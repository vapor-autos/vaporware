# EC25-AFXD Laptop Driver Gameplan

Date: 2026-09-02

## Goal

Develop and validate support for an external Quectel EC25-AFXD on the Linux
laptop before changing the comma four. The immediate goal is safe hardware
bring-up and a reusable modem service. The end goal is a controlled A/B test
against the comma four's internal EG916Q-GL using the same LTE carrier,
location, antennas, WebRTC workload, and metrics.

The external modem is intended to provide more uplink headroom for rover video:

```text
internal EG916Q-GL: LTE Cat 1 bis, nominal uplink about 5 Mbps
external EC25-AFXD: LTE Cat 4, nominal uplink about 50 Mbps
```

The Cat 4 rating is not itself proof of lower latency or loss. RF installation,
carrier scheduling, cell congestion, APN, and antenna quality still need an A/B
test.

## Bench Safety State

The module currently has no SIM and no antennas. It initially reported
`AT+CFUN? -> 1`, meaning full functionality. It was deliberately changed to
`AT+CFUN=0` and verified at `CFUN: 0` after the identity probe.

Treat that as a session-level safeguard, not a persistent guarantee. After any
power cycle or USB reconnect, query `CFUN` again and assume it may return to 1.

Keep it in this state until the RF hardware is ready:

- Unplug power before attaching or removing the small RF connectors.
- Connect the LTE `MAIN` antenna before returning to `CFUN=1`.
- Connect the LTE diversity/receive antenna too; it is strongly recommended for
  a useful performance comparison.
- Do not accidentally connect an LTE antenna to the GNSS port.
- Insert the SIM with the board unpowered unless the exact carrier board is
  confirmed to implement safe hot-swap behavior.
- Confirm that the carrier board's 5 V supply can handle modem transmit-current
  peaks. The USB descriptor advertises 500 mA, but that is not a substitute for
  verifying the carrier board's actual power design.

Until then, allowed work is USB discovery, local parser/state-machine work,
unit tests, identity queries, and radio-off behavior. Do not run registration,
signal, throughput, or WebRTC-over-cellular tests without the LTE antenna.

## Probe Results

The attached board is the expected device and Linux support is already present:

```text
USB ID:        2c7c:0125
USB product:   Quectel EC25-AFXD
AT model:      EC25
firmware:      EC25AFXDGAR07A01M1G
USB speed:     480 Mbps (USB 2 high speed)
USB mode:      QMI (AT+QCFG="usbnet" -> 0)
SIM state:     absent
radio state:   CFUN=0 after probe
```

ModemManager also reports the expected North American LTE bands:

```text
B2, B4, B5, B12, B13, B14, B66, B71
```

Linux exposed this composition:

| USB interface | Linux endpoint | Observed role | Phase-one use |
| --- | --- | --- | --- |
| `00` | `/dev/ttyUSB0` | Qualcomm diagnostic/QCDM | Ignore |
| `01` | `/dev/ttyUSB1` | GNSS/NMEA | Ignore |
| `02` | `/dev/ttyUSB2` | Primary AT | Radio control and metrics |
| `03` | `/dev/ttyUSB3` | Secondary AT/modem | First PPP data path; safe probe port while ModemManager owns primary |
| `04` | `/dev/cdc-wdm0` | QMI control | Optional later comparison |
| `04` | `wwan0` | QMI network interface | Optional later comparison |

The kernel bound `option`, `usb_wwan`, `qmi_wwan`, and `cdc_wdm` without custom
drivers. `/dev/serial/by-id/` aliases also exist for all four serial interfaces.

ModemManager discovered the modem but entered `failed: sim-missing`, which is
the correct broad diagnosis. It owns `/dev/ttyUSB2` exclusively; `/dev/ttyUSB3`
was available for the read-only AT probe. A direct `qmicli` attempt while
ModemManager owned the QMI endpoint ended with an endpoint hangup. Do not mix
independent QMI clients. One service must own QMI control at a time.

The presence of QMI endpoints is a capability, not a requirement to use them.
The production-parity plan below starts with PPP and only introduces QMI if a
controlled comparison proves that PPP is limiting the EC25.

## Live Comma Four Verification

The online comma four was inspected on 2026-09-02 using only short, read-only
shell commands and its producer-written modem state file.

The deployed hardware and port mapping are:

```text
internal modem:       Quectel EG916Q-GL
USB ID:               2c7c:6007
/dev/modem_at0:       /dev/ttyUSB2, USB interface 02
/dev/modem_at1:       /dev/ttyUSB3, USB interface 03
data bearer:          pppd on /dev/modem_at1
network interface:    ppp0
PPP speed argument:   460800
PPP MTU/MRU:          1500
PDP context:          CID 1, dialed with ATD*99***1#
```

The running driver is byte-for-byte identical to the local
`openpilot/common/hardware/tici/modem.py`. It is connected through `ppp0`,
installs the expected main and table-1000 routes, and registers DNS through
`systemd-resolved`. At inspection time it was registered on AT&T LTE Band 66
with strong RF values, showing that this is the active production path rather
than an unused compatibility mode.

Most importantly, `/etc/udev/rules.d/78-modem.rules` already contains support
for USB ID `2c7c:0125`, labeled there as EG25:

```text
interface 02 -> /dev/modem_at0
interface 03 -> /dev/modem_at1
```

The laptop EC25-AFXD has that exact USB ID and interface layout. This is direct
evidence that the existing comma PPP path was designed for the EC25/EG25 USB
family and should be the first integration target.

There is one important integration hazard: the rules for both `2c7c:6007` and
`2c7c:0125` create the same `/dev/modem_at0` and `/dev/modem_at1` names. If the
internal and external modems are present together, those links can become an
enumeration-order race. The external modem therefore needs distinct aliases or
the internal modem must be deliberately removed/disabled for the first test.

For the internal EG916Q, the same udev file explicitly unbinds its CDC Ethernet
function and the service uses PPP. This further confirms that native USB
networking is not part of the current production design.

The comma image also has built-in `CONFIG_USB_NET_QMI_WWAN=y`,
`CONFIG_USB_WDM=y`, `qmicli`, and `qmi-network`. QMI is technically available
for a later experiment. It is not used by the current driver. ModemManager is
runtime-masked while the comma modem service is running.

## What the Current Comma Driver Does

The current implementation is
`openpilot/common/hardware/tici/modem.py`. It is launched by manager only on
TICI-family hardware and provides the state consumed by `hardwared` and the
WebRTC metrics logger.

Its current flow is:

```text
/dev/modem_at0 -- Quectel AT control
/dev/modem_at1 -- PPP dial/data port
pppd            -- creates ppp0
modem.py         -- owns state machine, routes, DNS, polling, and recovery
/dev/shm/modem   -- atomic JSON state output
hardwared        -- maps JSON to openpilot network state
webrtcd          -- embeds JSON and parsed QENG radio metrics in test logs
```

Useful pieces to retain:

- Atomic `/dev/shm/modem` publication.
- The broad initialize/search/connect/disconnect state-machine shape.
- A shared lock around AT access.
- Existing APN and roaming parameter behavior.
- The existing JSON fields so `hardwared`, UI, and WebRTC metrics remain
  compatible.
- `QNWINFO`, `QENG="servingcell"`, `QTEMP`, and interface byte counters.

Parts that make it unsafe to run unchanged on this laptop:

- Device paths, `ppp0`, route table 1000, state path, and params path are hard
  coded.
- It stops and runtime-masks ModemManager globally.
- It uses `killall -9 pppd`, rather than terminating only its own child.
- It changes host routes and resolved configuration.
- Initialization requires both ICCID and IMEI, so a missing SIM is treated as
  an incomplete identity read and loops forever instead of becoming a stable
  `SIM_MISSING` state.
- The data plane is hard-coded to PPP. That is appropriate for the first EC25
  port, but the interface should still be configurable so PPP can be measured
  against QMI later without rewriting radio/state logic.
- LTE registration should prefer `CEREG`; the existing gate uses `CREG` plus
  `CGREG`.
- `CSQ` is coarse and detailed `QENG` values are stored as an opaque string.
- EG25-specific NV writes are model-gated today, which protects this EC25. That
  guard must remain; none of those writes should be generalized to EC25.

## Target Structure

Use one shared service with injected platform configuration and replaceable data
bearers:

```text
                         +--------------------+
/dev/ttyUSB2 ----------> | AT channel         |
                         | identity/SIM/radio |
                         | registration/RF    |
                         +---------+----------+
                                   |
                                   v
                         +--------------------+
                         | Modem service      |
                         | explicit states    |
                         | recovery policy    |
                         +----+----------+----+
                              |          |
             first/parity path |          | optional experiment
                              v          v
                   +-------------+  +-------------+
/dev/ttyUSB3 ----> | PPP bearer  |  | QMI bearer  | <--- /dev/cdc-wdm0
ppp0 <-------------|             |  |             | ----> wwan0
                   +------+------+  +------+------+
                          |                |
                          +-------+--------+
                                  v
                         configurable routing
                                  |
                                  v
                         /dev/shm/modem JSON
```

### Proposed code boundaries

```text
openpilot/common/hardware/modem/
  at.py                 serial transport, response framing, locking, redaction
  discovery.py          USB identity and stable endpoint resolution
  profiles.py           generic Quectel, EG25 legacy, and EC25 capabilities
  state.py              state enum, typed snapshot, atomic JSON publisher
  service.py            lifecycle and recovery state machine
  bearer.py             bearer interface
  ppp.py                existing PPP behavior, made instance-safe/configurable
  qmi.py                optional native QMI bearer, added only if justified

openpilot/common/hardware/tici/modem.py
  production entry point and comma-specific configuration

openpilot/tools/modem/ec25_bench.py
  laptop entry point; probe-only by default, no route replacement by default

openpilot/common/hardware/modem/tests/
  transcript/parser, state-machine, bearer, and recovery tests
```

Exact filenames can change during implementation, but the separation between
AT/radio state, data bearer, platform wiring, and publication should remain.

### Configuration object

Replace global paths and commands with an injected configuration containing at
least:

```text
AT endpoint
PPP endpoint or QMI control endpoint
network interface name
bearer type: qmi or ppp
state and lock paths
params/APN source
route table and route metric
whether default-route installation is allowed
whether ModemManager ownership may be changed
radio policy: keep-off, manual, or managed
```

The laptop defaults must be conservative:

```text
probe only
keep radio off when requested
do not stop ModemManager implicitly
do not replace the laptop default route
do not change DNS
do not start a bearer
```

State-changing flags should be explicit.

### Explicit service states

Use states that describe hardware and SIM conditions instead of retrying all
failures as initialization failures:

```text
ABSENT
PROBING
RADIO_OFF
SIM_MISSING
SIM_LOCKED
SEARCHING
REGISTERED
CONNECTING
CONNECTED
RECOVERING
STOPPING
```

`SIM_MISSING` must still publish model, firmware, USB identity, and radio power
state. It must not dial, modify routes, or spin through repeated destructive
initialization.

### Model profiles

Keep generic and model-specific initialization separate:

- Generic Quectel: echo/result formatting, registration URCs, read-only
  identity/capability queries.
- EG25 legacy: preserve only the existing, already proven comma configuration.
- EC25-AFXD: start with no persistent NV writes, no band lock, and no USB mode
  change. The current USB composition exposes both the serial ports required
  by PPP and the optional QMI endpoints, so it should be left unchanged.

Every persistent command needs a documented reason, model/firmware guard, and
an idempotence test before it is allowed on hardware.

### Data bearer boundary

Both bearers should implement the same small lifecycle:

```text
prepare()
connect(apn, roaming_allowed)
status()
disconnect()
recover()
```

The service should obtain interface name, assigned IP, gateway, DNS, and byte
counters from the selected bearer instead of assuming `ppp0`.

PPP is the first and default target because:

1. It is the active, production-tested comma four path.
2. The comma image already maps USB ID `2c7c:0125` interfaces 02 and 03 to the
   exact device names expected by the driver.
3. The EC25 exposes the same serial layout as the internal EG916Q.
4. The immediate WebRTC workload is roughly 1-3 Mbps, not tens of Mbps.
5. It avoids introducing QMI client/session/raw-IP recovery logic before the
   modem and antenna improvement itself has been measured.

The `460800` pppd argument should not be assumed to impose a literal 460.8 kbps
cap on a USB bulk serial function. Existing comma LTE tests already transported
more than 1 Mbps through this configuration. Actual EC25 PPP capacity still
needs measurement; it should not be inferred from either the baud argument or
the modem's Cat 4 headline.

QMI remains an optional second backend because it may improve peak throughput,
CPU use, or recovery behavior. It becomes worth implementing only if EC25 PPP
is healthy at the RF layer but fails the required workload or shows a clear
bearer bottleneck.

### Ownership

Only one of these may own the modem at a time:

```text
ModemManager/NetworkManager
custom EC25 service
manual qmicli/AT session
```

For an optional initial laptop carrier smoke test, ModemManager/NetworkManager
can provide a known-good OS-managed baseline. This does not select QMI for the
production design. For the PPP service, explicitly inhibit or stop
ModemManager for that device/session, use the two serial ports exclusively, and
restore it on exit. Never probe the QMI endpoint independently while
ModemManager owns it.

The custom service must terminate only its own processes and undo only routes,
rules, and DNS entries that it installed.

### Compatible state output

Keep the current fields and add structured fields without breaking existing
readers:

```text
existing:
  state, connected, ip_address, iccid, mcc_mnc, imei, modem_version
  signal_strength, signal_quality, network_type, operator, band, channel
  registration, temperatures, extra, tx_bytes, rx_bytes

new:
  sim_state, radio_power, bearer, interface, usb_id
  rsrp, rsrq, sinr, rssi, cell_id, pci, uplink_bandwidth
  last_error, recovery_count
```

Sensitive identifiers should be redacted in normal logs even if they remain
available to the local hardware API where required.

## Implementation and Test Phases

### Phase 0: Safe enumeration — complete

Acceptance criteria already met:

- Exact EC25-AFXD identity verified.
- Standard serial and QMI ports present.
- Kernel drivers loaded automatically.
- Primary/secondary AT roles identified.
- Firmware and USB mode read.
- Missing SIM detected.
- Radio left at `CFUN=0`.

### Phase 1: Probe-only laptop tool — complete

Implemented on branch `ec25afxd-modem`:

```text
openpilot/common/hardware/modem/at.py
openpilot/common/hardware/modem/discovery.py
openpilot/common/hardware/modem/probe.py
openpilot/tools/modem/ec25_bench.py
openpilot/common/hardware/modem/tests/
```

Completed behavior:

- Discover by USB VID/PID plus interface number, not `ttyUSB` enumeration order.
- Print endpoints, model, firmware, SIM state, USB mode, and radio state.
- Default to read-only commands.
- Support `--keep-radio-off` and refuse registration/data actions without an
  explicit radio-enable option.
- Detect ModemManager ownership and either use the unclaimed secondary AT port
  for limited probing or exit with a clear ownership message.
- Never change routes or start PPP/QMI in probe mode.

Laptop verification:

```text
USB discovery:       3-4.4, 2c7c:0125, EC25-AFXD
primary AT:          /dev/ttyUSB2, busy under ModemManager
selected fallback:   /dev/ttyUSB3
AT identity:         Quectel EC25, EC25AFXDGAR07A01M1G
SIM state:           missing
radio:               CFUN=0
USB network mode:    0 (QMI endpoints present; no QMI session started)
route/data changes:  none
unit tests:          10 passed
```

The query set intentionally excludes IMEI, IMSI, and ICCID. The CLI has no
arbitrary AT-command or radio-enable option. `--keep-radio-off` is its only
possible state-changing action.

### Phase 2: Extract and unit-test the reusable core

Refactor without changing comma-four production behavior yet.

Tests should cover:

- AT echo on/off, URCs interleaved with command responses, timeout, `ERROR`, and
  `+CME ERROR` parsing.
- Missing, locked, and ready SIM states.
- `CEREG`, `CGREG`, and `CREG` response variants.
- EC25 identity not triggering EG25 NV commands.
- Atomic state publication.
- Route and DNS cleanup affecting only resources owned by the instance.
- Child-process cleanup by PID rather than global `killall`.
- USB unplug/replug and port renumbering.

### Phase 3: Antenna/SIM carrier smoke test

Prerequisites:

1. Unplug the modem.
2. Attach labeled `MAIN` and diversity LTE antennas.
3. Insert the SIM.
4. Confirm adequate carrier-board power.
5. Reconnect USB and verify endpoints again.
6. Return to `CFUN=1` only after the antenna check.

#### Optional no-SIM RF smoke test

The antennas can be installed before the SIM arrives, but the result has a
strictly limited meaning. Quectel support notes that `QENG="servingcell"` data
seen after SIM removal may be historical, and that current serving-cell radio
values are normally available only when the modem is connected to or camped on
a network. Without a SIM, the module cannot perform normal authenticated
registration or establish a PDP/PPP data session.

After attaching both LTE antennas while the board is unpowered, a cold-booted,
bounded smoke test may do only this:

```text
verify CFUN=0 after reconnect
explicitly set CFUN=1
wait no more than 60 seconds for limited-service camping/search
query CPIN, CEREG, CSQ, and QENG="servingcell"
record any current LTE band/RSRP/RSRQ/RSSI/SINR response
return to CFUN=0 in a finally/cleanup path
```

Do not change scan order, band masks, USB mode, APN, or persistent modem
settings for this test. Do not run `COPS=?`, PPP, or throughput tests.

Interpretation:

- `SEARCH`, `LIMSRV`, unknown signal, or no serving-cell response is normal
  without a SIM and does not condemn the antennas.
- A plausible serving-cell response proves only that the receiver can observe
  or retain a cell; it is not an antenna benchmark.
- Do not compare antenna configurations by disconnecting RF leads while the
  modem is powered.
- Real antenna and link evaluation begins only after `AT+CPIN?` returns
  `READY`, normal LTE registration succeeds, and RF metrics can be correlated
  with PPP traffic.

The current `+CME ERROR: 13` formally means `SIM failure` in Quectel's EC25 AT
manual. Since the SIM is physically absent and `QSIMSTAT` also reports absent,
it is recorded as part of the present no-SIM state. If error 13 remains after a
known-good SIM is inserted, stop and troubleshoot the SIM socket/detection
before any network testing.

References:

- Quectel EC25 Series Hardware Design V2.4, including `ANT_MAIN`, `ANT_DIV`,
  and default-enabled receive diversity:
  <https://quectel.com/content/uploads/2024/02/Quectel_EC25_Series_Hardware_Design_V2.4-4.pdf>
- Quectel EC25/EC21 AT Commands Manual V1.3, including CME error codes:
  <https://quectel.com/content/uploads/2021/03/Quectel_EC25EC21_AT_Commands_Manual_V1.3.pdf>
- Quectel support discussion of retained serving-cell data without a SIM:
  <https://forums.quectel.com/t/at-qeng-servingcell-responds-with-details-even-when-no-sim/12244>

An optional short ModemManager/NetworkManager connection can separate RF, SIM,
APN, and carrier-provisioning problems from bugs in the new service. It is a
diagnostic baseline, not the production bearer decision. If ownership or QMI
behavior is messy, skip the data portion and proceed to the PPP phase after AT
registration is confirmed.

Acceptance criteria:

- SIM becomes ready without exposing identifiers in shared logs.
- LTE registration succeeds using the expected APN.
- Band and structured RSRP/RSRQ/SINR are reported.
- If the OS-managed data smoke test is used, it obtains an IP on `wwan0`.
- The first connection is configured as non-default or high-metric so it cannot
  unexpectedly replace the laptop's normal network.
- Bound `ping`, DNS, HTTPS, and upload tests work through `wwan0`.

### Phase 4: PPP production-parity backend

Adapt the existing service to use `/dev/ttyUSB2` for AT control and
`/dev/ttyUSB3` for `pppd`, matching the live comma setup. Keep the first laptop
connection isolated from the default route.

Required behavior:

- Claim/inhibit the device cleanly and restore ModemManager on exit.
- Configure CID 1 and dial `ATD*99***1#` through `/dev/ttyUSB3`.
- Own and terminate only the service's own `pppd` process.
- Read assigned local/peer IP and carrier DNS without replacing the laptop's
  normal route during initial tests.
- Make interface, route table, metric, and DNS behavior configurable.
- Publish `ppp0` status and byte counters through the existing state contract.
- Handle `SIM_MISSING` without repeatedly dialing or reinitializing.
- Prefer LTE `CEREG` for registration while retaining compatible fallbacks.

This phase proves how close the EC25 is to drop-in support for the current
driver. It is the first candidate for the production UGV path, not merely a
fallback.

### Phase 5: Recovery tests

Exercise these before comma-four deployment:

- Start with no SIM.
- Start with radio intentionally off.
- Wrong or missing APN.
- USB unplug/replug with changed `ttyUSB` numbers.
- Data-session process exit.
- Loss of LTE registration and later recovery.
- SIM removal only if the carrier board explicitly supports hot swap.
- Laptop suspend/resume.
- Repeated clean start/stop with no leaked routes, rules, DNS, clients, or
  processes.

### Phase 6: Comma four integration

Before copying the laptop design to the UGV, verify on the comma four:

- External EC25 enumeration on the physical comma USB host. Kernel QMI support
  and tools are already present if a later QMI test is needed.
- Exact external USB enumeration and stable udev naming.
- USB power budget and a robust mechanical/power connection.
- How the internal EG916Q and external EC25 are distinguished.

Use udev rules based on `2c7c:0125` plus USB interface number for the external
EC25. Do not rely on `ttyUSB2`/`ttyUSB3` numbers when the internal modem is also
present.

For the first UGV test, select exactly one active data modem. Do not let two
services race over `/dev/shm/modem`, default routes, DNS, or route table 1000.
Make external-primary/internal-fallback a later explicit controller feature,
not an accidental consequence of both drivers running.

Preserve the current `/dev/shm/modem` contract so existing producer-written
WebRTC metrics continue to capture RF state without adding any live Cereal or
msgq subscribers.

### Phase 7: Controlled performance A/B using PPP

Compare internal EG916Q and external EC25 in the same place and time window,
using the same SIM/carrier where practical and the same WebRTC profiles.

Network tests:

```text
idle RTT and jitter
TCP upload/download sanity
iperf3 UDP uplink at 0.5, 1, 2.25, 5, 10, and 20 Mbps
loss, jitter, reordering, and sustained thermal behavior
```

WebRTC tests:

```text
2 cameras x 500 kbps
2 cameras x 750 kbps
2 cameras x 1 Mbps
3 cameras x 500 kbps
3 cameras x 750 kbps only if the prior step is clean
```

For every run record:

```text
modem and bearer (PPP for the primary comparison)
antenna setup and location
carrier, LTE band, EARFCN, cell ID, PCI
RSRP, RSRQ, SINR, RSSI
assigned MTU and IP family
actual tx bitrate
RTT and jitter
packet loss, NACK, and PLI
modem temperature
selected ICE candidate pair
```

The known internal baseline is marginal even around two cameras at 500 kbps
each, with later RTT around 73-124 ms and jitter around 16-24 ms in recorded
indoor LTE runs. The EC25 should be judged against those measurements, not only
against a speed-test headline.

If EC25-over-PPP meets the real WebRTC targets with acceptable recovery and
CPU use, keep PPP and stop here. Simpler production behavior is a valid win.

### Controlled no-SIM RF comparison (2026-09-03)

With both LTE antennas attached to the EC25 carrier board, we collected three
time-adjacent `QENG="servingcell"` samples from the laptop EC25-AFXD and the
comma four's internal EG916Q-GL on each target band. The EC25 had no SIM and
therefore camped in limited-service mode; the comma was registered to AT&T and
connected through PPP.

Natural reselection repeatedly put the EC25 on Band 2 and the comma on Band
66. For a controlled comparison, temporarily restrict each modem with
`AT+QCFG="band"`. Quectel band-mask writes must omit the `0x` prefix even
though query responses include it. The original masks were recorded first:

```text
EC25-AFXD:  band=0x260, LTE=0x42000000000000381a, TDS=0x0
EG916Q-GL:  band=0x0,   LTE=0x2000001e20b0e18df
Band 2:     LTE mask 2
Band 66:    LTE mask 20000000000000000
```

The comma Band 2 test used an on-device four-minute rollback timer so loss of
the cellular SSH path could not leave a persistent restricted mask. Both
original masks were restored and read back after testing, the rollback timer
was canceled, the comma returned to `CONNECTED`, and the EC25 returned to
`CFUN=0`.

Three-sample averages:

| Band | Modem | PLMN / EARFCN / PCI | State | RSRP dBm | RSRQ dB | RSSI dBm | SINR dB |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: |
| 66 | EC25-AFXD | 310410 / 67086 / 218 | limited service | -72.0 | -9.7 | -44.3 | 28.0 |
| 66 | EG916Q-GL | 310410 / 67086 / 218 | registered/PPP | -75.7 | -8.3 | -49.7 | 20.7 |
| 2 | EC25-AFXD | 311480 / 1100 / 267 | limited service | -74.0 | -9.0 | -44.3 | 13.3 |
| 2 | EG916Q-GL | 310410 / 650 / 218 | registered/PPP | -81.7 | -10.0 | -54.0 | 14.0 |

The Band 66 comparison is the cleanest RF A/B: both modems selected the same
PLMN, EARFCN, PCI, and cell ID (`1B34918`). The EC25 measured about 3.7 dB
better RSRP, 5.3 dB stronger RSSI, and 7.3 dB better SINR; the comma measured
about 1.3 dB better RSRQ.

The Band 2 samples only match frequency band, not carrier or cell. The EC25
camped on PLMN 311480 while the comma registered to AT&T PLMN 310410, so the
apparent EC25 advantage in RSRP/RSSI is not a controlled modem-or-antenna
comparison. At this location and time, Band 66 was substantially cleaner than
Band 2 for both devices, but the next fair data-path comparison still requires
a provisioned SIM in the EC25 and PPP measurements on the same carrier.

### Comma-to-GCS Band 2/Band 66 throughput baseline (2026-09-03)

Before moving the active AT&T SIM, we measured the comma four's internal
EG916Q-GL/PPP path to the GCS at `100.99.187.99`. Tailscale reported a direct
peer path rather than DERP. Each band used the same UGV, SIM, antennas, GCS,
test order, and `iperf3` durations. Band selection was forced with the masks
above, with a ten-minute automatic all-band rollback armed for Band 66.

| Measurement | Band 2 | Band 66 |
| --- | ---: | ---: |
| AT&T cell | EARFCN 650 / PCI 218 / `1B3490A` | EARFCN 67086 / PCI 218 / `1B34918` |
| RF at test start (RSRP / RSRQ / RSSI / SINR dB) | -82 / -8 / -56 / 14 | -74 / -10 / -46 / 23 |
| RF at test end (RSRP / RSRQ / RSSI / SINR dB) | -81 / -7 / -57 / 14 | -75 / -6 / -51 / 25 |
| Idle ping, 20 packets | 0% loss; 44.4 ms avg; 5.9 ms mdev | 0% loss; 56.0 ms avg; 12.2 ms mdev |
| TCP uplink, 20 s (receiver) | 3.96 Mbps; 56 retransmits | 4.17 Mbps; 53 retransmits |
| TCP downlink, 20 s (receiver) | 7.71 Mbps; 2 sender retransmits | 8.28 Mbps; 5 sender retransmits |
| UDP uplink, 1 Mbps offered | 0.997 Mbps; 0% loss; 5.99 ms jitter | 0.989 Mbps; 0% loss; 10.87 ms jitter |
| UDP uplink, 2.25 Mbps offered | 2.24 Mbps; 0% loss; 2.95 ms jitter | 2.24 Mbps; 0.35% loss; 1.58 ms jitter |
| UDP uplink, 4.5 Mbps offered | 4.07 Mbps; 7.7% loss; 1.39 ms jitter | 4.20 Mbps; 5.8% loss; 1.85 ms jitter |
| Modem temperature | 46-47 C | 45-48 C |

Band 66 delivered about 5% more TCP uplink and 7% more TCP downlink, and lost
fewer packets at the overloaded 4.5 Mbps UDP point. Band 2 had the better idle
latency: 11.7 ms lower average RTT and roughly half the RTT variation. Both
bands carried 1 Mbps without loss. Band 2 also carried the 2.25 Mbps UDP point
without loss, while Band 66 lost 0.35%, so neither band is an unconditional
winner for the current video workload.

The useful production baseline is approximately 4.0-4.2 Mbps sustained uplink
and 7.7-8.3 Mbps downlink through the internal modem, with clean 1 Mbps UDP on
either band. This is a band comparison, not yet an EC25 data-path comparison.
Move the same SIM to the EC25 and repeat this exact matrix over PPP before
attributing any throughput gain to the EC25's stronger Band 66 RF results.

After the test, the original EG916Q-GL all-band mask
`0x2000001e20b0e18df` was restored and read back, PPP returned to `CONNECTED`,
and the rollback timer was canceled. The modem naturally reselected Band 66.

### Laptop EC25 PPP performance baseline (2026-09-03)

The same AT&T SIM was moved from the comma four into the laptop EC25-AFXD.
After correcting the physical SIM orientation, the modem reported `CPIN:
READY`, registered home on AT&T `310410`, and attached to packet service. CID 1
already contained the `broadband` APN. ModemManager was stopped temporarily so
it could not race the PPP data port, and PPP was dialed on `/dev/ttyUSB3` with
`ATD*99***1#`, CHAP, MTU/MRU 1500, and the existing CID 1 context.

The laptop's Ethernet default route remained metric 100 throughout. A
secondary PPP default was added at metric 700, and every measurement command
was explicitly bound to `ppp0`. Ordinary laptop, GCS, and Tailscale traffic
therefore continued over Ethernet. The EC25 assigned working IPv4 and IPv6
addresses through PPP; the nominal `460800` USB-serial line setting was not a
throughput cap.

The EC25 selected the same AT&T Band 2 cell used for the comma Band 2 baseline:
EARFCN 650, PCI 218, cell `1B3490A`. During the EC25 run, RSRP was -70 to -69
dBm, RSRQ -8 to -11 dB, RSSI -44 to -38 dBm, and SINR 17-19 dB. Reported
temperatures at the end were 37/34/34 C.

This laptop cannot test the EC25 back to itself at the GCS Tailscale address.
The EC25 measurements therefore used public New York endpoints: Cloudflare
EWR for bounded HTTP transfers and `nyc.speedtest.clouvider.net`,
`spd-uswb.hostkey.com`, and `speedtest.nyc1.us.leaseweb.net` for iperf3. The
endpoint and transport-path difference means this is not yet the final
apples-to-apples UGV-to-GCS test, but the observed margin is much larger than
that limitation.

| EC25 PPP bulk measurement | Result |
| --- | ---: |
| Cloudflare download, 10 MB | 15.8 Mbps |
| Leaseweb TCP download, 10 s | 21.6 Mbps receiver |
| Cloudflare upload, 5 MB | 15.1 Mbps |
| Cloudflare upload, 20 MB | 16.4 Mbps |
| Clouvider TCP upload, 5 s | 17.3 Mbps receiver; 49 retransmits |
| Hostkey TCP upload, 5 s | 18.2 Mbps receiver; 77 retransmits |

The Cloudflare 20 MB upload ran with a simultaneous PPP-bound ping. It
sustained 16.4 Mbps while the complete ping window remained at 0% loss, 47.0
ms average RTT, and 133.8 ms maximum RTT. A separate clean idle window measured
0% loss, 46.7 ms average, and 81.8 ms maximum. Other idle windows did contain
sporadic 180-660 ms cellular scheduling spikes, so the short clean windows do
not establish a long-term latency guarantee.

| UDP offered | Receiver throughput | Loss | Jitter | Simultaneous ping over complete window |
| ---: | ---: | ---: | ---: | ---: |
| 1 Mbps | 0.990 Mbps | 0 / 690, 0% | 7.78 ms | not recorded |
| 4.5 Mbps | 4.46 Mbps | 2 / 3,883, 0.052% | 3.12 ms | not recorded |
| 8 Mbps | 7.93 Mbps | 0 / 6,902, 0% | 1.37 ms | not recorded |
| 12 Mbps | 11.9 Mbps | 0 / 10,352, 0% | 0.92 ms | 0% loss; 38.2 ms avg; 129.1 ms max |
| 16 Mbps | 15.9 Mbps | 16 / 13,811, 0.12% | 0.52 ms | 0% loss; 41.5 ms avg; 118.6 ms max |
| 20 Mbps | 18.4 Mbps | 997 / 17,263, 5.8% | 0.75 ms | 2.78% loss; 105.8 ms avg; 182.8 ms max |

The EC25 remained effectively clean through 12 Mbps. At 16 Mbps, all 16
missing datagrams occurred in the first reported second and the remaining
nine seconds were loss-free. The clear saturation point appeared at 20 Mbps:
receiver throughput flattened at 18.4 Mbps, UDP loss reached 5.8%, and loaded
ping latency and loss increased materially. The useful short-window real-time
ceiling at this cell was therefore approximately 16-18 Mbps.

For comparison, the internal EG916Q delivered only 4.07-4.20 Mbps at 4.5 Mbps
offered while losing 5.8-7.7%. The EC25 delivered the same 4.5 Mbps load with
0.052% loss, then delivered 8 and 12 Mbps with no loss. Its measured TCP
uplink was approximately 3.7-4.4 times the internal modem's 3.96-4.17 Mbps,
and its clean UDP operating region was roughly four times larger. This is
direct evidence that EC25-over-PPP can exploit the Cat 4 headroom; QMI is not
required for the present 3.5-4 Mbps WebRTC workload.

These numbers strongly predict that the EC25 will move the two-camera WebRTC
queueing/loss knee upward, but they do not replace the final full-stack test.
The decisive validation is to connect the EC25 directly to the UGV, repeat
the same UGV-to-GCS WebRTC ladder with driving feedback active, and confirm
NACK/PLI and RTCP RTT rather than relying on raw throughput alone.

The test ended with the root-owned PPP service stopped, `ppp0` and its metric
700 route removed, ModemManager restored, and Ethernet/Tailscale still online.

### Reconciling the iperf baseline with full-stack WebRTC

The approximately 4 Mbps iperf uplink result is not inconsistent with the
roughly 1.5 Mbps observed during a usable full-stack run. The measurements are
at different operating points and include different traffic.

The live UGV was verified on `steer-assist-phase-2` commit `59361a666`, whose
transport layout is newer than the master-based EC25 development branch:

```text
two H264 camera tracks       -> WebRTC/SRTP over the selected ICE path
UGV UI/model feedback        -> packed/framed UDP over Tailscale
high-rate G29/assist state   -> packed UDP over Tailscale
one-shot commands/settings   -> reliable WebRTC SCTP
```

The active GCS configuration requests `wideRoad,driver` at `low` quality plus
the `ui_model,steer_assist` feedback profiles. WebRTC quality values are
**per active camera**, because `LivestreamEncoderBitrate` applies to every
livestream encoder:

```text
low:     0.5 Mbps per camera
medium:  1.5 Mbps per camera
high:    5.0 Mbps per camera by default
```

Therefore, `medium` with two cameras asks for about 3 Mbps of video, not 1.5
Mbps total. The current fixed `low` request asks for about 1 Mbps of video and
disables the loss controller's automatic quality selection for that session.

A 27-minute producer-written GCS capture from 2026-09-02
(`00000168--0f590f4eb8`) confirms the current low-quality video load:

| WebRTC video measurement | Result |
| --- | ---: |
| Two-camera RTP payload average | 0.989 Mbps |
| Complete WebRTC transport average | 1.035 Mbps |
| Transport p50 / p90 / p99 | 1.041 / 1.077 / 1.182 Mbps |
| Maximum two-second transport sample | 1.319 Mbps |
| RTCP NACK feedback | 382 requests covering 1,775 packet numbers |
| PLI / final cumulative packet loss | 0 / 0 |

The zero final loss does not mean the path was loss-free. NACK retransmission
repaired late/missing RTP packets, which preserves throughput but can still
produce a visible pause and additional queueing delay.

The current WebRTC capture excludes the independent UDP UI-feedback path.
During the 2026-09-02 full-stack tests, feedback was about 0.54-0.58 Mbps before
`controlsState` was restored from an experimental 50 Hz target to 20 Hz. That
change reduced its measured contribution from about 170 kbps to 88 kbps, which
puts the comparable post-change feedback estimate around 0.46-0.50 Mbps.
Together with the measured 1.035 Mbps video transport, the application traffic
is approximately 1.50-1.54 Mbps before outer UDP/IP/WireGuard overhead. That
matches the observed full-stack rate; it is the selected workload, not proof
that the modem cannot exceed 1.5 Mbps. This is the best synchronized historical
decomposition available, but an exact current on-wire total still requires
capturing `ppp0` counters during the same active session.

There are smaller background consumers too. The normal `uploader` process is
running and metered-network policy still permits completed `qlog.zst` files.
Current files are about 231 KB per one-minute segment, equivalent to about 31
kbps averaged over a minute if uploaded continuously, but they arrive as short
TCP bursts and can briefly compete with real-time media. Athena, Tailscale, and
SSH also contribute. A quiet 30-second PPP sample with no GCS session measured
only 2.1 kbps uplink, so steady idle traffic is not the main limit; burst
competition is the concern.

#### Why WebRTC RTT can exceed the idle ping

The paths in the compared captures are not identical:

```text
iperf/ping test:
  UGV 100.67.29.97 -> GCS 100.99.187.99
  direct Tailscale/WireGuard path, no DERP

recent WebRTC video capture:
  GCS 192.168.50.17:38674 host candidate
  UGV 74.254.84.113:39399 server-reflexive candidate
  direct ICE/STUN path, no TURN relay and not Tailscale
```

A same-minute idle check on Band 66 measured 53.2 ms average RTT over
Tailscale. The GCS public endpoint did not answer ICMP, so a simultaneous idle
ICMP A/B against the exact ICE underlay is not available.

That path difference can move the latency, but it is not the main explanation
for the oldest 100-180 ms WebRTC samples. The two reported RTTs also have
different semantics:

- The 44-56 ms values were small ICMP packets sent once per second while the
  uplink was idle.
- WebRTC `roundTripTime` is calculated from RTCP sender/receiver reports while
  video is actively filling the LTE uplink. aiortc applies an EWMA with
  `alpha=0.85`, so a queueing episode remains visible for several samples.
- RTP jitter is packet arrival variation in the 90 kHz video clock. It is not
  the standard deviation of ping RTT and should not be compared directly to
  ping `mdev`.

The newer full-stack evidence supports queueing/RF rather than a relay penalty:

- On weak Band 2 (`RSRP -92` to `-93`, `SINR 1-6`), WebRTC RTT reached
  129-146 ms, jitter reached 52-56 ms, and one interval lost 15.7%.
- After the modem returned to stronger Band 66 (`RSRP -84`, `SINR 13-14`), the
  same two low-quality cameras showed 49-78 ms RTT, 8-11 ms jitter, and zero
  interval loss.
- The controlled idle Band 66 ping averaged 56 ms. The healthy loaded WebRTC
  range overlaps it, showing that WebRTC or ICE does not inherently double the
  RTT.

#### Why visible trouble starts below the 4 Mbps speed-test ceiling

The 4 Mbps figure is a throughput ceiling, not a safe real-time operating
budget:

1. TCP iperf tolerates retransmission and queue growth. It delivered about 4
   Mbps despite roughly 53-56 retransmissions in 20 seconds.
2. UDP already showed the real-time margin: 4.5 Mbps offered produced 5.8-7.7%
   loss, and Band 66 showed measurable loss even at 2.25 Mbps in this sample.
3. Two medium-quality cameras plus current feedback would request roughly 3.5
   Mbps before outer overhead, leaving very little room below the measured
   ceiling. Three low-quality cameras plus feedback are already around 2 Mbps.
4. H264 uses a GOP of five frames at 20 fps, producing frequent keyframe bursts
   rather than perfectly smooth average traffic. NACK retransmits add more
   bursts after loss.
5. Cellular scheduling, RF, and carrier congestion vary moment to moment. A
   mean rate below capacity can still overrun the queue during a short fade,
   keyframe, retransmission wave, or `qlog` upload.

Older notes that report 1.1-1.4 Mbps of JSON feedback and 2.1-3.3 Mbps inside
the WebRTC transport predate the packed independent-UDP feedback design. Do
not use those numbers as the current steady-state split, though they remain
useful evidence for why the former reliable SCTP feedback channel stalled.

#### 2026-09-03 controlled Band 66 full-stack ladder

The first synchronized bitrate ladder was completed on 2026-09-03 before
moving the SIM. The laptop ran the normal GCS manager and the UGV ran deployed
commit `59361a666`. No extra Cereal/msgq subscriber or manager PTY was attached.
The test used the two configured cameras (`wideRoad,driver`), changed the typed
`LivestreamEncoderBitrate` parameter live, allowed ten seconds to settle, and
held each point for approximately 50 seconds. The setting is per camera.

All points remained on Band 66, EARFCN 67086, PCI 218. RSRP stayed between
-75 and -72 dBm and median SINR was 24.5-26 dB, so neither a band change nor
weak RF explains the differences. ICE selected a direct UDP pair from the GCS
LAN host candidate to the UGV's server-reflexive public candidate; video did
not use Tailscale or a TURN relay. GCS and UGV producer-written two-second
WebRTC records were correlated with the modem driver's `ppp0` byte counter.

| Requested bitrate per camera | Encoder RTP payload, both cameras | GCS WebRTC receive | Total `ppp0` uplink | RTCP RTT avg / p90 / max | RTP jitter avg / max | Positive loss deltas | NACK packet requests | PLI |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 500 kbps | 0.999 Mbps | 1.039 Mbps | 1.306 Mbps | 48.9 / 53.6 / 56.9 ms | 11.4 / 15.7 ms | 0 | 16 | 0 |
| 750 kbps | 1.502 Mbps | 1.554 Mbps | 1.725 Mbps | 50.0 / 56.8 / 62.5 ms | 14.6 / 18.7 ms | 0 | 143 | 0 |
| 1,000 kbps | 2.004 Mbps | 2.069 Mbps | 2.357 Mbps | 56.0 / 64.6 / 67.6 ms | 20.1 / 115.0 ms | 324 | 2,987 | 4 |
| 1,250 kbps | 2.518 Mbps | 2.659 Mbps | 2.737 Mbps | 57.9 / 75.7 / 92.7 ms | 19.0 / 37.6 ms | 25 | 2,266 | 3 |
| 1,500 kbps | 3.025 Mbps | 3.258 Mbps | 3.469 Mbps | 75.8 / 99.4 / 136.0 ms | 25.2 / 48.6 ms | 148 | 5,458 | 11 |

`Positive loss deltas` sums the increases reported in each two-second
receiver sample. Packets that arrive late after NACK repair can reduce the
later cumulative loss value, so this column describes transient missing RTP,
not final unrecovered loss. A NACK can also request the same missing packet
more than once; `NACK packet requests` is repair work, not a unique-loss count.

The encoder achieved every requested aggregate rate and total PPP egress rose
from 1.306 to 3.469 Mbps. This directly disproves a fixed 1.5 Mbps modem or
full-stack cap. It also locates a practical real-time knee in this short run:
750 kbps per camera remained clean apart from light NACK repair, while every
point at or above 1,000 kbps per camera produced PLI, heavy NACK activity, or
materially higher RTT/jitter. The lower raw loss count at 1,250 than at 1,000
is normal short-window LTE variability, not evidence that the higher rate is
safer; its repair traffic and p90 RTT remained elevated.

This result also explains why the approximately 4.17 Mbps Band 66 TCP iperf
number is not a safe video target. The highest ladder point consumed 3.469
Mbps, about 83% of that bulk TCP result, yet RTCP RTT was already 27 ms higher
on average than at low quality and repair demand was substantial. TCP reports
useful bytes after retransmission and tolerates queueing. Two bursty H264
streams need headroom for keyframes, RTP repair, cellular scheduling changes,
feedback, and background uploads.

One limitation matters when calling this a full driving-stack result. The
normal GCS and UGV processes were running, but the UGV feedback counters showed
only `deviceState`, `pandaStates`, `roadCameraState`, and
`wideRoadCameraState` producing data during the stationary test. Their packed
payload averaged about 25 kbps. `carState`, `controlsState`, `selfdriveState`,
`modelV2`, and the remaining driving services sent zero bytes in this window.
Consequently, the 1.306 Mbps low-quality PPP result is the true total for this
stationary full-stack session, but not the final moving/engaged total. Adding
the historical post-50-Hz feedback estimate predicts roughly another
0.44-0.48 Mbps of application payload during a real drive, before its outer
headers. The producer metrics should be used to measure that delta during the
next normal route without adding a live message subscriber.

For the internal modem, 500 kbps per camera remains the conservative fixed
production point. The 750 kbps point is a credible longer-route candidate,
but this 50-second result is too short to promote it. Fixed settings at or
above 1,000 kbps per camera do not preserve enough real-time margin even under
the excellent Band 66 RF observed here. The test ended with
`LivestreamEncoderBitrate=500000`, the temporary GCS manager stopped, and the
UGV still connected on Band 66.

#### Remaining moving-route measurement

Repeat the following on each forced LTE band before moving the SIM:

1. Run the normal GCS manager and deployed UGV stack for at least ten minutes.
2. Record start/end and periodic `ppp0` byte counters for the true total
   on-wire cellular rate; correlate them with existing GCS/UGV producer files.
3. Keep a one-packet-per-second Tailscale ping running during the video session
   and compare it with RTCP RTT. This separates load/bufferbloat from path and
   metric-definition differences.
4. Record WebRTC transport bitrate, per-camera RTP bitrate, feedback UDP bytes,
   NACK/PLI/loss/jitter, band/cell/RSRP/RSRQ/SINR, and completed qlog uploads.
5. Test two cameras at low first, then three cameras at low. Test two cameras
   at medium only if the prior step remains clean.

For the later EC25 comparison, repeat the identical full-stack matrix as well
as iperf. The decision metric is bounded RTT/loss and uninterrupted UI/control
feedback at the target camera load, not maximum bulk TCP throughput.

### Phase 8: Optional QMI comparison

Implement or benchmark a QMI backend only if the PPP results show a reason:

- PPP cannot sustain the target upload while RF metrics remain good.
- PPP CPU use is material on the comma four.
- PPP adds measurable latency/jitter relative to an OS-managed QMI run.
- PPP recovery is materially worse than QMI after a stalled session.
- A future workload needs throughput well beyond the current 1-3 Mbps video
  target.

The comma image already provides QMI kernel support and `qmicli`/`qmi-network`,
so this remains feasible without deciding it up front. If implemented, the QMI
backend must use the same service state and publication contract so only the
bearer changes in the comparison.

## Decisions Already Made

- The purchased device is the intended EC25-AFXD and has the desired band set.
- Keep USB composition in QMI mode; do not issue a persistent `QCFG="usbnet"`
  change.
- Use `/dev/ttyUSB2` as primary AT when the custom service owns the modem.
- Use `/dev/ttyUSB3` for the first PPP data path or limited secondary probing.
- Use PPP/`ppp0` as the first production-parity data path.
- Retain QMI as an optional measured optimization, not a prerequisite.
- Never apply the existing EG25-only NV writes to EC25 by default.
- Preserve `/dev/shm/modem` compatibility for downstream consumers.
- Do not run the stock driver unchanged on the laptop.

## Remaining Gates

- Identify and photograph the carrier board's `MAIN`, diversity, and GNSS port
  labels before attaching antennas.
- Confirm antenna connector type and LTE band coverage.
- Confirm the SIM carrier, APN, PIN state, and provisioning.
- Verify the carrier board power source and current capacity.
- If PPP results justify a QMI comparison, confirm that the external EC25 binds
  to `qmi_wwan` on the comma four as it does on the laptop.
- Decide the explicit production policy: external-only, external-primary with
  manual fallback, or automatic dual-modem failover.

The next implementation target is Phase 2: extend the now-tested AT/discovery
core with explicit service state and compatible atomic publication. This can
continue while the module remains safely in `CFUN=0` and before the SIM or
antennas are fitted.
