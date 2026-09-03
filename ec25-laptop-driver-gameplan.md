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
