# EC25 Laptop Tools

Probe the attached Quectel EC25-AFXD without starting a data session or changing
host routes:

```bash
./.venv/bin/python -m openpilot.tools.modem.ec25_bench
```

The probe discovers USB ID `2c7c:0125` through sysfs and maps ports by USB
interface number, so it does not depend on a fixed `ttyUSB` enumeration order.
It tries the primary AT port first and falls back to the secondary port when
ModemManager holds the primary port.

The default command set is read-only and deliberately excludes IMEI, IMSI, and
ICCID queries. It does not expose an arbitrary AT-command option.

When no LTE antenna is attached, explicitly leave the radio in minimum
functionality mode:

```bash
./.venv/bin/python -m openpilot.tools.modem.ec25_bench --keep-radio-off
```

`--keep-radio-off` may issue `AT+CFUN=0`; it is the tool's only state-changing
operation. Treat that as a session-level safeguard and recheck after every USB
reconnect or power cycle.
