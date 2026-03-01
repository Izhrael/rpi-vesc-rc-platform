# VESC USB Bring-Up Plan (Raspberry Pi 5, Debian 13, Python 3.13)

## 1) Recommended library strategy

### Recommendation
Use **`pyserial` + a tiny local VESC packet client** as the primary path for Python 3.13.

Why:
- `pyvesc` ecosystem packaging has had long-standing split/fork issues (`pyvesc`, `pyvesc-fix`, import shadowing).
- Python 3.13 compatibility is still uneven across older VESC wrappers.
- USB/CDC serial is simple enough to control directly with <200 lines of maintainable code.

### Should you downgrade to Python 3.11?
Not required for USB telemetry if you use raw protocol with `pyserial`.
Downgrade only if you have another dependency that strictly requires 3.11.

## 2) Clean dependency setup (no pyvesc conflicts)

```bash
# From your project root
python3.13 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel

# Remove any conflicting wrappers
python -m pip uninstall -y pyvesc pyvesc-fix

# Keep runtime minimal and deterministic
python -m pip install pyserial==3.5
python -m pip freeze > requirements.lock.txt
```

## 3) Safety-first architecture

- `scripts/vesc_protocol.py`
  - Low-level packet encode/decode (frame, CRC16, payload).
  - Commands used now:
    - `COMM_GET_VALUES` (read-only telemetry)
    - `COMM_SET_CURRENT_BRAKE(0.0)` and `COMM_SET_CURRENT(0.0)` for neutral/safe state
  - Optional gated command:
    - `COMM_SET_RPM` only after telemetry stability checks pass.

- `scripts/vesc_telemetry.py`
  - Read-only polling script.
  - Prints key fields and raw hex fallback for firmware variance.

- `scripts/vesc_control.py`
  - Explicit CLI command for neutral.
  - Separate subcommand for RPM with conservative default and confirmation flag.

## 4) Step-by-step validation sequence (no motor movement first)

1. **Device sanity**
   ```bash
   ls -l /dev/ttyACM0
   dmesg | tail -n 50
   ```

2. **Read-only telemetry only**
   ```bash
   source .venv/bin/activate
   python scripts/vesc_protocol.py --port /dev/ttyACM0 telemetry --samples 20 --interval 0.2
   ```
   Pass criteria:
   - Repeated valid `COMM_GET_VALUES` responses.
   - Stable voltage/current/RPM fields (or raw payload if firmware map differs).

3. **Neutral command check (still no spin command)**
   ```bash
   python scripts/vesc_protocol.py --port /dev/ttyACM0 neutral
   ```
   Pass criteria:
   - Command transmit succeeds.
   - Subsequent telemetry still works.

4. **Only then controlled RPM**
   ```bash
   # Very low magnitude first; keep wheels off ground.
   python scripts/vesc_protocol.py --port /dev/ttyACM0 rpm --value 300 --confirm
   ```
   Pass criteria:
   - RPM responds as expected.
   - Can return to neutral immediately.

5. **Fail-safe stop always available**
   ```bash
   python scripts/vesc_protocol.py --port /dev/ttyACM0 neutral
   ```

## 5) Reliability notes

- Use `timeout=0.2` and explicit frame parsing instead of line-based reads.
- Avoid mixed package environments; keep one venv per project.
- If telemetry intermittently fails, increase USB cable quality and test powered hub.
- Prefer fixed baud (typically `115200`) for CDC serial and keep it explicit.

## 6) Decision: pyvesc vs raw protocol

For your constraints (USB only, safety, Python 3.13, deterministic bring-up),
**raw protocol via `pyserial` is the minimal reliable path forward**.

You can adopt a wrapper later, but keep this low-level path as a known-good fallback.
