# GridSync — Grid-Tied Inverter with Anti-Islanding & Droop Control ⚡🔌

**The complete control core of a grid-connected inverter on ESP32: SOGI-PLL synchronisation, Proportional-Resonant current control, anti-islanding protection with automatic reconnection, and droop-based microgrid load sharing.**

Built and demonstrated on a **safe 24 V low-voltage demo bus** — the same control mathematics that runs in commercial solar inverters and microgrid controllers, at bench scale.

## 🧠 The four control systems

### 1. SOGI-PLL synchronisation
A Second-Order Generalised Integrator generates the grid voltage's orthogonal (α/β) components — implemented with proper **Tustin (bilinear) discretization** whose coefficients track the measured grid frequency every step — giving clean phase tracking even with a distorted or noisy grid waveform. The PLL reports tracked frequency and lock state continuously.

### 2. PR current control
Proportional-Resonant controllers track the sinusoidal current reference with (theoretically) zero steady-state error at 50 Hz — the standard reason PR beats PI in single-phase inverter control. The resonant term is **frequency-adaptive** (its peak follows the PLL's tracked frequency), with anti-windup and an optional 5th-harmonic resonant bank. P and Q setpoints are set independently from the dashboard.

### 3. Anti-islanding protection + reconnection
The safety function that stops an inverter energising a dead grid during an outage:
- Voltage window 0.88–1.10 pu and frequency window 47.5–51.5 Hz
- ROCOF (rate of change of frequency) detection
- **Active frequency shift** perturbation that pushes an islanded system out of its window so the passive checks trip fast
- A full **reconnection state machine**: TRIPPED → WAIT (30 s) → VERIFY (grid inside windows + PLL locked, continuously 5 s) → RESYNC/RAMP (current ramped up over 3 s) → RUN — no instant restart after a trip

### 4. Droop microgrid mode
With the grid absent, the inverter forms the bus for a **24 V DC-coupled microgrid demo**: classic f–P and V–Q droop laws let multiple inverters share load in proportion to their ratings with **no communication between them**.

## 📂 In this repo (multi-file firmware)

- `GridSync_Inverter.ino` — main sketch: sensing, modes, protection integration, dashboard
- `sogi_pll.h` — SOGI quadrature generator + PLL (Tustin discretization, DC-offset rejection, lock detector)
- `pr_control.h` — frequency-adaptive PR current controller with anti-windup
- `protection.h` — anti-islanding checks + reconnection state machine
- `components.md` — component list with the role of each part
- `protection-notes.md` — the anti-islanding design notes: windows, ROCOF, active shift, and honest limits

## 🖥️ Dashboard

Grid voltage/frequency, PLL lock state, P/Q output, mode (grid-tied / droop / island-tripped), protection state with reconnect countdown and trip reason, P & Q setpoint controls and an event log — served by the ESP32.

## 📚 Required libraries

Board support (once): **esp32 by Espressif Systems** — Arduino IDE → Boards Manager.

**No external libraries to install.** The firmware uses only libraries built into the ESP32 Arduino core (`WiFi`, `WebServer`) — the SOGI-PLL, PR controller and protection code are all in this repo.

## 🔌 Wiring — pin connections (ESP32)

| ESP32 pin | Connects to | Notes |
| --- | --- | --- |
| GPIO34 | ZMPT101B output | Grid (demo-bus) voltage sensing |
| GPIO35 | ACS712 output | Inverter output current sensing |
| GPIO25 | Full-bridge leg 1 — PWM | LEDC channel 0, 20 kHz |
| GPIO27 | Full-bridge leg 2 — PWM | LEDC channel 1, opposite phase |
| GPIO26 / GPIO14 | Bridge leg 1 / leg 2 low-side signals | Pin map reserves these where the driver board takes separate low-side inputs; the demo driver board generates complementary signals + dead-time in hardware |
| GPIO13 | Grid contactor relay | Closed only while the protection state machine allows output |
| GPIO32 | DC-bus voltage divider output | ADC input |
| GND | Sensor / driver ground | Common ground throughout |

## ⚠️ Safety — read first

**This is a control-core demonstration on an isolated 24 V bus. It is NOT a design for connecting equipment to the public grid.** Real grid connection in India requires type-tested, certified inverters and utility approval (CEA technical standards). Never connect a DIY inverter to mains wiring.

## 👤 Author

**Deepesh Chauhan** — B.Tech Electrical Engineering, RGGEC Kangra
[LinkedIn](https://www.linkedin.com/in/deepeshhh1)
