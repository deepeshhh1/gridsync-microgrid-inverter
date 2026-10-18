# GridSync — Grid-Tied Inverter with Anti-Islanding & Droop Control ⚡🔌

**The complete control core of a grid-connected inverter on ESP32: SOGI-PLL synchronisation, Proportional-Resonant current control, anti-islanding protection, and droop-based microgrid load sharing.**

Built and demonstrated on a **safe 24 V low-voltage demo bus** — the same control mathematics that runs in commercial solar inverters and microgrid controllers, at bench scale.

## 🧠 The four control systems

### 1. SOGI-PLL synchronisation
A Second-Order Generalised Integrator generates the grid voltage's orthogonal (α/β) components, giving clean phase tracking even with a distorted or noisy grid waveform. The PLL reports tracked frequency and lock state continuously.

### 2. PR current control
Proportional-Resonant controllers track the sinusoidal current reference with (theoretically) zero steady-state error at 50 Hz — the standard reason PR beats PI in single-phase inverter control. P and Q setpoints are set independently from the dashboard.

### 3. Anti-islanding protection
The safety function that stops an inverter energising a dead grid during an outage:
- Voltage window 0.88–1.10 pu and frequency window 47.5–51.5 Hz
- ROCOF (rate of change of frequency) detection
- **Active frequency shift** perturbation that pushes an islanded system out of its window so the passive checks trip fast

### 4. Droop microgrid mode
With the grid absent, the inverter forms the bus for a **24 V DC-coupled microgrid demo**: classic f–P and V–Q droop laws let multiple inverters share load in proportion to their ratings with **no communication between them**.

## 📂 In this repo

- `GridSync_Inverter.ino` — complete firmware (SOGI-PLL, PR control, protection, droop, dashboard)
- `components.md` — component list with the role of each part
- `protection-notes.md` — the anti-islanding design notes: windows, ROCOF, active shift, and honest limits

## 🖥️ Dashboard

Grid voltage/frequency, PLL lock state, P/Q output, mode (grid-tied / droop / island-tripped), P & Q setpoint controls and an event log — served by the ESP32.

## ⚠️ Safety — read first

**This is a control-core demonstration on an isolated 24 V bus. It is NOT a design for connecting equipment to the public grid.** Real grid connection in India requires type-tested, certified inverters and utility approval (CEA technical standards). Never connect a DIY inverter to mains wiring.

## 👤 Author

**Deepesh Chauhan** — B.Tech Electrical Engineering, RGGEC Kangra
[LinkedIn](https://www.linkedin.com/in/deepeshhh1)
