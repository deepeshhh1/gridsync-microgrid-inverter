# Components — GridSync Grid-Tied Inverter (24 V Demo Bus)

| S.No. | Component | Purpose | Qty |
| --- | --- | --- | --- |
| 1 | ESP32 DevKit V1 (30-pin) | Control core (PLL, PR control, protection) + dashboard | 1 |
| 2 | ZMPT101B Voltage Sensor Module | Grid/demo-bus AC voltage sensing | 1 |
| 3 | ACS712 Current Sensor (5 A/20 A) | Inverter output current sensing | 1–2 |
| 4 | Gate Driver + MOSFET H-Bridge (or integrated inverter module) | Power stage switching the demo bus | 1 |
| 5 | LC Output Filter (inductor + capacitor) | Sine-wave shaping at the bridge output | 1 set |
| 6 | 24 V DC Bench Supply / Battery | Demo DC bus source | 1 |
| 7 | 24 V AC Source (isolation transformer / signal source) | Safe stand-in "grid" for synchronisation tests | 1 |
| 8 | Relay / Contactor Module | Physical connect-disconnect for islanding tests | 1 |
| 9 | AC Loads (lamps / small fan, demo ratings) | Visible power delivery + droop load sharing | 1 set |

**Scope reminder:** everything above is low-voltage demonstration hardware. No component in this list is rated or intended for public-grid connection.
