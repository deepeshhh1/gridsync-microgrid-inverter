# GridSync Anti-Islanding — Design Notes

## Why islanding matters
If the utility grid fails and a local inverter keeps energising its section, line workers can meet a "dead" line that is actually live, and the island can drift out of sync so reconnection damages equipment. Grid codes therefore require inverters to detect islanding and disconnect quickly (standards family: IEEE 1547 / IEC 62116; Indian CEA technical standards for connectivity).

## What GridSync implements
1. **Passive windows** — trip if V < 0.88 pu or V > 1.10 pu, or f < 47.5 Hz or f > 51.5 Hz. Simple and robust, but they have a **non-detection zone (NDZ)**: if the island's local load happens to match generation closely, V and f barely move.
2. **ROCOF** — trip on sustained |df/dt| > 2.5 Hz/s, catching fast frequency slides the window misses.
3. **Active frequency shift** — the inverter deliberately biases its output frequency slightly with load; on a stiff grid nothing happens, but in an island the bias accumulates, pushing f out of the window where the passive check trips. This shrinks the NDZ.

## Honest limits
- These are **demonstration-grade** implementations on a 24 V bus: thresholds and timings are set for teaching visibility, not copied from a certification test plan.
- IEC 62116 certification tests islanding against a tuned RLC load at exact power balance — passing it requires laboratory testing of complete hardware, which no firmware listing can claim.
- Real products add multiple redundant methods (impedance measurement, Sandia frequency shift with positive feedback, SMS) tuned per grid code.

## Droop note
In islanded microgrid mode the anti-islanding trip is **disabled by design** — the inverter is *supposed* to form the bus. Mode discipline matters: the firmware requires the grid-tied mode to be selected explicitly, and the island-trip event latches until the operator resets it.
