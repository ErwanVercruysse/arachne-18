# Frozen v4 component baseline

These five solids are the previously checked isolated leg/body prototypes, before full-robot integration. `check_v4.py` checks their equivalence to the current build. STEP comparison ignores only generated product labels and timestamps; remaining differences use a geometric subtraction check.

The corresponding fit reports retain the original file hashes. `../ST3215.step` is the manufacturer reference used for case, fastener and insertion checks. The assembled visual servo is a simplified valid exterior model in `src/v4_servo.py`: the vendor STEP includes an invalid internal solid and is not used as the native assembly part.
