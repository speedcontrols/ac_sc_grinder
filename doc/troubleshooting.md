Troubleshooting
===============

## Quick checklist

1. Check soldering defects. Consider cheap electric microscope, or microscope
   program for your mobile phone.
2. Check shunt amplifier orientation.
3. Check motor brushes are not broken.


## If calibration hangs at some point

Check current shunt amplifier output. The most frequent problem is reversed chip
orienation.


## If AC-DC input resistor(s) fires

You used wrong resistor type, not as recommended. Resistors should be one of:

- Fusible
- "High Pulse Withstanding" or "Anti-Surge" (SMD)
- MELF

And, unofficially... If you have absolutely no choice an need result urgent, try
usual 5% SMD resistors (without laser trimming) and mount several in vertical
column. This usually helps... but... no guarantees :)

Total resistance should be ~ 1K, as on schematic.
