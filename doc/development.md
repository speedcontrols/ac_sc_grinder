Development
===========

## Extra components

For gray Hilda 180W there is board with optional hall sensor and wireless
transmitter. That may help use board as lab stand to measure motor params.

This extra components may be needed:

&nbsp; | Name | Comment
-----|-------------|--------
1 | [Hall sensor](https://lcsc.com/product-detail/Magnetic-Sensors_HX-hengjiaxing-HX4913_C296270.html) | Tacho.
2 | [Magnetic resin](https://www.aliexpress.com/item/33017302814.html) | Do magnetic mark for hall sensor.
3 | [Magnetic field viewer](https://www.aliexpress.com/item/32967659973.html) |
4 | [RF sender](https://lcsc.com/product-detail/Wireless-Modules_nRF24L01-wireless-module_C84802.html) | Logs transmitter. Note, similar modules from AliExpress have dofferent power pinout.
5 | [RF receiver](https://www.aliexpress.com/item/4000112750588.html) | Logs receiver.
6 | [1.27mm 8 pin female conn](https://lcsc.com/product-detail/Pin-Header-Female-Header_BOOMELE-Boom-Precision-Elec-C92297_C92297.html) |
7 | [1.27mm male pins](https://lcsc.com/product-detail/Pin-Header-Female-Header_BOOMELE-Boom-Precision-Elec-1-27mm-1x50P_C3408.html) |
8 | [1.27mm pogo pins clip](https://www.aliexpress.com/item/32959606674.html) | SWD debug.
9 | [USB isolator](https://www.aliexpress.com/af/usb-isolator.html) | MUST be used with ST-Link/V2.


## Rework for different power/components

Base schematic is for 100-200W motors. For higher power you need to tune current
shunt & update triac.


### Current shunt

Existing 0.01R shunt is for ~ 200W motors @ 220v. For higher power increase
shunt value proportionally.


### Triac

Schematic with direct triac dive from MCU (via capacitor) allows only 10ma (max)
opening triacs - Z0409, ACST410. Those have 4A RMS and ok for most of needs.
For higher power you can:

- Use ACST610 / ACS1210
- Use 35ma opening triacs (BTA12-600CW), but control it via optocoupler with
  snubber network (as been done at v1 board).

Note, you may need to add (or remove) `-D REVERSE_TRIAC` flag in `platform.ini`,
to set proper polarity of triac control signal.

- MCU direct-driven triacs (v2 boards) need reversed polarity.
- v1 board with optocoupler needs non-reversed polarity (but optocoupler's led
  can be routed for reversed control).


### AC-DC input resistor(s)

When power on, input capacitor charge cause short, but very high pulse of power.
Ordinary resistors will fire from time to time. Use this kinds:

- Fusible.
- "High Pulse Withstanding" or "Anti-Surge" (SMD).
- MELF.

As quick hack (but not recommended) - try several ordinary 5% (not laser rimmed)
SMD resistors, mounted in vertical column.

Total value should be ~1K (as max as possible). Input cap should be 2.2uF (as
low as possible).


### LNK3204 replacement

You can use LNK304 or MP157, but you should update feedback resistors
(according to chip datasheet) to make output 4.0-4.2 volts.
