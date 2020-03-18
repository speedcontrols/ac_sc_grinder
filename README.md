Grinder speed control with stable RPM <!-- omit in toc -->
=====================================

[![Build Status](https://travis-ci.org/speedcontrols/ac_sc_grinder.svg?branch=master)](https://travis-ci.org/speedcontrols/ac_sc_grinder)
[![Gitter chat](https://badges.gitter.im/speedcontrols/ac_sc_grinder.svg)](https://gitter.im/speedcontrols/ac_sc_grinder)

> Advanced speed control for grinder's AC brushed motor. With RPM stabilization
> via Back EMF.


Video:

[![Video](https://i.ytimg.com/vi/6eNhbyeh3mg/hqdefault.jpg)](https://youtu.be/6eNhbyeh3mg)

Schematic/PCB:

- [EasyEda project page](https://easyeda.com/speed/AC-speed-control-for-grinder) -
  you can order PCB & all components there in couple of clicks!

Note! Due size restrictions, it's impossible to create universal PCB for all
kinds of grinders. We prepared PCB for `Hilda 180W` - it's very popular and
cheap. Boards for other models are left to volunteers.


## Build manual

[See illustrated instruction](https://github.com/speedcontrols/ac_sc_grinder/blob/master/doc/assembly.md).


## Development notes

Everything is done in `PlatformIO`. If you wish to update yaml config and
regenerate headers, you may need `node.js` and run `npm run config`.

Note, this PCB has no AC isolation! That's ok for normal operation, but if you
plan to debug firmware via USB, you MUST use [USB isolator](https://www.aliexpress.com/wholesale?SearchText=usb+isolator)
module.
