Device assembly <!-- omit in toc -->
===============

- [Required components](#required-components)
- [Extract some parts from original board](#extract-some-parts-from-original-board)
- [Solder PCB top and bottom](#solder-pcb-top-and-bottom)
- [Cleanup PCB](#cleanup-pcb)
- [Install the rest and attach wires](#install-the-rest-and-attach-wires)
- [Upload firmware and test](#upload-firmware-and-test)
- [Cover PCB with protective coating](#cover-pcb-with-protective-coating)


## Required components

\- | Name | Comment
-----|-------------|--------
1 | [Hilda 180W](https://www.aliexpress.com/af/hilda-180w.html?SortType=total_tranpro_desc) grinder | Such grinders are available under other labels. Any, exactly as on photos below, will be ok.
2 | [PCB & Components](https://easyeda.com/speed/AC-speed-control-for-grinder) | Go to EasyEda project page and order both in couple of clicks. If you order PCB first, components second, then you will be able to join delivery and save some bucks.
3 | [Cheap ST-link/V2 programmer](https://www.aliexpress.com/af/st-link-v2.html?SortType=total_tranpro_desc) | Required to upload firmware, only 2$. You can also order it at [LCSC](https://lcsc.com/search?q=st-link) with other components.
4 | PCB protective coating | [Plastik 70 CRC](https://www.google.com/search?q=Plastik+70+CRC) or any other acrylic [insulating lacquer](https://www.google.com/search?q=insulating+lacquer).
5 | [Male](https://www.aliexpress.com/item/-/32700932502.html) & [female](https://www.aliexpress.com/item/-/32593170276.html) 2.8x0.5mm power terminals | Optional. You can solder power wires directly.


**Note on order at LCSC.** When you use "Order at LCSC" button to import BOM,
check "Part Match Confidence" column to filter garbage. Correct items usually
have 100% match confidence value.

![BOM import match confidence](./images/lcsc_import.png)

Remove items `PIN 1 MM`, `PIN 3*0.5 MM` & `HILDA POTENTIOMETER`.


## Extract some parts from original board

You need:

- Potentiometer.
- Motor terminals.
- Filter capacitor (0.1uF 275v, attached in parallel to power).

![Reused components from original board](./images/old_pcb_components.jpg)


## Solder PCB top and bottom

We recommend to install all components, except regulating knob. Because on
flux cleanup phase, some solvents can wash off speed marks.

PCB top:

![PCB top side](./images/pcb_top.jpg)

PCB bottom:

![PCB bottom side](./images/pcb_bottom.jpg)


## Cleanup PCB

You should remove the rest of flux, to add protective coating later. Methods and
solvents are not commented. We hope, if you use flux, you know how to remove it.


## Install the rest and attach wires

Now you can install potentiometer, filtering capacitor, and check everything
fits into case. Don't forget to remove flux again.

PCB with speed knob & filter capacitor:

![PCB with all components](./images/pcb_full.jpg)

PCB in drill body:

![PCB in case](./images/pcb_and_case.jpg)


## Upload firmware and test

**WARNING! You MUST unplug power cord prior to firmware upload.** Turning power
off via grinder switch is not enough. If you plug programmer into computer
while AC plug in power socket, your USB interface may be damaged!

1. Install [VS Code](https://code.visualstudio.com/).
2. Clone this repo or download as zip archive and unpack somewhere.
3. Open folder with project AND after VS Code suggests to install plugins - agree
   with everything. That should install PlatformIO and all required dependencies.
4. Make sure you installed & configured ST-link/V2 drivers:
   - [Linux](http://docs.platformio.org/en/latest/installation.html#troubleshooting)
     instructions (how to configure `udev` rules).
   - [Windows](https://www.st.com/en/development-tools/stsw-link009.html) drivers.
5. Attach ST-link/V2 with 4 wires, named as `VCC`, `SWCLK`, `GND`, `SWDIO`.
6. Run via VS Code menu: `Terminal` → `Run Task...` → `PlatformIO: Upload`,
   and wait until complete.

Now you can assemble drill, run self-calibration and try how your "new drill"
works.

**IMPORTANT**. When you turn device on after firmware upload, motor will run at
slow speed and will not react on knob. That means, motor calibration required
(it's done only once, don't worry).

To run calibration:

- Move knob to zero.
- Move knob shortly up-and-down 3 times (in 3 seconds).
- Wait couple of minutes until magic finishes and motor stops. Be patient.

If everything works as needed, you can go to final step - protect PCB from dust.


## Cover PCB with protective coating

When you work, drill is actively cooled with air flow. Produced dust can be
partially absorbed and cause regulator damage. It's strongly recommended to
shield PCB with protective coating.

**Warning!** Coating should be done after you are 100% sure everything
works as expected. You will not be able to solder PCB after that.

Prior to apply coating, don't forget to clean PCB with isopropyl alcohol or
acetone. That's important for good adhesion.

You can use special PCB coatings or alternatives. See below.


### Special PCB coatings <!-- omit in toc -->

Such products are "officially" declared as suitable for PCB protection. This
can be spray like [Plastik 70 CRC](https://www.google.com/search?q=Plastik+70+CRC),
acrylic [insulating lacquer](https://www.google.com/search?q=insulating+lacquer),
or something else.

![Protective coat in spray and liquid form](./images/protective_coat.jpg)

If your coater is spray - put it to pepsi cap first and use cosmetic brush to
cover PCB. Don't apply spray directly, because you need to keep terminals and
knob internals clean.

Usually, lacquers need 3-5 layers for best result.

Note, a lot of ordinary lacquers from your local shop can be ok. BUT,
some may have not suitable electric resistance. Don't use untested things, if
you are not 100% sure. See more safe alternatives below.


### Alternate protection <!-- omit in toc -->

If you have no special PCB chemistry, it may be more convenient to find
alternate & more cheap materials in your local shop.

**RTV1 non-corrosive silicon glues:**

- [RTV704](https://aliexpress.com/af/704-silicone-glue.html)
- [RTV705](https://aliexpress.com/af/705-silicone-glue.html)

IMPORTANT! You should not use any random silicone glue, because those can
contain acid. Advised glues were tested and are neutral.

Disadvantage: all silicone glues need 1 day to dry.

**UV glues & UV gels for nails**

TBD


## The end :) <!-- omit in toc -->

![Assembled grinder with new PCB](./images/hilda_assembled.jpg)


Enjoy!
