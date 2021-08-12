## Speed calculation based on back-EMF

Power balance of the AC commutator motor is described by this equation.
Integrals can be replaced by sums:

<!--

Use https://github.com/masakiaota/tex_image_link_generator
to create snippets from LaTeX

https://tex-image-link-generator.herokuapp.com/

-->

![\begin{align*}
& \int_{0}^{T} Voltage(t)\cdot Current(t)\cdot dt = R_\Sigma \int_{0}^{T} Current^2(t)\cdot dt\\
\\
\\
& \sum_{0}^{T} Voltage\cdot Current = R_\Sigma \sum_{0}^{T} Current^2
\end{align*}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%2A%7D%0A%26+%5Cint_%7B0%7D%5E%7BT%7D+Voltage%28t%29%5Ccdot+Current%28t%29%5Ccdot+dt+%3D+R_%5CSigma+%5Cint_%7B0%7D%5E%7BT%7D+Current%5E2%28t%29%5Ccdot+dt%5C%5C%0A%5C%5C%0A%5C%5C%0A%26+%5Csum_%7B0%7D%5E%7BT%7D+Voltage%5Ccdot+Current+%3D+R_%5CSigma+%5Csum_%7B0%7D%5E%7BT%7D+Current%5E2%0A%5Cend%7Balign%2A%7D)


R<sub>&Sigma;</sub> - equivalent summary resistance of motor circuit.

Sums must be calculated from one zero-crossing of current to next zero-crossing of current.

The motor speed can be calculated as follows:


![\begin{align*}
& R_\Sigma = R_{ekv} + R_{motor}\\
\\
\\
& R_{ekv} = \frac{\sum_{0}^{T} Voltage\cdot Current}{\sum_{0}^{T} Current^2} - R_{motor}\\
\\
\\
& RPM = \frac{R_{ekv}}{K}
\end{align*}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%2A%7D%0A%26+R_%5CSigma+%3D+R_%7Bekv%7D+%2B+R_%7Bmotor%7D%5C%5C%0A%5C%5C%0A%5C%5C%0A%26+R_%7Bekv%7D+%3D+%5Cfrac%7B%5Csum_%7B0%7D%5E%7BT%7D+Voltage%5Ccdot+Current%7D%7B%5Csum_%7B0%7D%5E%7BT%7D+Current%5E2%7D+-+R_%7Bmotor%7D%5C%5C%0A%5C%5C%0A%5C%5C%0A%26+RPM+%3D+%5Cfrac%7BR_%7Bekv%7D%7D%7BK%7D%0A%5Cend%7Balign%2A%7D)


- R<sub>ekv</sub> - equivalent resistance which is created by the back-EMF.
- R<sub>motor</sub> - motor resistance in Ohms.
- K - constructive coefficient of the motor.

__The motor speed is proportional to the R<sub>ekv</sub>.__


## Autocalibration

First, we should determine motor's resistance. That's done in
stopped state, passing 1 pulse (positive period of sine wave) and observing
current's behavior.


### Motor's resistance (R) calibration

If motor isn't rotating, active power is equivalent to Joule power
on motor resistance. Integrals can be replaced by sums.


![\begin{align*}
& \int_{0}^{t} Current*Voltage*dt = R *\int_{0}^{t} Current^2 *dt \\
\\
\\
& \sum_{0}^{N} (Current*Voltage) = R * \sum_{0}^{N} (Current^2)
\end{align*}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%2A%7D%0A%26+%5Cint_%7B0%7D%5E%7Bt%7D+Current%2AVoltage%2Adt+%3D+R+%2A%5Cint_%7B0%7D%5E%7Bt%7D+Current%5E2+%2Adt+%5C%5C%0A%5C%5C%0A%5C%5C%0A%26+%5Csum_%7B0%7D%5E%7BN%7D+%28Current%2AVoltage%29+%3D+R+%2A+%5Csum_%7B0%7D%5E%7BN%7D+%28Current%5E2%29%0A%5Cend%7Balign%2A%7D)


Count all ticks from triac opening to moment
when current crosses zero.


![R = \frac{\sum_{0}^{N} (Current*Voltage)}{\sum_{0}^{N} (Current^2)}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+R+%3D+%5Cfrac%7B%5Csum_%7B0%7D%5E%7BN%7D+%28Current%2AVoltage%29%7D%7B%5Csum_%7B0%7D%5E%7BN%7D+%28Current%5E2%29%7D)


**Compensating armature frequency-related losses**

In ideal world, equations above should work. But in reality R is not constant
and not linear it - depends on triac phase. Motor has additional losses (caused
by eddy currents) when "frequency" rises. Measured R at zero phase is 1.5x
smaller than at full phase.

So, we have to measure resistance at multiple phases and build interpolation
table. Then, to calculate speed - use interpolated value, appropriate to current
triac phase. This helps a lot to calculate small speeds right. In theory, losses
may depend on motor load, but described compensation is enough for good result.

Note, after each "positive" measuring pulse it worth to make negative pulse to
demagnetize armature. Also, it worth to make small pause between measurements,
to keep rotor stopped.

It's enough to measure R at 0%-50% of "speed" (triac phase). On high speed R
does not affect calculated speed too much. Measure at 100% pulse can make motor
rotate and return wrong value. So, we measure only in 0%-50% range and
propagate last value to max (100%) speed.


**Implementation notes**

It's important to count sum until current become zero, not until voltage become
zero! Since we can't measure negative voltage, we use simple trick: record
positive value into memory, and replay after zero cross.

If you need to implement this on low memory devices, it's possible to record
only 1/4 of positive voltage wave. But we had no so big restrictions, and
recorded full wave (both voltage and current) for simplicity.

Also, you may have overflow with fixed point math. Since performance is not
critical with recorded data, using `float` types is preferable.

Signal may be noisy at short pulses. To filter accidental peaks, measure
multiple times, until 3 consecutive results become stable.


### ADRC-control and calibration

Grinder motors are very inconvenient for PID-based systems:

- RPM/Volts response is non linear, and varies significantly between different
  grinder models. The same about zero speed offset.
- Cooling fan adds extra distortion.

So, we use ADRC instead of PID. Adaptive character of the ADRC system makes it
possible to use without the RPM/Volts response linearization. Deviations from
the linear RPM/Volts response are eliminated by the generalized disturbance
observer (they are part of the generalized disturbance).

First-order ADRC system consists of linear proportional controller with `Kp` gain
and 2 state observers - speed observer, generalized disturbance observer.
Observers are integrators with `Kobservers` gain and `L1`, `L2` time constants.
Output of the linear proportional controller is corrected by generalized
disturbance signal. This correction eliminates (in the steady state) the motor
speed deviation caused by mechanical motor load, motor parameters deviation,
inaccurate motor parameters estimation.

ADRC system parameters and equations:

`b0` = `K / T`, where `K=1` due to speed and triac setpoint normalization,
`T` - desired optimal motor time constant.

`L1` - time constant of the speed observer.

![L_1 =  2 \cdot  K_p \cdot K_{observers}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+L_1+%3D++2+%5Ccdot++K_p+%5Ccdot+K_%7Bobservers%7D)

`L2` - time constant of the generalized disturbance observer.

![L_2 =  (K_p \cdot K_{observers})^2](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+L_2+%3D++%28K_p+%5Ccdot+K_%7Bobservers%7D%29%5E2)

`u0` - output of linear proportional controller in ADRC system.

![u_0 = (Knob_{normalized} - Speed_{estimated}) \cdot K_p
](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+u_0+%3D+%28Knob_%7Bnormalized%7D+-+Speed_%7Bestimated%7D%29+%5Ccdot+K_p%0A)

Speed observer equation:

![Speed_{estimated} = \int (u_0 + L_1 \cdot (Speed - Speed_{estimated})) \cdot dt
](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+Speed_%7Bestimated%7D+%3D+%5Cint+%28u_0+%2B+L_1+%5Ccdot+%28Speed+-+Speed_%7Bestimated%7D%29%29+%5Ccdot+dt%0A)

Generalized disturbance observer equation:

![ADRC_{correction} =  \int (Speed - Speed_{estimated}) \cdot L_2 \cdot dt
](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+ADRC_%7Bcorrection%7D+%3D++%5Cint+%28Speed+-+Speed_%7Bestimated%7D%29+%5Ccdot+L_2+%5Ccdot+dt%0A)

`P_correction` - proportional correction signal, makes reaction to motor load change significantly faster.

![P_{correction} = (Speed - Speed_{estimated}) \cdot P_{corrcoeff}
](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+P_%7Bcorrection%7D+%3D+%28Speed+-+Speed_%7Bestimated%7D%29+%5Ccdot+P_%7Bcorrcoeff%7D%0A)

Output signal of ADRC regulator:

![Output = (u_0 - ADRC_{correction} - P_{correction}) / b_0
](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+Output+%3D+%28u_0+-+ADRC_%7Bcorrection%7D+-+P_%7Bcorrection%7D%29+%2F+b_0%0A)

General steps of ADRC calibration are:

1. Measure motor start/stop time. Then use it for understand max possible
   oscillations period.
2. Find `Kp` in range [0.3\*b0..0.3\*b0+4] by halving (division by 2) method.
   Criteria - "no oscillations" (speed dispersion should not exceed normal noise).
3. Find `Kobservers` in range [0..max] by halving method. Criteria -
   "no oscillations".
4. Find `P_corrcoeff` in range [0..max] by halving method. Criteria -
   "no oscillations".

**Implementation notes**

Basic start/stop time measure:

- We should select begin/end setpoints, where speed can be measured good enougth.
  Use 0.35 and 0.7.
- Common criteria of interval end is "when speed reach 2% window of desired
  setpoint". But signal is noisy, and we use simple trick: wait 15% window and
  multiply to `log(0.15)/log(0.02)` (because speed growth is ~ logarithmic).

Measure `Kp`:

- Set `Kobservers` to safe value (1.0), `Kp` min possible (0.3\*b0),
  `P_corrcoeff` to 0.0, measure all at minimal allowed speed.
- Wait for stable speed
- Measure noise amplitude, abs(max - min) for period ~ start/stop time.
- Use starting step = +4.0, and halving method. Check noise amplitude not exceed
  110% of initial value.
- Make 7 iterations total (last step will be 0.1 - good precision).
- Use 0.6 of final value as safe.

It would be better to count dispersion instead, but that's more complicated.
Counting min/max after median filter seems to work. Note, speed value is very
noisy. Applying some filter before min/max check is mandatory.

Measure `Kobservers`:

- Set `Kobservers` to 0.0, `Kp` to calibrated, `P_corrcoeff` to 0.0,
  measure all at minimal allowed speed.
- Wait for stable speed
- Measure noise amplitude, abs(max - min) for period ~ start/stop time.
- Use starting step = +4.0, and halving method. Check noise amplitude not exceed
  110% of initial value.
- Make 7 iterations total (last step will be 0.1 - good precision).
- Use 0.6 of final value as safe.

Measure `P_corrcoeff`:

- Set `Kobservers` to calibrated, `Kp` to calibrated, `P_corrcoeff` to 0.0,
  measure all at minimal allowed speed.
- Wait for stable speed
- Measure noise amplitude, abs(max - min) for period ~ start/stop time.
- Use starting step = +4.0, and halving method. Check noise amplitude not exceed
  110% of initial value.
- Make 7 iterations total (last step will be 0.1 - good precision).
- Use 0.6 of final value as safe.


# More to read

Links below are not mandatory, but can simplify understating of this document.

1. [AN863, Improved sensorless control with the ST62 MCU for universal motor](https://www.st.com/resource/en/application_note/cd00003969.pdf).
   Simplified algorithm, based on zero-cross current measure. Table-based,
   requires preliminary motor testing with stand. More cheap for mass production,
   but less universal. Recommended for reading, to undestand alternatives.
2. [DC Chopper control vs. Phase Angle control for appliance and power Tool Applications--Who wins?](https://www.edn.com/Home/PrintView?contentItemId=4214636)
   Has comparison of triac & chopper regulators, with details about
   harmonic-related losses. Helps to understand, why "active resistance" depends
   on triac control angle.
3. [Panasonic Application Note 030, Driving Triacs with Phototriacs](https://www.panasonic-electric-works.com/cps/rde/xbcr/pew_eu_en/dd_x615_en_an_030.pdf).
4. [ST AN440. Triac control with a microcontroller powered from a positive supply](https://www.st.com/resource/en/application_note/cd00003866-triac-control-with-a-microcontroller-powered-from-a-positive-supply-stmicroelectronics.pdf)
5. [A Simulative Study on Active Disturbance Rejection Control (ADRC) as a Control Tool for Practitioners - Gernot Herbst](https://arxiv.org/pdf/1908.04596.pdf)
