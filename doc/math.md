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


### Motor's RPM/volts response linearization & scaling.

Motor's speed depends on volts in non linear way. That may cause problems for
PID control. Been tuned on low speed, PID will not be optimal at opposite side.
Motor speed can be measured on different voltages for next corrections. That's
not ideal but good enough.

Note, we have 2 transforms for triac phase in the end:

1. DC volts (0..max) to AC phase of sine wave.
2. Motor non-linearity compensation.

It would be nice to scan speed in all voltage range and build inverse
compensation function.

See `rpms_vs_volts_idling.ods` with scan results from real tacho (red) and our
"speed sensor" (blue). There are 2 problems:

1. Bad noise at low volts. Seems, can not be dismissed with this motor type.
2. Unexpected fall in middle range. No ideas about nature of this effect.

Bad news is, we should compensate both deviations. Now we do this via "magical
constants", selecting ranges with plausible data. But this was not tested with
other motors and should be improved somehow.

Current algorithm is:

- Scan RPMs with small step in low range, and search interval [0.2..0.4]
  at Y axis. Build extrapolation (line) for low range. Use 2 points for linear
  interpolation.
- Scan 3 points at high range, use for linear interpolation.
- Connect the gap between via spline and use 2 points for linear interpolation
  (that's enough).

So, we have 7 points to interpolate RPMs good enough. See
`rpms_approximated.ods`. Finally, those can be used to build inverse data.

Note about scaling factor. That depends on constructional coefficient K of motor.
In real world everything is very simple. "Speed sensor" should produce data in
desired range. In our case, that's [0..1]. For our 180W grinder, scaling factor
is ~ 400-500. For any other motors it should fit in [300..2000] range.

**Implementation notes**

How to detect that speed become stable:

- Collect data ~ 0.25 secs & apply median filter.
- Try 3 times and make sure difference is < 0.3% (1% is not enough).
- Limit total number of measurements to 3 secs (if jitter is too big and
  stability condition not satisfied)


### PI-control calibration

Since our system has "fast" response, it's enough to use direct iterative
method to find `P` and `I` coefficients. Implementation is simple & takes acceptable
time to run.

General steps are:

1. Measure motor start/stop time. Then use it for 2 things:
   - Understand max possible `I`.
   - Understand max possible oscillations period.
2. Find `P` in range [1..5] by halving (division by 2) method. Criteria -
   "no oscillations" (speed dispersion should not exceed normal noise)
3. Find `I` in range [0..max] by halving method. Criteria - no over-compensation.


**Implementation notes**

Basic start/stop time measure:

- We should select begin/end setpoints, where speed can be measured good enougth.
  Use 0.35 and 0.7.
- Common criteria of interval end is "when speed reach 2% window of desired
  setpoint". But signal is noisy, and we use simple trick: wait 15% window and
  multiply to `log(0.15)/log(0.02)` (because speed growth is ~ logarithmic).

Measure `P`:

- Set `I` to max possible, `P` min possible (1), measure all at 0.3 setpoint.
- Wait for stable speed
- Measure noise amplitude, abs(max - min) for period ~ start/stop time.
- Use starting step = +4.0, and halving method. Check noise amplitude not exceed
  110% of initial value.
- Make 7 iterations total (last step will be 0.1 - good precision).
- Use 0.75 of final value as safe.

It would be better to count dispersion instead, but that's more complicated.
Counting min/max after median filter seems to work. Note, speed value is very
noisy. Applying some filter before min/max check is mandatory.

Measure `I`:

- Set `I` to max possible, `P` to calibrated.
- Apply 5Hz low pass Butterworth filter to input signal (median filter
  has less predictable response)
- Change setpoint from 0.3 to 0.8 and measure max reached speed.
- Find `I` with halving method. On each step repeat setpoint change from 0.3
  to 0.8, and check if overshoot not exceed 15% of initial max speed.
- Use 1/0.75 of final value as safe.


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