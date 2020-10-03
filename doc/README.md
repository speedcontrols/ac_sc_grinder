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


## Data flow

[diagram](https://www.draw.io/?lightbox=1&highlight=0000ff&edit=_blank&layers=1&nav=1&page=1#R%3Cmxfile%3E%3Cdiagram%20id%3D%229bc443ba-6071-7a84-ef12-a10f0f5354f1%22%20name%3D%22Main%22%3E7Vxtc5s4EP41nrn7gEcSSMDHxE2bm7m2mSa99j51MMg2F2I8GOft159kkEES2NgWidM2nqawgMDaZ%2FfZXS0Z2KO7xw9ZsJh9TCOaDBCIHgf2uwFC0MM2%2B49LngoJxk4hmGZxVJ5UCa7jZ1oKQSldxRFdSifmaZrk8UIWhul8TsNckgVZlj7Ip03SRL7rIphSTXAdBoku%2FRZH%2BayQesit5Jc0ns7EnSHxiyPjILydZulqXt5vgOzJ%2Bqc4fBeIscovupwFUfpQE9kXA3uUpWlebN09jmjC51ZMW3Hd%2B5ajm%2BfO6DzvcoHjhK7veT6IIJ74xLNKxd0HyYqKr7B%2B0PxJTA6dR2d8jtlemATLZRwO7PNZfpcwAWSb7NbZ03e2A8TOv3xniDDb1x9Q6DfIpjRveypcnEQjSW%2Fld%2FpA0zvKbsNOeKi0hcspntUUJWQZTYI8vpe1HZSgmW6G29zhKo3Z4yJQAtwW45TwdjxliGW6ykJaXlWffmUgx3GGHnahg4rfvjQsxmAIqoOeK9%2BkmDHtJkw3wVPttAU%2FYVk%2Fg23UJq0SrbHRESfQ04ByleZMqTEflmYDxMZks4Q1%2BOT0MZcBk9Fl%2FByM1ydwzJRPzM7G5wP8jkmCVZ4uCy%2FBLwiSeDpn2wmd8KHuaZbHzHjPSnGeLph0uQjCeD694TvvLIdJJuk8vy4fpMFSShEfjT5uRZnQHsSSvjyn3K%2BhEMIGGArrb0KcpKR9NCIsdW%2FTXTIg5UI8T%2Be0mzljcen7mD9kcfgxzr%2BL69h2caa71fALWylFzht0BrbsDIhAhWFn4PlDt3bY6eQMTJi6mO8asK4XlEacf4MkXLHZSzMNamsSpFEJjIdZnNNrZpH86AMLGmSMmTBGRzZGDIBujKjJGPEWlR9sjQ5snbQRc0NZmiT09SfNdWXsYpvbtTptSCCgPm2Q9OHEHD3%2BuMniIBwgknBXP2ZzRqZ8K6zNonrs1ecVbty%2BmFiCGya2EY9%2BL%2FPq61zA3KdgwzTLZ%2Bk0nQfJRSU9l2etTgmNbh63cIlKEv%2FRPH8qY35O7ExU3f%2FvdM3XW%2FVQZ4xWBO1khyPdPrQ92eEQuLGdgyO0tlu5XsutKjwUgxpz%2BoIca4a4WCVLlpQh8AfkvhVY7N8aweDy%2BU8NXj9PmAeRs9GsMGe%2FyU9C1OQnnXZkHWzOIt2Uou8H7gpPipJZyCLP2ybB3%2BUE%2B2EXTydl3S3uyG45ukZpwqaXn277If%2BsvVyW3tKmI60zeSoBrC8rybU1R9Y9hN05VI%2BBqqdnQOFkOvxy9fHHx7PvZjxU6XtCpktmbrpTuoujaE2eSTCmyfmmHFQDRlkQaoeSDph2JKvGuKmJlV9jUK8rNRkpGEKHOEaAZLmqn7SgYsbpZLKk5hXfY9VqnbwaNPo3ldoiJbX1PDN%2BgagVkD69gtPoFT5%2Bvvn85ceXi%2Bu%2Frm%2FOPo0uioLV59ndz%2BUnhGUc7Scs5igIsRXzNuM3bKQM67%2BI19BLmKMkDW%2BZyOFR7e3ls%2B5Fjk%2BaikRJpE1QTZqUulyZN8E98qaapyszMCap5V%2FtTqs70LpkYp2d2YaJAIDECKIwUCqzoGscUg1lFHwEhtj3EcU%2BDiKXji09VDHGWEess4i84lT4R11ncRXaMFNa9SE%2BoLTayzqLhhOdvLrhpPei%2FnaU7V3UPzXkqUV9YJuJfHzSY0akoQf26GawxGm%2B70jwsImAyxXN4mJJsBjiODLZCiLntBJrTJSqkavENV1B5IIdA%2FUJIT23Ygm14eVdk9Gyjq92szASFWPfTBiM1WqZ5amuop9YBOok80%2Ba5LxHhhd7Ibi32DzcFzrHwCK%2FQM2XKJEHaSz4ej0t7usq0tdgR6ts%2FQWZiiSdcCVxgINxzA9%2B%2FevTzU%2BsKEsj6ldWFNEJ9vCcsWNmuCvjO3rFre%2FFNIyUfK1zlLTvWpq64k0w2Uq9WCmO73u%2BR6CCp9pCXeen2xUxHMICkzGwMYF0gqMgGm86%2F06k2GEK0kpJI1j%2F9LRcc6QR%2BCxJVcCD%2BjID%2FV7uDmDvvGIHtPn1EGPoIgfarm%2FLlUTig2Ehh8B2HE9JwXuEfVOCXfSTRPG9aCipwqFnmqU8TMxSBtz5lHMtAJZYBheXsqeoXW2Gfk3GyFGcsSA3Tvl1bG45TTJbzoNSZPmgoRo44p%2FmALvFl5hYnaq6ro%2BtMkMVwJD0E1%2Brs6Ev1nM8LdtzqF5ac3aWfHa07TTWCSqnvE%2BFutXFbu%2Fs6VLB7Nz%2BIzBwZKrfL3JEomqalNFvVu7MygCqvOf2xsr6vbC9rdOreQROoOxKjyDgngjLNlSRwsn09IjRJAmaqzKBoYvEG1Sik8oQJxJpVEvtXTfi1tDYJeOxHdqAWetkHFqH5MhHcRx6BZLTfBk44x8dU%2BGEfzTHfRhJinUUM3Xy%2FeC1r7sjSu1GtD63pSDq%2BdjdklsfgFPYSyQGhjb26nQ7BGs4mlvO70TdE7ZZ93iYf7ZS7sGLN6Ke3Dc1I0dOIl11edgQLWv3kV%2BqPBp3qJEch9eXXz%2FdvO0OJY1OmxLPBkyJareJ%2FBEBuXneEgnXsS8AIGlYWxnACHNCtxePpPHmC4f%2Fms9hk8k%2Bp%2BZf1EUg%2FtoFAQ7BEBLk%2BqDb%2B7iHqF0vSlWrPYVwbQlr%2B8g0iLx0nz8CCi97zW9HNKzBmGjzb3gVWbeaHU0Ob7d7v7bgVa1VyoN0bTPQh7I7MuohGNerYmX3JWe51gbMt0N1DcARQDXRdmA72FRdVLZeS3%2BPzUwWqL9RtreZvmTd6KV6hDTzde2hX%2Fs5sOVeH7irMe8bHjd8hbLcs8%2BzmX2fcUNickh9VbwIxD3MtyAvuzVPyr3UA%2BQHusyPq0ltbM6Ex3E9JNegRBPz0Q5IeavARv04IKRhonxpcrMOA04PEK0O7Hidumo9xdsSjbXoThnCUtVvRnOOzhS%2FcG%2BCQl0N6DCfCrEYfyhbv%2BvhYbc21L3fd0daIOoiKK%2BDHHTRjsUTqCYyBGtf0dwaiShD%2FO482JPgOjs%2BhQudrc7wDXQasN3qL8MVp1d%2Ffs%2B%2B%2BB8%3D%3C%2Fdiagram%3E%3Cdiagram%20id%3D%22a006dd9b-79ae-cdf8-b110-d7cd4f0e9d1a%22%20name%3D%22Speed%20Controller%22%3E7VxZb9s4EP41BnYfbPAQdTzmahug6QZNse3uS6DYjK2tbBqScv76JS3SEkk5VmzKsdvYQCKNSIrizHxzcOQePpk%2Bfszi%2BeSCjWjaQ2D02MOnPYRg4Hn8n6A8lZTIQyVhnCUj2agiXCXPVBKBpN4lI5prDQvG0iKZ68Qhm83osNBocZaxB73ZLUv1u87jMbUIV8M4tanfk1ExKakhCir6J5qMJ%2BrO0I%2FKKzfx8Oc4Y3czeb8ewreLT3l5Gqux5IPmk3jEHmokfNbDJxljRXk0fTyhqVhbtWxlvw8rri7nndFZ0aYDJnIexZN6djriSyFPWVZM2JjN4vSsoh4vno%2BKEQA%2FmxTTlB9Cfkgfk%2BKHJIvjf8TxABF%2BmhdxVhwJ1nDaMI3zPBkq8ockXY4wG6lGMzajJUVeF6P%2BR4viScpLfFcwTqrm%2BJmxuRwnLzL2k56wlGWLp8IIii%2B%2FYq%2BQXLSc3WVDuQZSWsVS1NrIJfxI2ZQW2RNvkNE0LpJ7XWpiKXzjZbtl10uW8LsiIBXFj6QYSDUhARh4%2BiB8fca0kP0qRvKD2kQq0oK9K1gduGT1Oz%2Bb%2BOnr%2FPQhbMVNvkTxU63ZXDTIV98ngKTxPh9eOa9KmMoZbCxaYTn8fZzeybW7ZAVnSCJGoVkP8SH4FGywKehjoQtWRvPkOb5ZNBASIteCtybHPXLKKXGajGdC6vgN%2BNj4%2BJ5mRcLB%2B0hemCaj0UJ60%2FiGpsdLSK6JjwTlJvFReiJGpY%2BaJEhzI2enQbYmRrIXGEDs6ZySA20pZ2igj9qHUFHUIOz2NqfbQobXiXWAmnUgC0zgU1oaDnGyynJIEKnDDNBhpsKiCmngzpGmVHS5ip5r9NFY%2BhL%2FlChXevk396Lyg9JHJYQu9NEHUahrDnajkKGpkQh0opE%2BtFjmwl8bBKSmlPtg36Oh%2BPIrt2xWNNH3ye5Hnj%2FwUEgCWP7FmiyEAHTl0yHb7p6florBFzRNhekFf1zNKR39aQmOLhYPk6SgV%2FN4sXQPPKzTRWXlcltquVL9gsDwdvFSRx6qIGsZkkzqAZYPVvOkLRp2E%2BqURuoNVcdQkHjxeVGlVvKyaz0JDQkIAstvceQhW3cy4%2BDXtdc95LW9eVxnPleJSi503rd0fng7HnC9v77cP3OesYILDBPd%2BhFogPMT8W2USYdmP0DE0xiEnFj9PvR1ow%2B7MPlOw%2Fb9sfgHBFsRAIZ7F4KugCsCBhBBde9V0LW2xxrwsp%2BuU%2FiyUwUKvs7LsCSnw%2FyXAjKH%2BYQABaEj6NJ91D72O4lXVAKsxm4bzpqC9xqnd4kUNUeUSLWq%2B6GKtq0fhA2VjbyWCrd2qAbddeVSmZPGeA0y2XMjLyHTJuJlB0ACTb5eXlxfHP24%2Fnx%2Bcf5t%2F7BklGRc60sweaB5sR28wFWB2Kvxpc8BJkS6p9R35SohY1jcBdrgd7RpUEPTQdgCbcDO0Mac9Hq0sebmHG281Whz%2FuU3QRulYvuNNmY2Fnfj3XSUUnqPzdrHZqGu9W2Tra%2BOy0LYeJ%2BWMZbRu9MICzZniE7Pjk6v%2F%2F3ry9n19%2FPTb5%2F2D6i2QSWnySJnMZbO9GW6yCkG4agTDOodeAUPjHaDP76R%2Bo38aBDi6tPOO9pkbx5YWn7JHg6t1iJy6E2gQKVanBZb9I0otm8O4abUwu9CjduXWgSHV2qhVHxPai3sxOYB1loo18GFHY28SFMcR769ETHAbgotCO7erNo6d7CFFm9lbiEgYIB3YW%2BJHfxDEQPsp2q3ZyZxGdAjqKq1tt9atWocjcDOjZrvane1rX6u0%2FOWcLFBpP4W6hwRf4ABN%2FyEhBBHxCiWhVG7YH6TIFl5ni%2BXTi1c6j0onTKyHJHySXdSOKUs93vl1EpWdq4oEA1UdaH4q1exQhhAgb3mZdc5sKrMfHljaT5apsFefgoQRoOgqqJUvrz7DBmy94l%2BzRoqpbl7XUSFrdrpLgJstdPwnqt%2FMxALA12%2Fka7%2BCOwCxPgkDAgD0bqdxbVd1taHvvjgu8M9lSb%2FfYqvlNrv185AZEIe7mRrwN4I%2Bnp54TghxaEpL1EK1tie0tuigemFgK3jnLvKyWz8bYFhfc%2BRiwx1F5kQ20VWznDdQ0YOHGSV6Oo6eXtY%2BVn3udhmbYKYmHvA3sALAAlDHhOhyGv3Eu4m2QN7D%2BYAc75O92A8YhSWuvESAysbFHQAmKSjt%2BtMF%2FEX2oRRSl7fhEGd6P1rHT2IjP330H%2F5HaDtO0C0pofvge06%2BMTtm%2FtEBYgVgn1mw58lgN0wltJ4dgggRlb5Cxu4ecYbwm52kYkeQvT9jt7Yt3cpLhKbg7vOX0IYmi%2BZLN8GXeOfBQ78M2Ine3vIT4UQj5J7fjgWh8J0i98FQuCZZkxITsY4tM7Gqi2%2FTa15wwj8fzwVqze7yec7OQcQgD4UIg0%2BPa%2Ba5964%2BWvjuLUpivYiFwZWqtKzBA7iBonDDiTOtz3DJomb77%2FEgXKXsy%2BODDm7ycyZvUtezW1YIp3a1upA8Php9WNhpb2ofpENn%2F0P%3C%2Fdiagram%3E%3C%2Fmxfile%3E)


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