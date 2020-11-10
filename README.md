This is a Zephyr RTOS driver for the [HC-SR04 Ultrasonic Ranging Module](https://www.sparkfun.com/products/15569). It conforms to the [Sensor subsystem API](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/peripherals/sensor.html) and was built from the v1.4.0 tag of the [nRF Connect SDK (NCS)](https://github.com/nrfconnect/sdk-nrf).

### About the sensor
The HC-SR04 ranging module is an inexpensive sensor for measuring distance in a variety of applications. It is relatively easy to use:

 1. Set the TRIG pin high for at least 10us
 1. Wait for the ECHO pin to go high and then measure how long it remains high
 1. Convert the measured time to meters using the speed of sound

Although the HC-SR04 is a 5V device, its TRIG pin works fine when driven at 3V and it's trivial to use a couple of resistors as a voltage divider on the ECHO pin. The following configuration works fine with an [nRF52840 DK](https://www.nordicsemi.com/Software-and-Tools/Development-Kits/nRF52840-DK): 

<p align="center"><img src="https://user-images.githubusercontent.com/6494431/98627864-10787500-22ca-11eb-9a64-a5d4383ecc3b.png" width="256"></p>

The TRIG (top, 11us) and ECHO (bottom, 9.3ms) pulses look like this when the sensor is pointed at a ceiling that is ~1.5M away:

<p align="center"><img src="https://user-images.githubusercontent.com/6494431/98500542-a1374e00-2201-11eb-9783-fd52ad7a6a71.png" width="768"></p>

If the sensor can't get a valid measurement -- because the target is too close or too far away -- then the ECHO pulse is 128.6ms long followed by a second 6us pulse about 145us later. This error pulse can't be truncated so when it occurs it effectively reduces the sensor's 40Hz working rate.

Perhaps the biggest consideration when using these devices is that performing measurements using multiple HC-SR04 devices simultaneously can cause erroneous results because the individual sensors can't differentiate their own pulses from the others.

### About the driver
The driver assumes --and uses a mutex to enforce-- that only one HC-SR04 will be actively measuring at any given time. There are two variants of the driver:
 - **HC_SR04 uses a pin-change interrupt to measure by calling k_cycle_get_32 in rising-edge and falling-edge interrupts.**
   - PRO: Should work reasonably well on most platforms
   - PRO: Uses the standard GPIO driver
   - CON: Any latency when servicing the pin-change interrupt will affect the measurement
 - **HC_SR04_NRFX uses GPIOTE, TIMER, PPI, and EGU peripherals to measure using a hardware timer and then triggers an interrupt after completion.**
   - PRO: Only one CPU interrupt per measurement
   - PRO: Measurement is not affected if interrupt is delayed
   - CON: Uses nRF52-specific hardware peripherals
   - CON: Uses GPIOTE driver --but not the GPIOTE interrupt-- so it's not entirely compatible with standard GPIO driver

### Using the HC_SR04 variant
This is an example DT entry when using **HC_SR04**:
```C
us0: hc-sr04 {
    compatible = "elecfreaks,hc-sr04";
    label = "HC-SR04_0";
    trig-gpios = <&gpio0 26 GPIO_ACTIVE_HIGH>;
    echo-gpios = <&gpio0 27 GPIO_ACTIVE_HIGH>;
    status = "okay";
};
```
Then add the following to **prj.conf**:
```
CONFIG_SENSOR=y
CONFIG_GPIO=y
CONFIG_HC_SR04=y
```
### Using the HC_SR04_NRFX variant
The **HC_SR04_NRFX** version is similar but uses NRFX-style pin numbers instead:
```C
us0_nrfx: hc-sr04_nrfx {
    compatible = "elecfreaks,hc-sr04_nrfx";
    label = "HC-SR04_NRFX_0";
    trig-pin = <26>;
    echo-pin = <27>;
    status = "okay";
};
```
Then add the following to **prj.conf**: 
```
CONFIG_SENSOR=y
CONFIG_GPIO=n
CONFIG_HC_SR04_NRFX=y
```
NOTE: the project will compile normally if CONFIG_GPIO is enabled but **unexpected side effects will happen if the native GPIO driver is used to configure pin change interrupts. The GPIOTE driver should be used instead.**
