This is a Zephyr driver for [HC-SR04 Ultrasonic Ranging Module](https://www.sparkfun.com/products/15569) that conforms to the [Sensor subsystem API](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/peripherals/sensor.html). Two variants are included:
 - HC_SR04 uses a pin change interrupt to perform measurement by calling k_cycle_get_32
 - HC_SR04_NRFX uses TIMER, PPI, and EGU peripherals to limit CPU activity while measuring and enhance precision
 
Builds from from the v1.4.0 tag of [NCS](https://github.com/nrfconnect/sdk-nrf).

![ss](https://user-images.githubusercontent.com/6494431/98500542-a1374e00-2201-11eb-9783-fd52ad7a6a71.png)
