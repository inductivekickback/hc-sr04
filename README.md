# hc-sr04
Zephyr driver for HC-SR04 Ultrasonic Ranging Module

Conforms to the Sensor interface. Two variants are included:
 - HC_SR04 uses a pin change interrupt to perform measurement by calling k_cycle_get_32
 - HC_SR04_NRFX uses TIMER, PPI, and EGU peripherals to limit CPU activity while measuring and enhance precision
 
