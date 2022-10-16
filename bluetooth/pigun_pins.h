#include <bcm2835.h>

#ifndef PIGUN_PINS
#define PIGUN_PINS


// Button definitions - GPIO pins
#define PIN_TRG RPI_V2_GPIO_P1_16	// trigger 
#define PIN_RLD RPI_V2_GPIO_P1_29	// reload  (this is the little clip button like in real beretta M9)
#define PIN_MAG RPI_V2_GPIO_P1_18	// handmag (this should be connected under the handle)
#define PIN_AX1 RPI_V2_GPIO_P1_37	// AUX1 BT (this will be the one in the front of the magazine, under the barrel)
#define PIN_AX2 RPI_V2_GPIO_P1_31	// AUX2 BT 
#define PIN_CAL RPI_V2_GPIO_P1_33	// calibr  
// not used:
#define PIN_AX6 RPI_V2_GPIO_P1_38	// AUX4 BT goes on PIN #38 == GPIO 20
#define PIN_AX7 RPI_V2_GPIO_P1_40	// AUX5 BT goes on PIN #40 == GPIO 21


// GPIO for LEDs
#define PIN_OUT_ERR RPI_V2_GPIO_P1_11 // maybe use a red led?
#define PIN_OUT_CAL RPI_V2_GPIO_P1_13 // not soldered right?
#define PIN_OUT_AOK RPI_V2_GPIO_P1_15 // green LED
#define PIN_OUT_SOL RPI_V2_GPIO_P1_07

#endif
