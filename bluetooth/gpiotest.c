#include <stdio.h>
#include <bcm2835.h>

// Input on RPi pin GPIO 0 of pizero
#define PIN RPI_BPLUS_GPIO_J8_11

int main(void) {

    // If you call this, it will not actually access the GPIO
    // Use for testing
    //    bcm2835_set_debug(1);

    if (!bcm2835_init())
        return -1;

    printf("BCM inited!\n");

    // Set RPI pin P1-15 to be an input
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_INPT);
    //  with a pullup
    bcm2835_gpio_set_pud(PIN, BCM2835_GPIO_PUD_UP);
    // And a low detect enable
    bcm2835_gpio_len(PIN);


    printf("BCM looping...\n");
    while (1)
    {
        if (bcm2835_gpio_eds(PIN))
        {
            // Now clear the eds flag by setting it to 1
            bcm2835_gpio_set_eds(PIN);
            printf("low event detect for pin 0\n");
        }

        // wait a bit
        delay(500);
    }

    bcm2835_close();

    return 0;
}
