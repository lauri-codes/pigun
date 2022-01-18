#include <stdio.h>
#include <bcm2835.h>
#include <signal.h>


// Input on RPi pin GPIO 0 of pizero
#define PIN 17 //RPI_BPLUS_GPIO_J8_11

int stop;

void inthand(int signum) {
    stop = 1;
}

int main(void) {

    // If you call this, it will not actually access the GPIO
    // Use for testing
    //    bcm2835_set_debug(1);

    if (!bcm2835_init())
        return -1;

    printf("BCM inited!\n");

    stop = 0;
    signal(SIGINT, inthand);

    // Set RPI pin P1-15 to be an input
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_INPT);
    //  with a pullup
    bcm2835_gpio_set_pud(PIN, BCM2835_GPIO_PUD_UP);
    // And detect 
    bcm2835_gpio_fen(PIN);
    bcm2835_gpio_ren(PIN);


    printf("BCM looping...\n");
    while (!stop)
    {
        //printf("loop begins: %i \n", bcm2835_gpio_lev(PIN));
        if (bcm2835_gpio_eds(PIN)) // check the event detect flag
        {
            printf("eds on PIN worked! current is: %i\n", bcm2835_gpio_lev(PIN));
            // Now clear the eds flag by setting it to 1
            bcm2835_gpio_set_eds(PIN);
            printf("bcm2835_gpio_fen event detect for pin 0\n");
        }
        else {
            //printf("no event...\n");
        }

        // wait a bit
        delay(500);
    }

    bcm2835_gpio_clr_fen(PIN);
    bcm2835_gpio_clr_ren(PIN);
    bcm2835_close();
    printf("graceful shutdown\n");
    return 0;
}
