#include <assert.h>
#include <string.h>

#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "sysinit/sysinit.h"

#include "lsm9ds1/lsm9ds1.h"

int
main(int argc, char **argv)
{
    int rc;

    /* Initialize the OS */
    sysinit();

#if MYNEWT_VAL(LSM9DS1_CLI)
    lsm9ds1_shell_init();
#endif

    /* Configure the LED GPIO as an output and HIGH (On) */
    hal_gpio_init_out(LED_BLINK_PIN, 1);

    while (1) {
        /* Run the event queue to process background events */
        os_eventq_run(os_eventq_dflt_get());
    }

    return rc;
}