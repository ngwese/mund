
#include <string.h>
#include <errno.h>
#include "os/mynewt.h"
#include "console/console.h"
#include "shell/shell.h"
#include "hal/hal_gpio.h"
#include "lsm9ds1/lsm9ds1.h"
#include "lsm9ds1_priv.h"
#include "sensor/sensor.h"
#include "sensor/accel.h"
#include "sensor/mag.h"
#include "sensor/quat.h"
#include "sensor/euler.h"
#include "hal/hal_i2c.h"
#include "parse/parse.h"

#if MYNEWT_VAL(LSM9DS1_CLI)

static int lsm9ds1_shell_cmd(int argc, char **argv);

static struct shell_cmd lsm9ds1_shell_cmd_struct = {
    .sc_cmd = "lsm9ds1",
    .sc_cmd_func = lsm9ds1_shell_cmd
};


static int
lsm9ds1_shell_err_too_many_args(char *cmd_name)
{
    console_printf("Error: too many arguments for command \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
lsm9ds1_shell_err_unknown_arg(char *cmd_name)
{
    console_printf("Error: unknown argument \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

/*
static int
lsm9ds1_shell_err_invalid_arg(char *cmd_name)
{
    console_printf("Error: invalid argument \"%s\"\n",
                   cmd_name);
    return EINVAL;
}
*/

static int
lsm9ds1_shell_help(void)
{
    console_printf("%s cmd [flags...]\n", lsm9ds1_shell_cmd_struct.sc_cmd);
    console_printf("cmd:\n");
    console_printf("\tchip_id\n");
    console_printf("\treset\n");

    return 0;
}

static int
lsm9ds1_shell_cmd_get_chip_id(int argc, char **argv)
{
    uint8_t id;
    int rc;

    if (argc > 2) {
        return lsm9ds1_shell_err_too_many_args(argv[1]);
    }

    /* Display the chip id */
    if (argc == 2) {
        rc = lsm9ds1_get_chip_id(&g_lsm9ds1_i2c_0_itf, &id);
        if (rc) {
            console_printf("Read failed %d", rc);
        }
        console_printf("0x%02X\n", id);
    }

    return 0;
}

static int
lsm9ds1_shell_cmd_reset(int argc, char **argv)
{

    console_printf("not implemented\n");
    return 0;
}

static int
lsm9ds1_shell_cmd(int argc, char **argv)
{
    if (argc == 1) {
        return lsm9ds1_shell_help();
    }

    /* Chip ID command */
    if (argc > 1 && strcmp(argv[1], "chip_id") == 0) {
        return lsm9ds1_shell_cmd_get_chip_id(argc, argv);
    }

    /* Reset command */
    if (argc > 1 && strcmp(argv[1], "reset") == 0) {
        return lsm9ds1_shell_cmd_reset(argc, argv);
    }

    return lsm9ds1_shell_err_unknown_arg(argv[1]);
}

int
lsm9ds1_shell_init(void)
{
    int rc;

    rc = shell_cmd_register(&lsm9ds1_shell_cmd_struct);
    SYSINIT_PANIC_ASSERT(rc == 0);

    return rc;
}

#endif
