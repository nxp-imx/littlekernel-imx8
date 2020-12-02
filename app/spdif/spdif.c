/*
 * Copyright 2019-2020 NXP
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <app.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>
#include <dev/class/spdif.h>
#include <assert.h>


#if WITH_LIB_CONSOLE

#include <lib/console.h>

static int cmd_spdif(int argc, const cmd_args *argv);

STATIC_COMMAND_START
STATIC_COMMAND("spdif", "spdif commands", &cmd_spdif)
STATIC_COMMAND_END(spdif);

#define DO_CMD_SPDIF(func) \
    if (!strcmp(argv[2].str, #func)) { \
        err = class_spdif_##func(dev, true); \
        printf("SPDIF func on bus %d returned %d\n", bus, err); \
        return 0; \
    }

static int cmd_spdif(int argc, const cmd_args *argv)
{
    int err;

    if (argc < 3) {
        printf("not enough arguments\n");
usage:
        printf("%s bus test bit_format len repeat\n", argv[0].str);
        printf("%s bus open\n", argv[0].str);
        printf("%s bus close\n", argv[0].str);
        printf("%s bus start\n", argv[0].str);
        printf("%s bus stop\n", argv[0].str);
        printf("%s bus setup\n", argv[0].str);
        printf("%s bus read len <dump>\n", argv[0].str);
        return -1;
    }

    int bus = argv[1].u;
    struct device *dev = class_spdif_get_device_by_id(bus);
    if (dev == NULL) {
        printf("No device found on SPDIF bus %d\n", bus);
        return 1;
    }

    DO_CMD_SPDIF(open)
    DO_CMD_SPDIF(close)
    DO_CMD_SPDIF(start)
    DO_CMD_SPDIF(stop)

    if (!strcmp(argv[2].str, "setup")) {
        spdif_hw_params_t fmt;
        fmt.fmt = SPDIF_FMT_IEC60958;
        err = class_spdif_setup(dev, true, &fmt);
        printf("SPDIF setup on bus %d returned %d\n", bus, err);
        return 0;
    }

    if (!strcmp(argv[2].str, "read")) { \
        if (argc < 4) {
            printf("Invalid command\n");
            goto usage;
        }
        size_t len = argv[3].u;
        void *buf = malloc(len);
        ASSERT(buf);
        err = class_spdif_read(dev, buf, len);
        printf("SPDIF read on bus %d returned %d\n", bus, err);

        if ((argc > 4) && (!strcmp(argv[4].str, "dump")))
            hexdump(buf, len);
        free(buf);
        return 0;
    }

    if ((!strcmp(argv[2].str, "test")) && (argc == 6)){
        uint64_t pcm_data_mask;
        uint32_t pcm_data_max_val;
        uint16_t bitshift;
        /* Check args */
        switch (argv[3].u) {
            case 16:
                pcm_data_mask = 0xFFFF000;
                pcm_data_max_val = 0X7FFF;
                bitshift = 12;
                break;
            case 24:
                pcm_data_mask = 0xFFFFFF0;
                pcm_data_max_val = 0X7FFFFF;
                bitshift = 4;
                break;
            default:
                printf("Invalid command: bit format not supported.\tSupported values: 16, 24.\n");
                return -1;
        }
        if (argv[4].u % 0x80 != 0) {
            printf("Invalid chunk length: provide value multiple of 128.\n");
            return -1;
        }

        /* Open */
        err = class_spdif_open(dev, true);
        printf("SPDIF open on bus %d returned %d\n", bus, err);
        if (err)
            goto out_test;

        /* setup */
        spdif_hw_params_t fmt;
        fmt.fmt = SPDIF_FMT_IEC60958;
        err = class_spdif_setup(dev, true, &fmt);
        printf("SPDIF setup on bus %d returned %d\n", bus, err);
        if (err)
            goto out_test;

        /* start */
        err = class_spdif_start(dev, true);
        printf("SPDIF start on bus %d returned %d\n", bus, err);
        if (err)
            goto out_test;

        /* Read */
        size_t len = argv[4].u;
        void *buf = malloc(len);
        ASSERT(buf);
        uint16_t i = argv[5].u;
        uint32_t first_buf = 0;
        uint32_t last_buf = 0;
        uint32_t delta;
        addr_t address;

        while (i--) {
            err = class_spdif_read(dev, buf, len);
            printf("SPDIF read on bus %d returned %d\n", bus, err);
            if (err) {
                goto end_test;
}
            /* check SPDIF data */
            address = (addr_t)buf;
            first_buf = (((const int32_t *)address)[0] & pcm_data_mask) >> bitshift;
            delta = abs(first_buf - last_buf);
            if ( i < argv[5].u - 1) {
                /* check for gap between chunks */
                if ( (last_buf != pcm_data_max_val && delta != 1) || (last_buf == pcm_data_max_val && delta != pcm_data_max_val) ) {
                    printf("SPDIF ERROR - gap detected %#x:  %#x - %#x\n", delta, first_buf, last_buf);
                }
            }
            last_buf = (((const int32_t *)(address + len - 4))[0] & pcm_data_mask) >> bitshift;
        }
        goto end_test;

end_test:
        /* stop */
        err = class_spdif_stop(dev, true);
        printf("SPDIF stop on bus %d returned %d\n", bus, err);
        if (err)
            goto out_test_free;

        /* close */
        err = class_spdif_close(dev, true);
        printf("SPDIF close on bus %d returned %d\n", bus, err);
        if (err)
            goto out_test_free;
out_test_free:
        free(buf);
out_test:
        return err;
    }

    printf("unrecognized subcommand\n");
    goto usage;
}

APP_START(spdif)
    .flags = 0,
APP_END

#endif // WITH_APP_CONSOLE
