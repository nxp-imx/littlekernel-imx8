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
#include <err.h>
#include <string.h>
#include <dev/class/dac.h>
#include <stdio.h>


#if WITH_LIB_CONSOLE

#include <lib/console.h>

static int cmd_dac(int argc, const cmd_args *argv);

STATIC_COMMAND_START
STATIC_COMMAND("dac", "dac commands", &cmd_dac)
STATIC_COMMAND_END(dac);

static int cmd_dac(int argc, const cmd_args *argv)
{
    int err;

    if (argc < 3) {
        printf("not enough arguments\n");
usage:
        printf("%s bus rpc\n", argv[0].str);
        return -1;
    }

    int bus = argv[1].u;
    struct device *dev = class_dac_get_device_by_id(bus);
    if (dev == NULL) {
        printf("No device found on dac bus %d\n", bus);
        return 1;
    }

    if (!strcmp(argv[2].str, "rpc")) {
        uint32_t capabilities;
        dac_audio_hw_params_t params = {
            .pcm_fmt = DAC_AUDIO_PCM_FMT_24,
            .fmt = DAC_AUDIO_FMT_AC97,
            .pkt = DAC_AUDIO_PKT_PCM,
            .num_ch = 8,
        };

        err = class_dac_open(dev);
        printf("dac open on bus %d returned  %#x\n", bus, err);
        if (err) {
            return 1;
        }

        err = class_dac_get_capabilities(dev, &capabilities);
        if (err)
            return 1;
        printf("dac get capabilities returns: %#x\n", capabilities);

        err = class_dac_set_format(dev, &params);
        if (err)
            return 1;
        printf("dac set format done: pcm_fmt/fmt/pkt/num_ch [%x:%x:%x:%x]\n",
                        params.pcm_fmt, params.fmt, params.pkt, params.num_ch);

        err = class_dac_close(dev);
        printf("dac close on bus %d returned  %#x\n", bus, err);
        if (err)
            return 1;

        return 0;
    }

    printf("unrecognized subcommand\n");
    goto usage;
}

APP_START(dac)
    .flags = 0,
APP_END

#endif // WITH_APP_CONSOLE


