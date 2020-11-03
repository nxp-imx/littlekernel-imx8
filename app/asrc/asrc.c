/*
 * Copyright 2020-2021 NXP
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
#include <dev/class/asrc.h>
#include <assert.h>
#ifndef NO_CIPC_DRIVER
#include <ivshmem-endpoint.h>
#include "cipc.h"
#endif
#include "platform.h"       // for current_time_hires

#define FILE_NAME_MAX                   128
#define CIPC_WRITE_BYTES                2048
#define MAX_RATIO_SUPPORTED             48      // 384 kHz to 8 kHz

#define INPUT_BUFFER_SIZE               4096
#define OUTPUT_BUFFER_SIZE              (INPUT_BUFFER_SIZE * 48)

#define APP_ASRC_BENCHMARK
#define APP_ASRC_FLUSH

#if WITH_LIB_CONSOLE

#include <lib/console.h>

static int cmd_asrc(int argc, const cmd_args *argv);
static int test_asrc(struct device *dev, int bus,
        const char *in_file, uint32_t nch, uint32_t type,
        uint32_t in_freq, uint32_t out_freq, const char *out_file);

static const struct {
    const char *in_file;    /* Input file */
    uint32_t nch;           /* Number of channels */
    uint32_t type;          /* Type of audio data (0: int32, 1: float) */
    uint32_t in_freq;       /* Input frequency */
    uint32_t out_freq;      /* Output frequency */
    const char *out_file;   /* Output file */
} validation_tests[] = {
    {"sine_1000Hz_isr_48000_ch_2_10sec_f32le.raw", 2, 1, 48000, 96000,  "out_2ch_flt_48_96.raw"},
    {"sine_1000Hz_isr_32000_ch_2_10sec_f32le.raw", 2, 1, 32000, 48000,  "out_2ch_flt_32_48.raw"},
    {"sine_1000Hz_isr_48000_ch_2_10sec_f32le.raw", 2, 1, 48000, 192000, "out_2ch_flt_48_192.raw"},
    {"sine_1000Hz_isr_44100_ch_2_10sec_f32le.raw", 2, 1, 44100, 96000,  "out_2ch_flt_44p1_96.raw"},
    {"sine_1000Hz_isr_32000_ch_2_10sec_f32le.raw", 2, 1, 32000, 192000, "out_2ch_flt_32_192.raw"},

    {"sine_1000Hz_isr_48000_ch_2_10sec_f32le.raw", 2, 1, 48000, 44100,  "out_2ch_flt_48_44p1.raw"},
    {"sine_1000Hz_isr_96000_ch_2_10sec_f32le.raw", 2, 1, 96000, 48000,  "out_2ch_flt_96_48.raw"},
    {"sine_1000Hz_isr_192000_ch_2_10sec_f32le.raw", 2, 1, 192000, 48000, "out_2ch_flt_192_48.raw"},
    {"sine_1000Hz_isr_192000_ch_2_10sec_f32le.raw", 2, 1, 192000, 44100, "out_2ch_flt_192_44p1.raw"},
    {"sine_1000Hz_isr_88200_ch_2_10sec_f32le.raw", 2, 1, 88200, 32000,  "out_2ch_flt_88p2_32.raw"},
};

#define     FILENAME_LENGTH     64

static const struct {
    uint32_t freq;          /* frequency */
} freqs_list[] = {
    {32000},
    {44100},
    {48000},
    {88200},
    {96000},
    {176400},
    {192000},
};

STATIC_COMMAND_START
STATIC_COMMAND("asrc", "asrc driver test", &cmd_asrc)
STATIC_COMMAND_END(asrc);

#define DO_CMD_ASRC(func) \
    if (!strcmp(argv[2].str, #func)) { \
        err = class_asrc_##func(dev); \
        printf("ASRC func on bus %d returned %d\n", bus, err); \
        return 0; \
    }

static int cmd_asrc(int argc, const cmd_args *argv)
{
    int err;

    if (argc < 3) {
        printf("Not enough arguments\n");
usage:
        printf("%s <bus> open\n", argv[0].str);
        printf("%s <bus> close\n", argv[0].str);
        printf("%s <bus> setup\n", argv[0].str);
        printf("%s <bus> stop\n", argv[0].str);
        printf("%s <bus> test <in_file> <n_ch> <data_type> <in_rate> <out_rate> <out_file>\n", argv[0].str);
        printf("%s <bus> validation\n", argv[0].str);
        printf("%s <bus> validation_full [<n_ch>] [<data_type>]\n", argv[0].str);
        printf("   optional <n_ch>: 0 < n_ch <= 32, default: 2\n");
        printf("   optional <data_type>: 0 for integer32, 1 for float32 (default)\n");
        return -1;
    }

    int bus = argv[1].u;
    struct device *dev = class_asrc_get_device_by_id(bus);
    if (dev == NULL) {
        printf("No device found on ASRC bus %d\n", bus);
        return 1;
    }

    DO_CMD_ASRC(open)
    DO_CMD_ASRC(close)
    DO_CMD_ASRC(stop)

    if (!strcmp(argv[2].str, "setup")) {
        asrc_audio_params_t audio_params;

        audio_params.num_channels = 16;
        audio_params.in_sample_rate = 44100;
        audio_params.out_sample_rate = 4800000;
        audio_params.data_type = ASRC_DataTypeInteger;

        printf("I/O freq: %d/%d\n", audio_params.in_sample_rate, audio_params.out_sample_rate);
        err = class_asrc_setup(dev, &audio_params);
        printf("ASRC setup on bus %d returned %d\n", bus, err);

        audio_params.in_sample_rate = 44100;
        audio_params.out_sample_rate = 48000;
        printf("I/O freq: %d/%d\n", audio_params.in_sample_rate, audio_params.out_sample_rate);
        err = class_asrc_setup(dev, &audio_params);
        printf("ASRC setup on bus %d returned %d\n", bus, err);

        audio_params.in_sample_rate = 4410;
        audio_params.out_sample_rate = 48000;
        printf("I/O freq: %d/%d\n", audio_params.in_sample_rate, audio_params.out_sample_rate);
        err = class_asrc_setup(dev, &audio_params);
        printf("ASRC setup on bus %d returned %d\n", bus, err);

        audio_params.in_sample_rate = 4410;
        audio_params.out_sample_rate = 480;
        printf("I/O freq: %d/%d\n", audio_params.in_sample_rate, audio_params.out_sample_rate);
        err = class_asrc_setup(dev, &audio_params);
        printf("ASRC setup on bus %d returned %d\n", bus, err);

        audio_params.in_sample_rate = 4410000;
        audio_params.out_sample_rate = 48000;
        printf("I/O freq: %d/%d\n", audio_params.in_sample_rate, audio_params.out_sample_rate);
        err = class_asrc_setup(dev, &audio_params);
        printf("ASRC setup on bus %d returned %d\n", bus, err);
        return 0;
    }

    if (!strcmp(argv[2].str, "test")) {
        int argc_expected = 9;

        if (argc != argc_expected) {
            printf("Invalid command, expect %d parameters (received %d)\n", argc_expected, argc);
            goto usage;
        }

        printf("(Do not forget to launch ivshm_binary on Linux side)\n");

        /* Fill args */
        const char *in_file     = argv[3].str;
        uint32_t nch            = argv[4].u;
        uint32_t type           = argv[5].u;
        uint32_t in_freq        = argv[6].u;
        uint32_t out_freq       = argv[7].u;
        const char *out_file    = argv[8].str;

        err = test_asrc(dev, bus, in_file, nch, type, in_freq, out_freq, out_file);
        return err;
    }

    if (!strcmp(argv[2].str, "validation")) {
        int argc_expected = 3;

        if (argc != argc_expected) {
            printf("Invalid command, expect %d parameters (received %d)\n", argc_expected, argc);
            goto usage;
        }

        printf("(Do not forget to launch ivshm_binary on Linux side)\n");

        for (unsigned i = 0; i < ARRAY_SIZE(validation_tests); i++) {
            err = test_asrc(dev, bus,
                    validation_tests[i].in_file,
                    validation_tests[i].nch,
                    validation_tests[i].type,
                    validation_tests[i].in_freq,
                    validation_tests[i].out_freq,
                    validation_tests[i].out_file);

            if (err) {
                printf("ERROR: Error during test validation %d\n", i);
                goto out_validation;
            }
        }

out_validation:
        return err;
    }

    if (!strcmp(argv[2].str, "validation_full")) {
        int argc_expected_min = 3;

        char in_file[FILE_NAME_MAX], out_file[FILE_NAME_MAX];
        uint32_t nch = 2;       // 2 channels
        uint32_t type = 1;      // Default: float32

        if (argc < argc_expected_min) {
            printf("Invalid command, expect %d parameters minimum (received %d)\n", argc_expected_min, argc);
            goto usage;
        }

        if (argc >= argc_expected_min + 1) // nch provided by arg
            nch = argv[3].u;
        if (argc >= argc_expected_min + 2) // data_type provided by arg
            type = argv[4].u;

        printf("(Do not forget to launch ivshm_binary on Linux side)\n");

        for (unsigned i = 0; i < ARRAY_SIZE(freqs_list); i++) {
            for (unsigned j = 0; j < ARRAY_SIZE(freqs_list); j++) {

                snprintf(in_file, FILE_NAME_MAX, "sine_1000Hz_isr_%d_ch_%d_10sec_%s.raw",
                                                    freqs_list[i].freq,
                                                    nch,
                                                    (type) ? "f32le" : "s32le");
                snprintf(out_file, FILE_NAME_MAX, "out_%d_%s_%d_%d.raw",
                                                    nch,
                                                    (type) ? "flt" : "int",
                                                    freqs_list[i].freq,
                                                    freqs_list[j].freq);

                err = test_asrc(dev, bus,
                        in_file,
                        nch,
                        type,
                        freqs_list[i].freq,
                        freqs_list[j].freq,
                        out_file);

                if (err) {
                    printf("ERROR: Error during test validation %d => %d\n", freqs_list[i].freq, freqs_list[j].freq);
                    goto out_validation_full;
                }
            }
        }

out_validation_full:
        return err;
    }

    printf("unrecognized subcommand\n");
    goto usage;
    return -1;
}

APP_START(asrc)
    .flags = 0,
APP_END

#ifndef NO_CIPC_DRIVER
static int test_asrc(struct device *dev, int bus,
        const char *in_file, uint32_t nch, uint32_t type,
        uint32_t in_freq, uint32_t out_freq, const char *out_file)
{
    int err = 0;
    asrc_audio_data_type_t data_type;

    /* Check args */
    if (nch == 0 || nch > 32) {
        printf("Invalid number of channels (%d), shall be 0 < n_ch <= 32.\n", nch);
        return -1;
    }
    switch (type) {
        case 0:
            data_type = ASRC_DataTypeInteger;
            break;
        case 1:
            data_type = ASRC_DataTypeFloat;
            break;
        default:
            printf("Invalid data type (%d not supported), supported values: 0 for 32-bits integer, 1 for float.\n", type);
            return -1;
    }

    printf("Input file/Output file: %s/%s\n", in_file, out_file);
    printf("Number of channels: %d\n", nch);
    printf("Data type: %s\n", (type == 0) ? "integer" : "float");
    printf("Input freq/Output freq: %d/%d\n", in_freq, out_freq);

    /* Open */
    err = class_asrc_open(dev);
    if (err) {
        printf("ASRC open on bus %d returned %d\n", bus, err);
        goto end_valid_test;
    }

    /* Setup */
    asrc_audio_params_t audio_params = {
        .num_channels = nch,
        .in_sample_rate = in_freq,
        .out_sample_rate = out_freq,
        .data_type = data_type,
    };
    err = class_asrc_setup(dev, &audio_params);
    printf("ASRC setup on bus %d returned %d\n", bus, err);
    if (err)
        goto out_valid_test;

    /* Check if input file exists */
    ssize_t input_size = cipc_size_file(IVSHM_EP_ID_BINARY, (char *) in_file);
    if (input_size < 0) {
        printf("Error: Not able to open %s file, returned error: %ld\n", in_file, input_size);
        err = -1;
        goto out_valid_test;
    }
    if (input_size == 0) {
        printf("Error: file %s is empty\n", in_file);
        err = -1;
        goto out_valid_test;
    }
    printf("Input file size of %s file is %ld\n", in_file, input_size);

    input_size = 1000000;   // Clip input size to 1 Mbytes

    // Ensure input size is a multiple of samples group (= size(samples) * nch)
    input_size -= input_size % (sizeof(float) * nch);

    asrc_transfer_params_t transfer_params;
    asrc_out_params_t out_params;
    size_t in_len, in_cur_len;
    ssize_t read_len;
    size_t out_len, out_cur_len;
    ssize_t write_len;

    int32_t *in_buf = calloc(1, input_size);
    int32_t *out_buf = calloc(1, OUTPUT_BUFFER_SIZE);
    ASSERT(in_buf);
    ASSERT(out_buf);

    int32_t *in_cur_buf, *out_cur_buf;

    printf("Size to read from file: %ld bytes\n", input_size);
    read_len = cipc_read_file(IVSHM_EP_ID_BINARY, in_buf, input_size, (char *)in_file);
    if (read_len <= 0) {
        printf("Issue during CIPC read, returned value: %ld\n", read_len);
        return -1;
    }
    if (read_len != (ssize_t)input_size) {
        printf("Issue during CIPC read: read %ld, expected %ld\n", read_len, input_size);
        return -1;
    }

    in_cur_buf = in_buf;
    in_len = input_size;
#ifdef APP_ASRC_BENCHMARK
    lk_bigtime_t delta_acc = 0, us_start, us_stop, delta;
    size_t in_len_acc = 0;
#endif

    do {
        in_cur_len = (in_len > INPUT_BUFFER_SIZE) ? INPUT_BUFFER_SIZE : in_len;
        // Ensure buffer size is a multiple of samples group (= size(samples) * nch)
        in_cur_len -= in_cur_len % (sizeof(float) * nch);
        in_len -= in_cur_len;

        transfer_params.in_buffer = (int32_t *) in_cur_buf;
        transfer_params.out_buffer = (int32_t *) out_buf;
        transfer_params.in_size_bytes = in_cur_len;

        // Update for next read
        in_cur_buf = (int32_t *) ((int8_t *)in_cur_buf + in_cur_len);

#ifdef APP_ASRC_BENCHMARK
        us_start = current_time_hires();
#endif // APP_ASRC_BENCHMARK
        /* Start */
        err = class_asrc_start(dev, &transfer_params, &out_params);

#ifdef APP_ASRC_BENCHMARK
        us_stop = current_time_hires();
        delta = us_stop - us_start;
        delta_acc += delta;
        in_len_acc += in_cur_len;
#if 0
        float us_per_samples = (float) delta / (in_cur_len / nch / 4);

        printf("class_asrc_start = %llu usecs\n", delta);
        printf(" => %f us per sample, so %f MHz for %d Hz\n",
                    us_per_samples,
                    1500.0 * us_per_samples * in_freq / 1000000,
                    in_freq);
#endif
#endif // APP_ASRC_BENCHMARK

        if (err) {
            printf("Error during asrc_start, returned %d\n", err);
            goto out_valid_test_free;
        }

        if (out_params.out_size_bytes > OUTPUT_BUFFER_SIZE) {
            printf("Error: Produced buffer is too big for buffer, %d but sized to %d bytes\n",
                out_params.out_size_bytes,
                OUTPUT_BUFFER_SIZE);
            goto out_valid_test_free;
        }

        out_len = out_params.out_size_bytes;
        out_cur_buf = (int32_t *) out_buf;

        while (out_len > 0) {
            out_cur_len = (out_len > CIPC_WRITE_BYTES) ? CIPC_WRITE_BYTES : out_len;
            write_len = cipc_write_file(IVSHM_EP_ID_BINARY, out_cur_buf, out_cur_len, (char *)out_file, FILE_O_APPEND);
            if (write_len <= 0) {
                printf("Issue during CIPC write, returned value: %ld\n", write_len);
                return -1;
            }
            if (write_len != (ssize_t)out_cur_len) {
                printf("Issue during write CIPC: write %ld, expected: %ld\n", write_len, out_cur_len);
                return -1;
            }

            out_cur_buf = (int32_t *) ((int8_t *)out_cur_buf + out_cur_len);
            out_len -= out_cur_len;
        }

    } while(in_len > 0);

#ifdef APP_ASRC_BENCHMARK
    float us_per_sample = (float) delta_acc / (in_len_acc / nch / 4);

    printf("%llu usec for %ld input bytes\n", delta_acc, in_len_acc);
    printf(" => %f us per sample, so %f MHz for %d Hz\n",
                us_per_sample,
                1500.0 * us_per_sample * in_freq / 1000000,
                in_freq);
#endif // APP_ASRC_BENCHMARK

#ifdef APP_ASRC_FLUSH
    /* Flush */
    asrc_flush_params_t flush_params = {
        .out_buffer = (int32_t *) out_buf,
    };
    asrc_out_params_t out_flush_params;
    err = class_asrc_flush(dev, &flush_params, &out_flush_params);
    if (err) {
        printf("ASRC flush on bus %d returned %d\n", bus, err);
        goto end_valid_test;
    }

    if (out_flush_params.out_size_bytes > OUTPUT_BUFFER_SIZE) {
        printf("Error: Flush buffer is too big for buffer, %d but sized to %d bytes\n",
            out_flush_params.out_size_bytes,
            OUTPUT_BUFFER_SIZE);
        goto out_valid_test_free;
    }

    out_len = out_flush_params.out_size_bytes;
    out_cur_buf = (int32_t *) out_buf;

    while (out_len > 0) {
        out_cur_len = (out_len > CIPC_WRITE_BYTES) ? CIPC_WRITE_BYTES : out_len;
        write_len = cipc_write_file(IVSHM_EP_ID_BINARY, out_cur_buf, out_cur_len, (char *)out_file, FILE_O_APPEND);
        if (write_len <= 0) {
            printf("Issue during CIPC write, returned value: %ld\n", write_len);
            return -1;
        }
        if (write_len != (ssize_t)out_cur_len) {
            printf("Issue during write CIPC: write %ld, expected: %ld\n", write_len, out_cur_len);
            return -1;
        }

        out_cur_buf = (int32_t *) ((int8_t *)out_cur_buf + out_cur_len);
        out_len -= out_cur_len;
    }
#endif // APP_ASRC_FLUSH

out_valid_test_free:
    free(in_buf);
    free(out_buf);

out_valid_test:
    /* Stop */
    err = class_asrc_stop(dev);
    printf("ASRC stop on bus %d returned %d\n", bus, err);
    if (err)
        goto end_valid_test;
    /* Close */
    err = class_asrc_close(dev);
    printf("ASRC close on bus %d returned %d\n", bus, err);
    if (err)
        goto end_valid_test;

end_valid_test:
    return err;
}
#else // NO_CIPC_DRIVER
static int test_asrc(struct device *dev, int bus,
        const char *in_file, uint32_t nch, uint32_t type,
        uint32_t in_freq, uint32_t out_freq, const char *out_file)
{
    (void) dev;
    (void) bus;
    (void) in_file;
    (void) nch;
    (void) type;
    (void) in_freq;
    (void) out_freq;
    (void) out_file;

    return -1;
}
#endif // NO_CIPC_DRIVER

#endif // WITH_APP_CONSOLE
