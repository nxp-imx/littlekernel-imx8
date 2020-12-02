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
#include <err.h>
#include <string.h>
#include <stdlib.h>
#include <kernel/thread.h>

#include "fsl_gpio.h"

#include <dev/class/gpt.h>
#include <assert.h>
#include <gpt.h>

#if WITH_LIB_CONSOLE

#include <lib/console.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#ifndef GPT1_BASE
#define GPT1_BASE (0x302D0000u)
#endif

/* ECSPI2_MOSI - gpio5.IO[11] - J1003 #19 */
#define CAPTURE_EVT_PIN (GPIO5)
#define CAPTURE_EVT_PIN_NUM (11)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int cmd_gpt(int argc, const cmd_args *argv);

/*******************************************************************************
 * Variables
 ******************************************************************************/
STATIC_COMMAND_START
STATIC_COMMAND("gpt", "gpt commands", &cmd_gpt)
STATIC_COMMAND_END(gpt);

#define DO_CMD_GPT(func) \
    if (!strcmp(argv[2].str, #func)) { \
        err = gpt_##func(bus); \
        printf("GPT func on bus %d returned %#x\n", bus, err); \
        return 0; \
    }

static thread_t *gpio_thread;
static int volatile g_gpio_toggle_enable = 0; /* flag to stop the gpio_thread thread */
static int volatile g_gpio_toggle_event = 0; /* capture event counter */
static int volatile g_capture_counter_prev = 0; /* previous value for capture event counter */
static int volatile g_capture_counter_err_cnt = 0; /* error count on capture between 2 events */

/*******************************************************************************
 * Code
 ******************************************************************************/

/** This thread toggles GPIO5 pin11 (ECSPI2_MOSI) every <arg> milliseconds.
 *  The thread exits when global variable g_gpio_toggle_enable is false.
 *  @param arg frequency to toggle GPIO (500 creates a 1-second period
 *             square wave)
 */
static int gpio_toggle_thread_entry(void *arg)
{
    uint32_t freq;
    printf("Entering GPIO thread...\n");
    do {
        freq = (intptr_t) arg;
        /* set GPIO on (ECSPI2_MOSI pin) */
        GPIO_PinWrite(CAPTURE_EVT_PIN, CAPTURE_EVT_PIN_NUM, 1);
        thread_sleep(freq);
        /* set GPIO off */
        GPIO_PinWrite(CAPTURE_EVT_PIN, CAPTURE_EVT_PIN_NUM, 0);
        thread_sleep(freq);
        //printlk(LK_INFO, "%s:%d: GPT1_SR = 0x%08x\n", __PRETTY_FUNCTION__, __LINE__, *(uint32_t *)(GPT1_BASE+0x08));
        //printlk(LK_INFO, "%s:%d: GPT1_IR = 0x%08x\n", __PRETTY_FUNCTION__, __LINE__, *(uint32_t *)(GPT1_BASE+0x0C));
    } while (g_gpio_toggle_enable);
    thread_exit(0);
    return 0;
}

/** callback function for capture ISR.
  * This function print current capture counter and the number of clocks since the last event.
  * It stops the GPIO toggle thread after 10 events.
  * @param userData pointer to a GPT_Type pointer.
  */
void my_gpt_capture_callback(void *userData)
{
    GPT_Type *base;
    uint32_t cur_cnt;
    uint32_t cnt_diff;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    base = (GPT_Type*) userData;
    g_gpio_toggle_event++;
    cur_cnt = *(&base->ICR[0] + kGPT_InputCapture_Channel1);
    cnt_diff = (cur_cnt - g_capture_counter_prev);
    printf("counter value: 0x%0x 0x%0x diff=%d\n",
         cur_cnt, g_capture_counter_prev, cnt_diff);
    g_capture_counter_prev = cur_cnt; /* save capture counter */

    /* stop toggling GPIO after 10 events */
    if (g_gpio_toggle_event >= 10) {
        g_gpio_toggle_enable = 0;
    }

    /* check capture counter validity
     * The timer counter is @24.576MHz (XTAL), events are at 1Hz
     * -> except 24576 between 2 events
     * (drop first 2 events)
     */
    if (g_gpio_toggle_event > 2) {
        /* found an error */
        if ((cnt_diff < 24572000) || (cnt_diff > 24580000)) {
            g_capture_counter_err_cnt++;
        }
    }
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
    return;
}

/* GPT command
 */
static int cmd_gpt(int argc, const cmd_args *argv)
{
    int err;

    if (argc < 3) {
        printf("not enough arguments\n");
usage:
        //printf("%s bus test len repeat\n", argv[0].str);
        printf("%s bus open\n",  argv[0].str);
        printf("%s bus close\n", argv[0].str);
        printf("%s bus start\n", argv[0].str);
        printf("%s bus stop\n",  argv[0].str);
        printf("%s bus get_counter\n",  argv[0].str);
        printf("%s bus setup_capture\n", argv[0].str);
        printf("%s bus enable_capture ch on\n", argv[0].str);
        printf("%s bus get_capture\n",  argv[0].str);
        return -1;
    }

    int bus = argv[1].u;

    DO_CMD_GPT(start)
    DO_CMD_GPT(stop)

    if (!strcmp(argv[2].str, "open")) {
        gpt_config_t cfg;
        cfg.clockSource = kGPT_ClockSource_Periph;
        cfg.divider = 1U;
        cfg.enableFreeRun = false; /*!< true: FreeRun mode, false: Restart mode. */
        cfg.enableRunInWait = true;  /*!< GPT enabled in wait mode. */
        cfg.enableRunInStop = true;  /*!< GPT enabled in stop mode. */
        cfg.enableRunInDoze = false; /*!< GPT enabled in doze mode. */
        cfg.enableRunInDbg = false;  /*!< GPT enabled in debug mode. */
        cfg.enableMode = true;  /*!< true: counter reset to 0 when enabled; */
        err = gpt_open(bus, &cfg, NULL, NULL);
        printf("GPT open on bus %d returned %#x\n", bus, err);
        return 0;
    }

    if (!strcmp(argv[2].str, "get_counter")) {
        uint32_t counter = 0;
        err = gpt_get_counter(bus, &counter);
        printf("GPT get_counter on bus %d counter 0x%x returned %#x\n", bus, counter, err);
        return 0;
    }

    if (!strcmp(argv[2].str, "setup_capture")) {
        gpt_capture_config_t cfg;
        cfg.channel = kGPT_InputCapture_Channel1;
        cfg.mode = kGPT_InputOperation_RiseEdge;
        cfg.irq_enable = true;
        err = gpt_setup_capture(bus, &cfg, my_gpt_capture_callback, ((uintptr_t *)GPT1_BASE));
        printf("GPT setup_capture on bus %d returned %#x\n", bus, err);
        return 0;
    }

    if ((!strcmp(argv[2].str, "enable_capture")) && (argc ==5)) {
        int channel = argv[3].u;
        bool on = argv[4].b;
        err = gpt_enable_capture(bus, channel, on);
        printf("GPT enable_capture on bus %d returned %#x\n", bus, err);
        return 0;
    }

    /** toggle GPIO ECSPI2_MOSI
     *  @param bus unused
     *  @param "toggle_ecspi2_mosi"
     *  @param enable GPIO toggle
     *  @param toggle frequency
     */
    if ((!strcmp(argv[2].str, "toggle_ecspi2_mosi")) && (argc == 5)) {
        int enable = argv[3].u; /* enable/disable toggle */
        int freq = argv[4].u; /* toggle frequency for GPIO */
        printlk(LK_INFO, "%s:%d: GPT toggle_ecspi2_mosi on bus %d enable=%d freq=%d\n",
                __PRETTY_FUNCTION__, __LINE__, bus, enable, freq);
        gpio_pin_config_t gpioConfig = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
        GPIO_PinInit(CAPTURE_EVT_PIN, CAPTURE_EVT_PIN_NUM, &gpioConfig);

        gpio_thread = thread_create(
            "gpio_toggle",
            &gpio_toggle_thread_entry,
            (void *)(intptr_t)freq,
            DEFAULT_PRIORITY,
            DEFAULT_STACK_SIZE);

        g_gpio_toggle_enable = enable;
        if (enable){
            printlk(LK_INFO, "%s:%d: enabling gpio_thread\n", __PRETTY_FUNCTION__, __LINE__);
            thread_detach(gpio_thread);
            thread_resume(gpio_thread);
        } else {
            printlk(LK_INFO, "%s:%d: killing gpio_thread\n", __PRETTY_FUNCTION__, __LINE__);
            thread_join(gpio_thread, NULL, 1000);
        }
        return 0;
    }

    /* GPT test:
     * - toggle a GPIO at 1Hz.
     * - physically connect GPIO to GPT1.CAPTURE1 pin.
     * - toggling GPIO triggers capture interrupts.
     * - capture counter is displayed.
     * - stops after 10 captures (10 s).
    */
    if ((!strcmp(argv[2].str, "test")) && (argc == 3)) {
        /* open */
        gpt_config_t cfg;
        cfg.clockSource = kGPT_ClockSource_Periph;
        cfg.divider = 1U;
        cfg.enableFreeRun = false; /*!< true: FreeRun mode, false: Restart mode. */
        cfg.enableRunInWait = true;  /*!< GPT enabled in wait mode. */
        cfg.enableRunInStop = true;  /*!< GPT enabled in stop mode. */
        cfg.enableRunInDoze = false; /*!< GPT enabled in doze mode. */
        cfg.enableRunInDbg = false;  /*!< GPT enabled in debug mode. */
        cfg.enableMode = true;  /*!< true: counter reset to 0 when enabled; */
        err = gpt_open(bus, &cfg, NULL, NULL);
        printf("GPT open on bus %d returned %#x\n", bus, err);
        if (err)
            return err;

        /* start */
        err = gpt_start(bus);
        printf("GPT start on bus %d returned  %#x\n", bus, err);
        if (err)
            return err;
        /* get counter */
        uint32_t cnt = 0;
        err = gpt_get_counter(bus, &cnt);
        printf("GPT get_counter on bus %d counter 0x%x returned %#x\n", bus, cnt, err);

        /* open capture */
        gpt_capture_config_t ccfg;
        ccfg.channel = kGPT_InputCapture_Channel1;
        ccfg.mode = kGPT_InputOperation_RiseEdge;
        ccfg.irq_enable = true;
        err = gpt_setup_capture(bus, &ccfg, my_gpt_capture_callback, ((uintptr_t *)GPT1_BASE));
        printf("GPT open_capture on bus %d returned %#x\n", bus, err);
        if (err)
            return err;

        gpt_enable_capture(bus, ccfg.channel, false);
        /* toggle GPIO5 pin11 (ECSPI2_MOSI) @ 1Hz */
        /* (ECSPI2_MOSI is located on J1003 pin 19) */
        printf("Toggling ECSPI2_MOSI at 1Hz.\n");
        gpio_pin_config_t gpioConfig = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
        GPIO_PinInit(CAPTURE_EVT_PIN, CAPTURE_EVT_PIN_NUM, &gpioConfig);
        int freq = 500; /* 1Hz square wave */
        g_gpio_toggle_enable = 1;
        g_gpio_toggle_event = 0; /* reset event counter */
        g_capture_counter_prev = 0; /* reset previous capture value */
        g_capture_counter_err_cnt = 0; /* reset error count */

        gpio_thread = thread_create(
            "gpio_toggle",
            &gpio_toggle_thread_entry,
            (void *)(intptr_t)freq,
            DEFAULT_PRIORITY,
            DEFAULT_STACK_SIZE);
        /* GPT1.CAPTURE1 (SAI3.RXFS) is physically located on the
         * back side of EVK at TP218 (silkscreen label "RXFS")
         */
        printf("Please connect ECSPI2_MOSI to GPT1.CAPTURE1\n");

        thread_resume(gpio_thread);

        thread_sleep(5000);
        gpt_enable_capture(bus, ccfg.channel, true);

        /* wait until 10 events are received */
        thread_join(gpio_thread, NULL, INFINITE_TIME);

        /* stop */
        err = gpt_stop(bus);
        printf("GPT stop on bus %d returned %#x\n", bus, err);
        if (err)
            return err;

        /* close */
        err = gpt_close(bus);
        printf("GPT close on bus %d returned %#x\n", bus, err);
        if (err)
            return err;

        /* test result */
        printf("GPT test result: %d events, %d errors.\n",
            g_gpio_toggle_event, g_capture_counter_err_cnt);
        if ((g_capture_counter_err_cnt != 0) || (g_gpio_toggle_event == 0)) {
            printf("FAILED\n");
            return ERR_GENERIC;
        } else {
            printf("PASSED\n");
        }

        return 0;
    }

    printf("unrecognized subcommand\n");
    goto usage;
}

APP_START(gpt)
    .flags = 0,
APP_END

#endif // WITH_APP_CONSOLE

