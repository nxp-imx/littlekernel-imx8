/*
 * Copyright 2020 NXP
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
#ifndef __IMX_PLATFORM_COMMON
#define __IMX_PLATFORM_COMMON

struct imx_hw_state_s {
    spin_lock_t lock;
    unsigned state;
};

static inline void imx_init_hw_state(struct imx_hw_state_s *hw_state)
{
    hw_state->state = 0;
    spin_lock_init(&hw_state->lock);
}

static inline void imx_change_hw_state(struct imx_hw_state_s *hw_state,
                                                    unsigned state, bool set)
{

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&hw_state->lock, lock_state);

    if (set)
        hw_state->state |= state;
    else
        hw_state->state &= ~state;

    smp_wmb();
    spin_unlock_irqrestore(&hw_state->lock, lock_state);
}

static inline void imx_set_hw_state(struct imx_hw_state_s *hw_state,
                                                    unsigned state)
{
    imx_change_hw_state(hw_state, state, true);
}

static inline void imx_clear_hw_state(struct imx_hw_state_s *hw_state,
                                                    unsigned state)
{
    imx_change_hw_state(hw_state, state, false);
}

static inline bool imx_is_hw_state(struct imx_hw_state_s *hw_state, unsigned state)
{
    return ((hw_state->state & state) == state);
}

static inline uint32_t imx_get_hw_state(struct imx_hw_state_s *hw_state)
{
    return hw_state->state;
}

static status_t _imx_check_hw_state(
                                    struct imx_hw_state_s *hw_state,
                                    uint32_t expected_state,
                                    const char *msg,
                                    bool strict)
{
    bool cond;
    if (strict)
        cond = (imx_get_hw_state(hw_state) == expected_state);
    else
        cond = imx_is_hw_state(hw_state, expected_state);

    if (!cond) {
        printlk(LK_INFO, "%s:%d: Wrong state (%x) while %s the interface\n",
                __PRETTY_FUNCTION__, __LINE__, imx_get_hw_state(hw_state), msg);
        return ERR_BAD_STATE;
    }

    return 0;
}

static inline status_t imx_check_hw_state(
                                    struct imx_hw_state_s *hw_state,
                                    uint32_t expected_state,
                                    const char *msg)
{

    return _imx_check_hw_state(hw_state, expected_state, msg, false);
}

static inline status_t imx_check_strict_hw_state(
                                    struct imx_hw_state_s *hw_state,
                                    uint32_t expected_state,
                                    const char *msg)
{

    return _imx_check_hw_state(hw_state, expected_state, msg, true);
}



#endif /* __IMX_PLATFORM_COMMON */

