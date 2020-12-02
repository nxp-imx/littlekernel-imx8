/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_clock.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.clock"
#endif

#define FracPLL_GNRL_CTL_Offset (0U)
#define FracPLL_FDIV_CTL0_Offset (4U)
#define FracPLL_FDIV_CTL1_Offset (8U)

#define IntegerPLL_GNRL_CTL_Offset (0U)
#define IntegerPLL_DIV_CTL_Offset (4U)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * brief Gets the clock frequency for a specific clock name.
 *
 * This function checks the current clock configurations and then calculates
 * the clock frequency for a specific clock name defined in clock_name_t.
 *
 * param clockName Clock names defined in clock_name_t
 * return Clock frequency value in hertz
 */
uint32_t CLOCK_GetFreq(clock_name_t clockName)
{
    uint32_t freq;

    switch (clockName)
    {
        case kCLOCK_CoreM7Clk:
            freq = CLOCK_GetCoreM7Freq();
            break;
        case kCLOCK_AxiClk:
            freq = CLOCK_GetAxiFreq();
            break;
        case kCLOCK_AhbClk:
            freq = CLOCK_GetAhbFreq();
            break;
        case kCLOCK_IpgClk:
            freq = CLOCK_GetAhbFreq() / CLOCK_GetRootPostDivider(kCLOCK_RootIpg);
            break;
        default:
            freq = 0U;
            break;
    }
    return freq;
}

/*!
 * brief Get the CCM Cortex M7 core frequency.
 *
 * return  Clock frequency; If the clock is invalid, returns 0.
 */
uint32_t CLOCK_GetCoreM7Freq(void)
{
    uint32_t freq;
    uint32_t pre  = CLOCK_GetRootPreDivider(kCLOCK_RootM7);
    uint32_t post = CLOCK_GetRootPostDivider(kCLOCK_RootM7);

    switch (CLOCK_GetRootMux(kCLOCK_RootM7))
    {
        case kCLOCK_M7RootmuxOsc24M:
            freq = OSC24M_CLK_FREQ;
            break;
        case kCLOCK_M7RootmuxSysPll2Div5:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 5U;
            break;
        case kCLOCK_M7RootmuxSysPll2Div4:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 4U;
            break;
        case kCLOCK_M7RootmuxSysPll1Div3:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 3U;
            break;
        case kCLOCK_M7RootmuxSysPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl);
            break;
        case kCLOCK_M7RootmuxAudioPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_AudioPll1Ctrl);
            break;
        case kCLOCK_M7RootmuxVideoPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_VideoPll1Ctrl);
            break;
        case kCLOCK_M7RootmuxSysPll3:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll3Ctrl);
            break;
        default:
            return 0;
    }

    return freq / pre / post;
}

/*!
 * brief Get the CCM Axi bus frequency.
 *
 * return  Clock frequency; If the clock is invalid, returns 0.
 */
uint32_t CLOCK_GetAxiFreq(void)
{
    uint32_t freq;
    uint32_t pre  = CLOCK_GetRootPreDivider(kCLOCK_RootAxi);
    uint32_t post = CLOCK_GetRootPostDivider(kCLOCK_RootAxi);

    switch (CLOCK_GetRootMux(kCLOCK_RootAxi))
    {
        case kCLOCK_AxiRootmuxOsc24M:
            freq = OSC24M_CLK_FREQ;
            break;
        case kCLOCK_AxiRootmuxSysPll2Div3:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 3U;
            break;
        case kCLOCK_AxiRootmuxSysPll2Div4:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 4U;
            break;
        case kCLOCK_AxiRootmuxSysPll2:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl);
            break;
        case kCLOCK_AxiRootmuxAudioPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_AudioPll1Ctrl);
            break;
        case kCLOCK_AxiRootmuxVideoPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_VideoPll1Ctrl);
            break;
        case kCLOCK_AxiRootmuxSysPll1Div8:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 8;
            break;
        case kCLOCK_AxiRootmuxSysPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl);
            break;
        default:
            return 0;
    }

    return freq / pre / post;
}

/*!
 * brief Get the CCM Ahb bus frequency.
 *
 * return  Clock frequency; If the clock is invalid, returns 0.
 */
uint32_t CLOCK_GetAhbFreq(void)
{
    uint32_t freq;
    uint32_t pre  = CLOCK_GetRootPreDivider(kCLOCK_RootAhb);
    uint32_t post = CLOCK_GetRootPostDivider(kCLOCK_RootAhb);

    switch (CLOCK_GetRootMux(kCLOCK_RootAhb))
    {
        case kCLOCK_AhbRootmuxOsc24M:
            freq = OSC24M_CLK_FREQ;
            break;
        case kCLOCK_AhbRootmuxSysPll1Div6:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 6U;
            break;
        case kCLOCK_AhbRootmuxSysPll1Div2:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 2U;
            break;
        case kCLOCK_AhbRootmuxSysPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl);
            break;
        case kCLOCK_AhbRootmuxSysPll2Div8:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 8U;
            break;
        case kCLOCK_AhbRootmuxSysPll3:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll3Ctrl);
            break;
        case kCLOCK_AhbRootmuxAudioPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_AudioPll1Ctrl);
            break;
        case kCLOCK_AhbRootmuxVideoPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_VideoPll1Ctrl);
            break;
        default:
            return 0;
    }

    return freq / pre / post;
}

uint32_t CLOCK_GetSaiFreq(clock_root_control_t sai_root)
{
    switch (sai_root)
    {
        case kCLOCK_RootSai2:
        case kCLOCK_RootSai3:
        case kCLOCK_RootSai5:
        case kCLOCK_RootSai6:
        case kCLOCK_RootSai7:
            break;
        default:
            return 0;
    }
    uint32_t freq;
    uint32_t pre = CLOCK_GetRootPreDivider(sai_root);
    uint32_t post = CLOCK_GetRootPostDivider(sai_root);

    switch (CLOCK_GetRootMux(sai_root))
    {
        case kCLOCK_SaiRootmuxOsc24M:
            freq = OSC24M_CLK_FREQ;
            break;
        case kCLOCK_SaiRootmuxSysPll1Div6:
            freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 6U;
            break;
        case kCLOCK_SaiRootmuxAudioPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_AudioPll1Ctrl);
            break;
        case kCLOCK_SaiRootmuxAudioPll2:
            freq = CLOCK_GetPllFreq(kCLOCK_AudioPll2Ctrl);
            break;
        case kCLOCK_SaiRootmuxVideoPll1:
            freq = CLOCK_GetPllFreq(kCLOCK_VideoPll1Ctrl);
            break;
        case kCLOCK_SaiRootmuxOsc26m:
            freq = 26000000U;
            break;
        case kCLOCK_SaiRootmuxExtClk1:
        case kCLOCK_SaiRootmuxExtClk2:
        default:
            return 0;
    }

    return freq / pre / post;
}



/*!
 * brief Gets PLL reference clock frequency.
 *
 * param type fractional pll type.

 * return  Clock frequency
 */
uint32_t CLOCK_GetPllRefClkFreq(clock_pll_ctrl_t ctrl)
{
    uint32_t refClkFreq = 0U;
    uint8_t clkSel      = 0U;

    if (ctrl < kCLOCK_ArmPllCtrl)
    {
        clkSel = (CCM_ANALOG_TUPLE_REG(CCM_ANALOG, ctrl) & CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_REF_CLK_SEL_MASK);
    }
    else
    {
        clkSel = (CCM_ANALOG_TUPLE_REG(CCM_ANALOG, ctrl) & CCM_ANALOG_SYS_PLL1_GEN_CTRL_PLL_REF_CLK_SEL_MASK);
    }

    switch (clkSel)
    {
        case kANALOG_PllRefOsc24M:
            refClkFreq = OSC24M_CLK_FREQ;
            break;

        case kANALOG_PllPadClk:
            /* The value of PAD CLK need user to define according to the actual condition. */
            refClkFreq = CLKPAD_FREQ;
            break;

        default:
            break;
    }

    return refClkFreq;
}

/*!
 * brief Gets PLL clock frequency.
 *
 * param type fractional pll type.

 * return  Clock frequency
 */
uint32_t CLOCK_GetPllFreq(clock_pll_ctrl_t pll)
{
    uint32_t pllFreq       = 0U;
    uint32_t pllRefFreq    = 0U;
    bool intergerPllBypass = false;
    bool fracPllBypass     = false;

    pllRefFreq = CLOCK_GetPllRefClkFreq(pll);

    switch (pll)
    {
        /* Integer PLL frequency */
        case kCLOCK_ArmPllCtrl:
            intergerPllBypass = CLOCK_IsPllBypassed(CCM_ANALOG, kCLOCK_ArmPllPwrBypassCtrl);
            break;
        case kCLOCK_SystemPll1Ctrl:
            intergerPllBypass = CLOCK_IsPllBypassed(CCM_ANALOG, kCLOCK_SysPll1InternalPll1BypassCtrl);
            break;
        case kCLOCK_SystemPll2Ctrl:
            intergerPllBypass = CLOCK_IsPllBypassed(CCM_ANALOG, kCLOCK_SysPll2InternalPll1BypassCtrl);
            break;
        case kCLOCK_SystemPll3Ctrl:
            intergerPllBypass = CLOCK_IsPllBypassed(CCM_ANALOG, kCLOCK_SysPll3InternalPll1BypassCtrl);
            break;
        /* Fractional PLL frequency */
        case kCLOCK_AudioPll1Ctrl:
            fracPllBypass = CLOCK_IsPllBypassed(CCM_ANALOG, kCLOCK_AudioPll1BypassCtrl);
            break;
        case kCLOCK_AudioPll2Ctrl:
            fracPllBypass = CLOCK_IsPllBypassed(CCM_ANALOG, kCLOCK_AudioPll2BypassCtrl);
            break;
        case kCLOCK_VideoPll1Ctrl:
            fracPllBypass = CLOCK_IsPllBypassed(CCM_ANALOG, kCLOCK_VideoPll1BypassCtrl);
            break;
        case kCLOCK_DramPllCtrl:
            fracPllBypass = CLOCK_IsPllBypassed(CCM_ANALOG, kCLOCK_DramPllInternalPll1BypassCtrl);
            break;
        default:
            break;
    }
    if (pll < kCLOCK_ArmPllCtrl)
    {
        if (fracPllBypass)
        {
            pllFreq = pllRefFreq;
        }
        else
        {
            pllFreq = CLOCK_GetFracPllFreq(CCM_ANALOG, pll, pllRefFreq);
        }
    }
    else
    {
        if (intergerPllBypass)
        {
            /* if PLL is bypass, return reference clock directly */
            pllFreq = pllRefFreq;
        }
        else
        {
            pllFreq = CLOCK_GetIntegerPllFreq(CCM_ANALOG, pll, pllRefFreq, false);
        }
    }

    return pllFreq;
}

/*
 * Fout = Fin/p * (m + k / 65536) / (1 << s)
 * Foutnew = Fout * ( 1 + ppb / pow(10, 9))
 * Foutnew = Fin/p * (m + knew / 65536) / (1 << s)
 * knew = 65536 * ((Foutnew * (1 << s) * p / Fin) - m)
 * knew = 65536 * (( Fout * ( 1 + ppb / pow(10, 9)) * (1 << s) * p / Fin) - m)
 */

static inline void CLOCK_getFracPllConfig(CCM_ANALOG_Type *base,
                                clock_pll_ctrl_t pll,
                                ccm_analog_frac_pll_config_t *config)
{
    uint32_t fracCfg1 = CCM_ANALOG_TUPLE_REG_OFF(base, pll, FracPLL_FDIV_CTL0_Offset);
    uint32_t fracCfg2 = CCM_ANALOG_TUPLE_REG_OFF(base, pll, FracPLL_FDIV_CTL1_Offset);

    uint32_t m = CCM_BIT_FIELD_EXTRACTION(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV_MASK,
                                                CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV_SHIFT);
    uint8_t p = CCM_BIT_FIELD_EXTRACTION(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV_MASK,
                                              CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV_SHIFT);
    uint8_t s = CCM_BIT_FIELD_EXTRACTION(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV_MASK,
                                               CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV_SHIFT);
    int32_t k = CCM_BIT_FIELD_EXTRACTION(fracCfg2, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_MASK,
                                            CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_SHIFT);

    uint8_t clkSel = (CCM_ANALOG_TUPLE_REG(CCM_ANALOG, pll) & CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_REF_CLK_SEL_MASK);

    config->refSel = clkSel;
    config->mainDiv = m;
    config->dsm = k;
    config->preDiv = p;
    config->postDiv = s;
}

uint32_t CLOCK_AdjustPllFreq(clock_pll_ctrl_t pll, int ppb)
{
    /* 1 Extract current pll parameters */
    CCM_ANALOG_Type *base = CCM_ANALOG;
    uint32_t fracCfg1 = CCM_ANALOG_TUPLE_REG_OFF(base, pll, FracPLL_FDIV_CTL0_Offset);
    uint32_t fracCfg2 = CCM_ANALOG_TUPLE_REG_OFF(base, pll, FracPLL_FDIV_CTL1_Offset);

    uint32_t m = CCM_BIT_FIELD_EXTRACTION(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV_MASK,
                                                CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV_SHIFT);
    uint8_t p = CCM_BIT_FIELD_EXTRACTION(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV_MASK,
                                              CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV_SHIFT);
    uint8_t s = CCM_BIT_FIELD_EXTRACTION(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV_MASK,
                                               CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV_SHIFT);
    int16_t k = CCM_BIT_FIELD_EXTRACTION(fracCfg2, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_MASK,
                                            CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_SHIFT);

    /* 2. Get input frequency */
    uint32_t Fin = CLOCK_GetPllRefClkFreq(pll);

    /* 3. Get the actual PLL frequency */
    uint32_t Fout = CLOCK_ComputeFracPllFreq(Fin, m, p, s, k);

    /* 4. Compute desired frequency */
    float Foutnew = (float) Fout * (1 + ppb / 1000000000.0);

    printlk(LK_VERBOSE, "Applying %d ppb, set to from %u %f \n", ppb, Fout, Foutnew);
    /* 5. Get the new k */
    int32_t _knew = 65536 * ((Foutnew * (1 << s) * p / Fin) - m);
    int16_t knew = _knew;

    if (_knew < INT16_MIN)
        knew = INT16_MIN;
    else if (_knew > INT16_MAX)
        knew = INT16_MAX;

    printlk(LK_VERBOSE, "Setting k from %hd to %hd\n", k, knew);

    /* 6. Apply new k if safe to do */
    CCM_ANALOG_TUPLE_REG_OFF(base, pll, FracPLL_FDIV_CTL1_Offset) =
        (fracCfg2 & (~(CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_MASK))) |
        CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM(knew);

    uint32_t newF = CLOCK_ComputeFracPllFreq(Fin, m, p, s, knew);
    printlk(LK_VERBOSE, "New Frequency computed to %u\n", newF);
    return newF;
}

/*!
 * brief Initializes the ANALOG ARM PLL.
 *
 * param config Pointer to the configuration structure(see ref ccm_analog_integer_pll_config_t enumeration).
 *
 * note This function can't detect whether the Arm PLL has been enabled and
 * used by some IPs.
 */
void CLOCK_InitArmPll(const ccm_analog_integer_pll_config_t *config)
{
    assert(config);

    /* Integer PLL configuration */
    CLOCK_InitIntegerPll(CCM_ANALOG, config, kCLOCK_ArmPllCtrl);
    /* Disable PLL bypass */
    CLOCK_SetPllBypass(CCM_ANALOG, kCLOCK_ArmPllPwrBypassCtrl, false);
    /* Enable and power up PLL clock. */
    CLOCK_EnableAnalogClock(CCM_ANALOG, kCLOCK_ArmPllClke);

    /* Wait for PLL to be locked. */
    while (!CLOCK_IsPllLocked(CCM_ANALOG, kCLOCK_ArmPllCtrl))
    {
    }
}

/*!
 * brief De-initialize the ARM PLL.
 */
void CLOCK_DeinitArmPll(void)
{
    CLOCK_PowerDownPll(CCM_ANALOG, kCLOCK_ArmPllCtrl);
}

#define PLL_1443X_RATE(_rate, _m, _p, _s, _k)       \
    {                       \
        .rate   =   (_rate),        \
        .mdiv   =   (_m),           \
        .pdiv   =   (_p),           \
        .sdiv   =   (_s),           \
        .kdiv   =   (_k),           \
    }

/* NOTE: Rate table should be kept sorted in descending order. */
struct imx_int_pll_rate_table {
    unsigned int rate;
    unsigned int pdiv;
    unsigned int mdiv;
    unsigned int sdiv;
    unsigned int kdiv;
};

void CLOCK_InitFracPllx(clock_pll_ctrl_t pll_id,
                            const ccm_analog_frac_pll_config_t *config)

{
    CCM_ANALOG_Type *base = CCM_ANALOG;
    ccm_analog_frac_pll_config_t curr_cfg;
    clock_pll_bypass_ctrl_t bypass_id;
    clock_pll_clke_t enable_id;
    const char *pll_name;

    switch (pll_id) {
    case kCLOCK_AudioPll2Ctrl:
        bypass_id = kCLOCK_AudioPll2BypassCtrl;
        enable_id = kCLOCK_AudioPll2Clke;
        pll_name = "AudioPll2";
        break;
    case kCLOCK_AudioPll1Ctrl:
        bypass_id = kCLOCK_AudioPll1BypassCtrl;
        enable_id = kCLOCK_AudioPll1Clke;
        pll_name = "AudioPll1";
        break;
    case kCLOCK_VideoPll1Ctrl:
        bypass_id = kCLOCK_VideoPll1BypassCtrl;
        enable_id = kCLOCK_VideoPll1Clke;
        pll_name = "VideoPll1";
        break;
    default:
        printlk(LK_INFO, "Pll with %#x is not a fractional PLL\n", pll_id);
        return;
    }

    bool is_pll_locked = CLOCK_IsPllLocked(base, pll_id);

    if (is_pll_locked) {
        printlk(LK_INFO, "%s is on\n", pll_name);
        CLOCK_getFracPllConfig(CCM_ANALOG, pll_id, &curr_cfg);
        if ((config->refSel == curr_cfg.refSel)
                && (config->mainDiv == curr_cfg.mainDiv)
                && (config->preDiv == curr_cfg.preDiv)
                && (config->postDiv == curr_cfg.postDiv)) {

            printlk(LK_INFO, "%s hot config possible\n", pll_name);
            /* Adjust k and exit */
            uint32_t fracCfg2 = CCM_ANALOG_TUPLE_REG_OFF(base, pll_id, FracPLL_FDIV_CTL1_Offset);
            CCM_ANALOG_TUPLE_REG_OFF(base, pll_id, FracPLL_FDIV_CTL1_Offset) =
                (fracCfg2 & (~(CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_MASK))) |
                    CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM(config->dsm);

            /* Disable PLL bypass if needed */
            if (CLOCK_IsPllBypassed(base, pll_id))
                CLOCK_SetPllBypass(CCM_ANALOG, bypass_id, false);
            return;
        }
    }

    printlk(LK_INFO, "Complete %s reprogramming\n", pll_name);

    /* Enable PLL bypass */
    CLOCK_SetPllBypass(CCM_ANALOG, bypass_id, true);

    /* Fractional pll configuration */
    CLOCK_InitFracPll(CCM_ANALOG, config, pll_id);
    /* Enable and power up PLL clock. */
    CLOCK_EnableAnalogClock(CCM_ANALOG, enable_id);

    /* Wait for PLL to be locked. */
    while (!CLOCK_IsPllLocked(CCM_ANALOG, pll_id))
    {
    }

    /* Disable PLL bypass */
    CLOCK_SetPllBypass(CCM_ANALOG, bypass_id, false);
}


/* Set AUDIO_PLL1 to 786432000 Hz (48kHz * 16384) */
void CLOCK_SetupAudioPll1(unsigned long rate)
{
    struct imx_int_pll_rate_table imx8mm_audiopll_tbl =
            PLL_1443X_RATE(786432000U, 655, 5, 2, 23593);

    ccm_analog_frac_pll_config_t audioPll1Config = {
        .refSel = kANALOG_PllRefOsc24M, /*!< PLL reference OSC24M */
        .mainDiv = imx8mm_audiopll_tbl.mdiv,
        .dsm = imx8mm_audiopll_tbl.kdiv,
        .preDiv = imx8mm_audiopll_tbl.pdiv,
        .postDiv = imx8mm_audiopll_tbl.sdiv,
    };

    assert(imx8mm_audiopll_tbl.rate == rate);

    CLOCK_InitAudioPll1(&audioPll1Config);
}

/*!
 * brief Initializes the ANALOG AUDIO PLL1.
 *
 * param config Pointer to the configuration structure(see ref ccm_analog_frac_pll_config_t enumeration).
 *
 * note This function can't detect whether the AUDIO PLL has been enabled and
 * used by some IPs.
 */
void CLOCK_InitAudioPll1(const ccm_analog_frac_pll_config_t *config)
{
    assert(config);

    CLOCK_InitFracPllx(kCLOCK_AudioPll1Ctrl, config);
}

/*!
 * brief De-initialize the Audio PLL1.
 */
void CLOCK_DeinitAudioPll1(void)
{
    CLOCK_PowerDownPll(CCM_ANALOG, kCLOCK_AudioPll1Ctrl);
}

/* Set AUDIO_PLL2 to 722534400Hz (44.1kHz * 16384) */
void CLOCK_SetupAudioPll2(unsigned long rate)
{
    struct imx_int_pll_rate_table imx8mm_audiopll_tbl =
            PLL_1443X_RATE(722534400U, 301, 5, 1, 3670);

    ccm_analog_frac_pll_config_t audioPll2Config = {
        .refSel = kANALOG_PllRefOsc24M, /*!< PLL reference OSC24M */
        .mainDiv = imx8mm_audiopll_tbl.mdiv,
        .dsm = imx8mm_audiopll_tbl.kdiv,
        .preDiv = imx8mm_audiopll_tbl.pdiv,
        .postDiv = imx8mm_audiopll_tbl.sdiv,
    };

    assert(imx8mm_audiopll_tbl.rate == rate);

    CLOCK_InitAudioPll2(&audioPll2Config);
}

/*!
 * brief Initializes the ANALOG AUDIO PLL2.
 *
 * param config Pointer to the configuration structure(see ref ccm_analog_frac_pll_config_t enumeration).
 *
 * note This function can't detect whether the AUDIO PLL has been enabled and
 * used by some IPs.
 */
void CLOCK_InitAudioPll2(const ccm_analog_frac_pll_config_t *config)
{
    assert(config);
    CLOCK_InitFracPllx(kCLOCK_AudioPll2Ctrl, config);
}

/*!
 * brief De-initialize the Audio PLL2.
 */
void CLOCK_DeinitAudioPll2(void)
{
    CLOCK_PowerDownPll(CCM_ANALOG, kCLOCK_AudioPll2Ctrl);
}

/*!
 * brief Initializes the ANALOG VIDEO PLL1.
 *
 * param config Pointer to the configuration structure(see ref ccm_analog_frac_pll_config_t enumeration).
 *
 */
void CLOCK_InitVideoPll1(const ccm_analog_frac_pll_config_t *config)
{
    assert(config);

    CLOCK_InitFracPllx(kCLOCK_VideoPll1Ctrl, config);
}

/*!
 * brief De-initialize the Video PLL1.
 */
void CLOCK_DeinitVideoPll1(void)
{
    CLOCK_PowerDownPll(CCM_ANALOG, kCLOCK_VideoPll1Ctrl);
}

/*!
 * brief Initializes the ANALOG SYS PLL1.
 *
 * param config Pointer to the configuration structure(see ref ccm_analog_integer_pll_config_t enumeration).
 *
 * note This function can't detect whether the SYS PLL has been enabled and
 * used by some IPs.
 */
void CLOCK_InitSysPll1(const ccm_analog_integer_pll_config_t *config)
{
    assert(config);

    /* Integer PLL configuration */
    CLOCK_InitIntegerPll(CCM_ANALOG, config, kCLOCK_SystemPll1Ctrl);
    /* Disable PLL bypass */
    CLOCK_SetPllBypass(CCM_ANALOG, kCLOCK_SysPll1InternalPll1BypassCtrl, false);
    /* Enable and power up PLL clock. */
    CLOCK_EnableAnalogClock(CCM_ANALOG, kCLOCK_SystemPll1Clke);

    /* Wait for PLL to be locked. */
    while (!CLOCK_IsPllLocked(CCM_ANALOG, kCLOCK_SystemPll1Ctrl))
    {
    }
}

/*!
 * brief De-initialize the System PLL1.
 */
void CLOCK_DeinitSysPll1(void)
{
    CLOCK_PowerDownPll(CCM_ANALOG, kCLOCK_SystemPll1Ctrl);
}

/*!
 * brief Initializes the ANALOG SYS PLL2.
 *
 * param config Pointer to the configuration structure(see ref ccm_analog_integer_pll_config_t enumeration).
 *
 * note This function can't detect whether the SYS PLL has been enabled and
 * used by some IPs.
 */
void CLOCK_InitSysPll2(const ccm_analog_integer_pll_config_t *config)
{
    assert(config);

    /* Integer PLL configuration */
    CLOCK_InitIntegerPll(CCM_ANALOG, config, kCLOCK_SystemPll2Ctrl);
    /* Disable PLL bypass */
    CLOCK_SetPllBypass(CCM_ANALOG, kCLOCK_SysPll2InternalPll1BypassCtrl, false);
    /* Enable and power up PLL clock. */
    CLOCK_EnableAnalogClock(CCM_ANALOG, kCLOCK_SystemPll2Clke);

    /* Wait for PLL to be locked. */
    while (!CLOCK_IsPllLocked(CCM_ANALOG, kCLOCK_SystemPll2Ctrl))
    {
    }
}

/*!
 * brief De-initialize the System PLL2.
 */
void CLOCK_DeinitSysPll2(void)
{
    CLOCK_PowerDownPll(CCM_ANALOG, kCLOCK_SystemPll2Ctrl);
}

/*!
 * brief Initializes the ANALOG SYS PLL3.
 *
 * param config Pointer to the configuration structure(see ref ccm_analog_integer_pll_config_t enumeration).
 *
 * note This function can't detect whether the SYS PLL has been enabled and
 * used by some IPs.
 */
void CLOCK_InitSysPll3(const ccm_analog_integer_pll_config_t *config)
{
    assert(config);

    /* Integer PLL configuration */
    CLOCK_InitIntegerPll(CCM_ANALOG, config, kCLOCK_SystemPll3Ctrl);
    /* Disable PLL bypass */
    CLOCK_SetPllBypass(CCM_ANALOG, kCLOCK_SysPll3InternalPll1BypassCtrl, false);
    /* Enable and power up PLL clock. */
    CLOCK_EnableAnalogClock(CCM_ANALOG, kCLOCK_SystemPll3Clke);

    /* Wait for PLL to be locked. */
    while (!CLOCK_IsPllLocked(CCM_ANALOG, kCLOCK_SystemPll3Ctrl))
    {
    }
}

/*!
 * brief De-initialize the System PLL3.
 */
void CLOCK_DeinitSysPll3(void)
{
    CLOCK_PowerDownPll(CCM_ANALOG, kCLOCK_SystemPll3Ctrl);
}

/*!
 * brief Initializes the ANALOG Fractional PLL.
 *
 * param base CCM ANALOG base address.
 * param config Pointer to the configuration structure(see ref ccm_analog_frac_pll_config_t enumeration).
 * param type fractional pll type.
 *
 */
void CLOCK_InitFracPll(CCM_ANALOG_Type *base, const ccm_analog_frac_pll_config_t *config, clock_pll_ctrl_t type)
{
    assert(config);
    assert((config->mainDiv >= 64U) && (config->mainDiv <= 1023U));
    assert((config->preDiv >= 1U) && (config->preDiv <= 63U));
    assert(config->postDiv <= 6U);

    assert(type < kCLOCK_ArmPllCtrl);

    uint32_t fracCfg0 = CCM_ANALOG_TUPLE_REG_OFF(base, type, FracPLL_GNRL_CTL_Offset) &
                        ~((uint32_t)1 << CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_RST_SHIFT);
    uint32_t fracCfg1 = CCM_ANALOG_TUPLE_REG_OFF(base, type, FracPLL_FDIV_CTL0_Offset);
    uint32_t fracCfg2 = CCM_ANALOG_TUPLE_REG_OFF(base, type, FracPLL_FDIV_CTL1_Offset);

    /* power down the fractional PLL first */
    CCM_ANALOG_TUPLE_REG_OFF(base, type, FracPLL_GNRL_CTL_Offset) = fracCfg0;

    CCM_ANALOG_TUPLE_REG_OFF(base, type, FracPLL_FDIV_CTL0_Offset) =
        (fracCfg1 &
         (~(CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV_MASK | CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV_MASK |
            CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV_MASK))) |
        CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV(config->mainDiv) |
        CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV(config->preDiv) |
        CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV(config->postDiv);

    CCM_ANALOG_TUPLE_REG_OFF(base, type, FracPLL_FDIV_CTL1_Offset) =
        (fracCfg2 & (~(CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_MASK))) |
        CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM(config->dsm);

    /* power up the fractional pll */
    CCM_ANALOG_TUPLE_REG_OFF(base, type, FracPLL_GNRL_CTL_Offset) |= CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_RST_MASK;
}

/*!
 * brief Gets the ANALOG Fractional PLL clock frequency.
 *
 * param base CCM_ANALOG base pointer.
 * param type fractional pll type.
 * param fractional pll reference clock frequency
 *
 * return  Clock frequency
 */
uint32_t CLOCK_GetFracPllFreq(CCM_ANALOG_Type *base, clock_pll_ctrl_t type, uint32_t refClkFreq)
{
    assert(type < kCLOCK_ArmPllCtrl);

    uint32_t fracCfg1 = CCM_ANALOG_TUPLE_REG_OFF(base, type, FracPLL_FDIV_CTL0_Offset);
    uint32_t fracCfg2 = CCM_ANALOG_TUPLE_REG_OFF(base, type, FracPLL_FDIV_CTL1_Offset);
    uint64_t fracClk  = 0U;

    uint32_t mainDiv = CCM_BIT_FIELD_EXTRACTION(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV_MASK,
                                                CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV_SHIFT);
    uint8_t preDiv   = CCM_BIT_FIELD_EXTRACTION(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV_MASK,
                                              CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV_SHIFT);
    uint8_t postDiv  = CCM_BIT_FIELD_EXTRACTION(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV_MASK,
                                               CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV_SHIFT);
    uint32_t dsm     = CCM_BIT_FIELD_EXTRACTION(fracCfg2, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_MASK,
                                            CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_SHIFT);

    fracClk = (uint64_t)refClkFreq * (mainDiv * 65536 + dsm) / (65536 * preDiv * (1 << postDiv));

    return fracClk;
}

uint32_t CLOCK_ComputeFracPllFreq(uint32_t refClkFreq, uint32_t m, uint8_t p, uint8_t s, int16_t k)
{

    uint64_t fracClk = 0U;
    fracClk = (uint64_t)refClkFreq * (m * 65536 + k) / (65536 * p * (1 << s));

    return fracClk;
}


/*!
 * brief Initializes the ANALOG Integer PLL.
 *
 * param base CCM ANALOG base address
 * param config Pointer to the configuration structure(see ref ccm_analog_integer_pll_config_t enumeration).
 * param type integer pll type
 *
 */
void CLOCK_InitIntegerPll(CCM_ANALOG_Type *base, const ccm_analog_integer_pll_config_t *config, clock_pll_ctrl_t type)
{
    assert(config);
    assert((config->mainDiv >= 64U) && (config->mainDiv <= 1023U));
    assert((config->preDiv >= 1U) && (config->preDiv <= 63U));
    assert(config->postDiv <= 6U);

    assert(type >= kCLOCK_SystemPll1Ctrl);

    uint32_t integerCfg0 = CCM_ANALOG_TUPLE_REG_OFF(base, type, IntegerPLL_GNRL_CTL_Offset) &
                           ~((uint32_t)1 << CCM_ANALOG_SYS_PLL1_GEN_CTRL_PLL_RST_SHIFT);
    uint32_t integerCfg1 = CCM_ANALOG_TUPLE_REG_OFF(base, type, IntegerPLL_DIV_CTL_Offset);

    /* power down the Integer PLL first */
    CCM_ANALOG_TUPLE_REG_OFF(base, type, IntegerPLL_GNRL_CTL_Offset) = integerCfg0;

    /* pll mux configuration */
    CCM_ANALOG_TUPLE_REG_OFF(base, type, IntegerPLL_GNRL_CTL_Offset) =
        (integerCfg0 & (~CCM_ANALOG_SYS_PLL1_GEN_CTRL_PLL_REF_CLK_SEL_MASK)) | config->refSel;

    /* divider configuration */
    CCM_ANALOG_TUPLE_REG_OFF(base, type, IntegerPLL_DIV_CTL_Offset) =
        (integerCfg1 &
         (~(CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_MAIN_DIV_MASK | CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_PRE_DIV_MASK |
            CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_POST_DIV_MASK))) |
        CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_MAIN_DIV(config->mainDiv) |
        CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_PRE_DIV(config->preDiv) |
        CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_POST_DIV(config->postDiv);

    /* power up the Integer PLL */
    CCM_ANALOG_TUPLE_REG_OFF(base, type, IntegerPLL_GNRL_CTL_Offset) |= CCM_ANALOG_SYS_PLL1_GEN_CTRL_PLL_RST_MASK;
}

/*!
 * brief Get the ANALOG Integer PLL clock frequency.
 *
 * param base CCM ANALOG base address.
 * param type integer pll type
 * param pll1Bypass pll1 bypass flag
 *
 * return  Clock frequency
 */
uint32_t CLOCK_GetIntegerPllFreq(CCM_ANALOG_Type *base, clock_pll_ctrl_t type, uint32_t refClkFreq, bool pll1Bypass)
{
    assert(type >= kCLOCK_SystemPll1Ctrl);

    uint32_t integerCfg1 = CCM_ANALOG_TUPLE_REG_OFF(base, type, IntegerPLL_DIV_CTL_Offset);
    uint64_t pllOutClock = 0U;

    uint32_t mainDiv = CCM_BIT_FIELD_EXTRACTION(integerCfg1, CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_MAIN_DIV_MASK,
                                                CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_MAIN_DIV_SHIFT);
    uint8_t preDiv   = CCM_BIT_FIELD_EXTRACTION(integerCfg1, CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_PRE_DIV_MASK,
                                              CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_PRE_DIV_SHIFT);
    uint8_t postDiv  = CCM_BIT_FIELD_EXTRACTION(integerCfg1, CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_POST_DIV_MASK,
                                               CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_POST_DIV_SHIFT);

    if (pll1Bypass)
    {
        pllOutClock = refClkFreq;
    }

    else
    {
        pllOutClock = (uint64_t)refClkFreq * mainDiv / (((uint64_t)(1U) << postDiv) * preDiv);
    }

    return pllOutClock;
}

/*!
 * brief Set root clock divider
 * Note: The PRE and POST dividers in this function are the actually divider, software will map it to register value
 *
 * param ccmRootClk Root control (see ref clock_root_control_t enumeration)
 * param pre Pre divider value (1-8)
 * param post Post divider value (1-64)
 */
void CLOCK_SetRootDivider(clock_root_control_t ccmRootClk, uint32_t pre, uint32_t post)
{
    assert((pre <= 8U) && (pre != 0U));
    assert((post <= 64U) && (post != 0U));

    CCM_REG(ccmRootClk) = (CCM_REG(ccmRootClk) & (~(CCM_TARGET_ROOT_PRE_PODF_MASK | CCM_TARGET_ROOT_POST_PODF_MASK))) |
                          CCM_TARGET_ROOT_PRE_PODF(pre - 1U) | CCM_TARGET_ROOT_POST_PODF(post - 1U);
}

/*!
 * brief Update clock root in one step, for dynamical clock switching
 * Note: The PRE and POST dividers in this function are the actually divider, software will map it to register value
 *
 * param ccmRootClk Root control (see ref clock_root_control_t enumeration)
 * param root mux value (see ref _ccm_rootmux_xxx enumeration)
 * param pre Pre divider value (0-7, divider=n+1)
 * param post Post divider value (0-63, divider=n+1)
 */
void CLOCK_UpdateRoot(clock_root_control_t ccmRootClk, uint32_t mux, uint32_t pre, uint32_t post)
{
    assert((pre <= 8U) && (pre != 0U));
    assert((post <= 64U) && (post != 0U));

    CCM_REG(ccmRootClk) =
        (CCM_REG(ccmRootClk) &
         (~(CCM_TARGET_ROOT_MUX_MASK | CCM_TARGET_ROOT_PRE_PODF_MASK | CCM_TARGET_ROOT_POST_PODF_MASK))) |
        CCM_TARGET_ROOT_MUX(mux) | CCM_TARGET_ROOT_PRE_PODF(pre - 1U) | CCM_TARGET_ROOT_POST_PODF(post - 1U);
}

/*!
 * brief Enable CCGR clock gate and root clock gate for each module
 * User should set specific gate for each module according to the description
 * of the table of system clocks, gating and override in CCM chapter of
 * reference manual. Take care of that one module may need to set more than
 * one clock gate.
 *
 * param ccmGate Gate control for each module (see ref clock_ip_name_t enumeration).
 */
void CLOCK_EnableClock(clock_ip_name_t ccmGate)
{
    uintptr_t ccgr = CCM_TUPLE_CCGR(ccmGate);

    CCM_REG_SET(ccgr) = kCLOCK_ClockNeededAll;
#if !(defined(NOT_CONFIG_CLK_ROOT) && NOT_CONFIG_CLK_ROOT)
    uintptr_t rootClk = CCM_TUPLE_ROOT(ccmGate);
    /* if root clock is 0xFFFFU, then skip enable root clock */
    if (rootClk != 0xFFFFU)
    {
        CCM_REG_SET(rootClk) = CCM_TARGET_ROOT_SET_ENABLE_MASK;
    }
#endif
}

/*!
 * brief Disable CCGR clock gate for the each module
 * User should set specific gate for each module according to the description
 * of the table of system clocks, gating and override in CCM chapter of
 * reference manual. Take care of that one module may need to set more than
 * one clock gate.
 *
 * param ccmGate Gate control for each module (see ref clock_ip_name_t enumeration).
 */
void CLOCK_DisableClock(clock_ip_name_t ccmGate)
{
    uintptr_t ccgr = CCM_TUPLE_CCGR(ccmGate);

    CCM_REG(ccgr) = kCLOCK_ClockNotNeeded;
#if !(defined(NOT_CONFIG_CLK_ROOT) && NOT_CONFIG_CLK_ROOT)
    uintptr_t rootClk = CCM_TUPLE_ROOT(ccmGate);
    /* if root clock is 0xFFFFU, then skip disable root clock */
    if (rootClk != 0xFFFFU)
    {
        CCM_REG_CLR(rootClk) = CCM_TARGET_ROOT_CLR_ENABLE_MASK;
    }
#endif
}
