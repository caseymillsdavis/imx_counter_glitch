/* This is a test of the general purpose timer on the RT10xx family of
 * processors. We observe that the GPT counter can glitch backwards or forwards
 * under certain clock configurations.
 *
 * This has been tested by loading and running in RAM via GDB.
 *
 * The program will configure different AHB speeds and then take a bunch of
 * GPT1->CNT readings and check for any glitching. The array
 *
 * Refer to Figure 14-2 Clock Tree in the RT1060 ref manual.
 * We are sourcing the AHB and Arm core clock from PLL1 (also called USB PLL)
 * through a series of muxes and dividers. We are testing GPT1 sourced from the
 * 32K. We are assuming a setup similar to EVK dev boards which have external
 * 24M and 32K oscillators.
 *
 * In GDB, we define a command for loading to RAM
 *   mon reset halt
 *   load
 *   set $pc = vector_table[1]
 *   set $sp = vector_table[0]
 */

#include "MIMXRT1062.h"
#include <stddef.h>

void *memset(void *ptr, int value, size_t num)
{
    unsigned char *p = (unsigned char *)ptr;
    while (num--)
    {
        *p++ = (unsigned char)value;
    }
    return ptr;
}

void init_gpt1(void)
{
    // Enable GPT1 clock
    CCM->CCGR1 |= CCM_CCGR1_CG10_MASK;

    GPT1->CR = GPT_CR_SWR_MASK;
    GPT1->CR =
        GPT_CR_CLKSRC(4) |  // 32K clock
        GPT_CR_ENMOD_MASK |
        GPT_CR_FRR_MASK;
    GPT1->PR = 0;
    GPT1->CR |= GPT_CR_EN_MASK;
}

void deinit_gpt1(void)
{
    CCM->CCGR1 |= CCM_CCGR1_CG10_MASK;
    GPT1->CR = GPT_CR_SWR_MASK;
    CCM->CCGR1 &= ~CCM_CCGR1_CG10_MASK;
}

/* The ARM PLL is derived from the 24M.
 * Output frequency is 24*ARM_DIV_SELECT/2/ARM_PODF/AHB_PODF
 *
 * Could augment this to include AHB_PODF as an argument. */
#define AHB_PODF 1
void init_clocks(unsigned arm_div_select, unsigned arm_podf)
{
    /* set periph_clk2 to osc_clk */
    CCM->CBCMR = (CCM->CBCMR & ~CCM_CBCMR_PERIPH_CLK2_SEL_MASK) | CCM_CBCMR_PERIPH_CLK2_SEL(1U);

    /* periph_clk_sel to periph_clk2 while we tweak things */
    CCM->CBCDR = (CCM->CBCDR & ~CCM_CBCDR_PERIPH_CLK_SEL_MASK) | CCM_CBCDR_PERIPH_CLK_SEL(1);

    /* configure ARM PLL */
    CCM_ANALOG->PLL_ARM_CLR = CCM_ANALOG_PLL_ARM_POWERDOWN_MASK;
    CCM_ANALOG->PLL_ARM_SET = CCM_ANALOG_PLL_ARM_ENABLE_MASK | CCM_ANALOG_PLL_ARM_BYPASS_MASK;
    CCM_ANALOG->PLL_ARM_CLR = CCM_ANALOG_PLL_ARM_DIV_SELECT_MASK;
    CCM_ANALOG->PLL_ARM_SET = CCM_ANALOG_PLL_ARM_DIV_SELECT(arm_div_select);
    while ((CCM_ANALOG->PLL_ARM & CCM_ANALOG_PLL_ARM_LOCK_MASK) == 0);
    CCM_ANALOG->PLL_ARM_CLR = CCM_ANALOG_PLL_ARM_BYPASS_MASK;

    /* set ARM PLL divider */
    CCM->CACRR = (CCM->CACRR & ~CCM_CACRR_ARM_PODF_MASK) | CCM_CACRR_ARM_PODF(arm_podf-1);
    while (CCM->CDHIPR & CCM_CDHIPR_ARM_PODF_BUSY_MASK);

    /* set AHB divider; reproduces with 0 or 1 (div 1 or 2). */
    CCM->CBCDR = (CCM->CBCDR & ~CCM_CBCDR_AHB_PODF_MASK) | CCM_CBCDR_AHB_PODF(AHB_PODF-1);
    while (CCM->CDHIPR & CCM_CDHIPR_AHB_PODF_BUSY_MASK);


    /* set IPG divider to 1 */
    CCM->CBCDR = (CCM->CBCDR & ~CCM_CBCDR_IPG_PODF_MASK) | CCM_CBCDR_IPG_PODF(0);

    /* Handle setting up the perclk in main prior to initializing the GPT. */
    //CCM->CSCMR1 = (CCM->CSCMR1 & ~CCM_CSCMR1_PERCLK_PODF_MASK) | CCM_CSCMR1_PERCLK_PODF(0);
    //CCM->CSCMR1 = (CCM->CSCMR1 & ~CCM_CSCMR1_PERCLK_CLK_SEL_MASK) | CCM_CSCMR1_PERCLK_CLK_SEL(1);

    /* now switch periph_clk back to pre_periph_clk */
    CCM->CBCDR = (CCM->CBCDR & ~CCM_CBCDR_PERIPH_CLK_SEL_MASK) | CCM_CBCDR_PERIPH_CLK_SEL(0);
}

void disable_rtwdog(void)
{
    if ((RTWDOG->CS & RTWDOG_CS_EN_MASK) || ((RTWDOG->CS & RTWDOG_CS_CMD32EN_MASK) == 0))
    {
        if (RTWDOG->CS & RTWDOG_CS_CMD32EN_MASK)
        {
            // unlock using 32-bit update sequence
            RTWDOG->CNT = RTWDOG_UPDATE_KEY;
        }
        else
        {
            // unlock using 16-bit update sequence
            RTWDOG->CNT = RTWDOG_UPDATE_KEY & 0xffff;
            RTWDOG->CNT = (RTWDOG_UPDATE_KEY >> 16) & 0xffff;
        }
        RTWDOG->TOVAL = 0xFFFF;
        RTWDOG->CS = RTWDOG_CS_UPDATE_MASK | RTWDOG_CS_CMD32EN_MASK; // clears EN bit
    }
}

void enable_clock_outs(void)
{
    /* This will put the AHB clock out on CLKO1 and the osc_clk out on CLKO2,
     * both divided by 8. These are at TP27/28 on the IMXRT1050-EVKB. */
    CCM->CCGR4 |= CCM_CCGR4_CG1_MASK;  // Enable IOMUXC clock

    /* IOMUXC_GPIO_SD_B0_04_CCM_CLKO1 */
    *((volatile uint32_t *)0x401F81CC) = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6);
    *((volatile uint32_t *)0x401F83BC) = 0x10B0;
    /* IOMUXC_GPIO_SD_B0_05_CCM_CLKO2 */
    *((volatile uint32_t *)0x401F81D0) = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6);
    *((volatile uint32_t *)0x401F83C0) = 0x10B0;

    /* Configure clock outs. */
    CCM->CCOSR = 0x01ee00fb;
}

typedef struct
{
    uint32_t prev;
    uint32_t next;
    unsigned sample_idx;
} pair_t;

#define NUM_PAIRS 128
#define NUM_SAMPLES 1024 
pair_t gpt_pairs[NUM_PAIRS];
uint32_t gpt_pair_idx;
uint32_t gpt_samples[NUM_SAMPLES];

typedef struct
{
    unsigned arm_podf;
    unsigned arm_div_select;
    unsigned freq_mhz;
    unsigned glitch_count;
} test_point_t;

#define ARM_PODF_START     8
#define ARM_PODF_END       2
#define ARM_DIV_SEL_START 69
#define ARM_DIV_SEL_END   85

/* Different boards show different glitches and some take more time to
 * reproduce than others. Here we specify the number of seconds to spend in
 * each test point sampling the GPT. */
#define SECONDS_PER_TEST_POINT 5

#define NUM_TEST_PTS ((ARM_PODF_START - ARM_PODF_END)*(ARM_DIV_SEL_END - ARM_DIV_SEL_START))
test_point_t test_points[NUM_TEST_PTS];

void gpt_test(void)
{
    unsigned test_idx = 0;

    /* Zero init. We are assuming there is no cstartup. */
    memset(test_points, 0, sizeof(test_points));
    memset(gpt_pairs, 0, sizeof(gpt_pairs));
    memset(&gpt_pair_idx, 0, sizeof(gpt_pair_idx));

    for (unsigned arm_podf = ARM_PODF_START; arm_podf > ARM_PODF_END; arm_podf--)
    {
        for (unsigned arm_div_select = ARM_DIV_SEL_START; arm_div_select < ARM_DIV_SEL_END; arm_div_select++)
        {
            uint32_t start;
            test_point_t *tp = &test_points[test_idx++];
            tp->arm_podf = arm_podf;
            tp->arm_div_select = arm_div_select;
            tp->freq_mhz = 24*arm_div_select/2/arm_podf;

            init_clocks(arm_div_select, arm_podf);

            start = GPT1->CNT;
            while ((GPT1->CNT - start) < 32768*SECONDS_PER_TEST_POINT)
            {
                /* Take a bunch of samples */
                for (unsigned i = 0; i < NUM_SAMPLES/8; i++)
                {
                    gpt_samples[i*8+0] = GPT1->CNT;
                    gpt_samples[i*8+1] = GPT1->CNT;
                    gpt_samples[i*8+2] = GPT1->CNT;
                    gpt_samples[i*8+3] = GPT1->CNT;
                    gpt_samples[i*8+4] = GPT1->CNT;
                    gpt_samples[i*8+5] = GPT1->CNT;
                    gpt_samples[i*8+6] = GPT1->CNT;
                    gpt_samples[i*8+7] = GPT1->CNT;
                }

                /* See if consecutive reads every moved backwards. */
                uint32_t cnt_prev = gpt_samples[0];
                for (unsigned i = 1; i < NUM_SAMPLES; i++)
                {
                    uint32_t cnt_now = gpt_samples[i];

                    /* Detect backwards count */
                    if (cnt_now < cnt_prev)
                    {
                        tp->glitch_count += 1;

                        gpt_pairs[gpt_pair_idx].prev = cnt_prev;
                        gpt_pairs[gpt_pair_idx].next = cnt_now;
                        gpt_pairs[gpt_pair_idx].sample_idx = i;
                        gpt_pair_idx++;
                        if (gpt_pair_idx == NUM_PAIRS)
                        {
                            gpt_pair_idx = 0;
                        }
                    }
                    cnt_prev = cnt_now;
                }
            }
        }
    }
}


int main(void)
{
    /* Disable watchdogs. */
    WDOG1->WMCR &= ~WDOG_WMCR_PDE_MASK;
    WDOG2->WMCR &= ~WDOG_WMCR_PDE_MASK;
    disable_rtwdog();

    /* Make sure GPT isn't running when we change its clock */
    deinit_gpt1();

    /* Set the perclk to use osc_clk. */
    CCM->CSCMR1 = (CCM->CSCMR1 & ~CCM_CSCMR1_PERCLK_CLK_SEL_MASK) | CCM_CSCMR1_PERCLK_CLK_SEL(1);

    init_gpt1();

    /* Useful for verifying that clocks are configured as expected. */
    //enable_clock_outs();

    gpt_test();

    while(1);
}

__attribute__((naked))
void Default_Handler(void) { while (1); }
void Reset_Handler(void);

/* Vector table */
__attribute((aligned(4096)))
__attribute__((section(".vectors")))
void (* const vector_table[])(void) =
{
    (void (*)(void))(0x20208000),    // Initial stack pointer
    Reset_Handler,                   // Reset handler
    Default_Handler,                 // NMI handler
    Default_Handler,                 // HardFault handler
    Default_Handler,                 // MemManage handler
    Default_Handler,                 // BusFault handler
    Default_Handler,                 // UsageFault handler
    Default_Handler,                 // Reserved
    Default_Handler,                 // Reserved
    Default_Handler,                 // Reserved
    Default_Handler,                 // Reserved
    Default_Handler,                 // SVC handler
    Default_Handler,                 // DebugMon handler
    Default_Handler,                 // Reserved
    Default_Handler,                 // PendSV handler
    Default_Handler,                 // SysTick handler
};

__attribute__((naked))
void Reset_Handler(void)
{
    SCB->VTOR = (uint32_t)&vector_table;
    main();
}
