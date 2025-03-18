# RT10xx GPT Glitch Finder 

This repository contains a **bare-metal firmware** for the **NXP RT10xx EVK**,
running entirely from **OCRAM** with minimal dependencies. It initializes
a General Purpose Timer (**GPT1**) and detects **counter glitches** where the
counter does not increment by 1 as expected but instead can jump forwards or
backwards by as much as 4096.

The glitch is transient and subsequent reads of **GPT1->CNT** will read back
the expected value.

Different hardware shows various behavior, including the likelihood of glitches
and the CPU frequencies at which they occur.

## **Features**
- Runs **entirely in RAM** (OCRAM), no Flash required.
- Single source file, no cstartup.
- Barebones linker script included.
- Initializes **GPT1 (32KHz source)** in free-running mode.
- Sweeps over CPU frequencies and runs an N second test where GPT1->CNT is
  sampled.
- Collected samples are inspected for glitches.

## **Buidling**
`make`

## **Flashing**
OpenOCD can be used to flash an EVK board.
`./openocd -f interface/cmsis-dap.cfg -f target/imxrt.cfg`

You may need a udev rule to find the DAPLink device.

The imxrt.cfg file comes from
https://github.com/sysprogs/openocd/blob/master/tcl/target/imxrt.cfg.

## **Running**
Loading in GDB:

```
mon reset halt
load
set $pc = vector_table[1]
set $sp = vector_table[0]
continue
```

After all of the test points (various CPU speeds) are run, the binary will
`while(1);`. 

Results can be printed in GDB, i.e. the number of glitches which were
discovered in each test point:

```
(gdb) p test_points
$246 = {
  {arm_podf = 8, arm_div_select = 69, freq_mhz = 103, glitch_count = 271},
  {arm_podf = 8, arm_div_select = 70, freq_mhz = 105, glitch_count = 40},
  {arm_podf = 8, arm_div_select = 71, freq_mhz = 106, glitch_count = 384},
  {arm_podf = 8, arm_div_select = 72, freq_mhz = 108, glitch_count = 0},
  {arm_podf = 8, arm_div_select = 73, freq_mhz = 109, glitch_count = 208},
  {arm_podf = 8, arm_div_select = 74, freq_mhz = 111, glitch_count = 0},
  {arm_podf = 8, arm_div_select = 75, freq_mhz = 112, glitch_count = 262},
  {arm_podf = 8, arm_div_select = 76, freq_mhz = 114, glitch_count = 0},
  {arm_podf = 8, arm_div_select = 77, freq_mhz = 115, glitch_count = 323},
  {arm_podf = 8, arm_div_select = 78, freq_mhz = 117, glitch_count = 493},
  {arm_podf = 8, arm_div_select = 79, freq_mhz = 118, glitch_count = 296},
  {arm_podf = 8, arm_div_select = 80, freq_mhz = 120, glitch_count = 3892},
  {arm_podf = 8, arm_div_select = 81, freq_mhz = 121, glitch_count = 269}
```

You can also inspect the glitches themselves:
```
(gdb) p/x gpt_pairs
$230 = {
  {prev = 0xf2bff, next = 0xf2800, sample_idx = 0x3aa},
  {prev = 0x2b5bff, next = 0x2b5800, sample_idx = 0x22a},
  {prev = 0x351bff, next = 0x351800, sample_idx = 0x65},
  {prev = 0x386bff, next = 0x386800, sample_idx = 0xd8},
  {prev = 0x3bb3ff, next = 0x3bb000, sample_idx = 0x354},
```

---

## **Dependencies**
Relies on an NXP header MIMXRT1062.h that functions fine for testing on both
1050 and 1060 SOCs.

A few headers from the ARM CMSIS library are included.

**ARM GCC** (arm-none-eabi-gcc)


