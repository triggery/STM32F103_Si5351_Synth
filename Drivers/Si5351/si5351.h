/**************************************************************************/
/*!
    @file     Adafruit_SI5351.h
    @author   K. Townsend (Adafruit Industries)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2014, Adafruit Industries
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifndef _SI5351_H_
#define _SI5351_H_



#include "si5351_errors.h"
#include "si5351_asserts.h"
#include "stdbool.h"

extern I2C_HandleTypeDef hi2c1;

#define SI5351_ADDRESS            (0x60) // Assumes ADDR pin = low
#define SI5351_READBIT            (0x01)

#define USE_FULL_ASSERT			  (0x01)

#define SI5351_BUS_BASE_ADDR            0x60
#define SI5351_XTAL_FREQ                25000000
#define SI5351_PLL_FIXED                80000000000ULL
#define SI5351_FREQ_MULT                100ULL
#define SI5351_DEFAULT_CLK              1000000000ULL

#define SI5351_PLL_VCO_MIN              600000000
#define SI5351_PLL_VCO_MAX              900000000
#define SI5351_MULTISYNTH_MIN_FREQ      500000
#define SI5351_MULTISYNTH_DIVBY4_FREQ   150000000
#define SI5351_MULTISYNTH_MAX_FREQ      225000000
#define SI5351_MULTISYNTH_SHARE_MAX     100000000
#define SI5351_MULTISYNTH_SHARE_MIN     1024000
#define SI5351_MULTISYNTH67_MAX_FREQ    SI5351_MULTISYNTH_DIVBY4_FREQ
#define SI5351_CLKOUT_MIN_FREQ          4000
#define SI5351_CLKOUT_MAX_FREQ          SI5351_MULTISYNTH_MAX_FREQ
#define SI5351_CLKOUT67_MS_MIN          SI5351_PLL_VCO_MIN / SI5351_MULTISYNTH67_A_MAX
#define SI5351_CLKOUT67_MIN_FREQ        SI5351_CLKOUT67_MS_MIN / 128
#define SI5351_CLKOUT67_MAX_FREQ        SI5351_MULTISYNTH67_MAX_FREQ

#define SI5351_PLL_A_MIN                15
#define SI5351_PLL_A_MAX                90
#define SI5351_PLL_B_MAX                (SI5351_PLL_C_MAX-1)
#define SI5351_PLL_C_MAX                1048575
#define SI5351_MULTISYNTH_A_MIN         6
#define SI5351_MULTISYNTH_A_MAX         1800
#define SI5351_MULTISYNTH67_A_MAX       254
#define SI5351_MULTISYNTH_B_MAX         (SI5351_MULTISYNTH_C_MAX-1)
#define SI5351_MULTISYNTH_C_MAX         1048575
#define SI5351_MULTISYNTH_P1_MAX        ((1<<18)-1)
#define SI5351_MULTISYNTH_P2_MAX        ((1<<20)-1)
#define SI5351_MULTISYNTH_P3_MAX        ((1<<20)-1)
#define SI5351_VCXO_PULL_MIN            30
#define SI5351_VCXO_PULL_MAX            240
#define SI5351_VCXO_MARGIN              103

#define SI5351_DEVICE_STATUS            0
#define SI5351_INTERRUPT_STATUS         1
#define SI5351_INTERRUPT_MASK           2
#define SI5351_STATUS_SYS_INIT          (1<<7)
#define SI5351_STATUS_LOL_B             (1<<6)
#define SI5351_STATUS_LOL_A             (1<<5)
#define SI5351_STATUS_LOS               (1<<4)
#define SI5351_OUTPUT_ENABLE_CTRL       3
#define SI5351_OEB_PIN_ENABLE_CTRL      9
#define SI5351_PLL_INPUT_SOURCE         15
#define SI5351_CLKIN_DIV_MASK           (3<<6)
#define SI5351_CLKIN_DIV_1              (0<<6)
#define SI5351_CLKIN_DIV_2              (1<<6)
#define SI5351_CLKIN_DIV_4              (2<<6)
#define SI5351_CLKIN_DIV_8              (3<<6)
#define SI5351_PLLB_SOURCE              (1<<3)
#define SI5351_PLLA_SOURCE              (1<<2)

#define SI5351_CLK0_CTRL                16
#define SI5351_CLK1_CTRL                17
#define SI5351_CLK2_CTRL                18
#define SI5351_CLK3_CTRL                19
#define SI5351_CLK4_CTRL                20
#define SI5351_CLK5_CTRL                21
#define SI5351_CLK6_CTRL                22
#define SI5351_CLK7_CTRL                23
#define SI5351_CLK_POWERDOWN            (1<<7)
#define SI5351_CLK_INTEGER_MODE         (1<<6)
#define SI5351_CLK_PLL_SELECT           (1<<5)
#define SI5351_CLK_INVERT               (1<<4)
#define SI5351_CLK_INPUT_MASK           (3<<2)
#define SI5351_CLK_INPUT_XTAL           (0<<2)
#define SI5351_CLK_INPUT_CLKIN          (1<<2)
#define SI5351_CLK_INPUT_MULTISYNTH_0_4 (2<<2)
#define SI5351_CLK_INPUT_MULTISYNTH_N   (3<<2)
#define SI5351_CLK_DRIVE_STRENGTH_MASK  (3<<0)
#define SI5351_CLK_DRIVE_STRENGTH_2MA   (0<<0)
#define SI5351_CLK_DRIVE_STRENGTH_4MA   (1<<0)
#define SI5351_CLK_DRIVE_STRENGTH_6MA   (2<<0)
#define SI5351_CLK_DRIVE_STRENGTH_8MA   (3<<0)

#define SI5351_CLK3_0_DISABLE_STATE     24
#define SI5351_CLK7_4_DISABLE_STATE     25
#define SI5351_CLK_DISABLE_STATE_MASK   3
#define SI5351_CLK_DISABLE_STATE_LOW    0
#define SI5351_CLK_DISABLE_STATE_HIGH   1
#define SI5351_CLK_DISABLE_STATE_FLOAT  2
#define SI5351_CLK_DISABLE_STATE_NEVER  3

#define SI5351_PARAMETERS_LENGTH        8
#define SI5351_PLLA_PARAMETERS          26
#define SI5351_PLLB_PARAMETERS          34
#define SI5351_CLK0_PARAMETERS          42
#define SI5351_CLK1_PARAMETERS          50
#define SI5351_CLK2_PARAMETERS          58
#define SI5351_CLK3_PARAMETERS          66
#define SI5351_CLK4_PARAMETERS          74
#define SI5351_CLK5_PARAMETERS          82
#define SI5351_CLK6_PARAMETERS          90
#define SI5351_CLK7_PARAMETERS          91
#define SI5351_CLK6_7_OUTPUT_DIVIDER    92
#define SI5351_OUTPUT_CLK_DIV_MASK      (7 << 4)
#define SI5351_OUTPUT_CLK6_DIV_MASK     (7 << 0)
#define SI5351_OUTPUT_CLK_DIV_SHIFT     4
#define SI5351_OUTPUT_CLK_DIV6_SHIFT    0
#define SI5351_OUTPUT_CLK_DIV_1         0
#define SI5351_OUTPUT_CLK_DIV_2         1
#define SI5351_OUTPUT_CLK_DIV_4         2
#define SI5351_OUTPUT_CLK_DIV_8         3
#define SI5351_OUTPUT_CLK_DIV_16        4
#define SI5351_OUTPUT_CLK_DIV_32        5
#define SI5351_OUTPUT_CLK_DIV_64        6
#define SI5351_OUTPUT_CLK_DIV_128       7
#define SI5351_OUTPUT_CLK_DIVBY4       (3<<2)

#define SI5351_SSC_PARAM0               149
#define SI5351_SSC_PARAM1               150
#define SI5351_SSC_PARAM2               151
#define SI5351_SSC_PARAM3               152
#define SI5351_SSC_PARAM4               153
#define SI5351_SSC_PARAM5               154
#define SI5351_SSC_PARAM6               155
#define SI5351_SSC_PARAM7               156
#define SI5351_SSC_PARAM8               157
#define SI5351_SSC_PARAM9               158
#define SI5351_SSC_PARAM10              159
#define SI5351_SSC_PARAM11              160
#define SI5351_SSC_PARAM12              161

#define SI5351_VXCO_PARAMETERS_LOW      162
#define SI5351_VXCO_PARAMETERS_MID      163
#define SI5351_VXCO_PARAMETERS_HIGH     164

#define SI5351_CLK0_PHASE_OFFSET        165
#define SI5351_CLK1_PHASE_OFFSET        166
#define SI5351_CLK2_PHASE_OFFSET        167
#define SI5351_CLK3_PHASE_OFFSET        168
#define SI5351_CLK4_PHASE_OFFSET        169
#define SI5351_CLK5_PHASE_OFFSET        170

#define SI5351_PLL_RESET                177
#define SI5351_PLL_RESET_B              (1<<7)
#define SI5351_PLL_RESET_A              (1<<5)

#define SI5351_CRYSTAL_LOAD             183
#define SI5351_CRYSTAL_LOAD_MASK        (3<<6)
#define SI5351_CRYSTAL_LOAD_0PF         (0<<6)
#define SI5351_CRYSTAL_LOAD_6PF			(1<<6)
#define SI5351_CRYSTAL_LOAD_8PF         (2<<6)
#define SI5351_CRYSTAL_LOAD_10PF        (3<<6)

#define SI5351_FANOUT_ENABLE            187
#define SI5351_CLKIN_ENABLE             (1<<7)
#define SI5351_XTAL_ENABLE              (1<<6)
#define SI5351_MULTISYNTH_ENABLE        (1<<4)


/* Macro definitions */

#define RFRAC_DENOM 1000000ULL

enum si5351_clock {SI5351_CLK0, SI5351_CLK1, SI5351_CLK2, SI5351_CLK3,
	SI5351_CLK4, SI5351_CLK5, SI5351_CLK6, SI5351_CLK7, SI5351_CLKNONE};

enum si5351_pll {SI5351_PLLA, SI5351_PLLB};

enum si5351_drive {SI5351_DRIVE_2MA, SI5351_DRIVE_4MA, SI5351_DRIVE_6MA, SI5351_DRIVE_8MA};

enum si5351_clock_source {SI5351_CLK_SRC_XTAL, SI5351_CLK_SRC_CLKIN, SI5351_CLK_SRC_MS0, SI5351_CLK_SRC_MS};

enum si5351_clock_disable {SI5351_CLK_DISABLE_LOW, SI5351_CLK_DISABLE_HIGH, SI5351_CLK_DISABLE_HI_Z, SI5351_CLK_DISABLE_NEVER};

enum si5351_clock_fanout {SI5351_FANOUT_CLKIN, SI5351_FANOUT_XO, SI5351_FANOUT_MS};

enum si5351_pll_input {SI5351_PLL_INPUT_XO, SI5351_PLL_INPUT_CLKIN};

enum si5351_out_state { SI5351_OUT_DISABLE, SI5351_OUT_ENABLE };

typedef struct {
	uint32_t p1;
	uint32_t p2;
	uint32_t p3;
} Si5351RegSet;

typedef struct {
	uint8_t SYS_INIT;
	uint8_t LOL_B;
	uint8_t LOL_A;
	uint8_t LOS;
	uint8_t REVID;
} Si5351Status;

typedef struct {
	uint8_t SYS_INIT_STKY;
	uint8_t LOL_B_STKY;
	uint8_t LOL_A_STKY;
	uint8_t LOS_STKY;
} Si5351IntStatus;

/*
 * Based on former asm-ppc/div64.h and asm-m68knommu/div64.h
 *
 * The semantics of do_div() are:
 *
 * uint32_t do_div(uint64_t *n, uint32_t base)
 * {
 *      uint32_t remainder = *n % base;
 *      *n = *n / base;
 *      return remainder;
 * }
 *
 * NOTE: macro parameter n is evaluated multiple times,
 *       beware of side effects!
 */

# define do_div(n,base) ({                                      \
        uint64_t __base = (base);                               \
        uint64_t __rem;                                         \
        __rem = ((uint64_t)(n)) % __base;                       \
        (n) = ((uint64_t)(n)) / __base;                         \
        __rem;                                                  \
 })


  void Si5351_init(uint8_t xtal_load_c, uint32_t ref_osc_freq);
  void Si5351_set_correction(int32_t);
  void Si5351_set_pll(uint64_t pll_freq, enum si5351_pll target_pll);
  uint8_t Si5351_set_freq(uint64_t freq, uint64_t pll_freq, enum si5351_clock clk);
  void Si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive);

  //void Si5351_OutEnable(bool enable, unsigned char channel);

  void Si5351_set_state_out(enum si5351_clock clock_out, enum si5351_out_state out_state);

#endif
