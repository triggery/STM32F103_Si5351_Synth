/**************************************************************************/
/*!
    @file     si5351.c

    @author   K. Townsend (Adafruit Industries)

    @brief    Driver for the SI5351 160MHz Clock Gen

    @section  REFERENCES

    Si5351A/B/C Datasheet:
    http://www.silabs.com/Support%20Documents/TechnicalDocs/Si5351.pdf

    Manually Generating an Si5351 Register Map:
    http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf

    @section  LICENSE

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
#include "main.h"
// CHANGE THIS FOR YOUR CHIP
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include <math.h>
#include <si5351.h>

Si5351Status dev_status;
Si5351IntStatus dev_int_status;

uint64_t plla_freq;
uint64_t pllb_freq;
uint64_t clk0_freq;
uint64_t clk1_freq;
uint64_t clk2_freq;
uint8_t clk0_int_mode, clk1_int_mode, clk2_int_mode;

int32_t ref_correction;
uint8_t lock_plla, lock_pllb;
uint32_t xtal_freq;

uint64_t Si5351_pllCalc(uint64_t freq, Si5351RegSet *reg, int32_t correction);
HAL_StatusTypeDef si5351_write_bulk(uint8_t addr, uint8_t bytes, uint8_t *data);

uint8_t select_r_div(uint64_t *freq);
uint64_t multisynth_calc(uint64_t freq, uint64_t pll_freq, Si5351RegSet *reg);
void set_ms(enum si5351_clock clk, Si5351RegSet ms_reg, uint8_t int_mode, uint8_t r_div, uint8_t div_by_4);
//void set_ms_source(enum si5351_clock clk, enum si5351_pll pll);
void set_int(enum si5351_clock clk, uint8_t enable);
void ms_div(enum si5351_clock clk, uint8_t r_div, uint8_t div_by_4);
void pll_reset(enum si5351_pll target_pll);

err_t si5351_write8(uint8_t reg, uint8_t value);
err_t si5351_read8(uint8_t reg, uint8_t *value);
/*
 * init(uint8_t xtal_load_c, uint32_t ref_osc_freq)
 *
 * Setup communications to the Si5351 and set the crystal
 * load capacitance.
 *
 * xtal_load_c - Crystal load capacitance. Use the SI5351_CRYSTAL_LOAD_*PF
 * defines in the header file
 * ref_osc_freq - Crystal/reference oscillator frequency in 1 Hz increments.
 * Defaults to 25000000 if a 0 is used here.
 *
 */
void Si5351_init(uint8_t xtal_load_c, uint32_t ref_osc_freq)
{
	lock_plla = SI5351_CLKNONE;
	lock_pllb = SI5351_CLKNONE;
	clk0_int_mode = 0;
	clk1_int_mode = 0;
	clk2_int_mode = 0;
	plla_freq = 0ULL;
	pllb_freq = 0ULL;
	clk0_freq = 0ULL;
	clk1_freq = 0ULL;
	clk2_freq = 0ULL;
	xtal_freq = SI5351_XTAL_FREQ;

	ref_correction = 0;

	// Set crystal load capacitance
	si5351_write8(SI5351_CRYSTAL_LOAD, xtal_load_c);

	// Change the ref osc freq if different from default
	if (ref_osc_freq != 0)
	{
		xtal_freq = ref_osc_freq;
	}

	// Initialize the CLK outputs according to flowchart in datasheet
	// First, turn them off
	si5351_write8(16, 0x80);
	si5351_write8(17, 0x80);
	si5351_write8(18, 0x80);

	// Turn the clocks back on...
	si5351_write8(16, 0x0c);
	si5351_write8(17, 0x0c);
	si5351_write8(18, 0x0c);

	// Then reset the PLLs
	pll_reset(SI5351_PLLA);
	pll_reset(SI5351_PLLB);
}

void Si5351_set_correction(int32_t corr)
{
	ref_correction = corr;
}

/*
 * pll_reset(enum si5351_pll target_pll)
 *
 * target_pll - Which PLL to reset
 *     (use the si5351_pll enum)
 *
 * Apply a reset to the indicated PLL.
 */
void pll_reset(enum si5351_pll target_pll)
{
	if(target_pll == SI5351_PLLA)
 	{
    	si5351_write8(SI5351_PLL_RESET, SI5351_PLL_RESET_A);
	}
	else if(target_pll == SI5351_PLLB)
	{
	    si5351_write8(SI5351_PLL_RESET, SI5351_PLL_RESET_B);
	}
}

uint64_t Si5351_pllCalc(uint64_t freq, Si5351RegSet *reg, int32_t correction)
{
	uint64_t ref_freq = xtal_freq * SI5351_FREQ_MULT;	//xtal_freq
	uint32_t a, b, c, p1, p2, p3;
	uint64_t lltmp, denom;

	// Factor calibration value into nominal crystal frequency
	// Measured in parts-per-billion

	ref_freq = ref_freq + (int32_t)((((((int64_t)correction) << 31) / 1000000000LL) * ref_freq) >> 31);

	// PLL bounds checking
	if (freq < SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT)
	{
		freq = SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT;
	}
	if (freq > SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT)
	{
		freq = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT;
	}

	// Determine integer part of feedback equation
	a = freq / ref_freq;

	if (a < SI5351_PLL_A_MIN)
	{
		freq = ref_freq * SI5351_PLL_A_MIN;
	}
	if (a > SI5351_PLL_A_MAX)
	{
		freq = ref_freq * SI5351_PLL_A_MAX;
	}

	// Find best approximation for b/c = fVCO mod fIN
	denom = 1000ULL * 1000ULL;
	lltmp = freq % ref_freq;
	lltmp *= denom;
	do_div(lltmp, ref_freq);

	b = (((uint64_t)(freq % ref_freq)) * RFRAC_DENOM) / ref_freq;
	c = b ? RFRAC_DENOM : 1;

	// Calculate parameters
    p1 = 128 * a + ((128 * b) / c) - 512;
    p2 = 128 * b - c * ((128 * b) / c);
    p3 = c;

	// Recalculate frequency as fIN * (a + b/c)
	lltmp  = ref_freq;
	lltmp *= b;
	do_div(lltmp, c);
	freq = lltmp;
	freq += ref_freq * a;

	reg->p1 = p1;
	reg->p2 = p2;
	reg->p3 = p3;
	return freq;
}


/*
 * set_freq(uint64_t freq, uint64_t pll_freq, enum si5351_clock output)
 *
 * Sets the clock frequency of the specified CLK output
 *
 * freq - Output frequency in Hz
 * pll_freq - Frequency of the PLL driving the Multisynth
 *   Use a 0 to have the function choose a PLL frequency
 * clk - Clock output
 *   (use the si5351_clock enum)
 */
uint8_t Si5351_set_freq(uint64_t freq, uint64_t pll_freq, enum si5351_clock clk)
{
	Si5351RegSet ms_reg;	//, pll_reg;
	//enum si5351_pll target_pll;
	//uint8_t write_pll = 0;
	//uint8_t reg_val;
	uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;
	uint8_t int_mode = 0;
	uint8_t div_by_4 = 0;

	// PLL bounds checking
	if(pll_freq != 0)
	{
		if ((pll_freq < SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT) || (pll_freq > SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT))
		{
			return 1;
		}
	}

	// Lower bounds check
	if(freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT)
	{
		freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;
	}

	// Upper bounds check
	if(freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT)
	{
		freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;
	}

	// Select the proper R div value
	r_div = select_r_div(&freq);

	// Calculate the synth parameters
	// If pll_freq is 0 and freq < 150 MHz, let the algorithm pick a PLL frequency
	if((pll_freq) && (freq < SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT))
	{
		multisynth_calc(freq, pll_freq, &ms_reg);
		//write_pll = 0;
		div_by_4 = 0;
		int_mode = 0;

		/*switch(clk)
		{
		case SI5351_CLK0:
			clk0_freq = freq;
			break;
		case SI5351_CLK1:
			clk1_freq = freq;
			break;
		case SI5351_CLK2:
			clk2_freq = freq;
			break;
		default:
			break;
		}*/
	}
	else
	{
		clk1_freq = 0;
		return 1;
		/*
		// The PLL must be calculated and set by firmware when 150 MHz <= freq <= 160 MHz
		if(freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT)
		{
			pll_freq = multisynth_calc(freq, 0, &ms_reg);
			write_pll = 1;
			div_by_4 = 1;
			int_mode = 1;
		}

		// Determine which PLL to use
		// CLK0 gets PLLA, CLK1 gets PLLB
		// CLK2 gets PLLB if necessary
		// Only good for Si5351A3 variant at the moment
		switch(clk)
		{
		case SI5351_CLK0:
			pll_freq = multisynth_calc(freq, 0, &ms_reg);
			target_pll = SI5351_PLLA;
			write_pll = 1;
			Si5351_set_ms_source(SI5351_CLK0, SI5351_PLLA);

			plla_freq = pll_freq;
			clk0_freq = freq;
			break;
		case SI5351_CLK1:
			// Check to see if PLLB is locked due to other output being < 1.024 MHz or >= 112.5 MHz
			if(lock_pllb == SI5351_CLK2)
			{
				// We can't have a 2nd output < 1.024 MHz or >= 112.5 MHz on the same PLL unless exact same freq, so exit
				if((freq >= SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT
					|| freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128) && freq != clk2_freq)
				{
					clk1_freq = 0;
					return 1;
				}
				// Else, set multisynth to same PLL freq as CLK2
				else
				{
					pll_freq = pllb_freq;
					multisynth_calc(freq, pll_freq, &ms_reg);
					write_pll = 0;
					Si5351_set_ms_source(SI5351_CLK1, SI5351_PLLB);
				}
			}
			else
			{
				pllb_freq = pll_freq;
				pll_freq = multisynth_calc(freq, 0, &ms_reg);
				write_pll = 1;
				Si5351_set_ms_source(SI5351_CLK1, SI5351_PLLB);
			}

			if(freq >= SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT || freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128)
			{
				lock_pllb = SI5351_CLK1;

				// Recalc and rewrite the multisynth parameters on CLK2
				if(clk2_freq != 0)
				{
					Si5351RegSet ms_temp_reg;
					r_div = select_r_div(&clk2_freq);
					multisynth_calc(clk2_freq, pllb_freq, &ms_temp_reg);
					set_ms(SI5351_CLK2, ms_temp_reg, 0, r_div, 0);
				}
			}
			else
			{
				lock_pllb = SI5351_CLKNONE;
			}

			target_pll = SI5351_PLLB;
			clk1_freq = freq;
			break;
		case SI5351_CLK2:
			// Check to see if PLLB is locked due to other output being < 1.024 MHz or >= 112.5 MHz
			if(lock_pllb == SI5351_CLK1)
			{
				// We can't have a 2nd output < 1.024 MHz  or >= 112.5 MHz on the same PLL unless exact same freq, so exit
				if((freq >= SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT
					|| freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128) && freq != clk2_freq)
				{
					clk2_freq = 0;
					return 1;
				}
				// Else, set multisynth to same PLL freq as CLK1
				else
				{
					pll_freq = pllb_freq;
					multisynth_calc(freq, pll_freq, &ms_reg);
					write_pll = 0;
					Si5351_set_ms_source(SI5351_CLK2, SI5351_PLLB);
				}
			}
			// need to account for CLK2 set before CLK1
			else
			{
				pllb_freq = pll_freq;
				pll_freq = multisynth_calc(freq, 0, &ms_reg);
				write_pll = 1;
				Si5351_set_ms_source(SI5351_CLK2, SI5351_PLLB);
			}

			if(freq >= SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT || freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128)
			{
				lock_pllb = SI5351_CLK2;

				if(clk1_freq != 0)
				{
					// Recalc and rewrite the multisynth parameters on CLK1
					Si5351RegSet ms_temp_reg;
					r_div = select_r_div(&clk1_freq);
					multisynth_calc(clk1_freq, pllb_freq, &ms_temp_reg);
					set_ms(SI5351_CLK1, ms_temp_reg, 0, r_div, 0);
				}
			}
			else
			{
				lock_pllb = SI5351_CLKNONE;
			}

			target_pll = SI5351_PLLB;
			clk2_freq = freq;
			break;
		default:
			return 1;
		} */
	}

	// Set multisynth registers (MS must be set before PLL)
	set_ms(clk, ms_reg, int_mode, r_div, div_by_4);

	// Set PLL if necessary
	/*if(write_pll == 1)
	{
		Si5351_set_pll(pll_freq, target_pll);
	}*/

	return 0;
}


uint8_t select_r_div(uint64_t *freq)
{
	uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;

	// Choose the correct R divider
	if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 2))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_128;
		*freq *= 128ULL;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 2) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 4))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_64;
		*freq *= 64ULL;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 4) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 8))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_32;
		*freq *= 32ULL;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 8) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 16))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_16;
		*freq *= 16ULL;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 16) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 32))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_8;
		*freq *= 8ULL;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 32) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 64))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_4;
		*freq *= 4ULL;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 64) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_2;
		*freq *= 2ULL;
	}

	return r_div;
}


uint64_t multisynth_calc(uint64_t freq, uint64_t pll_freq, Si5351RegSet *reg)
{
	uint64_t lltmp;
	uint32_t a, b, c, p1, p2, p3;
	uint8_t divby4;
	uint8_t ret_val = 0;

	// Multisynth bounds checking
	if (freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT)
	{
		freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;
	}
	if (freq < SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT)
	{
		freq = SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT;
	}

	divby4 = 0;
	if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT)
	{
		divby4 = 1;
	}

	if(pll_freq == 0)
	{
		// Find largest integer divider for max
		// VCO frequency and given target frequency
		if(divby4 == 0)
		{
			lltmp = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT;
			do_div(lltmp, freq);
			a = (uint32_t)lltmp;
		}
		else
		{
			a = 4;
		}

		b = 0;
		c = 1;
		pll_freq = a * freq;
	}
	else
	{
		// Preset PLL, so return the actual freq for these params instead of PLL freq
		ret_val = 1;

		// Determine integer part of feedback equation
		a = pll_freq / freq;

		if (a < SI5351_MULTISYNTH_A_MIN)
		{
			freq = pll_freq / SI5351_MULTISYNTH_A_MIN;
		}
		if (a > SI5351_MULTISYNTH_A_MAX)
		{
			freq = pll_freq / SI5351_MULTISYNTH_A_MAX;
		}

		b = (pll_freq % freq * RFRAC_DENOM) / freq;
		c = b ? RFRAC_DENOM : 1;
	}

	// Calculate parameters
	if (divby4 == 1)
	{
		p3 = 1;
		p2 = 0;
		p1 = 0;
	}
	else
	{
        p1 = 128 * a + ((128 * b) / c) - 512;
        p2 = 128 * b - c * ((128 * b) / c);
        p3 = c;
	}

	reg->p1 = p1;
	reg->p2 = p2;
	reg->p3 = p3;

	if(ret_val == 0)
	{
		return pll_freq;
	}
	else
	{
		return freq;
	}
}

/*
 * set_pll(uint64_t pll_freq, enum si5351_pll target_pll)
 *
 * Set the specified PLL to a specific oscillation frequency
 *
 * pll_freq - Desired PLL frequency
 * target_pll - Which PLL to set
 *     (use the si5351_pll enum)
 */
void Si5351_set_pll(uint64_t pll_freq, enum si5351_pll target_pll)
{
  Si5351RegSet pll_reg;

  Si5351_pllCalc(pll_freq, &pll_reg, ref_correction);

  // Derive the register values to write

  // Prepare an array for parameters to be written to
  uint8_t params[20];
  uint8_t i = 0;
  uint8_t temp;

  // Registers 26-27
  temp = ((pll_reg.p3 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p3  & 0xFF);
  params[i++] = temp;

  // Register 28
  temp = (uint8_t)((pll_reg.p1 >> 16) & 0x03);
  params[i++] = temp;

  // Registers 29-30
  temp = (uint8_t)((pll_reg.p1 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p1  & 0xFF);
  params[i++] = temp;

  // Register 31
  temp = (uint8_t)((pll_reg.p3 >> 12) & 0xF0);
  temp += (uint8_t)((pll_reg.p2 >> 16) & 0x0F);
  params[i++] = temp;

  // Registers 32-33
  temp = (uint8_t)((pll_reg.p2 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p2  & 0xFF);
  params[i++] = temp;

  // Write the parameters
  if(target_pll == SI5351_PLLA)
  {
    si5351_write_bulk(SI5351_PLLA_PARAMETERS, i, &params[0]); // uint8_t addr, uint8_t bytes, uint8_t *data
  }
  else if(target_pll == SI5351_PLLB)
  {
    si5351_write_bulk(SI5351_PLLB_PARAMETERS, i, &params[0]);
  }
}

/*
 * set_ms(enum si5351_clock clk, struct Si5351RegSet ms_reg, uint8_t int_mode, uint8_t r_div, uint8_t div_by_4)
 *
 * Set the specified multisynth parameters. Not normally needed, but public for advanced users.
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * int_mode - Set integer mode
 *  Set to 1 to enable, 0 to disable
 * r_div - Desired r_div ratio
 * div_by_4 - Set Divide By 4 mode
 *   Set to 1 to enable, 0 to disable
 */
void set_ms(enum si5351_clock clk, Si5351RegSet ms_reg, uint8_t int_mode, uint8_t r_div, uint8_t div_by_4)
{
	uint8_t params[20];
	uint8_t i = 0;
 	uint8_t temp;
 	uint8_t reg_val;

	// Registers 42-43 for CLK0
	temp = (uint8_t)((ms_reg.p3 >> 8) & 0xFF);
	params[i++] = temp;

	temp = (uint8_t)(ms_reg.p3  & 0xFF);
	params[i++] = temp;

	// Register 44 for CLK0
	//reg_val = si5351_read((SI5351_CLK0_PARAMETERS + 2) + (clk * 8));
	si5351_read8((SI5351_CLK0_PARAMETERS + 2) + (clk * 8), &reg_val);

	reg_val &= ~(0x03);
	temp = reg_val | ((uint8_t)((ms_reg.p1 >> 16) & 0x03));
	params[i++] = temp;

	// Registers 45-46 for CLK0
	temp = (uint8_t)((ms_reg.p1 >> 8) & 0xFF);
	params[i++] = temp;

	temp = (uint8_t)(ms_reg.p1  & 0xFF);
	params[i++] = temp;

	// Register 47 for CLK0
	temp = (uint8_t)((ms_reg.p3 >> 12) & 0xF0);
	temp += (uint8_t)((ms_reg.p2 >> 16) & 0x0F);
	params[i++] = temp;

	// Registers 48-49 for CLK0
	temp = (uint8_t)((ms_reg.p2 >> 8) & 0xFF);
	params[i++] = temp;

	temp = (uint8_t)(ms_reg.p2  & 0xFF);
	params[i++] = temp;

	// Write the parameters
	switch(clk)
	{
		case SI5351_CLK0:
			si5351_write_bulk(SI5351_CLK0_PARAMETERS, i, &params[0]);
			break;
		case SI5351_CLK1:
			si5351_write_bulk(SI5351_CLK1_PARAMETERS, i, &params[0]);
			break;
		case SI5351_CLK2:
			si5351_write_bulk(SI5351_CLK2_PARAMETERS, i, &params[0]);
			break;
		case SI5351_CLK3:
			si5351_write_bulk(SI5351_CLK3_PARAMETERS, i, &params[0]);
			break;
		case SI5351_CLK4:
			si5351_write_bulk(SI5351_CLK4_PARAMETERS, i, &params[0]);
			break;
		case SI5351_CLK5:
			si5351_write_bulk(SI5351_CLK5_PARAMETERS, i, &params[0]);
			break;
		case SI5351_CLK6:
			si5351_write_bulk(SI5351_CLK6_PARAMETERS, i, &params[0]);
			break;
		case SI5351_CLK7:
			si5351_write_bulk(SI5351_CLK7_PARAMETERS, i, &params[0]);
			break;
		default:
			return;
	}

	set_int(clk, int_mode);
	ms_div(clk, r_div, div_by_4);
}

/*
 * set_int(enum si5351_clock clk, uint8_t int_mode)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 *
 * Set the indicated multisynth into integer mode.
 */
void set_int(enum si5351_clock clk, uint8_t enable)
{
	uint8_t reg_val;
	//reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);
	si5351_read8((SI5351_CLK0_CTRL + (uint8_t)clk), &reg_val);

	if(enable == 1)
	{
		reg_val |= (SI5351_CLK_INTEGER_MODE);
	}
	else
	{
		reg_val &= ~(SI5351_CLK_INTEGER_MODE);
	}

	//si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
	si5351_write8(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);

	// Integer mode indication
	switch(clk)
	{
	case SI5351_CLK0:
		clk0_int_mode = enable;
		break;
	case SI5351_CLK1:
		clk1_int_mode = enable;
		break;
	case SI5351_CLK2:
		clk2_int_mode = enable;
		break;
	default:
		break;
	}
}

void ms_div(enum si5351_clock clk, uint8_t r_div, uint8_t div_by_4)
{
	uint8_t reg_val, reg_addr;

	switch(clk)
	{
		case SI5351_CLK0:
			reg_addr = SI5351_CLK0_PARAMETERS + 2;
			break;
		case SI5351_CLK1:
			reg_addr = SI5351_CLK1_PARAMETERS + 2;
			break;
		case SI5351_CLK2:
			reg_addr = SI5351_CLK2_PARAMETERS + 2;
			break;
		case SI5351_CLK3:
			reg_addr = SI5351_CLK3_PARAMETERS + 2;
			break;
		case SI5351_CLK4:
			reg_addr = SI5351_CLK4_PARAMETERS + 2;
			break;
		case SI5351_CLK5:
			reg_addr = SI5351_CLK5_PARAMETERS + 2;
			break;
		case SI5351_CLK6:
			return;
		case SI5351_CLK7:
			return;
		default:
			return;
	}

	//reg_val = si5351_read(reg_addr);
	si5351_read8(reg_addr, &reg_val);

	// Clear the relevant bits
	reg_val &= ~(0x7c);

	if(div_by_4 == 0)
	{
		reg_val &= ~(SI5351_OUTPUT_CLK_DIVBY4);
	}
	else
	{
		reg_val |= (SI5351_OUTPUT_CLK_DIVBY4);
	}

	reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);

	//si5351_write(reg_addr, reg_val);
	si5351_write8(reg_addr, reg_val);
}

/*
 * set_ms_source(enum si5351_clock clk, enum si5351_pll pll)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * pll - Which PLL to use as the source
 *     (use the si5351_pll enum)
 *
 * Set the desired PLL source for a multisynth.
 */
void Si5351_set_ms_source(enum si5351_clock clk, enum si5351_pll pll)
{
	uint8_t reg_val;

	//reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);
	si5351_read8((SI5351_CLK0_CTRL + (uint8_t)clk), &reg_val);

	if(pll == SI5351_PLLA)
	{
		reg_val &= ~(SI5351_CLK_PLL_SELECT);
	}
	else if(pll == SI5351_PLLB)
	{
		reg_val |= SI5351_CLK_PLL_SELECT;
	}

	//si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
	si5351_write8(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}


HAL_StatusTypeDef si5351_write_bulk(uint8_t addr, uint8_t bytes, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_OK;

	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(SI5351_ADDRESS<<1), 3, 100) != HAL_OK) { }

    status = HAL_I2C_Mem_Write(&hi2c1,							// i2c handle
    						  (uint8_t)(SI5351_ADDRESS<<1),		// i2c address, left aligned
							  addr,								// register address
							  I2C_MEMADD_SIZE_8BIT,				// si5351 uses 8bit register addresses
							  data,								// write returned data to this variable
							  bytes,							// how many bytes to expect returned
							  100);								// timeout
    return status;
}

/*
 * drive_strength(enum si5351_clock clk, enum si5351_drive drive)
 *
 * Sets the drive strength of the specified clock output
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * drive - Desired drive level
 *   (use the si5351_drive enum)
 */
void Si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive)
{
  uint8_t reg_val;
  const uint8_t mask = 0x03;

  //reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);
  si5351_read8((SI5351_CLK0_CTRL + (uint8_t)clk), &reg_val);
  reg_val &= ~(mask);

  switch(drive)
  {
  case SI5351_DRIVE_2MA:
    reg_val |= 0x00;
    break;
  case SI5351_DRIVE_4MA:
   reg_val |= 0x01;
    break;
  case SI5351_DRIVE_6MA:
    reg_val |= 0x02;
    break;
  case SI5351_DRIVE_8MA:
    reg_val |= 0x03;
    break;
  default:
    break;
  }

  si5351_write8(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

//void Si5351_OutEnable(bool enable, unsigned char channel) {
//	uint8_t reg_val;
//
//	if (channel > 2)
//		return;
//
//	si5351_read8((SI5351_OUTPUT_ENABLE_CTRL), &reg_val);
//
//	if(enable)
//	{
//		reg_val &= ~(1 << channel);
//	}
//	else
//	{
//		reg_val |= (1 << channel);
//	}
//
//	si5351_write8(SI5351_OUTPUT_ENABLE_CTRL, reg_val);
//}

/*
 * Si5351_set_state_out(enum si5351_clock clock_out, enum si5351_out_state out_state)
 * set output enable control
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * out_state - output state
 *   (use the enum si5351_out_state)
 */
void Si5351_set_state_out(enum si5351_clock clock_out, enum si5351_out_state out_state) {
	uint8_t reg_val;

	si5351_read8((SI5351_OUTPUT_ENABLE_CTRL), &reg_val);

	switch(clock_out) {
	case SI5351_CLK0:
	    out_state == SI5351_OUT_ENABLE ? (reg_val &= ~(1 << 0)) : (reg_val |= (1 << 0));
		break;
	case SI5351_CLK1:
		out_state == SI5351_OUT_ENABLE ? (reg_val &= ~(1 << 1)) : (reg_val |= (1 << 1));
		break;
	case SI5351_CLK2:
		out_state == SI5351_OUT_ENABLE ? (reg_val &= ~(1 << 2)) : (reg_val |= (1 << 2));
		break;
	default:
		break;
	}

	si5351_write8(SI5351_OUTPUT_ENABLE_CTRL, reg_val);
}

/* ---------------------------------------------------------------------- */
/* PRUVATE FUNCTIONS                                                      */
/* ---------------------------------------------------------------------- */

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
err_t si5351_write8 (uint8_t reg, uint8_t value)
{
	//HAL_StatusTypeDef status = HAL_OK;
  
	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(SI5351_ADDRESS<<1), 3, 100) != HAL_OK) { }
	//status =
    HAL_I2C_Mem_Write(&hi2c1,							// i2c handle
    						  (uint8_t)(SI5351_ADDRESS<<1),		// i2c address, left aligned
							  (uint8_t)reg,						// register address
							  I2C_MEMADD_SIZE_8BIT,				// si5351 uses 8bit register addresses
							  (uint8_t*)(&value),				// write returned data to this variable
							  1,								// how many bytes to expect returned
							  100);								// timeout

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
err_t si5351_read8(uint8_t reg, uint8_t *value)
{
	//HAL_StatusTypeDef status = HAL_OK;

	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(SI5351_ADDRESS<<1), 3, 100) != HAL_OK) { }
	//status =
    HAL_I2C_Mem_Read(&hi2c1,							// i2c handle
    						  (uint8_t)(SI5351_ADDRESS<<1),		// i2c address, left aligned
							  (uint8_t)reg,						// register address
							  I2C_MEMADD_SIZE_8BIT,				// si5351 uses 8bit register addresses
							  value,				// write returned data to this variable
							  1,								// how many bytes to expect returned
							  100);								// timeout

  return ERROR_NONE;
}

