/* -*- c++ -*- */
/* 
 * Copyright 2014 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "gps_codegen_c_impl.h"

#include <cstdio>

namespace gr {
  namespace gps {

    gps_codegen_c::sptr
    gps_codegen_c::make(float samplerate, int code, int datamode)
    {
      return gnuradio::get_initial_sptr
        (new gps_codegen_c_impl(samplerate, code, datamode));
    }

    /*
     * The private constructor
     */
    gps_codegen_c_impl::gps_codegen_c_impl(float samplerate, int code, int datamode)
      : gr::sync_block("gps_codegen_c",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1,1, sizeof(gr_complex)))
    {
		set_code(code);
		
		set_sample_rate(samplerate);
		code_phase = 0;

		datagen_mode = datamode;
		data_counter = 0;

		for(int i = 0; i < 22 ; ++i)
			random_data[i] = 1;
	}

    /*
     * Our virtual destructor.
     */
    gps_codegen_c_impl::~gps_codegen_c_impl()
    {
    }

	void gps_codegen_c_impl::set_code(int c)
	{
		int g1_tap0 = 0;
		int g1_tap1 = 0;
		
		code_selection = c;
	
		// code generator and tap coefficients from 
		// Matjaz Vidmar
		// http://lea.hamradio.si/~s53mv/navsats/theory.html
		
		switch(c) 
		{
			case 1:
			g1_tap0=1;
			g1_tap1=5;
			break;
			
			case 2:
			g1_tap0=2;
			g1_tap1=6;
			break;
			
			case 3:
			g1_tap0=3;
			g1_tap1=7;
			break;
			
			case 4:
			g1_tap0=4;
			g1_tap1=8;
			break;
			
			case 5:
			g1_tap0=0;
			g1_tap1=8;
			break;
			
			case 6:
			g1_tap0=1;
			g1_tap1=5;
			break;
			
			case 7:
			g1_tap0=0;
			g1_tap1=7;
			break;
			
			case 8:
			g1_tap0=1;
			g1_tap1=8;
			break;
			
			case 9:
			g1_tap0=2;
			g1_tap1=9;
			break;
			
			case 10:
			g1_tap0=1;
			g1_tap1=2;
			break;
			
			case 11:
			g1_tap0=2;
			g1_tap1=3;
			break;
			
			case 12:
			g1_tap0=4;
			g1_tap1=5;
			break;
			
			case 13:
			g1_tap0=5;
			g1_tap1=6;
			break;
			
			case 14:
			g1_tap0=6;
			g1_tap1=7;
			break;
			
			case 15:
			g1_tap0=7;
			g1_tap1=8;
			break;
			
			case 16:
			g1_tap0=8;
			g1_tap1=9;
			break;
			
			case 17:
			g1_tap0=0;
			g1_tap1=3;
			break;
			
			case 18:
			g1_tap0=1;
			g1_tap1=4;
			break;
			
			case 19:
			g1_tap0=2;
			g1_tap1=5;
			break;
			
			case 20:
			g1_tap0=3;
			g1_tap1=6;
			break;
			
			case 21:
			g1_tap0=4;
			g1_tap1=7;
			break;
			
			case 22:
			g1_tap0=5;
			g1_tap1=8;
			break;
			
			case 23:
			g1_tap0=0;
			g1_tap1=2;
			break;
			
			case 24:
			g1_tap0=3;
			g1_tap1=5;
			break;
			
			case 25:
			g1_tap0=4;
			g1_tap1=6;
			break;
			
			case 26:
			g1_tap0=5;
			g1_tap1=7;
			break;
			
			case 27:
			g1_tap0=6;
			g1_tap1=8;
			break;
			
			case 28:
			g1_tap0=7;
			g1_tap1=9;
			break;
			
			case 29:
			g1_tap0=0;
			g1_tap1=5;
			break;
			
			case 30:
			g1_tap0=1;
			g1_tap1=6;
			break;
			
			case 31:
			g1_tap0=2;
			g1_tap1=7;
			break;
			
			case 32:
			g1_tap0=3;
			g1_tap1=8;
			break;
	
			default: // default
			code_selection=1;
			g1_tap0=1;
			g1_tap1=5;
		}

		// calculate our code LUT
		
		char g0[10] = {1,1,1,1,1,1,1,1,1,1};
		char g1[10] = {1,1,1,1,1,1,1,1,1,1};
		char g1_out = 0;
		
		
		int i = 0, j = 0;
		for(i = 0 ; i < 1023 ; ++i)
		{
			g1_out = g1[g1_tap0] ^ g1[g1_tap1];
		
			code_LUT[i] = (gr_complex) (2.0 * ((g1_out ^ g0[9]) - 0.5));
			
			char g0_state0_next = g0[2] ^ g0[9];
			char g1_state0_next = g1[1] ^ g1[2] ^ g1[5]
					 			^ g1[7] ^ g1[8] ^ g1[9];
			
			for(j = 9; j >= 0; --j)
			{
				if(j == 0)
				{
					g0[j] = g0_state0_next;
					g1[j] = g1_state0_next;
				}
				else
				{
					g0[j] = g0[j-1];
					g1[j] = g1[j-1];
				}
			}
			
			
			
		}

	}

	void gps_codegen_c_impl::set_sample_rate(float samplerate)
	{
		// fixed point with 40 bits fractional part
		code_phase_increment = (unsigned long)(( 1023e3 / samplerate) * (float)(1L << 40));  

	}

    void
    gps_codegen_c_impl::set_datamode(int mode)
    {
		datagen_mode = mode;
    }

    void
	gps_codegen_c_impl::advance_random_data()
    {
		// 22-bit m-sequence LFSR
		char new_bit = (random_data[21] ^ random_data[20]);

		for(int i = 21; i >= 0; --i)
		{
			if(i == 0)
			{
				random_data[i] = new_bit & 1;
			}
			else
			{
				random_data[i] = random_data[i-1];
			}

		}

    }

    int
    gps_codegen_c_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        gr_complex *out = (gr_complex *) output_items[0];

        // Do <the thing>
		int i;
		int code_sel = 0;

		for(i = 0; i < noutput_items; ++i)
		{
			code_sel = (int) ((code_phase + (1L << 39)) >> 40 );

			if(datagen_mode == 1)
				out[i] = ( (gr_complex)(((float)random_data[21] - 0.5f) * 2.0f) ) * code_LUT[code_sel];
			else
				out[i] = code_LUT[code_sel];

			code_phase += code_phase_increment;

			long limit = (1023L << 40);

			if( code_phase >=  limit)
			{
				if(datagen_mode == 1)
				{
					if(data_counter == 20)
					{
						data_counter = 1;
						advance_random_data();
					}
					else
						data_counter++;
				}

				code_phase -= (1023L << 40);

			}
		}

        // Tell runtime system how many output items we produced.
        return noutput_items;
    }

  } /* namespace gps */
} /* namespace gr */

