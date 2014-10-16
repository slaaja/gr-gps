/* -*- c++ -*- */
/* 
 * Copyright 2014 Samu Laaja.
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

#include <pthread.h>
#include <string.h>


#include <gnuradio/io_signature.h>
#include <gnuradio/blocks/control_loop.h>
#include <gnuradio/expj.h>
#include <gnuradio/sincos.h>
#include <gnuradio/math.h>
#include "gps_despread_impl.h"

#define DEBUG_OUT




namespace gr {
  namespace gps {

	void max_abs(gr_complex *b, int l, float &a, int &ix, float &snr)
	{
		unsigned long i = 0;

		snr = 0;
		a = 0;
		ix = 0;
	
		for(i = 0;i < l; ++i)
		{
			snr += abs(b[i]);	
			if(abs(b[i]) > a)
			{
				a = abs(b[i]);
				ix = i;
			}
		}
	
		snr = a * (l-1) / (snr - a);
	
	}

	void calculate_product(gr_complex *in, gr_complex *c, gr_complex *out, int l, int shift )
	{
		int i = 0;
	
		for(i = 0; i < l; ++i)
		{
			int code_ix = i - shift;
		
			while(code_ix < 0)
			{
				code_ix = code_ix + l;
			}
		
			while(code_ix >= l)
			{
				code_ix = code_ix - l;
			}
		
			out[i] = in[i] * c[code_ix];		
		}
	
	}

	void *search_worker(void *s)
	{


		int d = 0;
		int f = 0;
		int i;

		search_data_t *search_data = (search_data_t *) s;

		search_data->best_delay = 0;
		search_data->snr = -1;
		search_data->best_freq = 0;	
		search_data->running = 1;

		int data_len = search_data->data_len;
		int osr_int = search_data->osr;
		int freq_search_Nsteps = search_data->freq_search_Nsteps;

		printf("Searching...\n");

		gr::fft::fft_complex *fft_c = new gr::fft::fft_complex(data_len, 1, 4);
		gr::fft::fft_complex *ifft_c = new gr::fft::fft_complex(data_len, 0, 4);	

		gr_complex *input_buffer = fft_c->get_inbuf();
		gr_complex *output_buffer = fft_c->get_outbuf();
			
		//
		// calculate fft of input data and spreading code
		//
		gr_complex *code_fft_conj = new gr_complex[data_len];
		gr_complex *input_data_fft = new gr_complex[data_len];

		for(i = 0; i < data_len; ++i)
		{

			input_buffer[i] = search_data->code_LUT[((int)(i/osr_int)) % 1023];
		}

		fft_c->execute();

			
		for(i = 0; i < data_len; ++i)
		{
			code_fft_conj[i] = conj(output_buffer[i]);
			input_buffer[i] = search_data->data[i];
		}

			
		fft_c->execute();

			
		for(i = 0; i < data_len ; ++i)
		{
			input_data_fft[i] = output_buffer[i];
		}
			
			
		float freq_step = 1000.0f;

		
	
		input_buffer = ifft_c->get_inbuf();
		output_buffer = ifft_c->get_outbuf();

		float peak;
		float best_power = -1;

		for(i = -(freq_search_Nsteps >> 1) ; i < (freq_search_Nsteps >> 1) ; ++i)
		{
			calculate_product(input_data_fft, code_fft_conj, input_buffer, data_len, i);
					
			ifft_c->execute();
					
			int peak_offset;
			float snr;
					
			max_abs(output_buffer, data_len, peak, peak_offset, snr);

					
			printf("i: %d, peak: %f , peak_offset: %d, snr: %f\n", i, peak, peak_offset, snr);
			if(peak > best_power)
			{				
				best_power = peak;
				search_data->best_freq = i * freq_step;
				search_data->best_delay = peak_offset;
				search_data->snr = snr;
			}
		}


		printf("search:: best_freq: %f, best_delay: %d\n", search_data->best_freq, osr_int*1023-search_data->best_delay );
		
		delete fft_c;
		delete ifft_c;		

		delete code_fft_conj;
		delete input_data_fft;
		
		search_data->done = 1;
		search_data->running = 0;

		pthread_exit(NULL);
	}


    gps_despread::sptr
    gps_despread::make(int code_sel, int osr_in)
    {
      return gnuradio::get_initial_sptr
        (new gps_despread_impl(code_sel, osr_in));
    }

    /*
     * The private constructor
     */
    gps_despread_impl::gps_despread_impl(int code_sel, int osr_in)
      : gr::block("gps_despread",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)) )
    {
		// reset delay counter
		delay_selection = 0;
			
		// fill the code lut
		generate_codes();
		set_code(code_sel);

		code_counter = 0;
		
		osr_int = osr_in;

		// search params
		freq_search_Nsteps = 21;
		int i;
		search_data.running = 0;
		search_data.done = 0;
		search_acq_counter = 0;
		
		// state machine
		fsm = state_start_search;

		// code tracking		
		for(i = 0; i < 5 ; ++i)
			track_integrator[i] = 0;
		track_counter = 0;
		track_mode = 1;
		lockdet_counter = 0;


		// buffers for FFT fine frequency correction

		for(int i = 0; i < freq_Nsamples ; ++i)
		{
			freq_corr_integrator[i] = 0;
			freq_corr_integrator_d[i] = 0;
			freq_corr_integrator_filtered[i] = 0;
		}

		freq_corr_integrator_filtered_d = 0;
		freq_Nsamples = FREQ_SAMPLING_RATE;
		fix_done = 0;



		search_avg_selection = 0;
		
		// frequency tracking loop
		nco_freq = 0;
		lf_int = 0;
		lf_zero_d = 0;
		lf_pole_d = 0;
		freqerror_trunc_d = 0;

		
		nco_phase = 0;
    }
	
	

    /*
     * Our virtual destructor.
     */
    gps_despread_impl::~gps_despread_impl()
    {

    }
	
	
	void gps_despread_impl::set_delay(int d)
	{

		if(d >= 0)
			delay_selection = d % (1023 * osr_int);
		else
			delay_selection = osr_int * 1023 + (d % (1023 * osr_int));

		//printf("## Delay changed to %d\n", delay_selection);
	}
	
	int gps_despread_impl::delay() const
	{
		
		return delay_selection;
	}
	
	int gps_despread_impl::osr() const
	{
		return osr_int;
		
	}
	
	void gps_despread_impl::set_osr(int o)
	{
		//this->osr_int = o;
		
	}
	
	void gps_despread_impl::set_freq(float f)
	{
		//printf("## NCO frequency fixed part set to: %f\n", f);
		nco_freq_fixed = 2 * M_PI * f / (1023e3 * osr_int);
	}

	int gps_despread_impl::code() const 
	{
		return code_selection;
		
	}

	void gps_despread_impl::set_code(int c)
	{
		if(c < 32 && c > 0)
		{
			printf("## Code set to: %d\n", c);
			code_selection = c;
		}
		else
			printf("ERROR: Unknown code selection: %d\n", c);
	}

	void
	gps_despread_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
	{
		unsigned ninputs = ninput_items_required.size();

		// we need at least a whole sequence
		for(int i = 0; i < ninputs; ++i)
		{
			ninput_items_required[i] = noutput_items * 1023 * osr_int;
		}
	}


	void
	gps_despread_impl::update_pll(float freqerror)
	{
		float freqerror_trunc = freqerror; 

		while(freqerror_trunc > M_PI/2)
			freqerror_trunc -= M_PI;
		while(freqerror_trunc < -M_PI/2)
			freqerror_trunc += M_PI;

		phase_error += freqerror_trunc;
		
		float fix = 0;//freqerror_trunc_d + freqerror_trunc;

		// remove navigation data from phase tracking:
		// ignore fast > pi/2 radian phase jumps
		if(!fix_done)
		{
			if(freqerror_trunc+freqerror_trunc_d > M_PI/2 )
			{
				fix = -M_PI;
				fix_done = 1;
				track_counter = 0; // align code tracking with input data
			}
			else if(freqerror_trunc+freqerror_trunc_d < -M_PI/2 )
			{
				fix = M_PI;
				fix_done = 1;
				track_counter = 0;  // align code tracking with input data
			}
	
		}
		else if(abs(freqerror_trunc+freqerror_trunc_d) < M_PI/2 )
			fix_done = 0;
	
		phase_error += fix;

		freqerror_trunc_d = freqerror_trunc;

		// two pole, one zero loop filter

		// Fs = 2kHz (PhaM = 55deg)
		//float Kp = 0.905082;
		//float Kz = 0.990143;

		// Fs = 4kHz (PhaM = 55deg)
		float Kp = 0.95139;
		float Kz = 0.99506;

		lf_int = lf_int + (lf_pole_d - lf_zero_d + phase_error)/512.0f / 2000.0; // type-II mode
		//lf_int = (lf_pole_d - lf_zero_d + phase_error)/512.0f / 2000.0; // type-I mode
		lf_pole_d =  Kp * (lf_pole_d - lf_zero_d + phase_error);		 
		lf_zero_d = Kz * phase_error; // type-II mode
		//lf_zero_d = 0; // type-I mode		

		nco_freq = lf_int;

		FILE *fid = fopen("/home/samu/testi_out.txt", "a+");
		fprintf(fid, "%f,%f,%f,%d,\n", (nco_freq + nco_freq_fixed) * 1023e3 * osr_int / 2 / M_PI, phase_error, fix, fix_done);
		fclose(fid);

		// lock detection
		if(lockdet_counter == 499)
		{
			lockdet_counter = 0;
			
			if(abs(nco_freq * 1023e3 * osr_int / 2 / M_PI) > 4000)
			{
				printf("# Frequency correction PLL unlocked.\n");
				fsm = state_start_search;
			}

		}
		else
		{
			lockdet_counter++;
		}

		//printf("nco_freq: %8.4f\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b", (nco_freq + nco_freq_fixed) * 1023e3 * osr_int / 2 / M_PI);
	}

	void
	gps_despread_impl::track(const gr_complex *in, gr_complex *out, int noutput_items, int &noutputs)
	{
		int consumed;
		noutputs = 0;

		for(int i = 0; i < noutput_items*1023*osr_int; ++i)
		{
			int code_index = ((code_counter + delay_selection) / osr_int) % 1023;
			
			if (code_index < 0)
				code_index += 1023;
			
			nco_phase = nco_phase + nco_freq + nco_freq_fixed;
			
			while(nco_phase > 2*M_PI)
				nco_phase -= 2*M_PI;
			while(nco_phase < -2*M_PI)
				nco_phase += 2*M_PI;
			
			gr_complex nco = gr_expj(-nco_phase);

			gr_complex s = nco * in[i] * code_LUT[code_index][code_selection - 1];
			out[noutputs] += s / (float)osr_int ;
			
			// code rate correction
			for(int j = -2; j < 3 ; ++j)
			{
				int code_index_track = ((code_counter + delay_selection + j) / osr_int) % 1023;

				if(code_index_track < 0)
					code_index_track += 1023;
					

				track_integrator[j + 2] += nco * in[i] * code_LUT[code_index_track][code_selection - 1]; 
			}
		
			// LO freq error correction
			//
			// [despread]---->[lowpass]---->[freq sampling]			
			
			float a1 = 0.996933745203311f;
			float b0 = 0.00153312739834435f;
			float b1 = 0.00153312739834435f;
			//float a1 = 0.857139174832348;
			//float b0 = 0.0714304125838262;
			//float b1 = 0.0714304125838262;

			for(int j = 0; j < freq_Nsamples; ++j)
			{
				int offset_index = (j + 1) * (osr_int * 1023)/freq_Nsamples - 1;

				if( (code_counter + delay_selection) == offset_index) 
				{
					float freq_error = arg(freq_corr_integrator_filtered[j]) - arg(freq_corr_integrator_filtered_d) ;

					freq_corr_integrator_filtered_d = freq_corr_integrator_filtered[j];
				
					update_pll(freq_error);

					freq_corr_integrator[j] = 0;
					freq_corr_integrator_d[j] = 0;
					freq_corr_integrator_filtered[j] = 0;
				}
				else
				{
					freq_corr_integrator[j] += s;
					freq_corr_integrator_filtered[j] = freq_corr_integrator_filtered[j] * a1 
													 + freq_corr_integrator[j] * b0 + freq_corr_integrator_d[j] * b1;
					freq_corr_integrator_d[j] = freq_corr_integrator[j];


				}
			}


				
			// code wrapped around -> sample ready
			if( (code_counter + delay_selection) == (osr_int * 1023 - 1) )
			{
				// Integrate 20 periods: 1kHz / 20 = 50Hz
				// Code tracking PLL automatically aligns the integrator
				if(track_counter == 19)
				{	
					noutputs++;
					out[noutputs] = 0;

					track_counter = 0;
				}
				else
				{
					track_counter++;
				}

				// code tracking
				if(track_counter % 4 == 0)				
				{
					int best_ix;
					float best_power;
					float avg;

					max_abs(track_integrator, 5, best_power, best_ix, avg);

					//printf("code track:: [ ");
					//for(int j = 0; j < 5 ; ++j)
					//	printf("%5.1f ", abs(track_integrator[j]));
					//printf("]\n");

					if(best_ix - 2 < 0)
						set_delay(delay_selection - 1);
					else if(best_ix - 2 > 0)
						set_delay(delay_selection + 1);

					for(int j = 0; j < 5 ; ++j)
						track_integrator[j] = 0; 

				}
				
			}		
			
			// This counter is our reference point for the delay selection.
			//
			// Even though search consumes the samples it uses for searching,
			// this counter remains in sync because the number of
			// samples used for search is always exactly one full rotation
			// of this counter.		
			if(code_counter == (1023 * osr_int) - 1)
			{
				code_counter = 0;
			}
			else
				code_counter++;

			consumed++;
		}

		consume_each(noutput_items*1023*osr_int);
	}

    int
    gps_despread_impl::general_work(int noutput_items, 
			  gr_vector_int &ninput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out = (gr_complex *) output_items[0];

		int noutputs = 0;
		
		out[noutputs] = 0; // reset integrator

		switch(fsm)
		{
			case state_track:
				track(in, out, noutput_items, noutputs);
				break;
		


			case state_wait_search:
				// keep the sample counter in sync while waiting for results.
				// consume samples to 
				for(int i = 0; i < noutput_items*1023*osr_int; ++i)
				{
					if(code_counter == (1023 * osr_int) - 1)
					{
						code_counter = 0;
					}
					else
						code_counter++;

				}	
				consume_each( noutput_items*1023*osr_int	);

				if(search_data.done)
				{
					// free the data buffer
					delete search_data.data;

					float threshold = 4.0f;
					
					if(abs(search_data.snr) > threshold)	
					{
						// satellite found. start tracking

						set_freq(search_data.best_freq);
						set_delay(osr_int * 1023 - search_data.best_delay);
						//set_delay(search_data.best_delay);


						fsm = state_track;
					}
					else
					{
						// nothing found, retry search

						fsm = state_start_search;
					}
					
				}

				break;
					

			case state_start_search:
			default:


				// launch search 
				if(!search_data.running)
				{

					if(search_acq_counter == 0)
					{
						// reset freq tracking PLL
						phase_error = 0;
						lf_int = 0;
						lf_pole_d = 0;
						nco_freq = 0;


						for(int i = 0; i < freq_Nsamples ; ++i)
						{
							freq_corr_integrator[i] = 0;
							freq_corr_integrator_d[i] = 0;
							freq_corr_integrator_filtered[i] = 0;
						}
						freq_corr_integrator_filtered_d = 0;
				

						// reset code tracking integrators
						for(int i = 0; i < 5 ; ++i)
							track_integrator[i] = 0;
						track_counter = 0;
						track_mode = 1;
						lockdet_counter = 0;

						search_data.done = 0;
	
					
						search_data.data = new gr_complex[1023 * osr_int];
						for(int i = 0; i < 1023*osr_int ; ++i)
							search_data.data[i] = in[i];

						consume_each(1023 * osr_int);

					}
					else
					{
						for(int i = 0; i < 1023*osr_int ; ++i)
							search_data.data[i] = in[i];
						consume_each(1023 * osr_int);
						
					}

					printf("%d ", search_acq_counter);

					if(++search_acq_counter == 1)
					{	
						printf("\n");

						search_data.code_LUT = new gr_complex[1023];
						for(int i = 0; i < 1023 ; ++i)
							search_data.code_LUT[i] = code_LUT[i][code_selection - 1];

						search_data.osr = osr_int;
						search_data.data_len = 1023 * osr_int;
				
						search_data.freq_search_Nsteps = freq_search_Nsteps;

						pthread_create(&search_thread, NULL, search_worker, (void *) &search_data);
	
						search_acq_counter = 0;
						fsm = state_wait_search;
					}

				}
				else
				{
					fsm = state_wait_search;
				}

				break;		
		}
		
        return noutputs;
    }

	
	void gps_despread_impl::generate_codes()
	{
		int g1_tap0 = 0;
		int g1_tap1 = 0;
		int c;

		for(c = 0; c < 32 ; ++c)
		{		
			switch(c + 1) 
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
			}
		

			// calculate our code LUT
		
			char g0[10] = {1,1,1,1,1,1,1,1,1,1};
			char g1[10] = {1,1,1,1,1,1,1,1,1,1};
			char g1_out = 0;
		
		
			int i = 0, j = 0;
			for(i = 0 ; i < 1023 ; ++i)
			{
				g1_out = g1[g1_tap0] ^ g1[g1_tap1];
			
			
				code_LUT[i][c] = (gr_complex) (2.0 * ((g1_out ^ g0[9]) - 0.5));
				//printf("%d ", g1_out ^ g0[9]);
			
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

	}

  } /* namespace gps */
} /* namespace gr */

