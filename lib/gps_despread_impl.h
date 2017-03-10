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

#ifndef INCLUDED_GPS_GPS_DESPREAD_IMPL_H
#define INCLUDED_GPS_GPS_DESPREAD_IMPL_H

#define DEBUG_OUT
#define MAX_SEARCH_THREADS 4


#define FREQ_SAMPLING_RATE 4

#include <gps/gps_despread.h>
#include <gnuradio/fft/fft.h>
#include <stdio.h>


typedef enum
{
	state_start_search,
	state_wait_search,
	state_track
} fsm_state;

typedef struct 
{
	// inputs
	gr_complex *data;
	gr_complex *code_LUT;
	int osr;
	int data_len;
	int freq_search_Nsteps;

	// outputs
	int best_delay;
	float best_freq;
	float snr;

	int done;
	int running;
	
} search_data_t;

namespace gr {
  namespace gps {

    class gps_despread_impl : public gps_despread
    {
	private:

		// code generator
		gr_complex code_LUT[1023][32];
		int code_selection;
		int code_counter;
		int increment_counter;
		 
		int osr_int;
		 
		int delay_selection;
		 
		 // search
		int freq_search_Nsteps;

		search_data_t search_data;
		pthread_t search_thread;
		int search_acq_counter;

		int search_avg_selection;

		// doppler correction
		float nco_freq;
		float nco_freq_fixed;
		float nco_phase;
	
		// code tracking
		gr_complex track_integrator[5];
		int track_counter;
		int track_mode;
		int lockdet_counter; // PLL lock detection

		// freq tracking
		gr_complex freq_corr_integrator[FREQ_SAMPLING_RATE];
		gr_complex freq_corr_integrator_d[FREQ_SAMPLING_RATE]; 
		gr_complex freq_corr_integrator_filtered[FREQ_SAMPLING_RATE];
		gr_complex freq_corr_integrator_filtered_d;
		int fix_done;
			
		int freq_Nsamples; // number of frequency samples per code period

		float phase_error;
		float lf_int;
		float lf_zero_d;
		float lf_pole_d;
		float freqerror_trunc_d;

		fsm_state fsm;

		void generate_codes();

		void track(const gr_complex *, gr_complex *, int, int &);
		void update_pll(float);

	public:
		gps_despread_impl(int, int);
		~gps_despread_impl();

		void set_code(int);
		int code(void) const;
	  
		void set_osr(int);
		int osr(void) const;
	  
		void set_freq(float);

		void set_delay(int);
		int delay(void) const;
	  
		void start_search();
	
		void forecast(int, gr_vector_int &);
      // Where all the action really happens
		int general_work(int, gr_vector_int &, gr_vector_const_void_star &, gr_vector_void_star &);
    };

  } // namespace gps
} // namespace gr

#endif /* INCLUDED_GPS_GPS_DESPREAD_IMPL_H */

