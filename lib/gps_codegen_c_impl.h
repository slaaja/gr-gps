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

#ifndef INCLUDED_GPS_GPS_CODEGEN_C_IMPL_H
#define INCLUDED_GPS_GPS_CODEGEN_C_IMPL_H

#include <gps/gps_codegen_c.h>

namespace gr {
  namespace gps {

    class gps_codegen_c_impl : public gps_codegen_c
    {
     private:

		gr_complex code_LUT[1023];
		long code_phase;
		long code_phase_increment;

		int code_selection;
		int datagen_mode;
		
		char random_data[22];
		int data_counter;
		void advance_random_data();

     public:
		gps_codegen_c_impl(float, int, int);
		~gps_codegen_c_impl();

		void set_code(int);
		void set_sample_rate(float);

		void set_datamode(int);

      // Where all the action really happens
		int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace gps
} // namespace gr

#endif /* INCLUDED_GPS_GPS_CODEGEN_C_IMPL_H */

