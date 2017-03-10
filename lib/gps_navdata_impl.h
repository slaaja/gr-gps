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

#ifndef INCLUDED_GPS_GPS_NAVDATA_IMPL_H
#define INCLUDED_GPS_GPS_NAVDATA_IMPL_H

#include <gps/gps_navdata.h>
#include <stdio.h>


typedef enum
{
	search_preamble,
	read_tlm_word,
	read_how_word,
	read_subframe
} frame_state;

namespace gr {
  namespace gps {

    class gps_navdata_impl : public gps_navdata
    {
     private:

      	frame_state fsm;
		unsigned char polarity; // 0 = right way around, 1 = upside down
		int bit_counter;
		int word_counter;
		unsigned char word[30];
		unsigned char last2[2];

	int check_preamble(unsigned char *, unsigned char &);
	int check_checksum(unsigned char *, unsigned char *);

     public:
      gps_navdata_impl();
      ~gps_navdata_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace gps
} // namespace gr

#endif /* INCLUDED_GPS_GPS_NAVDATA_IMPL_H */

