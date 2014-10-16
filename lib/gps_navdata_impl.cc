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

#include <gnuradio/io_signature.h>
#include "gps_navdata_impl.h"

namespace gr {
  namespace gps {

    gps_navdata::sptr
    gps_navdata::make()
    {
      return gnuradio::get_initial_sptr
        (new gps_navdata_impl());
    }

    /*
     * The private constructor
     */
    gps_navdata_impl::gps_navdata_impl()
      : gr::block("gps_navdata",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(1, 1, sizeof(char)))
    {
		fsm = search_preamble;

		for(int i = 0; i < 30 ; ++i)
			word[i] = 0;

		last2[0] = 0;
		last2[1] = 0;

	}

    /*
     * Our virtual destructor.
     */
    gps_navdata_impl::~gps_navdata_impl()
    {
    }

    void
    gps_navdata_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      
		ninput_items_required[0] = noutput_items;
    }

	int
	gps_navdata_impl::check_preamble(unsigned char *d, unsigned char &p)
	{
		unsigned char preamble[8] = {1, 0, 0, 0, 1, 0, 1, 1};

		p = 0;

		int ok = 1;
		for(int i = 0; i < 8 ; ++i)
		{
			if(d[i] != preamble[i])
				ok = 0;
		}	

		if(ok)
		{
			p = 0;
			return 1;
		}

		ok = 1;
		for(int i = 0; i < 8 ; ++i)
		{
			if(d[i] != (preamble[i] ^ 1))
				ok = 0;
		}

		if(ok)
		{
			p = 1;
			return 1;
		}

		return 0;
			
	}

	int
	gps_navdata_impl::check_checksum(unsigned char *d, unsigned char *D2)
	{
		unsigned char D[6];

		//for(int i = 0; i < 24 ; ++i)
		//{
		//	D[i] = d[i] ^ d[29];
		//}
		
		D[0] = D2[0] ^ d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[ 9] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[16] ^ d[17] ^ d[19] ^ d[22];
		D[1] = D2[1] ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[17] ^ d[18] ^ d[20] ^ d[23];
		D[2] = D2[0] ^ d[0] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[ 7] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[21];
		D[3] = D2[1] ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[ 8] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[19] ^ d[20] ^ d[22];
		D[4] = D2[1] ^ d[0] ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[ 8] ^ d[ 9] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[23];
		D[5] = D2[1] ^ d[2] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[ 9] ^ d[10] ^ d[12] ^ d[14] ^ d[18] ^ d[21] ^ d[22] ^ d[23];

		int ok = 1;

		for(int i = 0; i < 6; ++i)
		{
			if(D[i] != d[i + 24])
				ok = 0;
		}

		return ok;

	}

    int
    gps_navdata_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const char *in = (const char *) input_items[0];
        char*out = (char *) output_items[0];

		

        // Do <+signal processing+>
        // Tell runtime system how many input items we consumed on
        // each input stream.
		for(int i = 0; i < noutput_items;++i)
		{

	
			switch(fsm)
			{

				case search_preamble:
					last2[0] = last2[1];
					last2[1] = word[0];
					
					for(int j = 0; j < 7 ; ++j)
						word[j] = word[j+1];

					word[7] = in[i];
		
					if(check_preamble(word, polarity))
					{
						// preamble found

						printf("Preamble found. \n");

						bit_counter = 8;
						fsm = read_tlm_word;
					}

					break;

				case read_tlm_word:

					word[bit_counter++] = in[i] ^ polarity;
					
					if(bit_counter == 30)
					{
						if(!check_preamble(word, polarity))
						{	
							printf("TLM word has wrong preamble.\n");
		

							fsm = search_preamble;

							last2[1] = word[29];
							last2[0] = word[28];
						} 
						else if(!check_checksum(word, last2))
						{
							printf("TLM word checksum failed.\n");

							last2[1] = word[29];
							last2[0] = word[28];
							fsm = search_preamble;
						}						
						else
						{
							printf("TLM word checksum OK!\n");

							last2[1] = word[29];
							last2[0] = word[28];
							fsm = read_how_word;
						}

						bit_counter = 0;
					}

					break;

				case read_how_word:
					word[bit_counter++] = in[i] ^ polarity;

					if(bit_counter == 30)
					{
						if(!check_checksum(word, last2))
						{
							printf("HOW word checksum failed.\n");

							last2[1] = word[29];
							last2[0] = word[28];
							//fsm = search_preamble;
						}						
						else
						{
							printf("HOW word checksum OK!\n");

							last2[1] = word[29];
							last2[0] = word[28];
							//fsm = read_subframe;
						}

						fsm = read_subframe;
						bit_counter = 0;
						word_counter = 2;
					}
					break;

				case read_subframe:
					word[bit_counter++] = in[i] ^ polarity;					
					
					if(bit_counter == 30)
					{
						if(!check_checksum(word, last2))
						{
							printf("Word %d checksum failed.\n", word_counter);

							last2[1] = word[29];
							last2[0] = word[28];
							//fsm = search_preamble;
						}						
						else
						{
							printf("Word %d checksum OK!\n", word_counter);

							last2[1] = word[29];
							last2[0] = word[28];
							//fsm = read_how_word;
						}

						if(++word_counter == 10)
						{
							word_counter = 0;
							fsm = read_tlm_word;
						} 

						bit_counter = 0;
					}
					break;

				default:
					fsm = search_preamble;
			}

		}

		fflush(stdout);
        consume_each (noutput_items);

        // Tell runtime system how many output items we produced.
        return noutput_items;
    }

  } /* namespace gps */
} /* namespace gr */

