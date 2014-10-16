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


#ifndef INCLUDED_GPS_GPS_DESPREAD_H
#define INCLUDED_GPS_GPS_DESPREAD_H

#include <gps/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace gps {

    /*!
     * \brief <+description of block+>
     * \ingroup gps
     *
     */
    class GPS_API gps_despread : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<gps_despread> sptr;


	  virtual void set_delay(int) = 0;
	  virtual int delay() const = 0;
	  virtual int osr() const = 0;
	  virtual void set_osr(int) = 0;
	  virtual int code() const = 0;
	  virtual void set_code(int) = 0;	


      /*!
       * \brief Return a shared_ptr to a new instance of gps::gps_despread.
       *
       * To avoid accidental use of raw pointers, gps::gps_despread's
       * constructor is in a private implementation
       * class. gps::gps_despread::make is the public interface for
       * creating new instances.
       */
      static sptr make(int, int);
    };

  } // namespace gps
} // namespace gr

#endif /* INCLUDED_GPS_GPS_DESPREAD_H */

