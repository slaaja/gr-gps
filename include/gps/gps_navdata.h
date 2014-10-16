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


#ifndef INCLUDED_GPS_GPS_NAVDATA_H
#define INCLUDED_GPS_GPS_NAVDATA_H

#include <gps/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace gps {

    /*!
     * \brief <+description of block+>
     * \ingroup gps
     *
     */
    class GPS_API gps_navdata : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<gps_navdata> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of gps::gps_navdata.
       *
       * To avoid accidental use of raw pointers, gps::gps_navdata's
       * constructor is in a private implementation
       * class. gps::gps_navdata::make is the public interface for
       * creating new instances.
       */
      static sptr make();
    };

  } // namespace gps
} // namespace gr

#endif /* INCLUDED_GPS_GPS_NAVDATA_H */

