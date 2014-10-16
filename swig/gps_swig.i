/* -*- c++ -*- */

#define GPS_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "gps_swig_doc.i"

%{
#include "gps/gps_despread.h"
#include "gps/gps_codegen_c.h"
#include "gps/gps_navdata.h"
%}


%include "gps/gps_despread.h"
GR_SWIG_BLOCK_MAGIC2(gps, gps_despread);
%include "gps/gps_codegen_c.h"
GR_SWIG_BLOCK_MAGIC2(gps, gps_codegen_c);

%include "gps/gps_navdata.h"
GR_SWIG_BLOCK_MAGIC2(gps, gps_navdata);
