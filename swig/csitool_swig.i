/* -*- c++ -*- */

#define CSITOOL_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "csitool_swig_doc.i"

%{
#include "csitool/ofdm_sync_and_decode_header.h"
#include "csitool/ofdm_sync_and_decode_header_history.h"
#include "csitool/ofdm_equalize_symbols.h"
#include "csitool/ofdm_equalize_symbol_simpledfe.h"
%}


%include "csitool/ofdm_sync_and_decode_header.h"
GR_SWIG_BLOCK_MAGIC2(csitool, ofdm_sync_and_decode_header);
%include "csitool/ofdm_sync_and_decode_header_history.h"
GR_SWIG_BLOCK_MAGIC2(csitool, ofdm_sync_and_decode_header_history);
%include "csitool/ofdm_equalize_symbols.h"
GR_SWIG_BLOCK_MAGIC2(csitool, ofdm_equalize_symbols);
%include "csitool/ofdm_equalize_symbol_simpledfe.h"
GR_SWIG_BLOCK_MAGIC2(csitool, ofdm_equalize_symbol_simpledfe);
