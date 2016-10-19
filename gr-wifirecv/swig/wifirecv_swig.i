/* -*- c++ -*- */

#define WIFIRECV_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "wifirecv_swig_doc.i"

%{
#include "wifirecv/ofdm_sync_parse_header.h"
#include "wifirecv/ofdm_equalize_symbols.h"
#include "wifirecv/ofdm_parse_payload.h"
%}

%include "wifirecv/ofdm_sync_parse_header.h"
GR_SWIG_BLOCK_MAGIC2(wifirecv, ofdm_sync_parse_header);
%include "wifirecv/ofdm_equalize_symbols.h"
GR_SWIG_BLOCK_MAGIC2(wifirecv, ofdm_equalize_symbols);
%include "wifirecv/ofdm_parse_payload.h"
GR_SWIG_BLOCK_MAGIC2(wifirecv, ofdm_parse_payload);
