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

#ifndef INCLUDED_WIFIRECV_OFDM_SYNC_PARSE_HEADER_IMPL_H
#define INCLUDED_WIFIRECV_OFDM_SYNC_PARSE_HEADER_IMPL_H

#include <wifirecv/ofdm_sync_parse_header.h>
#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/fft/fft.h>
#include <volk/volk.h>

#include <list>
#include <tuple>
#include <itpp/itcomm.h>

namespace gr {
  namespace wifirecv {

    class ofdm_sync_parse_header_impl : public ofdm_sync_parse_header
    {
     private:
      // Nothing to declare in this block.
      //! Signal detection threshold
      const double d_threshold;
      //! Min detection plateau
      const unsigned int d_min_plateau;
      //! Long sync window size
      const int d_sync_length;
      
      //! VOLK align byte
      int d_align;
      //! Block status
      enum {SEARCH, SYNC, PARSE, COPY} d_state;
      //! Current plateau width
      int d_plateau;
      //! Number of octet in PSDU
      int d_n_psdu;
      //! Data rate
      int d_rate;
      //! Header buffer
      gr_complex *d_header_time_buf;
      //! Header buffer
      gr_complex *d_header_freq_buf;
      //! Header code bits
      double *d_header_code_bit;
      //! Header data bits
      itpp::bvec d_header_data_bit;
      //! Deinterleave buffer
      double *d_inter_buf;

      //! Number of samples of current detected packet
      int d_to_copy;
      //! Current offset from the start of long sync
      int d_offset;
      //! Absolute offset from the start of the packet;
      int d_abs_offset;
      //! Offset between the position of 1st data sym and start of the long sync
      int d_frame_start;
      //! Corse frequency estimation
      gr_complex d_coarse_freq_est;
      //! Fine frequency estimation
      gr_complex d_fine_freq_est;
      //! Final phase shift per sample
      float d_phase_shift;
      //! Temporary history of correlation of long sync
      gr_complex *d_correlation;
      //! Temporary history of corse frequency estimation
      gr_complex *d_coarse_freq_est_buf;
      //! History of correlation of long sync
      std::list<std::pair<double, int>> d_cor;
      //! Long sync correlator
      gr::filter::kernel::fir_filter_ccc *d_fir;
      //! Channel estimation
      gr_complex* d_cfr;
      //! long sync estimation buffer;
      gr_complex* d_long_sync_buf;
      //! FFT executor
      fft::fft_complex *d_fft;
      //! Timing offset
      int d_timing_off;

      // TODO: make these parameters editable
      //! FFT length
      const static int d_fft_length = 64;
      //! Cyclic prefix length
      const static int d_cyclic_prefix_length = 16;
      //! Short sync length
      const static int d_short_sync_length = 16;
      //! Long sync length
      const static int d_long_sync_length = 64;
      //! Number of data subcarrier
      const static int d_n_data_subcarrier = 48;
      //! long sync in time field
      const static std::vector<gr_complex> d_long_sync_time;
      //! long sync in freq field unaligned
      const static std::vector<float> d_long_sync_freq;
      //! Interleave pattern
      const static std::vector<int> d_interleave_pattern;

     public:
      ofdm_sync_parse_header_impl(double threshold, unsigned int min_plateau, int timing_off, int sync_length = 320);
      ~ofdm_sync_parse_header_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);

      void search_frame_start();
      void fine_freq_sync(const gr_complex *in);
      void channel_estimation(const gr_complex *in);
      void header_deinterleave();
      void header_decode();
      bool header_parse();
    };
  } // namespace wifirecv
} // namespace gr

#endif /* INCLUDED_WIFIRECV_OFDM_SYNC_PARSE_HEADER_IMPL_H */

