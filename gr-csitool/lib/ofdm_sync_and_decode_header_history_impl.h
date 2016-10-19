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

#ifndef INCLUDED_CSITOOL_OFDM_SYNC_AND_DECODE_HEADER_HISTORY_IMPL_H
#define INCLUDED_CSITOOL_OFDM_SYNC_AND_DECODE_HEADER_HISTORY_IMPL_H

#include <csitool/ofdm_sync_and_decode_header_history.h>
#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/fft/fft.h>
#include <list>
#include <tuple>
#include <itpp/itcomm.h>

namespace gr {
  namespace csitool {

    class ofdm_sync_and_decode_header_history_impl : public ofdm_sync_and_decode_header_history
    {
     private:
      // Nothing to declare in this block.
      const double d_threshold;
      const unsigned int d_min_plateau;
      const int d_sync_length;
      const int d_timing_offset;

      const static int d_fft_length = 64;
      const static int d_cyclic_prefix_length = 16;
      const static int d_short_sync_length = 16;
      const static int d_long_sync_length = 64;
      const static int d_n_data_subcarrier = 48;
      const static std::vector<float> d_long_sync;
      const static std::vector<gr_complex> d_long_sync_taps;
      const static std::vector<int> d_interleave_pattern;

      enum{SEARCH, SYNC, PARSE, COPY} d_state;

      // Parameters for SEARCH stage
      int d_plateau;

      // Parameters for SYNC stage
      int d_frame_start;
      int d_index;
      gr_complex d_coarse_phase_shift;
      gr_complex d_fine_phase_shift;
      float d_phase_shift;
      gr_complex *d_correlation;
      gr_complex *d_coarse_phase_shift_buffer;
      std::list<std::pair<double, int>> d_correlation_index;
      gr::filter::kernel::fir_filter_ccc *d_fir;
      gr_complex d_initial_cfr[d_fft_length];
      gr_complex d_header_cfr[d_fft_length];
      fft::fft_complex *d_fft;

      // Parameters for PARSE stage
      int d_psdu;
      int d_rate;
      gr_complex d_header_time[d_fft_length];
      gr_complex d_header_freq[d_n_data_subcarrier];
      double d_header_code[d_n_data_subcarrier];
      itpp::bvec d_header_data;
      double d_header_interleave[d_n_data_subcarrier];
      float d_alpha;

      // Parameters for COPY stage
      int d_to_copy;
      int d_offset;
      int d_absolute_offset;

     public:
      ofdm_sync_and_decode_header_history_impl(double threshold, unsigned int min_plateau, int timing_offset, int sync_length);
      ~ofdm_sync_and_decode_header_history_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);

      void search_frame_start();
      void fine_freq_sync(const gr_complex *in_long_sync, bool fine);
      void channel_estimation(const gr_complex *in_long_sync);
      void header_deinterleave();
      void header_decode();
      bool header_parse();
    };

  } // namespace csitool
} // namespace gr

#endif /* INCLUDED_CSITOOL_OFDM_SYNC_AND_DECODE_HEADER_HISTORY_IMPL_H */

