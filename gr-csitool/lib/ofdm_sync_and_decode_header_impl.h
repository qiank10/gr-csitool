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

#ifndef INCLUDED_CSITOOL_OFDM_SYNC_AND_DECODE_HEADER_IMPL_H
#define INCLUDED_CSITOOL_OFDM_SYNC_AND_DECODE_HEADER_IMPL_H

#include <csitool/ofdm_sync_and_decode_header.h>

#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/fft/fft.h>
#include <list>
#include <tuple>
#include <itpp/itcomm.h>

namespace gr {
  namespace csitool {

    class ofdm_sync_and_decode_header_impl : public ofdm_sync_and_decode_header
    {
     private:
      // Nothing to declare in this block.
      const double d_threshold;
      const unsigned int d_min_plateau;
      const int d_sync_length;
      const int d_timing_offset;

      const static int d_fft_length = 64;
      const static int d_long_sync_length = 64;
      const static int d_short_sync_length = 16;
      const static int d_cyclic_prefix_length = 16;
      const static int d_n_data_subcarrier = 48;
      const static std::vector<gr_complex> d_long_sync_tap;
      const static std::vector<float> d_long_sync;
      const static std::vector<int> d_interleave_pattern;

      int flag;

      // State Info.
      enum{SEARCH, ALIGN, SYNC, PARSE, COPY} d_state;

      // Used in Search stage.
      int d_plateau;

      // Used in Align stage.
      int d_frame_start;
      gr::filter::kernel::fir_filter_ccc *d_fir;
      gr_complex *d_correlation;
      gr_complex *d_coarse_phase_shift_buffer;
      std::list<std::pair<double, int>> d_correlation_offset;
      gr_complex d_coarse_phase_shift;
      int d_index;

      // Used in Sync stage
      gr_complex d_fine_phase_shift;
      float d_phase_shift;
      gr_complex d_initial_cfr[d_fft_length];
      gr_complex d_header_cfr[d_fft_length];
      fft::fft_complex *d_fft;

      // Used in Parse stage.
      gr_complex d_header_time[d_fft_length * sizeof(gr_complex)];
      gr_complex d_header_freq[d_n_data_subcarrier * sizeof(gr_complex)];
      double d_header_code[d_n_data_subcarrier * sizeof(double)];
      itpp::bvec d_header_data;
      double d_interleave[d_n_data_subcarrier * sizeof(double)];
      int d_absolute_offset;
      float d_alpha;

      // Used in copy stage.
      int d_to_copy;
      int d_offset;
      int d_rate;
      int d_psdu;
      int d_delay_left;

     public:
      ofdm_sync_and_decode_header_impl(double threshold, unsigned int min_plateau, int timing_offset, int sync_length, float alpha = 0.1);
      ~ofdm_sync_and_decode_header_impl();

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

#endif /* INCLUDED_CSITOOL_OFDM_SYNC_AND_DECODE_HEADER_IMPL_H */

