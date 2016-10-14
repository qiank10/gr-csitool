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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "ofdm_sync_and_decode_header_history_impl.h"

namespace gr {
  namespace csitool {

    ofdm_sync_and_decode_header_history::sptr
    ofdm_sync_and_decode_header_history::make(double threshold, unsigned int min_plateau, int timing_offset, int sync_length)
    {
      return gnuradio::get_initial_sptr
        (new ofdm_sync_and_decode_header_history_impl(threshold, min_plateau, timing_offset, sync_length));
    }

    /*
     * The private constructor
     */
    ofdm_sync_and_decode_header_history_impl::ofdm_sync_and_decode_header_history_impl(double threshold, unsigned int min_plateau, int timing_offset, int sync_length)
      : gr::block("ofdm_sync_and_decode_header_history",
              gr::io_signature::make2(2, 2, sizeof(gr_complex), sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_threshold(threshold),
      d_min_plateau(min_plateau),
      d_sync_length(sync_length),
      d_timing_offset(timing_offset),
      d_plateau(0),
      d_state(SEARCH),
      d_alpha(0.1)
    {
      set_tag_propagation_policy(gr::block::TPP_DONT);
      set_history(d_sync_length + 1);
      d_fir = new gr::filter::kernel::fir_filter_ccc(1, d_long_sync_taps);
      d_correlation = gr::fft::malloc_complex(8192);
      d_coarse_phase_shift_buffer = static_cast<gr_complex *>(std::malloc(sync_length * sizeof(gr_complex)));
      d_fft = new fft::fft_complex(d_long_sync_length, true, 1);
    }

    /*
     * Our virtual destructor.
     */
    ofdm_sync_and_decode_header_history_impl::~ofdm_sync_and_decode_header_history_impl()
    {
      delete d_fir;
      delete d_fft;
      gr::fft::free(d_correlation);
      free(d_coarse_phase_shift_buffer);
    }

    void
    ofdm_sync_and_decode_header_history_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      if(d_state == SYNC){
        ninput_items_required[0] = d_long_sync_length;
        ninput_items_required[1] = d_long_sync_length;
      }else if(d_state == PARSE){
        ninput_items_required[0] = d_fft_length + d_cyclic_prefix_length;
        ninput_items_required[1] = d_fft_length + d_cyclic_prefix_length;
      }else{
        ninput_items_required[0] = noutput_items;
        ninput_items_required[1] = noutput_items;
      }
      ninput_items_required[0] += d_sync_length;
      ninput_items_required[1] += d_sync_length;
    }

    int
    ofdm_sync_and_decode_header_history_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const gr_complex *in = ((const gr_complex *) input_items[0]) + d_sync_length;
        const float * in_threshold = ((const float *) input_items[1]) + d_sync_length;
        gr_complex *out = (gr_complex *) output_items[0];
        int nitems = std::min(ninput_items[0], ninput_items[1]) - d_sync_length;

        // Do <+signal processing+>
        // Tell runtime system how many input items we consumed on
        // each input stream.
        int i = 0;
        int o = 0;
        switch(d_state){
          case SEARCH:{
            for(i = 0; i < nitems; i++){
              if(in_threshold[i] > d_threshold){
                d_plateau++;
                if(d_plateau >= d_min_plateau){
                  d_state = SYNC;
                  d_plateau = 0;
                  d_index = 0;
                  d_coarse_phase_shift = gr_complex(0, 0);
                  i++;
                  break;
                }
              }else{
                d_plateau = 0;
              }
            }
            break;
          }
          case SYNC:{
            d_fir->filterN(d_correlation, in, std::min(d_sync_length, std::max(nitems - (d_long_sync_length - 1), 0)));
            while(i + d_long_sync_length - 1 < nitems){
              d_coarse_phase_shift += in[i] * conj(in[i + d_short_sync_length]);
              d_coarse_phase_shift_buffer[d_index] = d_coarse_phase_shift;
              d_correlation_index.push_back(std::pair<double, int>(abs(d_correlation[i]), d_index));
              i++;
              d_index++;
              if(d_index == d_sync_length){
                search_frame_start();
                if(d_frame_start > d_sync_length || d_frame_start - d_timing_offset < 2.5 * d_long_sync_length + 2 * d_short_sync_length){
                  d_index = 0;
                  d_coarse_phase_shift = gr_complex(0, 0);
                  i = 0;
                  d_state = SEARCH;
                }else{
                  d_frame_start -= d_timing_offset;
                  d_coarse_phase_shift = d_coarse_phase_shift_buffer[d_frame_start - int(2.5 * d_long_sync_length) - d_short_sync_length - 1];
                  const gr_complex *in_long_sync = in + i - d_sync_length + d_frame_start - 2 * d_long_sync_length;
                  fine_freq_sync(in_long_sync, false);
                  channel_estimation(in_long_sync);

                  d_frame_start = d_frame_start - d_sync_length + i;
                  if(d_frame_start <= 0){
                    i = 0;
                  }else{
                    i = d_frame_start;
                    d_frame_start = 0;
                  }
                  d_offset = 0;
                  d_state = PARSE;
                }
                break;
              }
            }
            break;
          }
          case PARSE:{
            const gr_complex *in_header = in + d_frame_start + d_cyclic_prefix_length;
            int absolute_offset = 2 * d_long_sync_length + d_cyclic_prefix_length;
            int len = (int)ceil(d_long_sync_length / 2.0);
            for(int j = 0; j < d_fft_length; j++){
              d_header_time[j] = in_header[j] * exp(gr_complex(0, (absolute_offset + j) * d_phase_shift));
            }
            memcpy(d_fft->get_inbuf(), d_header_time, d_fft_length * sizeof(gr_complex));
            d_fft->execute();
            memcpy(&d_header_time[0], &d_fft->get_outbuf()[len], sizeof(gr_complex) * (d_long_sync_length - len));
            memcpy(&d_header_time[d_fft_length - len], &d_fft->get_outbuf()[0], sizeof(gr_complex) * len);

            int k = 0;
            gr_complex phase_error = gr_complex(0, 0);
            gr_complex sym_est;
            for(int j = 0; j < d_fft_length; j++){
              if(j == 32 || j < 6 || j > 58){
                d_header_cfr[j] = 0;
                continue;
              }else if(j == 11 || j == 25 || j == 39){
                d_header_cfr[j] = d_alpha * d_initial_cfr[j] + (1 - d_alpha) * d_header_time[j];
                phase_error = phase_error + d_header_time[j] * conj(d_initial_cfr[j]);
              }else if(j == 53){
                d_header_cfr[j] = d_alpha * d_initial_cfr[j] - (1 - d_alpha) * d_header_time[j];
                phase_error = phase_error - d_header_time[j] * conj(d_initial_cfr[j]);
              }else{
                d_header_freq[k] = d_header_time[j] / d_initial_cfr[j];
                sym_est = real(d_header_freq[k]) > 0 ? 1 : -1;
                d_header_cfr[j] = d_alpha * d_initial_cfr[j] + (1 - d_alpha) * d_header_time[j] / sym_est;
                k++;
              }
            }
            for(k = 0; k < d_n_data_subcarrier; k++){
              d_header_freq[k] = d_header_freq[k] * exp(gr_complex(0, -arg(phase_error)));
              d_header_code[k] = -real(d_header_freq[k]);
            }
            header_deinterleave();
            header_decode();
            if(header_parse()){
              d_absolute_offset = 2 * d_long_sync_length + d_fft_length + d_cyclic_prefix_length;
              d_frame_start = (int)(in_header - in) + d_fft_length;
              // for(int j = 0; j < d_n_data_subcarrier; j++){
              //   std::cout << d_header_freq[j] << ",";
              // }
              // std::cout << std::endl;
              if(d_frame_start >= 0){
                i = d_frame_start;
                d_frame_start = 0;
              }else{
                i = 0;
              }
              d_offset = 0;
              d_state = COPY;
            }else{
              d_index = 0;
              d_offset = 0;
              d_coarse_phase_shift = gr_complex(0, 0);
              d_state = SEARCH;
              i = 0;
            }
            break;
          }
          case COPY:{
            const gr_complex *in_data = in + d_frame_start;
            nitems = std::min(noutput_items, (nitems - d_frame_start));
            o = 0;
            for(i = 0; i < nitems; i++){
              if(d_offset % (d_fft_length + d_cyclic_prefix_length) < d_cyclic_prefix_length){
                d_offset++;
                d_absolute_offset++;
              }else{
                if(d_offset == d_cyclic_prefix_length){
                  assert(o = 0);
                  add_item_tag(0, nitems_written(0),
                      pmt::string_to_symbol("ofdm_sample_index"),
                      pmt::from_uint64(nitems_read(0) + d_frame_start),
                      pmt::string_to_symbol(name()));
                  add_item_tag(0, nitems_written(0),
                      pmt::string_to_symbol("ofdm_data_rate"),
                      pmt::from_uint64(d_rate),
                      pmt::string_to_symbol(name()));
                  add_item_tag(0, nitems_written(0),
                    pmt::string_to_symbol("ofdm_psdu_length"),
                    pmt::from_uint64(d_psdu),
                    pmt::string_to_symbol(name()));
                  add_item_tag(0, nitems_written(0),
                    pmt::string_to_symbol("ofdm_sample_number"),
                    pmt::from_uint64(d_to_copy * d_n_data_subcarrier / (d_fft_length + d_cyclic_prefix_length)),
                    pmt::string_to_symbol(name()));
                  add_item_tag(0, nitems_written(0),
                    pmt::string_to_symbol("ofdm_channel_taps"),
                    pmt::init_c32vector(d_fft_length, d_initial_cfr),
                    pmt::string_to_symbol(name()));
                  add_item_tag(0, nitems_written(0),
                    pmt::string_to_symbol("ofdm_correlation_taps"),
                    pmt::init_c32vector(d_fft_length, d_header_cfr),
                    pmt::string_to_symbol(name()));
                }
                out[o] = in_data[i] * exp(gr_complex(0, d_absolute_offset * d_phase_shift));// (d_offset % (d_fft_length + d_cyclic_prefix_length) - d_cyclic_prefix_length) * d_phase_shift)); // d_absolute_offset * d_phase_shift));
                o++;
                d_offset++;
                d_absolute_offset++;
              }
              if(d_offset == d_to_copy){
                i++;
                break;
              }
            }
            if(d_offset < d_to_copy){
              d_frame_start = i + d_frame_start;
              if(d_frame_start >= 0){
                i = d_frame_start;
                d_frame_start = 0;
              }else{
                i = 0;
              }
            }else{
              assert(d_offset == d_to_copy);
              d_offset = 0;
              d_coarse_phase_shift = gr_complex(0, 0);
              d_state = SEARCH;
            }
            break;
          }
        }

        consume_each (i);

        // Tell runtime system how many output items we produced.
        return o;
    }

    void 
    ofdm_sync_and_decode_header_history_impl::header_deinterleave(){
      for(int j = 0; j < d_n_data_subcarrier; j++){
        d_header_interleave[j] = d_header_code[d_interleave_pattern[j]];
      }
    }

    void
    ofdm_sync_and_decode_header_history_impl::header_decode(){
      itpp::Convolutional_Code code;
      itpp::ivec generator(2);
      generator(0)=0133;
      generator(1)=0171;
      code.set_generator_polynomials(generator, 7);
      code.set_truncation_length(30);
      itpp::vec header_signal(d_header_interleave, d_n_data_subcarrier);
      code.reset();
      code.decode_tail(header_signal, d_header_data);
    }

    bool
    ofdm_sync_and_decode_header_history_impl::header_parse(){
      d_rate = 0;
      d_psdu = 0;
      bool parity = false;
      for(int j = 0; j < 17; j++){
        parity ^= (bool)d_header_data[j];
        if((j < 4) && d_header_data[j]){
          d_rate = d_rate | (1 << j);
        }else if(d_header_data[j] && j > 4 && j < 17){
          d_psdu = d_psdu | (1 << (j - 5));
        }
      }

      if(parity != (bool)d_header_data[17]){
        return false;
      }

      switch(d_rate){
        case 11:
          d_to_copy = ((int)ceil((16 + 8 * d_psdu + 6) / (double)24));
          break;
        case 15:
          d_to_copy = ((int)ceil((16 + 8 * d_psdu + 6) / (double)36));
          break;
        case 10:
          d_to_copy = ((int)ceil((16 + 8 * d_psdu + 6) / (double)48));
          break;
        case 14:
          d_to_copy = ((int)ceil((16 + 8 * d_psdu + 6) / (double)72));
          break;
        case 9:
          d_to_copy = ((int)ceil((16 + 8 * d_psdu + 6) / (double)96));
          break;
        case 13:
          d_to_copy = ((int)ceil((16 + 8 * d_psdu + 6) / (double)144));
          break;
        case 8:
          d_to_copy = ((int)ceil((16 + 8 * d_psdu + 6) / (double)192));
          break;
        case 12:
          d_to_copy = ((int)ceil((16 + 8 * d_psdu + 6) / (double)216));
          break;
        default:
          return false;
      }
      d_to_copy = d_to_copy * (d_cyclic_prefix_length + d_fft_length);
      return true;
    }

    void
    ofdm_sync_and_decode_header_history_impl::channel_estimation(const gr_complex *in_long_sync){
      int len = (int)ceil(d_long_sync_length / 2.0);
      gr_complex long_sync_buffer[d_long_sync_length];
      int absolute_offset = 0;

      for(int j = 0; j < d_long_sync_length; j++){
        long_sync_buffer[j] = in_long_sync[j] * exp(gr_complex(0, j * d_phase_shift));
        // std::cout << in_long_sync[j] << ",";
      }
      // std::cout << std::endl;
      memcpy(d_fft->get_inbuf(), long_sync_buffer, d_long_sync_length * sizeof(gr_complex));
      d_fft->execute();
      memcpy(&d_initial_cfr[0], &d_fft->get_outbuf()[len], (d_long_sync_length - len) * sizeof(gr_complex));
      memcpy(&d_initial_cfr[d_long_sync_length - len], &d_fft->get_outbuf()[0], len * sizeof(gr_complex));
      for(int j = 0; j < d_long_sync_length; j++){
        long_sync_buffer[j] = in_long_sync[j + d_long_sync_length] * exp(gr_complex(0, (d_long_sync_length + j) * d_phase_shift));
        // std::cout << in_long_sync[j + d_long_sync_length] << ",";
      }
      // std::cout << std::endl << std::endl;
      memcpy(d_fft->get_inbuf(), long_sync_buffer, d_long_sync_length * sizeof(gr_complex));
      d_fft->execute();
      memcpy(&long_sync_buffer[0], &d_fft->get_outbuf()[len], (d_long_sync_length - len) * sizeof(gr_complex));
      memcpy(&long_sync_buffer[d_long_sync_length - len], &d_fft->get_outbuf()[0], len * sizeof(gr_complex));

      for(int j = 0; j < d_long_sync_length; j++){
        d_initial_cfr[j] = (d_initial_cfr[j] + long_sync_buffer[j]) * d_long_sync[j];
      }
    }

    void 
    ofdm_sync_and_decode_header_history_impl::fine_freq_sync(const gr_complex *in_long_sync, bool fine){
      if(fine){
        d_fine_phase_shift = gr_complex(0, 0);
        gr_complex coarse_phase_shift_long = exp(gr_complex(0, arg(d_coarse_phase_shift) * d_long_sync_length / d_short_sync_length));
        for(int j = 0; j < d_long_sync_length; j++){
          d_fine_phase_shift += in_long_sync[j] * conj(in_long_sync[j + d_long_sync_length] * coarse_phase_shift_long);
        }
        d_phase_shift = arg(d_fine_phase_shift) / d_long_sync_length + arg(d_coarse_phase_shift) / d_short_sync_length;
      }else{
        d_phase_shift = arg(d_coarse_phase_shift) / d_short_sync_length;
      }
    }

    void
    ofdm_sync_and_decode_header_history_impl::search_frame_start(){
      d_correlation_index.sort();
      d_correlation_index.reverse();
      d_frame_start = d_sync_length + 1;
      std::pair<double, int> peak[4];
      std::list<std::pair<double, int>>::iterator it = d_correlation_index.begin();
      for(int j = 0; j < 4; j++, it++){
        peak[j] = *it;
      }
      d_correlation_index.clear();

      for(int j = 0; j < 3; j++){
        for(int k = 0; k < 4; k++){
          int interval = abs(std::get<1>(peak[j]) - std::get<1>(peak[k]));
          if(interval == d_long_sync_length){
            d_frame_start = std::max(std::get<1>(peak[j]), std::get<1>(peak[k])) + d_long_sync_length;
            return;
          }else if(interval == d_long_sync_length - 1){
            d_frame_start = std::max(std::get<1>(peak[j]), std::get<1>(peak[k])) + d_long_sync_length - 1;
            return;
          }else if(interval == d_long_sync_length + 1){
            d_frame_start = std::max(std::get<1>(peak[j]), std::get<1>(peak[k])) + d_long_sync_length;
            return;
          }
        }
      }
    }

    const std::vector<gr_complex> ofdm_sync_and_decode_header_history_impl::d_long_sync_taps = {
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), 
      // gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000), gr_complex(1.0000, 1.0000)
      // gr_complex( 1.3868,  0.0000), gr_complex(-0.0455, -1.0679), gr_complex( 0.3528, -0.9865), gr_complex( 0.8594,  0.7348),
      // gr_complex( 0.1874,  0.2475), gr_complex( 0.5309, -0.7784), gr_complex(-1.0218, -0.4897), gr_complex(-0.3401, -0.9423),
      // gr_complex( 0.8657, -0.2298), gr_complex( 0.4734,  0.0362), gr_complex( 0.0088, -1.0207), gr_complex(-1.2142, -0.4205),
      // gr_complex( 0.2172, -0.5195), gr_complex( 0.5207, -0.1326), gr_complex(-0.1995,  1.4259), gr_complex( 1.0583, -0.0363),
      // gr_complex( 0.5547, -0.5547), gr_complex( 0.3277,  0.8728), gr_complex(-0.5077,  0.3488), gr_complex(-1.1650,  0.5789),
      // gr_complex( 0.7297,  0.8197), gr_complex( 0.6173,  0.1253), gr_complex(-0.5353,  0.7214), gr_complex(-0.5011, -0.1935),
      // gr_complex(-0.3110, -1.3392), gr_complex(-1.0818, -0.1470), gr_complex(-1.1300, -0.1820), gr_complex( 0.6663, -0.6571),
      // gr_complex(-0.0249,  0.4773), gr_complex(-0.8155,  1.0218), gr_complex( 0.8140,  0.9396), gr_complex( 0.1090,  0.8662),
      // gr_complex(-1.3868,  0.0000), gr_complex( 0.1090, -0.8662), gr_complex( 0.8140, -0.9396), gr_complex(-0.8155, -1.0218),
      // gr_complex(-0.0249, -0.4773), gr_complex( 0.6663,  0.6571), gr_complex(-1.1300,  0.1820), gr_complex(-1.0818,  0.1470),
      // gr_complex(-0.3110,  1.3392), gr_complex(-0.5011,  0.1935), gr_complex(-0.5353, -0.7214), gr_complex( 0.6173, -0.1253),
      // gr_complex( 0.7297, -0.8197), gr_complex(-1.1650, -0.5789), gr_complex(-0.5077, -0.3488), gr_complex( 0.3277, -0.8728),
      // gr_complex( 0.5547,  0.5547), gr_complex( 1.0583,  0.0363), gr_complex(-0.1995, -1.4259), gr_complex( 0.5207,  0.1326),
      // gr_complex( 0.2172,  0.5195), gr_complex(-1.2142,  0.4205), gr_complex( 0.0088,  1.0207), gr_complex( 0.4734, -0.0362),
      // gr_complex( 0.8657,  0.2298), gr_complex(-0.3401,  0.9423), gr_complex(-1.0218,  0.4897), gr_complex( 0.5309,  0.7784),
      // gr_complex( 0.1874, -0.2475), gr_complex( 0.8594, -0.7348), gr_complex( 0.3528,  0.9865), gr_complex(-0.0455,  1.0679)
      gr_complex(-0.0455, -1.0679),gr_complex( 0.3528, -0.9865),gr_complex( 0.8594, +0.7348),gr_complex( 0.1874, +0.2475),
      gr_complex( 0.5309, -0.7784),gr_complex(-1.0218, -0.4897),gr_complex(-0.3401, -0.9423),gr_complex( 0.8657, -0.2298),
      gr_complex( 0.4734, +0.0362),gr_complex( 0.0088, -1.0207),gr_complex(-1.2142, -0.4205),gr_complex( 0.2172, -0.5195),
      gr_complex( 0.5207, -0.1326),gr_complex(-0.1995, +1.4259),gr_complex( 1.0583, -0.0363),gr_complex( 0.5547, -0.5547),
      gr_complex( 0.3277, +0.8728),gr_complex(-0.5077, +0.3488),gr_complex(-1.1650, +0.5789),gr_complex( 0.7297, +0.8197),
      gr_complex( 0.6173, +0.1253),gr_complex(-0.5353, +0.7214),gr_complex(-0.5011, -0.1935),gr_complex(-0.3110, -1.3392),
      gr_complex(-1.0818, -0.1470),gr_complex(-1.1300, -0.1820),gr_complex( 0.6663, -0.6571),gr_complex(-0.0249, +0.4773),
      gr_complex(-0.8155, +1.0218),gr_complex( 0.8140, +0.9396),gr_complex( 0.1090, +0.8662),gr_complex(-1.3868, +0.0000),
      gr_complex( 0.1090, -0.8662),gr_complex( 0.8140, -0.9396),gr_complex(-0.8155, -1.0218),gr_complex(-0.0249, -0.4773),
      gr_complex( 0.6663, +0.6571),gr_complex(-1.1300, +0.1820),gr_complex(-1.0818, +0.1470),gr_complex(-0.3110, +1.3392),
      gr_complex(-0.5011, +0.1935),gr_complex(-0.5353, -0.7214),gr_complex( 0.6173, -0.1253),gr_complex( 0.7297, -0.8197),
      gr_complex(-1.1650, -0.5789),gr_complex(-0.5077, -0.3488),gr_complex( 0.3277, -0.8728),gr_complex( 0.5547, +0.5547),
      gr_complex( 1.0583, +0.0363),gr_complex(-0.1995, -1.4259),gr_complex( 0.5207, +0.1326),gr_complex( 0.2172, +0.5195),
      gr_complex(-1.2142, +0.4205),gr_complex( 0.0088, +1.0207),gr_complex( 0.4734, -0.0362),gr_complex( 0.8657, +0.2298),
      gr_complex(-0.3401, +0.9423),gr_complex(-1.0218, +0.4897),gr_complex( 0.5309, +0.7784),gr_complex( 0.1874, -0.2475),
      gr_complex( 0.8594, -0.7348),gr_complex( 0.3528, +0.9865),gr_complex(-0.0455, +1.0679),gr_complex( 1.3868, +0.0000)
    };

    const std::vector<float> ofdm_sync_and_decode_header_history_impl::d_long_sync = {
      0, 0, 0, 0, 0, 0, 0.5, 0.5,-0.5,-0.5, 0.5, 0.5,-0.5, 0.5,-0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,-0.5,-0.5, 0.5, 0.5,-0.5, 0.5,-0.5, 0.5, 0.5, 0.5, 0.5,
      0, 0.5,-0.5,-0.5, 0.5, 0.5,-0.5, 0.5,-0.5, 0.5,-0.5,-0.5,-0.5,-0.5,-0.5, 0.5, 0.5,-0.5,-0.5, 0.5,-0.5, 0.5,-0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0
    };

    const std::vector<int> ofdm_sync_and_decode_header_history_impl:: d_interleave_pattern = {
      0,3,6,9,12,15,18,21,24,27,30,33,36,39,42,45,1,4,7,10,13,16,19,22,25,28,31,34,37,40,43,46,2,5,8,11,14,17,20,23,26,29,32,35,38,41,44,47
    };
  } /* namespace csitool */
} /* namespace gr */

