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
#include "ofdm_sync_parse_header_impl.h"

using namespace itpp;
using namespace std;

namespace gr {
  namespace wifirecv {

    ofdm_sync_parse_header::sptr
    ofdm_sync_parse_header::make(double threshold, unsigned int min_plateau, int timing_off, int sync_length){
      return gnuradio::get_initial_sptr
        (new ofdm_sync_parse_header_impl(threshold, min_plateau, timing_off, sync_length));
    }

    /*
     * The private constructor
     */
    ofdm_sync_parse_header_impl::ofdm_sync_parse_header_impl(double threshold, unsigned int min_plateau, int timing_off, int sync_length)
      : gr::block("ofdm_sync_parse_header",
              gr::io_signature::make2(2, 2, sizeof(gr_complex), sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
        d_threshold(threshold),
        d_min_plateau(min_plateau),
        d_sync_length(sync_length), //require d_sync_length >= 320
        d_plateau(0),
        d_n_psdu(0),
        d_rate(0),
        d_to_copy(0),
        d_offset(0),
        d_frame_start(0),
        d_coarse_freq_est(gr_complex(0,0)),
        d_fine_freq_est(gr_complex(0,0)),
        d_timing_off(timing_off),
        d_state(SEARCH)
    {

      d_align = volk_get_alignment();
      set_tag_propagation_policy(gr::block::TPP_DONT);
      set_history(d_sync_length + 1);
      d_fir = new gr::filter::kernel::fir_filter_ccc(1, d_long_sync_time);
      d_correlation = gr::fft::malloc_complex(8192);
      d_coarse_freq_est_buf = static_cast<gr_complex*>(std::malloc(sync_length * sizeof(gr_complex)));
      d_fft = new fft::fft_complex(d_long_sync_length, true, 1);
      d_long_sync_buf = static_cast<gr_complex*>(std::malloc(d_long_sync_length * sizeof(gr_complex)));
      d_cfr = static_cast<gr_complex*>(std::malloc(d_long_sync_length * sizeof(gr_complex)));
      d_header_time_buf = static_cast<gr_complex*>(std::malloc(d_fft_length * sizeof(gr_complex)));
      d_header_freq_buf = static_cast<gr_complex*>(std::malloc(d_n_data_subcarrier * sizeof(gr_complex)));
      d_header_code_bit = static_cast<double*>(std::malloc(d_n_data_subcarrier * sizeof(double)));
      d_inter_buf = static_cast<double*>(std::malloc(d_n_data_subcarrier * sizeof(double)));
    }

    /*
     * Our virtual destructor.
     */
    ofdm_sync_parse_header_impl::~ofdm_sync_parse_header_impl(){
      delete d_fir;
      gr::fft::free(d_correlation);
      free(d_coarse_freq_est_buf);
      delete d_fft;
      free(d_long_sync_buf);
      free(d_cfr);
      free(d_header_time_buf);
      free(d_header_freq_buf);
      free(d_header_code_bit);
      free(d_inter_buf);
    }

    void
    ofdm_sync_parse_header_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required){
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
    ofdm_sync_parse_header_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items){
        const gr_complex *in = ((const gr_complex*) input_items[0]) + d_sync_length;
        const float *in_threshold = ((const float*) input_items[1]) + d_sync_length;


        gr_complex *out = (gr_complex*) output_items[0];
        int noutput = noutput_items;
        int ninput = std::min(ninput_items[0], ninput_items[1]) - d_sync_length;

        // Do <+signal processing+>
        // Tell runtime system how many input items we consumed on
        // each input stream.
        // cout << nitems_read(0) << " " << ninput_items[0] << " " << ninput_items[1] << " " << ninput << endl;
        int i = 0;
        int o = 0;
        switch(d_state){
          case SEARCH:{
              for(i = 0; i < ninput; i++){
                if(in_threshold[i] > d_threshold){
                  d_plateau++;
                  if(d_plateau >= d_min_plateau){
                    d_state = SYNC;
                    d_plateau = 0;
                    d_offset = 0;
                    i++;
                    // cout << nitems_read(0) +i << endl;
                    break;
                  }
                }else{
                  d_plateau = 0;
                }
              }  
            }
            // cout << "SEARCH: " << nitems_read(0) << " " << i << endl;
            break;

          case SYNC:{
              d_fir->filterN(d_correlation, in, std::min(d_sync_length, std::max(ninput - (d_long_sync_length - 1), 0)));
              while(i + d_long_sync_length - 1 < ninput){
                d_coarse_freq_est += in[i] * conj(in[i + 16]);
                d_coarse_freq_est_buf[d_offset] = d_coarse_freq_est;
                d_cor.push_back(std::pair<double, int>(abs(d_correlation[i]),d_offset));
                i++;
                d_offset++;
                if(d_offset == d_sync_length){
                  search_frame_start();
                  // cout << "ALIGN " << nitems_read(0) + i - d_sync_length << " " << d_frame_start << " " << i << endl;
                  if(d_frame_start > d_sync_length || d_frame_start - d_timing_off < 2.5 * d_long_sync_length + 2 * d_short_sync_length){
                    d_offset = 0;
                    d_coarse_freq_est = gr_complex(0,0);
                    d_fine_freq_est = gr_complex(0,0);
                    i = 0;
                    d_state = SEARCH;
                  }else{
                    // cout << "ALIGN " << nitems_read(0) + i - d_sync_length + d_frame_start << " " << d_frame_start << " " << i << endl;
                    d_frame_start -= d_timing_off;
                    d_coarse_freq_est = d_coarse_freq_est_buf[d_frame_start - int(2.5 * d_long_sync_length) - d_short_sync_length - 1];
                    const gr_complex *in_long_sync = in + i - d_sync_length + d_frame_start - 2 * d_long_sync_length;
                    fine_freq_sync(in_long_sync);
                    channel_estimation(in_long_sync);
                    i = d_frame_start - d_sync_length + i;
                    if(i <= 0){
                      d_frame_start = i;
                      i = 0;
                    }else{
                      d_frame_start = 0;
                    }
                    d_offset = 0;
                    d_state = PARSE;
                  }
                  break;
                }
              }
            }
            // cout << "ALIGN" << nitems_read(0) + i - d_sync_length << " " << d_frame_start << " " << i << endl;
            break;
          case PARSE:{
              const gr_complex *in_header = in + d_frame_start + d_cyclic_prefix_length;
              int absolute_offset = 2*d_long_sync_length+d_cyclic_prefix_length;
              int len = (int)ceil(d_long_sync_length / 2.0);
              int j = 0;
              for(j = 0; j < d_fft_length; j++){
                d_header_time_buf[j] = in_header[j] * exp(gr_complex(0, (absolute_offset + j) * d_phase_shift));
              }
              memcpy(d_fft->get_inbuf(), d_header_time_buf, d_fft_length * sizeof(gr_complex));
              d_fft->execute();
              memcpy(&d_header_time_buf[0], &d_fft->get_outbuf()[len], sizeof(gr_complex) * (d_long_sync_length - len));
              memcpy(&d_header_time_buf[d_long_sync_length - len], &d_fft->get_outbuf()[0], sizeof(gr_complex) * len);

              int k = 0;
              gr_complex phase_err = gr_complex(0,0);
              for(j = 0; j < d_fft_length; j++){
                if(j == 32 || j < 6 || j > 58){
                  continue;
                }else if(j == 11 || j == 25 || j == 39){
                  phase_err = phase_err + d_header_time_buf[j] * conj(d_cfr[j]);
                }else if(j == 53){
                  phase_err = phase_err - d_header_time_buf[j] * conj(d_cfr[j]);
                }else{
                  d_header_freq_buf[k] = d_header_time_buf[j] / d_cfr[j];
                  k++;
                }
              }
              for(k = 0; k < d_n_data_subcarrier; k++){
                d_header_freq_buf[k] = d_header_freq_buf[k] * exp(gr_complex(0, -arg(phase_err)));
                // TODO: Use subcarrier amplitudes to weight the soft decisions before Viterbi decoding  

                // 0-->1, 1-->-1 in itpp.
                d_header_code_bit[k] = -real(d_header_freq_buf[k]);
              }
              header_deinterleave();
              header_decode();
              if(header_parse()){
                // cout << nitems_read(0) + d_frame_start << endl;
                d_abs_offset = 2 * d_long_sync_length + d_fft_length + d_cyclic_prefix_length;
                d_frame_start = (int)(in_header - in) + d_fft_length;
                if(d_frame_start >= 0){
                  i = d_frame_start;
                  d_frame_start = 0;
                }else{
                  i = 0;
                }
                d_offset = 0;
                d_state = COPY;
                // cout << arg(d_coarse_freq_est) / d_short_sync_length << " " << arg(d_fine_freq_est) / d_long_sync_length << endl;
              }else{
                d_offset = 0;
                d_coarse_freq_est = gr_complex(0,0);
                d_fine_freq_est = gr_complex(0,0);
                d_state = SEARCH;
                i = 0;
              }
            }
            break;
          case COPY:{
              const gr_complex* in_data = in + d_frame_start;
              int n_item = std::min(noutput, (ninput - d_frame_start));
              o = 0;
              for(i = 0; i < n_item; i++){
                if(d_offset % (d_fft_length + d_cyclic_prefix_length) < d_cyclic_prefix_length){
                  d_offset++;
                  d_abs_offset++;
                }else{
                  if(d_offset == d_cyclic_prefix_length){
                    assert(o = 0);
                    add_item_tag(0, nitems_written(0),
                      pmt::string_to_symbol("ofdm_data_rate"),
                      pmt::from_uint64(d_rate),
                      pmt::string_to_symbol(name()));
                    add_item_tag(0, nitems_written(0),
                      pmt::string_to_symbol("ofdm_psdu_length"),
                      pmt::from_uint64(d_n_psdu),
                      pmt::string_to_symbol(name()));
                    add_item_tag(0, nitems_written(0),
                      pmt::string_to_symbol("ofdm_sample_number"),
                      pmt::from_uint64(d_to_copy * d_n_data_subcarrier / (d_fft_length + d_cyclic_prefix_length)),
                      pmt::string_to_symbol(name()));
                    add_item_tag(0, nitems_written(0),
                      pmt::string_to_symbol("ofdm_channel_taps"),
                      pmt::init_c32vector(d_fft_length, d_cfr),
                      pmt::string_to_symbol(name()));
                  }
                  out[o] = in_data[i] * exp(gr_complex(0, d_abs_offset * d_phase_shift));
                  o++;
                  d_offset++;
                  d_abs_offset++;
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
              }else if(d_offset == d_to_copy){
                d_offset = 0;
                d_coarse_freq_est = gr_complex(0,0);
                d_fine_freq_est = gr_complex(0,0);
                d_state = SEARCH;
              }
            }
            break;
        }

        consume_each(i);
        // Tell runtime system how many output items we produced.
        return o;
    }

    void
    ofdm_sync_parse_header_impl::search_frame_start(){
      d_cor.sort();
      d_cor.reverse();

      d_frame_start = d_sync_length + 1;

      std::pair<double,int> peak[4];
      std::list<std::pair<double,int>>::iterator it = d_cor.begin();

      for(int j = 0; j < 4; j++,it++){
        peak[j] = *it;
        // cout << get<0>(peak[j]) << " " << get<1>(peak[j]) << endl;
      }
      d_cor.clear();

      for(int j = 0; j < 3; j++){
        for(int k = j + 1; k < 4; k++){
          int interval = abs(std::get<1>(peak[j]) - std::get<1>(peak[k]));
          if(interval == d_long_sync_length){
            d_frame_start = std::max(std::get<1>(peak[j]), std::get<1>(peak[k])) + d_long_sync_length;
            return;
          }else if(interval == d_long_sync_length - 1){ // We have cp
            d_frame_start = std::max(std::get<1>(peak[j]), std::get<1>(peak[k])) + d_long_sync_length - 1;
            return;
          }else if(interval == d_long_sync_length + 1){
            d_frame_start = std::max(std::get<1>(peak[j]), std::get<1>(peak[k])) + d_long_sync_length;
            return;
          }
        }
      }

    }

    void
    ofdm_sync_parse_header_impl::fine_freq_sync(const gr_complex *in){
      d_fine_freq_est = gr_complex(0,0);
      gr_complex coarse_phase_offset_long = exp(gr_complex(0, arg(d_coarse_freq_est) * d_long_sync_length / d_short_sync_length));
      for(int i = 0; i < d_long_sync_length; i++){
        d_fine_freq_est += in[i] * conj(in[i + d_long_sync_length] * coarse_phase_offset_long);
      }
      // d_phase_shift = arg(d_coarse_freq_est) / d_short_sync_length + arg(d_fine_freq_est) / d_long_sync_length;
      d_phase_shift = arg(d_coarse_freq_est) / d_short_sync_length;
    }

    void
    ofdm_sync_parse_header_impl::channel_estimation(const gr_complex *in){
      int len = (int)ceil(d_long_sync_length / 2.0);
      // First long sync
      int absolute_offset = 0;
      for(int i = 0; i < d_long_sync_length; i++){
        d_long_sync_buf[i] = in[i] * exp(gr_complex(0, (absolute_offset + i) * d_phase_shift));
      }
      memcpy(d_fft->get_inbuf(), d_long_sync_buf, d_long_sync_length * sizeof(gr_complex));
      d_fft->execute();
      memcpy(&d_cfr[0], &d_fft->get_outbuf()[len], sizeof(gr_complex) * (d_long_sync_length - len));
      memcpy(&d_cfr[d_long_sync_length - len], &d_fft->get_outbuf()[0], sizeof(gr_complex) * len);
      // Second long sync
      absolute_offset += d_long_sync_length;
      for(int i = 0; i < d_long_sync_length; i++){
        d_long_sync_buf[i] = in[i + d_long_sync_length] * exp(gr_complex(0, (absolute_offset + i) * d_phase_shift));
      }
      memcpy(d_fft->get_inbuf(), d_long_sync_buf, d_long_sync_length * sizeof(gr_complex));
      d_fft->execute();
      memcpy(&d_long_sync_buf[0], &d_fft->get_outbuf()[len], sizeof(gr_complex) * (d_long_sync_length - len));
      memcpy(&d_long_sync_buf[d_long_sync_length - len], &d_fft->get_outbuf()[0], sizeof(gr_complex) * len);

      for(int i = 0; i < d_long_sync_length; i++){
        d_cfr[i] = (d_cfr[i] + d_long_sync_buf[i]) * d_long_sync_freq[i];
      }
    }

    void 
    ofdm_sync_parse_header_impl::header_deinterleave(){
      for(int i = 0; i < d_n_data_subcarrier; i++){
        d_inter_buf[i] = d_header_code_bit[d_interleave_pattern[i]];
      }
      memcpy(d_header_code_bit, d_inter_buf, sizeof(double) * d_n_data_subcarrier);
    }

    void
    ofdm_sync_parse_header_impl::header_decode(){
      itpp::Convolutional_Code code;
      itpp::ivec generator(2);
      generator(0)=0133;
      generator(1)=0171;
      code.set_generator_polynomials(generator, 7);
      code.set_truncation_length(30);
      itpp::vec header_signal(d_header_code_bit, d_n_data_subcarrier);
      code.reset();
      code.decode_tail(header_signal, d_header_data_bit);
    }

    bool
    ofdm_sync_parse_header_impl::header_parse(){
      d_rate = 0;
      d_n_psdu = 0;
      bool parity = false;
      for(int i = 0; i < 17; i++){
        parity ^= (bool)d_header_data_bit[i];
        if((i < 4) && d_header_data_bit[i]){
          d_rate = d_rate | (1 << i);
        }else if(d_header_data_bit[i] && i > 4 && i < 17){
          d_n_psdu = d_n_psdu | (1 << (i - 5));
        }
      }

      if(parity != (bool)d_header_data_bit[17]){
        return false;
      }

      switch(d_rate){
        case 11:
          d_to_copy = ((int)ceil((16 + 8 * d_n_psdu + 6) / (double)24));
          break;
        case 15:
          d_to_copy = ((int)ceil((16 + 8 * d_n_psdu + 6) / (double)36));
          break;
        case 10:
          d_to_copy = ((int)ceil((16 + 8 * d_n_psdu + 6) / (double)48));
          break;
        case 14:
          d_to_copy = ((int)ceil((16 + 8 * d_n_psdu + 6) / (double)72));
          break;
        case 9:
          d_to_copy = ((int)ceil((16 + 8 * d_n_psdu + 6) / (double)96));
          break;
        case 13:
          d_to_copy = ((int)ceil((16 + 8 * d_n_psdu + 6) / (double)144));
          break;
        case 8:
          d_to_copy = ((int)ceil((16 + 8 * d_n_psdu + 6) / (double)192));
          break;
        case 12:
          d_to_copy = ((int)ceil((16 + 8 * d_n_psdu + 6) / (double)216));
          break;
        default:
          return false;
      }

      d_to_copy = d_to_copy * (d_cyclic_prefix_length + d_fft_length);
      return true;
    }

    const std::vector<gr_complex> ofdm_sync_parse_header_impl::d_long_sync_time = {
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

    const std::vector<float> ofdm_sync_parse_header_impl::d_long_sync_freq = {
      0, 0, 0, 0, 0, 0, 0.5, 0.5,-0.5,-0.5, 0.5, 0.5,-0.5, 0.5,-0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,-0.5,-0.5, 0.5, 0.5,-0.5, 0.5,-0.5, 0.5, 0.5, 0.5, 0.5,
      0, 0.5,-0.5,-0.5, 0.5, 0.5,-0.5, 0.5,-0.5, 0.5,-0.5,-0.5,-0.5,-0.5,-0.5, 0.5, 0.5,-0.5,-0.5, 0.5,-0.5, 0.5,-0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0
    };

    const std::vector<int> ofdm_sync_parse_header_impl::d_interleave_pattern = {
      0,3,6,9,12,15,18,21,24,27,30,33,36,39,42,45,1,4,7,10,13,16,19,22,25,28,31,34,37,40,43,46,2,5,8,11,14,17,20,23,26,29,32,35,38,41,44,47
    };
  } /* namespace wifirecv */
} /* namespace gr */

