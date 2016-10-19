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
#include "ofdm_equalize_symbols_impl.h"

namespace gr {
  namespace wifirecv {

    ofdm_equalize_symbols::sptr
    ofdm_equalize_symbols::make()
    {
      return gnuradio::get_initial_sptr
        (new ofdm_equalize_symbols_impl());
    }

    /*
     * The private constructor
     */
    ofdm_equalize_symbols_impl::ofdm_equalize_symbols_impl()
      : gr::block("ofdm_equalize_symbols",
              gr::io_signature::make(1, 1, d_fft_size * sizeof(gr_complex)),
              gr::io_signature::make(1, 1, d_data_size * sizeof(gr_complex))),
        d_pilot_index(0),
        d_state(STALL)
    {
      set_relative_rate(1);
    }

    /*
     * Our virtual destructor.
     */
    ofdm_equalize_symbols_impl::~ofdm_equalize_symbols_impl()
    {
    }

    int
    ofdm_equalize_symbols_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out = (gr_complex *) output_items[0];

        int nitem = std::min(ninput_items[0], noutput_items);
        std::vector<tag_t> tags;

        // Do <+signal processing+>
        // Tell runtime system how many input items we consumed on
        // each input stream.
        int i = 0;
        for(i = 0; i < nitem; i++){
          // [start,end)
          get_tags_in_range(tags, 0,
            nitems_read(0)+i*d_fft_size,
            nitems_read(0)+(i+1)*d_fft_size);
          if(tags.size() > 0){
            d_pilot_index = 1;
            d_state = WORK;
            int k = 0;
            for(k = 0; k < tags.size(); k++){
              if(pmt::symbol_to_string(tags[k].key) == "ofdm_channel_taps"){
                d_cfr = pmt::c32vector_elements(tags[k].value);
                break;
              }
            }
            if(k >= tags.size()){
              d_state = STALL;
              in += d_fft_size;
              out += d_data_size;
              continue;
            }
          }else{
            if(d_state == STALL){
              in += d_fft_size;
              out += d_data_size;
              continue;
            }
          }

          float p = d_pilot_scramble[d_pilot_index];
          d_pilot_index++;
          d_pilot_index %= d_pilot_scramble.size();

          // Track phase offset
          int j = 0;
          int l = 0;
          gr_complex phase_err = gr_complex(0,0);
          for(j = 0; j < d_fft_size; j++){
            if(j == 32 || j < 6 || j > 58){
              continue;
            }else if(j == 11 || j == 25 || j == 39){
              phase_err = phase_err + in[j] * conj(d_cfr[j]) * p;
            }else if(j == 53){
              phase_err = phase_err - in[j] * conj(d_cfr[j]) * p;
            }else{
              out[l] = in[j] / d_cfr[j];
              l++;
            }
          }
          for(l = 0; l < d_data_size; l++){
            out[l] = out[l] * exp(gr_complex(0,-arg(phase_err)));
          }

          in += d_fft_size;
          out += d_data_size;
        }

        consume_each (i);

        // Tell runtime system how many output items we produced.
        return i;
    }

    const std::vector<float> ofdm_equalize_symbols_impl::d_pilot_scramble = {
       1, 1, 1, 1,-1,-1,-1, 1,-1,-1,-1,-1, 1, 1,-1, 1,
      -1,-1, 1, 1,-1, 1, 1,-1, 1, 1, 1, 1, 1, 1,-1, 1,
       1, 1,-1, 1, 1,-1,-1, 1, 1, 1,-1, 1,-1,-1,-1, 1,
      -1, 1,-1,-1, 1,-1,-1, 1, 1, 1, 1, 1,-1,-1, 1, 1,
      -1,-1, 1,-1, 1,-1, 1, 1,-1,-1,-1, 1, 1,-1,-1,-1,
      -1, 1,-1,-1, 1,-1, 1, 1, 1, 1,-1, 1,-1, 1,-1, 1,
      -1,-1,-1,-1,-1, 1,-1, 1, 1,-1, 1,-1, 1, 1, 1,-1,
      -1, 1,-1,-1,-1, 1, 1, 1,-1,-1,-1,-1,-1,-1,-1
    };
  } /* namespace wifirecv */
} /* namespace gr */

