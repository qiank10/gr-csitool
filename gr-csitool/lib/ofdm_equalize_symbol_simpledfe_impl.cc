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
#include "ofdm_equalize_symbol_simpledfe_impl.h"

namespace gr {
  namespace csitool {

    ofdm_equalize_symbol_simpledfe::sptr
    ofdm_equalize_symbol_simpledfe::make(const std::string& taps_tag)
    {
      return gnuradio::get_initial_sptr
        (new ofdm_equalize_symbol_simpledfe_impl(taps_tag));
    }

    /*
     * The private constructor
     */
    ofdm_equalize_symbol_simpledfe_impl::ofdm_equalize_symbol_simpledfe_impl(const std::string& taps_tag)
      : gr::block("ofdm_equalize_symbol_simpledfe",
              gr::io_signature::make(1, 1, d_fft_length * sizeof(gr_complex)),
              gr::io_signature::make(1, 1, d_n_data_subcarrier * sizeof(gr_complex))),
      d_pilot_index(-1),
      d_taps_tag(taps_tag),
      d_constellation(-1),
      d_alpha(0.1)
    {
      set_relative_rate(1);
    }

    /*
     * Our virtual destructor.
     */
    ofdm_equalize_symbol_simpledfe_impl::~ofdm_equalize_symbol_simpledfe_impl()
    {}

    void
    ofdm_equalize_symbol_simpledfe_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      ninput_items_required[0] = noutput_items;
    }

    int
    ofdm_equalize_symbol_simpledfe_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out = (gr_complex *) output_items[0];

        int nitems = std::min(ninput_items[0], noutput_items);
        std::vector<tag_t> tags;

        // Do <+signal processing+>
        // Tell runtime system how many input items we consumed on
        // each input stream.
        int i = 0;
        for(i = 0; i < nitems; i++){
          get_tags_in_range(tags, 0, nitems_read(0) + i * d_fft_length, nitems_read(0) + (i + 1) * d_fft_length);
          if(tags.size() > 0){
            int k = 0;
            for(k = 0; k < tags.size(); k++){
              if(pmt::symbol_to_string(tags[k].key) == "ofdm_data_rate"){
                int rate = pmt::to_uint64(tags[k].value);
                switch(rate){
                  case 11:
                  case 15:
                    d_constellation = 1;
                    break;
                  case 10:
                  case 14:
                    d_constellation = 2;
                    break;
                  default:
                    d_constellation = -1;
                    break;
                }
              }
              if(pmt::symbol_to_string(tags[k].key) == d_taps_tag){
                d_cfr = pmt::c32vector_elements(tags[k].value);
                d_pilot_index = 1;
              }
            }
          }
          if(d_pilot_index < 0 || d_constellation < 0){
            in += d_fft_length;
            memset(out, 0, d_n_data_subcarrier * sizeof(gr_complex));
            out += d_n_data_subcarrier;
            continue;
          }
          float p = d_pilot_scramble[d_pilot_index];
          d_pilot_index++;
          d_pilot_index %= d_pilot_scramble.size();

          gr_complex sym_eq;
          gr_complex sym_est;
          int k = 0;
          for(int j = 0; j < d_fft_length; j++){
            if(j == 32 || j < 6 || j > 58){
              continue;
            }else if(j == 11 || j == 25 || j == 39){
              d_cfr[j] = d_alpha * d_cfr[j] + (1 - d_alpha) * in[j] * p;
            }else if(j == 53){
              d_cfr[j] = d_alpha * d_cfr[j] - (1 - d_alpha) * in[j] * p;
            }else{
              sym_eq = in[j] / d_cfr[j];
              switch(d_constellation){
                case 1:{
                  int index = (real(sym_eq) > 0);
                  sym_est = d_bpsk_constellation[index];
                  break;
                }
                case 2:{
                  int index = 2 * (imag(sym_eq) > 0) + (real(sym_eq) > 0);
                  sym_est = d_qpsk_constellation[index];
                  break;
                }
              }
              d_cfr[j] = d_alpha * d_cfr[j] + (1 - d_alpha) * in[j] / sym_est;
              out[k] = sym_eq;
              k++;
            }
          }

          in += d_fft_length;
          out += d_n_data_subcarrier;
        }

        consume_each (i);

        // Tell runtime system how many output items we produced.
        return i;
    }

    const std::vector<float> ofdm_equalize_symbol_simpledfe_impl::d_pilot_scramble = {
       1, 1, 1, 1,-1,-1,-1, 1,-1,-1,-1,-1, 1, 1,-1, 1,
      -1,-1, 1, 1,-1, 1, 1,-1, 1, 1, 1, 1, 1, 1,-1, 1,
       1, 1,-1, 1, 1,-1,-1, 1, 1, 1,-1, 1,-1,-1,-1, 1,
      -1, 1,-1,-1, 1,-1,-1, 1, 1, 1, 1, 1,-1,-1, 1, 1,
      -1,-1, 1,-1, 1,-1, 1, 1,-1,-1,-1, 1, 1,-1,-1,-1,
      -1, 1,-1,-1, 1,-1, 1, 1, 1, 1,-1, 1,-1, 1,-1, 1,
      -1,-1,-1,-1,-1, 1,-1, 1, 1,-1, 1,-1, 1, 1, 1,-1,
      -1, 1,-1,-1,-1, 1, 1, 1,-1,-1,-1,-1,-1,-1,-1
    };

    const std::vector<gr_complex> ofdm_equalize_symbol_simpledfe_impl::d_bpsk_constellation = {
      gr_complex(-1, 0), gr_complex( 1, 0)
    };

    const std::vector<gr_complex> ofdm_equalize_symbol_simpledfe_impl::d_qpsk_constellation = {
      gr_complex(-0.707107, -0.707107), gr_complex( 0.707107, -0.707107),
      gr_complex(-0.707107,  0.707107), gr_complex( 0.707107,  0.707107)
    };
  } /* namespace csitool */
} /* namespace gr */

