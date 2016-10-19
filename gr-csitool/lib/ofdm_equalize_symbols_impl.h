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

#ifndef INCLUDED_CSITOOL_OFDM_EQUALIZE_SYMBOLS_IMPL_H
#define INCLUDED_CSITOOL_OFDM_EQUALIZE_SYMBOLS_IMPL_H

#include <csitool/ofdm_equalize_symbols.h>

namespace gr {
  namespace csitool {

    class ofdm_equalize_symbols_impl : public ofdm_equalize_symbols
    {
     private:
      // Nothing to declare in this block.
      const static int d_fft_length = 64;
      const static int d_n_data_subcarrier = 48;
      const static std::vector<float> d_pilot_scramble;

      std::vector<gr_complex> d_cfr;
      int d_pilot_index;
      std::string d_taps_tag;

     public:
      ofdm_equalize_symbols_impl(const std::string &taps_tag);
      ~ofdm_equalize_symbols_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace csitool
} // namespace gr

#endif /* INCLUDED_CSITOOL_OFDM_EQUALIZE_SYMBOLS_IMPL_H */

