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

#ifndef INCLUDED_WIFIRECV_OFDM_PARSE_PAYLOAD_IMPL_H
#define INCLUDED_WIFIRECV_OFDM_PARSE_PAYLOAD_IMPL_H

#include <wifirecv/ofdm_parse_payload.h>
#include <itpp/itcomm.h>
#include <boost/crc.hpp>

namespace gr {
  namespace wifirecv {

    class ofdm_parse_payload_impl : public ofdm_parse_payload
    {
     private:
      // Nothing to declare in this block.

      //! Copy flag
      bool d_begin_copy;
      //! Storing samples of single packet.
      gr_complex d_sample_buf[70000];
      //! Storing demodulate bits
      itpp::vec d_bits;
      //! Storing deinterleaved coded bits
      double d_deinterleave_buf[70000];
      //! Storing decoded bits
      itpp::bvec d_decoded_bits;
      //! Storing descrambled bits;
      char d_data_bits[35000];
      //! Storing data octets
      char d_data_bytes[5000];
      //! Number of copied samples.
      int d_copied;
      //! Number of samples to copy.
      int d_to_copy;
      //! Data rate
      int d_rate;
      //! Psdu size
      int d_n_psdu;
      //! CFR
      std::vector<gr_complex> d_cfr;
      //! Coded bits per symbol
      int d_ncbps;
      //! MAC Address
      std::vector<char> d_mac;
      //! Sample Index
      uint64_t d_index;

      int d_n;

      itpp::Modulator<std::complex<double>> bpsk;
      itpp::Modulator<std::complex<double>> qpsk;
      itpp::Modulator<std::complex<double>> qam16;
      itpp::Modulator<std::complex<double>> qam64;

      const static int d_data_size = 48;
      const static int d_fft_size = 64;
      const static unsigned int d_crc32_magic_number = 558161692;
     public:
      ofdm_parse_payload_impl(const std::string &mac);
      ~ofdm_parse_payload_impl();

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);

      bool decode();
      void demodulate();
      void deinterleave();
      void deconvolute();
      void descramble();
    };

  } // namespace wifirecv
} // namespace gr

#endif /* INCLUDED_WIFIRECV_OFDM_PARSE_PAYLOAD_IMPL_H */

