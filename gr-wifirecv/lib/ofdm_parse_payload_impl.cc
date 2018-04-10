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
#include "ofdm_parse_payload_impl.h"
using namespace std;
namespace gr {
  namespace wifirecv {

    ofdm_parse_payload::sptr
    ofdm_parse_payload::make(const std::string &mac)
    {
      return gnuradio::get_initial_sptr
        (new ofdm_parse_payload_impl(mac));
    }

    /*
     * The private constructor
     */
    ofdm_parse_payload_impl::ofdm_parse_payload_impl(const std::string &mac)
      : gr::block("ofdm_parse_payload",
              gr::io_signature::make(1, 1, d_data_size * sizeof(gr_complex)),
              gr::io_signature::make2(2, 2, d_fft_size * sizeof(gr_complex), sizeof(uint64_t))),
        d_copied(0),
        d_to_copy(0),
        d_begin_copy(false),
        d_n(0)
    {
      message_port_register_out(pmt::mp("pdu"));
      set_min_output_buffer(4095);
      set_tag_propagation_policy(gr::block::TPP_DONT);
      d_mac.clear();
      const char *ptr = (const char *)&mac[0];
      for(int i = 0; i < 6; i++){
        char seg = 0;
        bool is_seg = false;
        while(*ptr){
          int digit;
          if(*ptr >= '0' && *ptr <= '9')
            digit = *ptr - '0';
          else if(*ptr >= 'A' && *ptr <= 'F')
            digit = *ptr - 'A' + 10;
          else if(*ptr >= 'a' && *ptr <= 'f')
            digit = *ptr - 'a' + 10;
          else
            break;
          is_seg = true;
          seg *= 16;
          seg += digit;
          ptr++;
        }
        if(is_seg)
          d_mac.push_back(seg);
        if(!(*ptr))
          break;
        ptr++;
      }
      if(d_mac.size() != 6){
        d_mac.clear();
      }

      int bpsk_bits[] = {0, 1};
      std::complex<double> bpsk_syms[2] = {
        std::complex<double>(-1.0, 0.0), std::complex<double>(1.0, 0.0)
      };
      bpsk.set(itpp::cvec(bpsk_syms, 2), itpp::ivec(bpsk_bits, 2));

      int qpsk_bits[] = {0, 1, 2, 3};
      std::complex<double> qpsk_syms[4] = {
        std::complex<double>(-0.7071, -0.7071), std::complex<double>(-0.7071, +0.7071),
        std::complex<double>(+0.7071, -0.7071), std::complex<double>(+0.7071, +0.7071)
      };
      qpsk.set(itpp::cvec(qpsk_syms, 4), itpp::ivec(qpsk_bits, 4));

      int qam16_bits[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
      std::complex<double> qam16_syms[16] = {
        std::complex<double>(-0.9487, -0.9487), std::complex<double>(-0.9487, -0.3162),
        std::complex<double>(-0.9487, 0.9487), std::complex<double>(-0.9487, 0.3162),
        std::complex<double>(-0.3162, -0.9487), std::complex<double>(-0.3162, -0.3162),
        std::complex<double>(-0.3162, 0.9487), std::complex<double>(-0.3162, 0.3162),
        std::complex<double>(0.9487, -0.9487), std::complex<double>(0.9487, -0.3162),
        std::complex<double>(0.9487, 0.9487), std::complex<double>(0.9487, 0.3162),
        std::complex<double>(0.3162, -0.9487), std::complex<double>(0.3162, -0.3162),
        std::complex<double>(0.3162, 0.9487), std::complex<double>(0.3162, 0.3162)
      };
      qam16.set(itpp::cvec(qam16_syms, 16), itpp::ivec(qam16_bits, 16));

      int qam64_bits[] = {
         0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
        32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
        48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63
      };
      std::complex<double> qam64_syms[64] = {
        std::complex<double>(-1.0801, -1.0801), std::complex<double>(-1.0801, -0.7715),
        std::complex<double>(-1.0801, -0.1543), std::complex<double>(-1.0801, -0.4629),
        std::complex<double>(-1.0801, 1.0801), std::complex<double>(-1.0801, 0.7715),
        std::complex<double>(-1.0801, 0.1543), std::complex<double>(-1.0801, 0.4629),
        std::complex<double>(-0.7715, -1.0801), std::complex<double>(-0.7715, -0.7715),
        std::complex<double>(-0.7715, -0.1543), std::complex<double>(-0.7715, -0.4629),
        std::complex<double>(-0.7715, 1.0801), std::complex<double>(-0.7715, 0.7715),
        std::complex<double>(-0.7715, 0.1543), std::complex<double>(-0.7715, 0.4629),
        std::complex<double>(-0.1543, -1.0801), std::complex<double>(-0.1543, -0.7715),
        std::complex<double>(-0.1543, -0.1543), std::complex<double>(-0.1543, -0.4629),
        std::complex<double>(-0.1543, 1.0801), std::complex<double>(-0.1543, 0.7715),
        std::complex<double>(-0.1543, 0.1543), std::complex<double>(-0.1543, 0.4629),
        std::complex<double>(-0.4629, -1.0801), std::complex<double>(-0.4629, -0.7715),
        std::complex<double>(-0.4629, -0.1543), std::complex<double>(-0.4629, -0.4629),
        std::complex<double>(-0.4629, 1.0801), std::complex<double>(-0.4629, 0.7715),
        std::complex<double>(-0.4629, 0.1543), std::complex<double>(-0.4629, 0.4629),
        std::complex<double>(1.0801, -1.0801), std::complex<double>(1.0801, -0.7715),
        std::complex<double>(1.0801, -0.1543), std::complex<double>(1.0801, -0.4629),
        std::complex<double>(1.0801, 1.0801), std::complex<double>(1.0801, 0.7715),
        std::complex<double>(1.0801, 0.1543), std::complex<double>(1.0801, 0.4629),
        std::complex<double>(0.7715, -1.0801), std::complex<double>(0.7715, -0.7715),
        std::complex<double>(0.7715, -0.1543), std::complex<double>(0.7715, -0.4629),
        std::complex<double>(0.7715, 1.0801), std::complex<double>(0.7715, 0.7715),
        std::complex<double>(0.7715, 0.1543), std::complex<double>(0.7715, 0.4629),
        std::complex<double>(0.1543, -1.0801), std::complex<double>(0.1543, -0.7715),
        std::complex<double>(0.1543, -0.1543), std::complex<double>(0.1543, -0.4629),
        std::complex<double>(0.1543, 1.0801), std::complex<double>(0.1543, 0.7715),
        std::complex<double>(0.1543, 0.1543), std::complex<double>(0.1543, 0.4629),
        std::complex<double>(0.4629, -1.0801), std::complex<double>(0.4629, -0.7715),
        std::complex<double>(0.4629, -0.1543), std::complex<double>(0.4629, -0.4629),
        std::complex<double>(0.4629, 1.0801), std::complex<double>(0.4629, 0.7715),
        std::complex<double>(0.4629, 0.1543), std::complex<double>(0.4629, 0.4629)
      };
      qam64.set(itpp::cvec(qam64_syms, 64), itpp::ivec(qam64_bits, 64));
    }

    /*
     * Our virtual destructor.
     */
    ofdm_parse_payload_impl::~ofdm_parse_payload_impl()
    {}

    int
    ofdm_parse_payload_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out = (gr_complex *)output_items[0];
        uint *out2 = (uint *)output_items[1];
        int i = 0;
        int o = 0;
        std::vector<tag_t> tags;

        // Do <+signal processing+>
        for(i = 0; i < ninput_items[0]; i++){
          if(d_copied == 0 && d_to_copy == 0){
            get_tags_in_range(tags, 0, nitems_read(0)+i*d_data_size, nitems_read(0)+(i+1)*d_data_size);
            if(tags.size()){
              for(int j = 0; j < tags.size(); j++){
                if(pmt::symbol_to_string(tags[j].key) == "ofdm_data_rate"){
                  d_rate = pmt::to_uint64(tags[j].value);
                }else if(pmt::symbol_to_string(tags[j].key) == "ofdm_psdu_length"){
                  d_n_psdu = pmt::to_uint64(tags[j].value);
                }else if(pmt::symbol_to_string(tags[j].key) == "ofdm_sample_number"){
                  d_to_copy = pmt::to_uint64(tags[j].value);
                  d_copied = 0;
                }else if(pmt::symbol_to_string(tags[j].key) == "ofdm_channel_taps"){
                  d_cfr = pmt::c32vector_elements(tags[j].value);
                }else if(pmt::symbol_to_string(tags[j].key) == "ofdm_sample_index"){
                  d_index = pmt::to_uint64(tags[j].value);
                }
              }
            }else{
              continue;
            }  
          }

          std::memcpy(&d_sample_buf[d_copied], in, d_data_size * sizeof(gr_complex));
          d_copied += d_data_size;
          in += d_data_size;

          if(d_copied == d_to_copy && d_copied > 0){
            if(decode()){
              if(d_mac.size() == 6){
                if(d_n_psdu >= 20){
                  int j = 0;
                  for(j = 0; j < 6; j++){
                    if(d_mac[j] != d_data_bytes[12 + j]){
                      break;
                    }
                  }
                  if(j >= 6){
                    memcpy(out, &d_cfr[0], d_fft_size * sizeof(gr_complex));
                    memcpy(out2, &d_index, sizeof(uint64_t));
                    out += d_fft_size;
                    out2 += sizeof(uint64_t) / sizeof(uint);
                    o += 1;
                    pmt::pmt_t data = pmt::make_blob(d_data_bytes + 2, d_n_psdu - 4);
                    pmt::pmt_t encode = pmt::from_uint64(d_rate);
                    pmt::pmt_t cfr = pmt::init_c32vector(d_fft_size, d_cfr);
                    pmt::pmt_t dict = pmt::make_dict();
                    dict = pmt::dict_add(dict, pmt::mp("encoding"), encode);
                    dict = pmt::dict_add(dict, pmt::mp("csi"), cfr);
                    message_port_pub(pmt::mp("pdu"), pmt::cons(dict, data));
                  }
                }
              }else{
                memcpy(out, &d_cfr[0], d_fft_size * sizeof(gr_complex));
                memcpy(out2, &d_index, sizeof(uint64_t));
                out += d_fft_size;
                out2 += sizeof(uint64_t) / sizeof(uint);
                o += 1;
                pmt::pmt_t data = pmt::make_blob(d_data_bytes + 2, d_n_psdu - 4);
                pmt::pmt_t encode = pmt::from_uint64(d_rate);
                pmt::pmt_t cfr = pmt::init_c32vector(d_fft_size, d_cfr);
                pmt::pmt_t dict = pmt::make_dict();
                dict = pmt::dict_add(dict, pmt::mp("encoding"), encode);
                dict = pmt::dict_add(dict, pmt::mp("csi"), cfr);
                message_port_pub(pmt::mp("pdu"), pmt::cons(dict, data));
                d_n++;
                // cout << d_n << endl;
                // for(int j = 0; j < d_to_copy; j++){
                //   cout << d_sample_buf[j] << ",";
                // }
                // cout << endl;
              }
            }
            d_copied = 0;
            d_to_copy = 0;
          }
        }
        // Tell runtime system how many input items we consumed on
        // each input stream.


        consume_each (i);

        // Tell runtime system how many output items we produced.
        return o;
    }

    bool
    ofdm_parse_payload_impl::decode(){
      demodulate();
      deinterleave();
      deconvolute();
      descramble();

      boost::crc_32_type crc_computer;
      crc_computer.process_bytes(d_data_bytes+2, d_n_psdu);
      if(crc_computer.checksum() != d_crc32_magic_number){
        return false;
      }
      // cout << d_bits.size() << endl;
      return true;
    }

    void
    ofdm_parse_payload_impl::demodulate(){
      itpp::cvec samples;
      samples.set_length(d_to_copy);
      for(int i = 0; i < d_to_copy; i++){
        samples[i] = std::complex<double>(d_sample_buf[i]);
      }

      switch(d_rate){
        case 11:
        case 15:
          d_bits = to_vec(bpsk.demodulate_bits(samples));
          break;
        case 10:
        case 14:
          d_bits = to_vec(qpsk.demodulate_bits(samples));
          break;
        case 9:
        case 13:
          d_bits = to_vec(qam16.demodulate_bits(samples));
          break;
        case 8:
        case 12:
          d_bits = to_vec(qam64.demodulate_bits(samples));
          break;
      }

      // 0-->1, 1-->-1 for deconvolution.
      d_bits = d_bits * (-2) + 1;
    }

    void 
    ofdm_parse_payload_impl::deinterleave(){
      d_ncbps = 0;
      switch(d_rate){
        case 11:
        case 15:
          d_ncbps = 48;
          break;
        case 10:
        case 14:
          d_ncbps = 96;
          break;
        case 9:
        case 13:
          d_ncbps = 192;
          break;
        case 8:
        case 12:
          d_ncbps = 288;
          break;
      }
      int permutation_pattern[d_ncbps];
      int interleave_pattern[d_ncbps];
      int s = std::max(d_ncbps / (d_data_size * 2), 1);

      for(int j = 0; j < d_ncbps; j++){
        permutation_pattern[j] = s * (j / s) + ((j + int(floor(16.0 * j / d_ncbps))) % s);
      }
      for(int j = 0; j < d_ncbps; j++){
        interleave_pattern[j] = 16 * permutation_pattern[j] - (d_ncbps - 1) * int(floor(16.0 * permutation_pattern[j] / d_ncbps));
      }

      int nsym = d_to_copy / d_data_size;
      for(int j = 0; j < nsym; j++){
        for(int i = 0; i < d_ncbps; i++){
          d_deinterleave_buf[j * d_ncbps + interleave_pattern[i]] = d_bits[j * d_ncbps + i];
        }
      }
    }

    void
    ofdm_parse_payload_impl::deconvolute(){
      itpp::Punctured_Convolutional_Code code;
      itpp::ivec generator(2);
      generator(0)=0133;
      generator(1)=0171;
      code.set_generator_polynomials(generator, 7);

      itpp::bmat puncture_matrix;
      switch(d_rate){
        case 11:
        case 10:
        case 9:
          puncture_matrix = "1 1; 1 1;";
          break;
        case 15:
        case 14:
        case 13:
        case 12:
          puncture_matrix = "1 1 0; 1 0 1;";
          break;
        case 8:
          puncture_matrix = "1 1; 1 0;";
          break;
      }
      code.set_puncture_matrix(puncture_matrix);
      code.set_truncation_length(30);
      int n_encoded_bits = d_to_copy / d_data_size * d_ncbps;
      itpp::vec rx_signal(d_deinterleave_buf, n_encoded_bits);
      code.reset();
      d_decoded_bits.set_length(n_encoded_bits);
      code.decode_tail(rx_signal, d_decoded_bits);
    }

    void
    ofdm_parse_payload_impl::descramble(){
      // S(x) = x^-7 + x^-4 + 1
      int state = 0;
      for(int i = 0; i < 7; i++){
        if(d_decoded_bits(i)){
          state |= 1 << (6 - i);
        }
      }
      
      int feedback;

      for(int i = 7; i < d_decoded_bits.size(); i++){
        feedback = (!!(state & 64)) ^ (!!(state & 8));
        d_data_bits[i] = feedback ^ (int)d_decoded_bits(i);
        state = ((state << 1) & 0x7E) | feedback;
      }

      for(int i = 0; i < d_decoded_bits.size(); i++){
        int bit = i % 8;
        int byte = i / 8;
        if(bit == 0){
          d_data_bytes[byte] = 0;
        }

        if(d_data_bits[i]){
          d_data_bytes[byte] |= (1 << bit);
        }
      }
    }
  } /* namespace wifirecv */
} /* namespace gr */

