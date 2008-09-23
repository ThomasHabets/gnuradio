/* -*- c++ -*- */
/*
 * Copyright 2008 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <usrp2_sink_16sc.h>
#include <usrp2/metadata.h>
#include <gr_io_signature.h>
#include <iostream>

usrp2_sink_16sc_sptr
usrp2_make_sink_16sc(const std::string &ifc, const std::string &mac_addr) 
  throw (std::runtime_error)
{
  return usrp2_sink_16sc_sptr(new usrp2_sink_16sc(ifc, mac_addr));
}

usrp2_sink_16sc::usrp2_sink_16sc(const std::string &ifc, const std::string &mac_addr) 
  throw (std::runtime_error)
  : usrp2_sink_base("usrp2_sink_16sc",
		    gr_make_io_signature(1, 1, sizeof(std::complex<int16_t>)),
		    ifc, mac_addr)
{
  // NOP
}

usrp2_sink_16sc::~usrp2_sink_16sc()
{
  // NOP
}

int
usrp2_sink_16sc::work(int noutput_items,
		      gr_vector_const_void_star &input_items,
		      gr_vector_void_star &output_items)
{
  std::complex<int16_t> *in = (std::complex<int16_t> *)input_items[0];

  usrp2::tx_metadata metadata;
  metadata.timestamp = -1;
  metadata.send_now = 1;
  metadata.start_of_burst = 1;

  bool ok = d_u2->tx_complex_int16(0,  // FIXME: someday, streams will have channel numbers
				   in, noutput_items, &metadata);
  if (!ok)
    std::cerr << "usrp2_sink_16sc: tx_complex_int16 failed" << std::endl;

  return noutput_items;
}
