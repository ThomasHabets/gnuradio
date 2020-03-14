/* -*- c++ -*- */
/*
 * Copyright (C) 2002,2012,2017 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "gnuradio/filter/interp_differentiator_taps.h"
#include <gnuradio/filter/mmse_interp_differentiator_cc.h>
#include <stdexcept>

namespace gr {
namespace filter {

mmse_interp_differentiator_cc::mmse_interp_differentiator_cc()
{
    filters.reserve(DNSTEPS + 1);

    for (int i = 0; i < DNSTEPS + 1; i++) {
        std::vector<float> t(&Dtaps[i][0], &Dtaps[i][DNTAPS]);
        filters.emplace_back(1, t);
    }
}

mmse_interp_differentiator_cc::~mmse_interp_differentiator_cc() {}

unsigned mmse_interp_differentiator_cc::ntaps() const { return DNTAPS; }

unsigned mmse_interp_differentiator_cc::nsteps() const { return DNSTEPS; }

gr_complex mmse_interp_differentiator_cc::differentiate(const gr_complex input[],
                                                        float mu) const
{
    int imu = (int)rint(mu * DNSTEPS);

    if ((imu < 0) || (imu > DNSTEPS)) {
        throw std::runtime_error("mmse_interp_differentiator_cc: imu out of bounds.");
    }

    return filters[imu].filter(input);
}

} /* namespace filter */
} /* namespace gr */
