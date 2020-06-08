/* -*- c++ -*- */
/*
 * Copyright 2008,2010,2013,2017,2018 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "moving_average_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <numeric>

namespace gr {
namespace blocks {

namespace {
template <typename T>
inline T volk_sum(T* tmp, const T* in, int num)
{
    return std::accumulate(in, in + num, T{});
}

template <>
inline float volk_sum<float>(float* tmp, const float* in, int num)
{
    volk_32f_accumulator_s32f(tmp, in, num);
    return tmp[0];
}


template <typename T>
inline void volk_add(T* out, const T* add1, const T* add2, unsigned int num)
{
    for (unsigned int elem = 0; elem < num; elem++) {
        out[elem] = add1[elem] + add2[elem];
    }
}

template <>
inline void
volk_add<float>(float* out, const float* add1, const float* add2, unsigned int num)
{
    volk_32f_x2_add_32f(out, add1, add2, num);
}

template <>
inline void volk_add<gr_complex>(gr_complex* out,
                                 const gr_complex* add1,
                                 const gr_complex* add2,
                                 unsigned int num)
{
    volk_32fc_x2_add_32fc(out, add1, add2, num);
}

template <typename T>
inline void volk_sub(T* out, const T* in, const T* sub, unsigned int num)
{
    for (unsigned int elem = 0; elem < num; elem++) {
        out[elem] = in[elem] - sub[elem];
    }
}

template <>
inline void
volk_sub<float>(float* out, const float* in, const float* sub, unsigned int num)
{
    volk_32f_x2_subtract_32f(out, in, sub, num);
}

// TODO: Add volk_sub specialization for gr_complex?

template <typename T>
inline void volk_scale(T* out, const T scale, unsigned int num)
{
    for (unsigned int elem = 0; elem < num; elem++) {
        out[elem] *= scale;
    }
}

template <>
inline void volk_scale<float>(float* out, const float scale, unsigned int num)
{
    volk_32f_s32f_normalize(out, scale, num);
}

template <>
inline void
volk_scale<gr_complex>(gr_complex* out, const gr_complex scale, unsigned int num)
{
    volk_32fc_s32fc_multiply_32fc(out, out, scale, num);
}

} // namespace

template <class T>
typename moving_average<T>::sptr
moving_average<T>::make(int length, T scale, int max_iter, unsigned int vlen)
{
    return gnuradio::get_initial_sptr(
        new moving_average_impl<T>(length, scale, max_iter, vlen));
}

template <class T>
moving_average_impl<T>::moving_average_impl(int length,
                                            T scale,
                                            int max_iter,
                                            unsigned int vlen)
    : sync_block("moving_average",
                 io_signature::make(1, 1, sizeof(T) * vlen),
                 io_signature::make(1, 1, sizeof(T) * vlen)),
      d_length(length),
      d_scratch(length),
      d_scale(scale),
      d_max_iter(max_iter),
      d_vlen(vlen),
      d_new_length(length),
      d_new_scale(scale),
      d_updated(false)
{
    this->set_history(length);
    // we store this vector so that work() doesn't spend its time allocating and freeing
    // vector storage
    if (d_vlen > 1) {
        d_sum = volk::vector<T>(d_vlen);
    }
}

template <class T>
moving_average_impl<T>::~moving_average_impl()
{
}

template <class T>
void moving_average_impl<T>::set_length_and_scale(int length, T scale)
{
    d_new_length = length;
    d_new_scale = scale;
    d_updated = true;
}

template <class T>
void moving_average_impl<T>::set_length(int length)
{
    d_new_length = length;
    d_updated = true;
}

template <class T>
void moving_average_impl<T>::set_scale(T scale)
{
    d_new_scale = scale;
    d_updated = true;
}

template <class T>
int moving_average_impl<T>::work(int noutput_items,
                                 gr_vector_const_void_star& input_items,
                                 gr_vector_void_star& output_items)
{
    if (d_updated) {
        d_length = d_new_length;
        d_scale = d_new_scale;
        this->set_history(d_length);
        d_updated = false;
        return 0; // history requirements might have changed
    }

    const T* in = static_cast<const T*>(input_items[0]);
    T* out = static_cast<T*>(output_items[0]);

    const unsigned int num_iter = std::min(noutput_items, d_max_iter);
    if (d_vlen == 1) {
        T sum = volk_sum(d_scratch.data(), in, d_length - 1);

        for (unsigned int i = 0; i < num_iter; i++) {
            sum += in[i + d_length - 1];
            out[i] = sum * d_scale;
            sum -= in[i];
        }

    } else { // d_vlen > 1
        volk_add(d_sum.data(), in, &in[d_vlen], d_vlen);
        for (int i = 2; i < d_length - 1; i++) {
            volk_add(d_sum.data(), d_sum.data(), &in[i * d_vlen], d_vlen);
        }

        for (unsigned int i = 0; i < num_iter; i++) {
            T* t = &out[i * d_vlen];
            volk_add(t, d_sum.data(), &in[(i + d_length - 1) * d_vlen], d_vlen);
            volk_scale(t, d_scale, d_vlen);
            volk_sub(d_sum.data(), t, &in[i * d_vlen], d_vlen);
        }
        // Benchmarking shows that it's faster to scale inside the loop than
        // here outside.
        // Otherwise: volk_scale(out, d_scale, num_iter * d_vlen);
    }
    return num_iter;
}

template class moving_average<std::int16_t>;
template class moving_average<std::int32_t>;
template class moving_average<float>;
template class moving_average<gr_complex>;
} /* namespace blocks */
} /* namespace gr */
