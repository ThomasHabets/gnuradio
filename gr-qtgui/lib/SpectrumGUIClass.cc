/* -*- c++ -*- */
/*
 * Copyright 2008-2011 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef SPECTRUM_GUI_CLASS_CPP
#define SPECTRUM_GUI_CLASS_CPP

#include <gnuradio/qtgui/SpectrumGUIClass.h>

// Added by qt3to4:
#include <volk/volk.h>
#include <QEvent>

const long SpectrumGUIClass::MAX_FFT_SIZE = 32768;
const long SpectrumGUIClass::MIN_FFT_SIZE = 256;

SpectrumGUIClass::SpectrumGUIClass(const uint64_t maxDataSize,
                                   const uint64_t fftSize,
                                   const double newCenterFrequency,
                                   const double newStartFrequency,
                                   const double newStopFrequency)
    : d_dataPoints(std::max(static_cast<uint64_t>(2), maxDataSize)),
      d_centerFrequency(newCenterFrequency),
      d_startFrequency(newStartFrequency),
      d_stopFrequency(newStopFrequency),
      d_lastDataPointCount(d_dataPoints),
      d_fftSize(fftSize),
      d_fftPoints(d_dataPoints),
      d_realTimeDomainPoints(d_dataPoints),
      d_imagTimeDomainPoints(d_dataPoints)
{
}

SpectrumGUIClass::~SpectrumGUIClass()
{
    // We don't need to delete this since as a QWidget, it is supposed to be destroyed
    // with it's parent. Deleting it causes a segmentation fault, and not deleting it
    // does not leave any extra memory.
    // if(getWindowOpenFlag()){
    // delete d_spectrumDisplayForm;
    //}
}

void SpectrumGUIClass::openSpectrumWindow(QWidget* parent,
                                          const bool frequency,
                                          const bool waterfall,
                                          const bool time,
                                          const bool constellation)
{
    d_mutex.lock();

    if (!d_windowOpennedFlag) {
        // Called from the Event Thread
        d_spectrumDisplayForm = new SpectrumDisplayForm(parent);

        // Toggle Windows on/off
        d_spectrumDisplayForm->toggleTabFrequency(frequency);
        d_spectrumDisplayForm->toggleTabWaterfall(waterfall);
        d_spectrumDisplayForm->toggleTabTime(time);
        d_spectrumDisplayForm->toggleTabConstellation(constellation);

        d_windowOpennedFlag = true;

        d_spectrumDisplayForm->setSystem(this, d_dataPoints, d_fftSize);

        qApp->processEvents();
    }
    d_mutex.unlock();


    setDisplayTitle(d_title);
    reset();

    qApp->postEvent(d_spectrumDisplayForm, new QEvent(QEvent::Type(QEvent::User + 3)));

    d_lastGUIUpdateTime = 0;

    // Draw Blank Display
    updateWindow(false, NULL, 0, NULL, 0, NULL, 0, gr::high_res_timer_now(), true);

    // Set up the initial frequency axis settings
    setFrequencyRange(d_centerFrequency, d_startFrequency, d_stopFrequency);

    // GUI Thread only
    qApp->processEvents();

    // Set the FFT Size combo box to display the right number
    int idx =
        d_spectrumDisplayForm->FFTSizeComboBox->findText(QString("%1").arg(d_fftSize));
    d_spectrumDisplayForm->FFTSizeComboBox->setCurrentIndex(idx);
}

void SpectrumGUIClass::reset()
{
    if (getWindowOpenFlag()) {
        qApp->postEvent(d_spectrumDisplayForm,
                        new SpectrumFrequencyRangeEvent(
                            d_centerFrequency, d_startFrequency, d_stopFrequency));
        qApp->postEvent(d_spectrumDisplayForm, new SpectrumWindowResetEvent());
    }
    d_droppedEntriesCount = 0;
    // Call the following function from the Spectrum Window Reset Event window
    // ResetPendingGUIUpdateEvents();
}

void SpectrumGUIClass::setDisplayTitle(const std::string newString)
{
    d_title.assign(newString);

    if (getWindowOpenFlag()) {
        qApp->postEvent(d_spectrumDisplayForm,
                        new SpectrumWindowCaptionEvent(d_title.c_str()));
    }
}

bool SpectrumGUIClass::getWindowOpenFlag()
{
    gr::thread::scoped_lock lock(d_mutex);
    bool returnFlag = false;
    returnFlag = d_windowOpennedFlag;
    return returnFlag;
}


void SpectrumGUIClass::setWindowOpenFlag(const bool newFlag)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_windowOpennedFlag = newFlag;
}

void SpectrumGUIClass::setFrequencyRange(const double centerFreq,
                                         const double startFreq,
                                         const double stopFreq)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_centerFrequency = centerFreq;
    d_startFrequency = startFreq;
    d_stopFrequency = stopFreq;

    qApp->postEvent(d_spectrumDisplayForm,
                    new SpectrumFrequencyRangeEvent(
                        d_centerFrequency, d_startFrequency, d_stopFrequency));
}

double SpectrumGUIClass::getStartFrequency()
{
    gr::thread::scoped_lock lock(d_mutex);
    double returnValue = 0.0;
    returnValue = d_startFrequency;
    return returnValue;
}

double SpectrumGUIClass::getStopFrequency()
{
    gr::thread::scoped_lock lock(d_mutex);
    double returnValue = 0.0;
    returnValue = d_stopFrequency;
    return returnValue;
}

double SpectrumGUIClass::getCenterFrequency()
{
    gr::thread::scoped_lock lock(d_mutex);
    double returnValue = 0.0;
    returnValue = d_centerFrequency;
    return returnValue;
}


void SpectrumGUIClass::updateWindow(const bool updateDisplayFlag,
                                    const float* fftBuffer,
                                    const uint64_t inputBufferSize,
                                    const float* realTimeDomainData,
                                    const uint64_t realTimeDomainDataSize,
                                    const float* complexTimeDomainData,
                                    const uint64_t complexTimeDomainDataSize,
                                    const gr::high_res_timer_type timestamp,
                                    const bool lastOfMultipleFFTUpdateFlag)
{
    // gr::thread::scoped_lock lock(d_mutex);
    uint64_t bufferSize = std::min(d_dataPoints, inputBufferSize);
    bool repeatDataFlag = false;
    int64_t timeDomainBufferSize = 0;

    if (updateDisplayFlag) {
        if ((fftBuffer != NULL) && (bufferSize > 0)) {
            memcpy(d_fftPoints.data(), fftBuffer, bufferSize * sizeof(d_fftPoints[0]));
        }

        // ALL OF THIS SHIT SHOULD BE COMBINED WITH THE FFTSHIFT
        // USE VOLK_32FC_DEINTERLEAVE_64F_X2_A TO GET REAL/IMAG FROM COMPLEX32
        // Can't do a memcpy since this is going from float to double data type
        if ((realTimeDomainData != NULL) && (realTimeDomainDataSize > 0)) {
            const float* realTimeDomainDataPtr = realTimeDomainData;

            double* realTimeDomainPointsPtr = d_realTimeDomainPoints.data();
            timeDomainBufferSize = realTimeDomainDataSize;

            memset(d_imagTimeDomainPoints.data(),
                   0x0,
                   realTimeDomainDataSize * sizeof(d_imagTimeDomainPoints[0]));
            for (uint64_t number = 0; number < realTimeDomainDataSize; number++) {
                *realTimeDomainPointsPtr++ = *realTimeDomainDataPtr++;
            }
        }

        if ((complexTimeDomainData != NULL) && (complexTimeDomainDataSize > 0)) {
            volk_32fc_deinterleave_64f_x2(d_realTimeDomainPoints.data(),
                                          d_imagTimeDomainPoints.data(),
                                          (const lv_32fc_t*)complexTimeDomainData,
                                          complexTimeDomainDataSize);
            timeDomainBufferSize = complexTimeDomainDataSize;
        }
    }

    // If bufferSize is zero, then just update the display by sending over the old data
    if (bufferSize < 1) {
        bufferSize = d_lastDataPointCount;
        repeatDataFlag = true;
    } else {
        // Since there is data this time, update the count
        d_lastDataPointCount = bufferSize;
    }

    const gr::high_res_timer_type currentTime = gr::high_res_timer_now();
    const gr::high_res_timer_type lastUpdateGUITime = getLastGUIUpdateTime();

    if ((currentTime - lastUpdateGUITime >
         (4 * d_updateTime) * gr::high_res_timer_tps()) &&
        (getPendingGUIUpdateEvents() > 0) && lastUpdateGUITime != 0) {
        // Do not update the display if too much data is pending to be displayed
        d_droppedEntriesCount++;
    } else {
        // Draw the Data
        incrementPendingGUIUpdateEvents();
        qApp->postEvent(d_spectrumDisplayForm,
                        new SpectrumUpdateEvent(d_fftPoints.data(),
                                                bufferSize,
                                                d_realTimeDomainPoints.data(),
                                                d_imagTimeDomainPoints.data(),
                                                timeDomainBufferSize,
                                                timestamp,
                                                repeatDataFlag,
                                                lastOfMultipleFFTUpdateFlag,
                                                currentTime,
                                                d_droppedEntriesCount));

        // Only reset the dropped entries counter if this is not
        // repeat data since repeat data is dropped by the display systems
        if (!repeatDataFlag) {
            d_droppedEntriesCount = 0;
        }
    }
}

float SpectrumGUIClass::getPowerValue()
{
    gr::thread::scoped_lock lock(d_mutex);
    float returnValue = 0;
    returnValue = d_powerValue;
    return returnValue;
}

void SpectrumGUIClass::setPowerValue(const float value)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_powerValue = value;
}

int SpectrumGUIClass::getWindowType()
{
    gr::thread::scoped_lock lock(d_mutex);
    int returnValue = 0;
    returnValue = d_windowType;
    return returnValue;
}

void SpectrumGUIClass::setWindowType(const int newType)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_windowType = newType;
}

int SpectrumGUIClass::getFFTSize()
{
    int returnValue = 0;
    returnValue = d_fftSize;
    return returnValue;
}

int SpectrumGUIClass::getFFTSizeIndex()
{
    gr::thread::scoped_lock lock(d_mutex);
    int fftsize = getFFTSize();
    int rv = 0;
    switch (fftsize) {
    case (1024):
        rv = 0;
        break;
    case (2048):
        rv = 1;
        break;
    case (4096):
        rv = 2;
        break;
    case (8192):
        rv = 3;
        break;
    case (16384):
        rv = 3;
        break;
    case (32768):
        rv = 3;
        break;
    default:
        rv = 0;
        break;
    }
    return rv;
}

void SpectrumGUIClass::setFFTSize(const int newSize)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_fftSize = newSize;
}

gr::high_res_timer_type SpectrumGUIClass::getLastGUIUpdateTime()
{
    gr::thread::scoped_lock lock(d_mutex);
    gr::high_res_timer_type returnValue;
    returnValue = d_lastGUIUpdateTime;
    return returnValue;
}

void SpectrumGUIClass::setLastGUIUpdateTime(const gr::high_res_timer_type newTime)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_lastGUIUpdateTime = newTime;
}

unsigned int SpectrumGUIClass::getPendingGUIUpdateEvents()
{
    gr::thread::scoped_lock lock(d_mutex);
    unsigned int returnValue = 0;
    returnValue = d_pendingGUIUpdateEventsCount;
    return returnValue;
}

void SpectrumGUIClass::incrementPendingGUIUpdateEvents()
{
    gr::thread::scoped_lock lock(d_mutex);
    d_pendingGUIUpdateEventsCount++;
}

void SpectrumGUIClass::decrementPendingGUIUpdateEvents()
{
    gr::thread::scoped_lock lock(d_mutex);
    if (d_pendingGUIUpdateEventsCount > 0) {
        d_pendingGUIUpdateEventsCount--;
    }
}

void SpectrumGUIClass::resetPendingGUIUpdateEvents()
{
    gr::thread::scoped_lock lock(d_mutex);
    d_pendingGUIUpdateEventsCount = 0;
}


QWidget* SpectrumGUIClass::qwidget()
{
    gr::thread::scoped_lock lock(d_mutex);
    return (QWidget*)d_spectrumDisplayForm;
}

void SpectrumGUIClass::setTimeDomainAxis(double min, double max)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_spectrumDisplayForm->setTimeDomainAxis(min, max);
}

void SpectrumGUIClass::setConstellationAxis(double xmin,
                                            double xmax,
                                            double ymin,
                                            double ymax)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_spectrumDisplayForm->setConstellationAxis(xmin, xmax, ymin, ymax);
}

void SpectrumGUIClass::setConstellationPenSize(int size)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_spectrumDisplayForm->setConstellationPenSize(size);
}


void SpectrumGUIClass::setFrequencyAxis(double min, double max)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_spectrumDisplayForm->setFrequencyAxis(min, max);
}

void SpectrumGUIClass::setUpdateTime(double t)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_updateTime = t;
    d_spectrumDisplayForm->setUpdateTime(d_updateTime);
}

void SpectrumGUIClass::enableRFFreq(bool en)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_spectrumDisplayForm->toggleRFFrequencies(en);
}

bool SpectrumGUIClass::checkClicked()
{
    gr::thread::scoped_lock lock(d_mutex);
    return d_spectrumDisplayForm->checkClicked();
}

float SpectrumGUIClass::getClickedFreq()
{
    gr::thread::scoped_lock lock(d_mutex);
    return d_spectrumDisplayForm->getClickedFreq();
}

#endif /* SPECTRUM_GUI_CLASS_CPP */
