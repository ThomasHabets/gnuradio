/* -*- c++ -*- */
/*
 * Copyright 2012 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef FREQ_DISPLAY_FORM_H
#define FREQ_DISPLAY_FORM_H

#include <gnuradio/filter/firdes.h>
#include <gnuradio/qtgui/FrequencyDisplayPlot.h>
#include <gnuradio/qtgui/spectrumUpdateEvents.h>
#include <QtGui/QtGui>
#include <vector>

#include <gnuradio/qtgui/displayform.h>

class FreqControlPanel;

/*!
 * \brief DisplayForm child for managing frequency (PSD) plots.
 * \ingroup qtgui_blk
 */
class FreqDisplayForm : public DisplayForm
{
    Q_OBJECT

public:
    FreqDisplayForm(int nplots = 1, QWidget* parent = 0);
    ~FreqDisplayForm();

    FrequencyDisplayPlot* getPlot();

    int getFFTSize() const;
    float getFFTAverage() const;
    gr::filter::firdes::win_type getFFTWindowType() const;

    // Trigger methods
    gr::qtgui::trigger_mode getTriggerMode() const;
    float getTriggerLevel() const;
    int getTriggerChannel() const;
    std::string getTriggerTagKey() const;


    // returns the frequency that was last double-clicked on by the user
    float getClickedFreq() const;

    // checks if there was a double-click event; reset if there was
    bool checkClicked();

public slots:
    void customEvent(QEvent* e);

    void setSampleRate(const QString& samprate);
    void setFFTSize(const int);
    void setFFTAverage(const float);
    void setFFTWindowType(const gr::filter::firdes::win_type);

    void setFrequencyRange(const double centerfreq, const double bandwidth);
    void setYaxis(double min, double max);
    void setYLabel(const std::string& label, const std::string& unit = "");
    void setYMax(const QString& m);
    void setYMin(const QString& m);
    void autoScale(bool en);
    void autoScaleShot();
    void setPlotPosHalf(bool half);
    void clearMaxHold();
    void clearMinHold();

    // Trigger slots
    void updateTrigger(gr::qtgui::trigger_mode mode);
    void setTriggerMode(gr::qtgui::trigger_mode mode);
    void setTriggerLevel(QString s);
    void setTriggerLevel(float level);
    void setTriggerChannel(int chan);
    void setTriggerTagKey(QString s);
    void setTriggerTagKey(const std::string& s);

    void setupControlPanel(bool en);
    void setupControlPanel();
    void teardownControlPanel();

    void notifyYAxisPlus();
    void notifyYAxisMinus();
    void notifyYRangePlus();
    void notifyYRangeMinus();
    void notifyFFTSize(const QString& s);
    void notifyFFTWindow(const QString& s);
    void notifyMaxHold(bool en);
    void notifyMinHold(bool en);
    void notifyTriggerMode(const QString& mode);
    void notifyTriggerLevelPlus();
    void notifyTriggerLevelMinus();

signals:
    void signalFFTSize(int size);
    void signalFFTWindow(gr::filter::firdes::win_type win);
    void signalClearMaxData();
    void signalClearMinData();
    void signalSetMaxFFTVisible(bool en);
    void signalSetMinFFTVisible(bool en);
    void signalTriggerMode(gr::qtgui::trigger_mode mode);
    void signalTriggerLevel(float level);
    void signalReplot();


private slots:
    void newData(const QEvent* updateEvent);
    void onPlotPointSelected(const QPointF p);

private:
    uint64_t d_num_real_data_points;
    QIntValidator d_int_validator;

    double d_samp_rate, d_center_freq;
    int d_fftsize;
    float d_fftavg;
    gr::filter::firdes::win_type d_fftwintype;
    double d_units;

    bool d_clicked;
    double d_clicked_freq;

    FFTSizeMenu* d_sizemenu;                // Managed by QT.
    FFTAverageMenu* d_avgmenu;              // Managed by QT.
    FFTWindowMenu* d_winmenu;               // Managed by QT.
    QAction *d_minhold_act, *d_maxhold_act; // Managed by QT.

    QMenu* d_triggermenu;                  // Managed by QT.
    TriggerModeMenu* d_tr_mode_menu;       // Managed by QT.
    PopupMenu* d_tr_level_act;             // Managed by QT.
    TriggerChannelMenu* d_tr_channel_menu; // Managed by QT.
    PopupMenu* d_tr_tag_key_act;           // Managed by QT.

    gr::qtgui::trigger_mode d_trig_mode;
    float d_trig_level;
    int d_trig_channel;
    std::string d_trig_tag_key;

    QAction* d_controlpanelmenu; // Managed by QT.
    std::unique_ptr<FreqControlPanel> d_controlpanel;
};

#endif /* FREQ_DISPLAY_FORM_H */
