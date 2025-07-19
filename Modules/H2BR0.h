#pragma once

#include "BOS.h"

/**************************************************************************************************/
/* H2BR0 User Interface ***************************************************************************/
/**************************************************************************************************/
struct H2BR0_ECG
{
    BOSStatus status;
    float sample = 0.0f;
    float filteredSample = 0.0f;
};

struct H2BR0_EOG
{
    BOSStatus status;
    float sample = 0.0f;
    float filteredSample = 0.0f;
};

struct H2BR0_EEG
{
    BOSStatus status;
    float sample = 0.0f;
    float filteredSample = 0.0f;
};

struct H2BR0_EMG
{
    BOSStatus status;
    float sample = 0.0f;
    float filteredSample = 0.0f;
    float rectifiedSample = 0.0f;
    float envelopeSample = 0.0f;
};

class H2BR0
{
public:
    static std::promise<H2BR0_ECG> ECGPromise;
    static std::promise<H2BR0_EOG> EOGPromise;
    static std::promise<H2BR0_EEG> EEGPromise;
    static std::promise<H2BR0_EMG> EMGPromise;

    static H2BR0_ECG RequestECG(uint8_t moduleID);
    static H2BR0_EOG RequestEOG(uint8_t moduleID);
    static H2BR0_EEG RequestEEG(uint8_t moduleID);
    static H2BR0_EMG RequestEMG(uint8_t moduleID);
};