/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//
// adc.c - Driver for the adc Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup adc_api adc
//! @{
//
//*****************************************************************************

#include "hw_memmap.h"

#ifdef __MSP430_HAS_ADC__
#include "adc.h"

#include <assert.h>

void ADC_init(uint16_t baseAddress,
              uint16_t sampleHoldSignalSourceSelect,
              uint8_t clockSourceSelect,
              uint16_t clockSourceDivider)
{
    //Turn OFF ADC Module & Clear Interrupt Registers
    HWREG16(baseAddress + OFS_ADCCTL0) &= ~(ADCON + ADCENC + ADCSC);
    HWREG16(baseAddress + OFS_ADCIE) &= 0x0000;  //Reset ALL interrupt enables
    HWREG16(baseAddress + OFS_ADCIFG) &= 0x0000; //Reset ALL interrupt flags

    //Set ADC Control 1
    HWREG16(baseAddress + OFS_ADCCTL1) = sampleHoldSignalSourceSelect //Setup the Sample-and-Hold Source
                                         + (clockSourceDivider & ADCDIV_7) //Set Clock Divider
                                         + clockSourceSelect; //Setup Clock Source

    //Set ADC Control 2
    HWREG16(baseAddress + OFS_ADCCTL2) = (clockSourceDivider & ADCPDIV_3) //Set Clock Pre-Divider
                                         + ADCRES; //Default resolution to 10-bits
}

void ADC_enable(uint16_t baseAddress)
{
    //Reset the ADCON bit to enable the ADC Module
    HWREG16(baseAddress + OFS_ADCCTL0) |= ADCON;
}

void ADC_disable(uint16_t baseAddress)
{
    //Set the ADCON bit to disable the ADC Module
    HWREG16(baseAddress + OFS_ADCCTL0) &= ~ADCON;
}

void ADC_setupSamplingTimer(uint16_t baseAddress,
                            uint16_t clockCycleHoldCount,
                            uint16_t multipleSamplesEnabled)
{
    HWREG16(baseAddress + OFS_ADCCTL1) |= ADCSHP;

    //Reset and Set CB Control 0 Bits
    HWREG16(baseAddress + OFS_ADCCTL0) &= ~(ADCSHT_15 + ADCMSC);
    HWREG16(baseAddress +
            OFS_ADCCTL0) |= clockCycleHoldCount + multipleSamplesEnabled;
}

void ADC_disableSamplingTimer(uint16_t baseAddress)
{
    HWREG16(baseAddress + OFS_ADCCTL1) &= ~(ADCSHP);
}

void ADC_configureMemory(uint16_t baseAddress,
                         uint8_t inputSourceSelect,
                         uint8_t positiveRefVoltageSourceSelect,
                         uint8_t negativeRefVoltageSourceSelect)
{
    //Make sure the ENC bit is cleared before configuring a Memory Buffer Control
    assert(!(HWREG16(baseAddress + OFS_ADCCTL0) & ADCENC));

    if(!(HWREG16(baseAddress + OFS_ADCCTL0) & ADCENC))
    {
#ifdef ADCPCTL9
        //Enable ADC input pin
        if(inputSourceSelect < ADCINCH_10)
        {
            HWREG16(SYS_BASE + OFS_SYSCFG2) |= (1 << inputSourceSelect);
        }
#elif ADCPCTL7
        if(inputSourceSelect < ADCINCH_8)
        {
            HWREG16(SYS_BASE + OFS_SYSCFG2) |= (1 << inputSourceSelect);
        }
#endif

        //Reset and Set the Memory Buffer Control Bits
        HWREG16(baseAddress +
                OFS_ADCMCTL0) = inputSourceSelect +
                                positiveRefVoltageSourceSelect
                                + negativeRefVoltageSourceSelect;
    }
}

void ADC_enableInterrupt(uint16_t baseAddress,
                         uint8_t interruptMask)
{
    HWREG16(baseAddress + OFS_ADCIE) |= interruptMask;
}

void ADC_disableInterrupt(uint16_t baseAddress,
                          uint8_t interruptMask)
{
    HWREG16(baseAddress + OFS_ADCIE) &= ~(interruptMask);
}

void ADC_clearInterrupt(uint16_t baseAddress,
                        uint8_t interruptFlagMask)
{
    HWREG16(baseAddress + OFS_ADCIFG) &= ~(interruptFlagMask);
}

uint8_t ADC_getInterruptStatus(uint16_t baseAddress,
                               uint8_t interruptFlagMask)
{
    return (HWREG16(baseAddress + OFS_ADCIFG) & interruptFlagMask);
}

void ADC_startConversion(uint16_t baseAddress,
                         uint8_t conversionSequenceModeSelect)
{
    //Reset the ENC bit to set the conversion mode sequence
    HWREG16(baseAddress + OFS_ADCCTL0) &= ~(ADCENC);

    HWREG16(baseAddress + OFS_ADCCTL1) |= conversionSequenceModeSelect;
    HWREG16(baseAddress + OFS_ADCCTL0) |= ADCENC | ADCSC;
}

void ADC_disableConversions(uint16_t baseAddress,
                            bool preempt)
{
    if(preempt)
    {
        HWREG16(baseAddress + OFS_ADCCTL1) &= ~(ADCCONSEQ_3);
        //Reset conversion sequence mode to single-channel, single-conversion
    }
    else if(!(HWREG16(baseAddress + OFS_ADCCTL1) & ADCCONSEQ_3))
    {
        //To prevent preemoption of a single-channel, single-conversion we must
        //wait for the ADC core to finish the conversion.
        while(HWREG16(baseAddress + OFS_ADCCTL1) & ADCBUSY)
        {
            ;
        }
    }

    HWREG16(baseAddress + OFS_ADCCTL0) &= ~(ADCENC);
}

int16_t ADC_getResults(uint16_t baseAddress)
{
    return((int16_t)(HWREG16(baseAddress + OFS_ADCMEM0)));
}

void ADC_setResolution(uint16_t baseAddress,
                       uint8_t resolutionSelect)
{
    HWREG16(baseAddress + OFS_ADCCTL2) &= ~(ADCRES);
    HWREG16(baseAddress + OFS_ADCCTL2) |= resolutionSelect;
}

void ADC_setSampleHoldSignalInversion(uint16_t baseAddress,
                                      uint16_t invertedSignal)
{
    HWREG16(baseAddress + OFS_ADCCTL1) &= ~(ADCISSH);
    HWREG16(baseAddress + OFS_ADCCTL1) |= invertedSignal;
}

void ADC_setDataReadBackFormat(uint16_t baseAddress,
                               uint16_t readBackFormat)
{
    HWREG16(baseAddress + OFS_ADCCTL2) &= ~(ADCDF);
    HWREG16(baseAddress + OFS_ADCCTL2) |= readBackFormat;
}

void ADC_setReferenceBufferSamplingRate(uint16_t baseAddress,
                                        uint16_t samplingRateSelect)
{
    HWREG16(baseAddress + OFS_ADCCTL2) &= ~(ADCSR);
    HWREG16(baseAddress + OFS_ADCCTL2) |= samplingRateSelect;
}

void ADC_setWindowComp(uint16_t baseAddress,
                       uint16_t highThreshold,
                       uint16_t lowThreshold)
{
    HWREG16(baseAddress + OFS_ADCHI) = highThreshold;
    HWREG16(baseAddress + OFS_ADCLO) = lowThreshold;
}

uint32_t ADC_getMemoryAddressForDMA(uint16_t baseAddress)
{
    return (baseAddress + OFS_ADCMEM0);
}

uint8_t ADC_isBusy(uint16_t baseAddress)
{
    return (HWREG16(baseAddress + OFS_ADCCTL1) & ADCBUSY);
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for adc_api
//! @}
//
//*****************************************************************************
