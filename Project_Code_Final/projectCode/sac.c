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
// sac.c - Driver for the sac Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup sac_api sac
//! @{
//
//*****************************************************************************

#include "hw_memmap.h"

#ifdef __MSP430_HAS_SACx_CONFIG_1__
#include "sac.h"

#include <assert.h>

void SAC_OA_init(uint16_t baseAddress,
                 uint16_t positiveInput,
                 uint16_t negativeInput)
{
    HWREG16(baseAddress + OFS_SAC0OA) &= ~(PSEL1 | PSEL0 | NSEL1 | NSEL0);

    if(positiveInput == SAC_OA_POSITIVE_INPUT_SOURCE_DISCONNECTED)
    {
        HWREG16(baseAddress + OFS_SAC0OA) &= ~PMUXEN;
    }
    else
    {
        HWREG16(baseAddress + OFS_SAC0OA) |= PMUXEN | positiveInput;
    }

    if(negativeInput == SAC_OA_NEGATIVE_INPUT_SOURCE_DISCONNECTED)
    {
        HWREG16(baseAddress + OFS_SAC0OA) &= ~NMUXEN;
    }
    else
    {
        HWREG16(baseAddress + OFS_SAC0OA) |= NMUXEN | negativeInput;
    }
}

void SAC_OA_selectPowerMode(uint16_t baseAddress,
                            uint16_t powerMode)
{
    HWREG16(baseAddress + OFS_SAC0OA) &= ~OAPM;
    HWREG16(baseAddress + OFS_SAC0OA) |= powerMode;
}

void SAC_OA_enable(uint16_t baseAddress)
{
    HWREG16(baseAddress + OFS_SAC0OA) |= OAEN;
}

void SAC_OA_disable(uint16_t baseAddress)
{
    HWREG16(baseAddress + OFS_SAC0OA) &= ~OAEN;
}

void SAC_enable(uint16_t baseAddress)
{
    HWREG16(baseAddress + OFS_SAC0OA) |= SACEN;
}

void SAC_disable(uint16_t baseAddress)
{
    HWREG16(baseAddress + OFS_SAC0OA) &= ~SACEN;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for sac_api
//! @}
//
//*****************************************************************************
