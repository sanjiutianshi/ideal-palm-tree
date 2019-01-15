/*
 * F2803x_Adc_Driver.c
 *
 *  Created on: 2015-11-12
 *      Author: RD01
 */

#include "F2803x_Project.h"     // Device Headerfile and Examples Include File

//---------------------------------------------------------------------------
// InitAdc:
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
// NOTE: ADC INIT IS DIFFERENT ON 2803x DEVICES COMPARED TO OTHER 28X DEVICES
//
void InitAdc(void)
{
    // *IMPORTANT*
    // The Device_cal function, which copies the ADC calibration values from TI reserved
    // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
    // Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
        (*Device_cal)();
        EDIS;

    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
    // after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
    // CPU_RATE define statement in the DSP2803x_Examples.h file must
    // contain the correct CPU clock period in nanoseconds.
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
    AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
    AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
    AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
//    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 1;      // External VREFHI/VREFLO pins used for reference generation.
    EDIS;

    Delay_US (1000);                        // Delay before converting ADC channels
}

//---------------------------------------------------------------------------
// unsigned int Start_Sample (void)
//---------------------------------------------------------------------------
//  开始ADS采样.
//
void Start_Sample (void)
{
    unsigned int i;

    // clear buffer and initialize global variables
    for (i=0;i<DutyMax;i++)
    {
        Get.PASS_Duty[i] = 0;
        Get.Duty[i] = 1;
    }

    Get.aveDuty  = 0;
    Get.RMS_Current = 0;
    Get.RMS_Voltage = 0;
    Get.RMS_Temp = 0;
    Get.RMS_Skewing = 0;

    temp = _IQ15 (1000.0);
    Get.realFreq = _IQ15mpy (Set.Freq,temp);

    /*****setup frequency output******/
    temp = _IQ15 (134.217);
    temp = _IQ15mpy (Set.Freq, temp);
    Get.DDSfreq = _IQ15int (temp) * 1000;      // setup register value

    tempA = _IQ15 (1000.0);
    tempB = _IQ15frac(temp);
    tempC = _IQ15mpy (tempA, tempB);
    Get.DDSfreq += _IQ15toF (tempC);

    Get.DDSfreq += (_IQ15frac (tempC)>= 0.5) ? 1 : 0;

    AD9834SIN_SetFQ (FREQUENCY_RES0,Get.DDSfreq);

    /*****Initialize global variables******/
    CpuTimer0.InterruptCount = 0;
    CpuTimer1.InterruptCount = 0;
    CpuTimer2.InterruptCount = 0;

    TestStatus = BUSY;
    UpdateStatus = BUSY;
    SampleStatus = BUSY;

    EnableECap1 ();

    ENABLE1_ACTIVE;
    ENABLE2_ACTIVE;
    SIN_DDS_ACTIVE;                                // start AD9834 sine output

    CpuTimer0Regs.TCR.all = 0x4000;                // start timer for update frequency

    ECap1Regs.ECCTL2.bit.TSCTRSTOP   = 1;          // Start capture

    PieCtrlRegs.PIEIER4.bit.INTx1    = 1;          // Enable ECap1 Interrupt
}

//---------------------------------------------------------------------------
// void Restart_Sample (void)
//---------------------------------------------------------------------------
//
void Restart_Sample (void)
{
    unsigned int i;

    // clear buffer and initialize global variables
    for (i=0;i<DutyMax;i++)
    {
        Get.PASS_Duty[i] = Vzero;
        Get.Duty[i] = 1;
    }

    Get.aveDuty  = 0;
    CpuTimer1.InterruptCount = 0;
    CpuTimer2.InterruptCount = 0;

    EnableECap1 ();

    ECap1Regs.ECCTL2.bit.TSCTRSTOP   = 1;          // Start capture
    PieCtrlRegs.PIEIER4.bit.INTx1    = 1;          // Enable ECap1 Interrupt
}

//---------------------------------------------------------------------------
// void Stop_Sample(void)
//---------------------------------------------------------------------------
// 停止ADS采样.
//
void Stop_Sample(void)
{
    SIN_DDS_RESET;                             // stop AD9834 sine output
    ENABLE1_INACTIVE;
    ENABLE2_INACTIVE;

    CpuTimer0Regs.TCR.all = 0x4010;            // stop update frequency at once

//    EPwm1Regs.TBCTL.bit.CTRMODE  = 3;          // Stop-freeze counter operation

    ECap1Regs.ECEINT.all = 0x0000;            // Disable all capture interrupts
    ECap1Regs.ECCLR.all = 0xFFFF;             // Clear all CAP interrupt flags
    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;         // Disable CAP1-CAP4 register loads
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;       // Make sure the counter is stopped
    PieCtrlRegs.PIEIER4.bit.INTx1  = 0;       // disable ECap1 Interrupt

    TestStatus = IDLE;
    UpdateStatus = IDLE;
    SampleStatus = IDLE;
}

//---------------------------------------------------------------------------
// __interrupt void ADC_INT1_Isr(void)
//---------------------------------------------------------------------------
//
__interrupt void ADC_INT1_Isr(void)
{
/**********************************************************************************************/
    // Vadc = [AdcResult.ADCRESULT / 4096] * (VREFHI – VREFLO) = [AdcResult.ADCRESULT / 4096] * 3.3v
    Get.RMS_Current = _IQ15 (AdcResult.ADCRESULT3);

    Get.RMS_Current = _IQ15div (Get.RMS_Current,VadcSCale);

    // Uin = [[(Uadc - 1.5) / SCALE] * 20] / 3 = [Uadc - 1.5]  * 6.6666
    Get.RMS_Current -= _IQ15 (1.5);

    Get.RMS_Current = _IQ15mpy (Get.RMS_Current,UinSCale);

    Get.Current = _IQ15toF (Get.RMS_Current);

/**********************************************************************************************/
    // Vadc = [AdcResult.ADCRESULT / 4096] * (VREFHI – VREFLO) = [AdcResult.ADCRESULT / 4096] * 3.3v
    Get.RMS_Voltage = _IQ15 (AdcResult.ADCRESULT2);

    Get.RMS_Voltage = _IQ15div (Get.RMS_Voltage,VadcSCale);

    // Uin = [[(Uadc - 1.5) / SCALE] * 20] / 3 = [Uadc - 1.5]  * 6.6666
    Get.RMS_Voltage -= _IQ15 (1.5);

    Get.RMS_Voltage = _IQ15mpy (Get.RMS_Voltage,UinSCale);

    Get.Voltage = _IQ15toF (Get.RMS_Voltage);

/**********************************************************************************************/
    Get.RMS_Temp = _IQ15 (AdcResult.ADCRESULT0);

    Get.RMS_Temp = _IQ15mpy (UinSCaleTemp,Get.RMS_Temp);

/**********************************************************************************************/
    Get.RMS_Skewing = _IQ15 (AdcResult.ADCRESULT1);

    SampleStatus = CompleteSample;

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;     //Clear ADCINT1 flag reinitialize for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
    return;
}





