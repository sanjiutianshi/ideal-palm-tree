/*
 * F2803x_I2C_Driver.c
 *
 *  Created on: 2017年12月22日
 *      Author: chenna
 */

#include "F2803x_Project.h"

//---------------------------------------------------------------------------
// Example: InitI2CGpio:
//---------------------------------------------------------------------------
void InitI2CGpio()
{

    EALLOW;

 /* Enable internal pull-up for the selected pins */
 // Pull-ups can be enabled or disabled disabled by the user.
 // This will enable the pullups for the specified pins.
 // Comment out other unwanted lines.

   GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;    // Enable pull-up for GPIO32 (SDAA)
   GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;    // Enable pull-up for GPIO33 (SCLA)

 /* Set qualification for selected pins to asynch only */
 // This will select asynch (no qualification) for the selected pins.
 // Comment out other unwanted lines.

   GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // Asynch input GPIO32 (SDAA)
   GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;  // Asynch input GPIO33 (SCLA)

 /* Configure I2C pins using GPIO regs*/
 // This specifies which of the possible GPIO pins will be I2C functional pins.
 // Comment out other unwanted lines.

   GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;   // Configure GPIO32 for SDAA operation
   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;   // Configure GPIO33 for SCLA operation

   EDIS;
}

//---------------------------------------------------------------------------
// Example: InitI2CGpio:
//---------------------------------------------------------------------------
// The DAC8571 supports the I2C serial bus and data transmission protocol,
// in all three defined modes:standard (100Kbps),fast (400kBps),and highspeed(3.4Mbps).
// but, Data transfer rate of from 10kbps up to 400kbps(Philips Fast-moderate)

void InitDAC8571(void)
{
   // Initialize I2C

   // I2C模块内部操作时钟 10MHz   I2caRegs.I2CPSC.all = (60M/ 10M - 1);
   I2caRegs.I2CPSC.all = 5;         // Prescaler - need 7-12 Mhz on module clk

   // I2C clock = I2C module clock frequency / (( ICCL+d)+(ICCH+d))
   //   IPSC              d
   //    0                7
   //    1                6
   //  Greater than 1     5
   // I2C 400kbps (Philips Fast-mode rate)
   I2caRegs.I2CCLKL = 10;           // NOTE: must be non zero
   I2caRegs.I2CCLKH = 5;            // NOTE: must be non zero

   I2caRegs.I2CIER.all = 0x24;      // Enable SCD & ARDY interrupts

   I2caRegs.I2CMDR.all = 0x0020;    // Take I2C out of reset
                                    // Stop I2C when suspended

   I2caRegs.I2CFFTX.all = 0x6000;   // Enable FIFO mode and TXFIFO
   I2caRegs.I2CFFRX.all = 0x2040;   // Enable RXFIFO, clear RXFFINT,

   // check i2c bus status
   while (I2caRegs.I2CSTR.bit.BB == 1) {}

   // set fast settling mode
//   DAC8571_PowerSetting(FAST_SETTLING);

   // clear dac output to zero
   DAC8571_DataSetting (_IQ15 (0.0));
}

//---------------------------------------------------------------------------
// 获取的数据是 1. TargetDA DAC8801 的设定值，需要send to M3
// 2. ADtagCurrent C28 ADC 的采样值 直接设定DA模式则不需要
// 3. actDA 在直接设定电流的工作模式中需要调整DAC的值 直接设定DA模式则不需要
//---------------------------------------------------------------------------
//
// void DAC8571_DataSetting(unsigned long voltage)
//---------------------------------------------------------------------------
void DAC8571_DataSetting (_iq15 voltage)
{
    _iq15 value;

    value = _IQ15(5.0) - voltage;

     // Vout = Vref * D / 65536       // DAC VREF 1.15 but select 5V
    value = _IQ15mpy (value,DACSCale);

    Data16.all = CalculateLossIQ15 (value);

    I2caRegs.I2CSAR = I2C_SLAVE_ADDR_DAC;

    I2caRegs.I2CCNT = 3;

   // Control byte Write temporary register and load Data DAC with data
    I2caRegs.I2CDXR = DAC_Data_Instance;

    I2caRegs.I2CDXR = Data16.Bytes.Uint1;

    I2caRegs.I2CDXR = Data16.Bytes.Uint0;

    I2caRegs.I2CMDR.all = 0x6E20;

    // Bus free time between a STOP and START condition
    Delay_US(5);
}

//---------------------------------------------------------------------------
// void DAC8571_PowerMode(unsigned char mode)
//---------------------------------------------------------------------------
void DAC8571_PowerSetting(unsigned char mode)
{
    I2caRegs.I2CSAR = I2C_SLAVE_ADDR_DAC;

    I2caRegs.I2CCNT = 3;

   // Control byte Write temporary register and load Data DAC with data
    I2caRegs.I2CDXR = DAC_PD_Instance;

    I2caRegs.I2CDXR = mode;

    I2caRegs.I2CDXR = 0x00;

    I2caRegs.I2CMDR.all = 0x6E20;

    Delay_US(100);
}
