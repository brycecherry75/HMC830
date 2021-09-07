/*!
   @file HMC830.h

   This is part of the Arduino Library for the HMC830 PLL wideband frequency synthesier

*/

#ifndef HMC830_H
#define HMC830_H
#include <Arduino.h>
#include <ShiftX.h>
#include <stdint.h>
#include <BigNumber.h>
#include <BitFieldManipulation.h>
#include <BeyondByte.h>

// All frequencies are in Hz
#define HMC830_PFD_MAX   100000000UL      ///< Maximum Frequency for Phase Detector with minimum PFD/VCO ratio of 20
#define HMC830_PFD_MIN   125000UL        ///< Minimum Frequency for Phase Detector
#define HMC830_REFIN_MIN   100000UL      ///< Minimum Reference Frequency
#define HMC830_REFIN_MAX   350000000UL   ///< Maximum Reference Frequency
#define HMC830_REF_FREQ_DEFAULT 10000000UL  /// < Default Reference Frequency

// common to all of the following subroutines
#define HMC830_ERROR_NONE 0

// setf
#define HMC830_ERROR_RF_FREQUENCY 1
#define HMC830_ERROR_POWER_LEVEL 2
#define HMC830_ERROR_OUTPUTS 3
#define HMC830_ERROR_ZERO_PFD_FREQUENCY 4
#define HMC830_ERROR_FRAC_RANGE 5
#define HMC830_ERROR_N_RANGE 6
#define HMC830_ERROR_PFD_VCO_RATIO_UNDER_20 7
#define HMC830_ERROR_FFCR_RANGE 8
#define HMC830_ERROR_RF_FREQUENCY_AND_STEP_FREQUENCY_HAS_REMAINDER 9
#define HMC830_ERROR_PRECISION_FREQUENCY_CALCULATION_TIMEOUT 10
#define HMC830_WARNING_FREQUENCY_ERROR 11

// setrf
#define HMC830_ERROR_R_RANGE 12
#define HMC830_ERROR_REF_FREQUENCY 13
#define HMC830_ERROR_PFD_LIMITS 14

// SetStepFreq
#define HMC830_ERROR_STEP_FREQUENCY_EXCEEDS_PFD 15

// common to setf and SetStepFreq
#define HMC830_ERROR_PFD_AND_STEP_FREQUENCY_HAS_REMAINDER 16

// per step for arrays for high speed frequency sweeping
#define HMC830_RegsToWrite 7UL // uint32_t
#define HMC830_SubregsToWrite 2UL // uint16_t

/*!
   @brief HMC830 chip device driver

   This class provides the overall interface for HMC830 chip. It is used
   to define the SPI connection, initalize the chip on power up, disable/enable
   frequency generation, and set the frequency and reference frequency.

   The PLL values and register values can also be set directly with this class,
   and current settings for the chip and PLL can be read.

   As a simple frequency generator, once the target frequency and desired channel step
   value is set, the library will perform the required calculations to
   set the PLL and other values, and determine the mode (Frac-N or Int-N)
   for the PLL loop. This greatly simplifies the use of the HMC830 chip.

   The HMC830 datasheet should be consulted to understand the correct
   register settings. While a number of checks are provided in the library,
   not all values are checked for allowed settings, so YMMV.

*/
class HMC830
{
  public:

uint8_t HMC830_PIN_SEN;
uint8_t HMC830_PIN_SDO;
uint8_t HMC830_PIN_SDI;
uint8_t HMC830_PIN_SCK;

HMC830();
void WriteRegs();
uint16_t ReadR();
uint32_t ReadN();
uint32_t ReadFraction();
uint16_t ReadFFCR(); // Fine Frequency Control Register
uint16_t ReadOutDivider();
void ReadSweepValues(uint32_t *regs, uint16_t *subregs);
void WriteSweepValues(const uint32_t *regs, const uint16_t *subregs);
double ReadPFDfreq();

void ManualInit(uint8_t SSpinToUse, uint8_t MOSIpinToUse, uint8_t MISOpinToUse, uint8_t SCKpinToUse);
void init(uint8_t SENpinToUse, uint8_t SDIpinToUse, uint8_t SDOpinToUse, uint8_t SCKpinToUse, uint8_t LockPinNumber, bool Lock_Pin_Used, uint8_t CEpin, bool CE_Pin_Used);
int SetStepFreq(uint32_t value);
int setf(char *freq, uint8_t PowerLevel, uint8_t Outputs, bool PrecisionFrequency, uint32_t CalculationTimeout) ; // set freq and power levels and output mode
int setrf(uint32_t f, uint16_t r) ; // set reference freq and reference divider (default is 10 MHz with divide by 1)
bool CheckSignature(); // check BIST signature

// power on defaults
uint32_t HMC830_reffreq = HMC830_REF_FREQ_DEFAULT;
uint32_t HMC830_R[13] = {0x00000000, 0x02000004, 0x04000002, 0x06000032, 0x08000002, 0x0A000000, 0x0C400794, 0x0E00199A, 0x10837DDE, 0x123FFFFF, 0x1400408C, 0x160F80C2, 0x18000000};
uint16_t HMC830_SubR5[7] = {0x0000, 0x0F88, 0x6F10, 0x2898, 0x60A0, 0x1628, 0x7FB0};
uint32_t HMC830_ChanStep = 100000UL;
float HMC830_Icp = 2.54; // mA

};

#endif