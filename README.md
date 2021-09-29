# HMC830
Arduino Library for the HMC830 Wideband Frequency Synthesizer chip

## Introduction

This library supports the HMC830 from Analog Devices on Arduinos. The chip is a wideband (24.193549 MHz to 3 GHz) Phase-Locked Loop (PLL) and Voltage Controlled Oscillator (VCO), covering a very wide range frequency range under digital control. Just add an external PLL loop filter, Reference frequency source and a power supply for a very useful frequency generator for applications as a Local Oscillator or Sweep Generator.  

The chip generates the frequency using a programmable Fractional-N and Integer-N Phase-Locked Loop (PLL) and Voltage Controlled Oscillator (VCO) with an external loop filter and frequency reference. The chip is controlled by a SPI interface, which is controlled by a microcontroller such as the Arduino.

The library provides an SPI control interface for the HMC830, and also provides functions to calculate and set the
frequency, which greatly simplifies the integration of this chip into a design. The calculations are done using the excellent 
[Big Number Arduino Library](https://github.com/nickgammon/BigNumber) by Nick Gammon.
The library also exposes all of the PLL variables, such as FRAC, Mod and INT, so they examined as needed.  

Requires the BitFieldManipulation library: http://github.com/brycecherry75/BitFieldManipulation

Requires the BeyondByte library: http://github.com/brycecherry75/BeyondByte

Requires the ShiftX library: http://github.com/brycecherry75/ShiftX

A low phase noise stable oscillator is required for this module. Typically, an Ovenized Crystal Oscillator (OCXO) in the 10 MHz to 100 MHz range is used.  

## Features

+ Frequency Range: 24.193549 MHz to 3 GHz
+ Output Level: 0 dB to -9 dB (in 3 dB steps) 
+ In-Band Phase Noise: -110 dBc/Hz
+ Signal On/Off control for the RF_P output
+ All HMC830_R[] and HMC830_SubR5[] registers and HMC830_Icp (float) can be accessed and manipulated - in HMC Mode (used for this library), HMC830_R values start at Bit 1 of each element (Bit 0 in HMC Mode (Write) is irrevalent as per HMC830 Product & Operating Guide section 4.17.3.1) and Bits 25-31 are not to be modified (Write/_Read flag and Address); all HMC830_SubR5 registers start at Bit 7 and Bits 0-6 are not to be modified (VCO Subsystem ID and Register Address) as per HMC830 Product & Operating Guide section 5.7

## Library Use

An example program using the library is provided in the source directory [example830.ino](examples/example830.ino).

ManualInit(SS_SEN_pin, MOSI_SDI_pin, MISO_SDO_pin, SCK_pin): special initialization required for boards other than the Uno/Nano Every/Mega/Leonardo/Zero/Due/ATmega8-based and derivatives which will not be fully initialized by the init() routine

init(SENpinToUse, SDIpinToUse, SDOpinToUse, SCKpinToUse, LockPinNumber, Lock_Pin_Used, CEpin, CE_Pin_Used): initialize the HMC830 with specified SEN/SDI/SDO/SCK pins, lock pin and true/false for lock pin use and CE pin use - CE pin is typically LOW (disabled) on reset if used unless a pullup resistor is used and the CE pin is connected to a microcontroller pin which will float on reset; depending on your board, this pin may have a pullup or pulldown resistor fitted

SetStepFreq(frequency): sets the step frequency in Hz - returns an error code

ReadR()/ReadFFCR()/ReadOutDivider(): returns a uint16_t value for the currently programmed register (FFCR: Fine Frequency Control Register)

ReadN()/ReadFraction(): returns a uint32_t value for the currently programmed register

ReadPFDfreq(): returns a double for the PFD value

setf(*frequency, PowerLevel, Outputs, PrecisionMode, FrequencyTimeout): set the frequency (in Hz with a char string), power level (1-4 from -9dB to 0dBm in 3dB steps) and output enables (2 to enable both outputs, 1 to enable RF_N output only, 0 to disable both outputs), precision frequency true/false, precision frequency calculation timeout (in mS with uint32_t - recommended value is 45000 in most cases) - returns an error or warning code

setrf(frequency, R_divider): set the reference frequency and reference divider R - default is 10 MHz/1 - a setf() is required to update reference divider R - returns an error code

ReadSignature(): returns a true or false if the HMC830 signature matches

WriteSweepValues(*regs, *subregs): high speed write for registers when used for frequency sweep (*regs is uint32_t and size is as per HMC830_RegsToWrite, *subregs is uint16_t and size is as per HMC830_SubregsToWrite)

ReadSweepValues(*regs, *subregs): high speed read for registers when used for frequency sweep (*regs is uint32_t and size is as per HMC830_RegsToWrite, *subregs is uint16_t and size is as per HMC830_SubregsToWrite)

Please note that you should install the provided BigNumber library in your Arduino library directory.

Unusual step frequencies in non-precision mode ((RF frequency * divider) / GCD(VCO, (REFIN / R))) (all values are in Hz) e.g. VCO (RF frequency * divider) = 1500.00353 MHz and PFD = 10 MHz (REFIN / R) should be avoided along with a PFD having decimal place(s) to avoid slow GCD calculations for FFCR and a subsequent FFCR range error - step sizes for a 10 MHz PFD which do not have this issue are any mulitple of 625/800/1000 Hz.

After power is applied to the HMC830, a rising edge of SCLK will select Open Mode (used by this library) or a rising edge of SEN (SPI SS) will select HMC Mode; it is essential that the init() function (and if your board is anything other than a Uno/Nano Every/Mega/Leonardo/Zero/Due or derivative, use of ManualInit() beforehand) is used before any other actions on any pins including SPI which are connected to the HMC830.

Since the bootloader of most Arduino boards will flash an LED connected to Pin 13 (which depending on your microcontroller can have an SPI SCLK hardware function) on reset which along with unwanted power-on glitches will subsequently cause the HMC830 to select a mode when not wanted, it is also essential that the +3.3V supply to the HMC830 and the +3.3V side (connected to the HMC830 +3.3V) of the logic level translator be applied after a one second delay after power is applied to the Arduino.

Default settings which may need to be changed as required BEFORE execution of MAX2870 library functions (defaults listed):

Phase Detector Polarity (Register 0x0B/Bit 4 = 0): Positive (passive or noninverting active loop filter)


Error codes:

Common to SetStepFreq/setf/setrf:

HMC830_ERROR_NONE


Common to setf/setrf:

HMC830_ERROR_PFD_AND_STEP_FREQUENCY_HAS_REMAINDER


SetStepFreq only:

HMC830_ERROR_STEP_FREQUENCY_EXCEEDS_PFD


setf only:

HMC830_ERROR_RF_FREQUENCY

HMC830_ERROR_POWER_LEVEL

HMC830_ERROR_OUTPUTS

HMC830_ERROR_ZERO_PFD_FREQUENCY

HMC830_ERROR_FRAC_RANGE

HMC830_ERROR_N_RANGE

HMC830_ERROR_PFD_VCO_RATIO_UNDER_20

HMC830_ERROR_RF_FREQUENCY_AND_STEP_FREQUENCY_HAS_REMAINDER

HMC830_WARNING_FREQUENCY_ERROR


setrf only:

HMC830_ERROR_R_RANGE

HMC830_ERROR_REF_FREQUENCY

HMC830_ERROR_PFD_LIMITS

## Installation
Copy the `src/` directory to your Arduino sketchbook directory (named the directory `example830`), and install the libraries in your Arduino library directory. You can also install the HMC830 files separatly as a library.

## References
+ Hittite PLLs with Integrated VCO RF Applications Product & Operating Guide covering HMC830 and certain others
+ [Big Number Arduino Library](https://github.com/nickgammon/BigNumber) by Nick Gammon