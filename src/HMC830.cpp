/*!

   @file HMC830.cpp

   @mainpage HMC830 Arduino library driver for Wideband Frequency Synthesizer

   @section intro_sec Introduction

   The HMC830 chip is a wideband freqency synthesizer integrated circuit that can generate frequencies
   from 24.193549 MHz to 3 GHz. It incorporates a PLL and VCO, along with prescalers, dividers and multipiers.
   The users add a PLL loop filter and reference frequency to create a frequency generator with a very wide
   range, that is tuneable in settable frequency steps.

   The HMC830 chip provides an SPI interface for setting the device registers that control the
   frequency and output levels, along with several IO pins for gathering chip status and
   enabling/disabling output and power modes.

   The HMC830 library provides an Arduino API for accessing the features of the HMC830 chip.

   @section dependencies Dependencies

   This library uses the BigNumber library from Nick Gammon
   Also requires the following libraries:
   BitFieldManipulation: https://github.com/brycecherry75/BitFieldManipulation
   BeyondByte: https://github.com/brycecherry75/BeyondByte
   BeyondByte: https://github.com/brycecherry75/ShiftX

   @section author Author

   Bryce Cherry

*/

#include "HMC830.h"

HMC830::HMC830() {
}

void HMC830::WriteRegs() { // order as per Hittite PLLs with Integrated VCO RF Applications Product & Operating Guide covering HMC830 and certain others
  for (int i = 0; i < 15; i++) {
    uint32_t DataToWrite;
    uint8_t RegisterToWrite;
    bool SubregWrite = false;
    switch (i) { // sequence as per HMC830 Product & Operating Guide section 4.18
      case 0:
        RegisterToWrite = 0x01; // Enables
        SubregWrite = true;
        break;
      case 1:
        RegisterToWrite = 0x02; // Reference
        break;
      case 2:
        RegisterToWrite = 0x06; // Delta-Sigma configruation
        break;
      case 3:
        RegisterToWrite = 0x07; // Lock Detect - fixed value
        break;
      case 4:
        RegisterToWrite = 0x0A; // VCO Autocal - fixed value
        break;
      case 5:
        RegisterToWrite = 0x0B; // Phase Detect
        break;
      case 6:
        RegisterToWrite = 0x09; // Charge Pump Current
        break;
      case 7:
        RegisterToWrite = 0x05; // VCO Configuration
        SubregWrite = true;
        break;
      case 8:
        RegisterToWrite = 0x04; // VCO Configuration
        SubregWrite = true;
        break;
      case 9:
        RegisterToWrite = 0x02; // VCO Configuration (RF Divide ratio/RF output buffer gain control)
        SubregWrite = true;
        break;
      case 10:
        RegisterToWrite = 0x03; // VCO Configuration (RF output enables)
        SubregWrite = true;
        break;
      case 11: // reset VCO subregister address to 0
        RegisterToWrite = 0x00; // VCO Configuration
        SubregWrite = true;
        break;
      case 12:
        RegisterToWrite = 0x03; // N
        break;
      case 13:
        RegisterToWrite = 0x04; // Fraction
        break;
      case 14:
        RegisterToWrite = 0x0C; // FFCR
        break;
    }
    if (SubregWrite == false) {
      DataToWrite = HMC830_R[RegisterToWrite];
    }
    else {
      DataToWrite = HMC830_SubR5[RegisterToWrite];
      DataToWrite <<= 1; // Bit 0 in HMC Mode (Write) is irrevalent as per HMC830 Product & Operating Guide section 4.17.3.1
      DataToWrite |= 0x0A000000; // write the required address bits
    }
    digitalWrite(HMC830_PIN_SEN, HIGH);
    delayMicroseconds(1);
    ShiftX.out_Dword(HMC830_PIN_SDI, HMC830_PIN_SCK, MSBFIRST, 32, 1, DataToWrite);
    delayMicroseconds(1);
    digitalWrite(HMC830_PIN_SEN, LOW);
    delayMicroseconds(1);
  }
}

void HMC830::ReadSweepValues(uint32_t *regs, uint16_t *subregs) {
  regs[0] = HMC830_R[0x02]; // Reference
  regs[1] = HMC830_R[0x06]; // Delta-Sigma configuration
  regs[2] = HMC830_R[0x09]; // Charge Pump Current
  regs[3] = HMC830_R[0x0B]; // Phase Detect
  subregs[0] = HMC830_SubR5[0x02]; // VCO Configuration (RF Divide ratio/RF output buffer gain control)
  subregs[1] = HMC830_SubR5[0x03]; // VCO Configuration (RF output enables)
  regs[4] = HMC830_R[0x03]; // N
  regs[5] = HMC830_R[0x04]; // Fraction
  regs[6] = HMC830_R[0x0C]; // FFCR
}

void HMC830::WriteSweepValues(const uint32_t *regs, const uint16_t *subregs) {
  HMC830_R[0x02] = regs[0]; // reference
  HMC830_R[0x06] = regs[1]; // Delta-Sigma configruation
  HMC830_R[0x09] = regs[2]; // Charge Pump Current
  HMC830_R[0x0B] = regs[3]; // Phase Detect
  HMC830_SubR5[0x02] = subregs[0x02]; // VCO Configuration (RF Divide ratio/RF output buffer gain control)
  HMC830_SubR5[0x03] = subregs[0x03]; // VCO Configuration (RF output enables)
  HMC830_R[0x03] = regs[4]; // N
  HMC830_R[0x04] = regs[5]; // Fraction
  HMC830_R[0x0C] = regs[6]; // FFCR
  WriteRegs();
}

uint16_t HMC830::ReadR() {
  return BitFieldManipulation.ReadBF_dword(1, 14, HMC830_R[0x02]);
}

uint32_t HMC830::ReadN() {
  return BitFieldManipulation.ReadBF_dword(1, 19, HMC830_R[0x03]);
}

uint32_t HMC830::ReadFraction() {
  return BitFieldManipulation.ReadBF_dword(1, 24, HMC830_R[0x04]);
}

uint16_t HMC830::ReadFFCR() {
  return BitFieldManipulation.ReadBF_dword(1, 14, HMC830_R[0x0C]);
}

uint16_t HMC830::ReadOutDivider() {
  return BitFieldManipulation.ReadBF_word((0 + 7), 6, HMC830_SubR5[0x02]);
}

double HMC830::ReadPFDfreq() {
  double value = HMC830_reffreq;
  uint16_t temp = ReadR();
  if (temp == 0) { // avoid division by zero
    return 0;
  }
  value /= temp;
  return value;
}

void HMC830::init(uint8_t SENpinToUse, uint8_t SDIpinToUse, uint8_t SDOpinToUse, uint8_t SCKpinToUse, uint8_t LockPinNumber, bool Lock_Pin_Used, uint8_t CEpin, bool CE_Pin_Used) {
  HMC830_PIN_SEN = SENpinToUse;
  HMC830_PIN_SDO = SDOpinToUse;
  HMC830_PIN_SDI = SDIpinToUse;
  HMC830_PIN_SCK = SCKpinToUse;
  delay(2000); // wait for power to the HMC830 to be enabled to deal with unwanted SCK pulse (to flash the LED) by the Arduino bootloader on reset and unwanted power-on glitches which can set the mode of the HMC830 when not needed
  pinMode(HMC830_PIN_SCK, OUTPUT);
  pinMode(HMC830_PIN_SEN, OUTPUT);
  pinMode(HMC830_PIN_SDI, OUTPUT);
  pinMode(HMC830_PIN_SDO, INPUT_PULLUP);
  digitalWrite(HMC830_PIN_SCK, LOW);
  digitalWrite(HMC830_PIN_SEN, LOW);
  digitalWrite(HMC830_PIN_SDI, LOW);
  delayMicroseconds(1);
  digitalWrite(HMC830_PIN_SEN, HIGH);
  delayMicroseconds(1);
  digitalWrite(HMC830_PIN_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(HMC830_PIN_SCK, LOW);
  digitalWrite(HMC830_PIN_SDI, LOW);
  digitalWrite(HMC830_PIN_SEN, LOW);
  delayMicroseconds(1);
  if (Lock_Pin_Used == true) {
    pinMode(LockPinNumber, INPUT_PULLUP);
  }
  if (CE_Pin_Used == true) {
    pinMode(CEpin, OUTPUT);
  }
}

int HMC830::SetStepFreq(uint32_t value) {
  if (value > ReadPFDfreq()) {
    return HMC830_ERROR_STEP_FREQUENCY_EXCEEDS_PFD;
  }
  uint32_t ReferenceFrequency = HMC830_reffreq;
  uint16_t Rvalue = ReadR();
  if (Rvalue == 0 || (HMC830_reffreq % value) != 0) {
    return HMC830_ERROR_PFD_AND_STEP_FREQUENCY_HAS_REMAINDER;
  }
  HMC830_ChanStep = value;
  return HMC830_ERROR_NONE;
}

int  HMC830::setf(char *freq, uint8_t PowerLevel, uint8_t Outputs, bool PrecisionFrequency, uint32_t CalculationTimeout) // 0 to disable RF output, 1-4 to enable with power setting
{
  //  calculate settings from freq
  if (PowerLevel < 0 || PowerLevel > 4) return HMC830_ERROR_POWER_LEVEL;
  if (Outputs < 0 || Outputs > 2) return HMC830_ERROR_OUTPUTS;
  if (ReadPFDfreq() == 0) return HMC830_ERROR_ZERO_PFD_FREQUENCY;
  if (PrecisionFrequency == false && ((HMC830_reffreq / ReadR()) % HMC830_ChanStep) != 0) return HMC830_ERROR_PFD_AND_STEP_FREQUENCY_HAS_REMAINDER;

  BigNumber::begin(20); // BigNumber(signed_int_limited_from_-32768_to_32767) / BigNumber("number_string_is_not_limited_to_signed_int")

  if (BigNumber(freq) > BigNumber("3000000000") || BigNumber(freq) < BigNumber("24193549")) { // lower frequency is rounded up to the nearest integer
    BigNumber::finish();
    return HMC830_ERROR_RF_FREQUENCY;
  }

  uint8_t FrequencyPointer = 0;
  while (true) { // null out any decimal places below 1 Hz increments to avoid GCD calculation input overflow
    if (freq[FrequencyPointer] == '.') { // change the decimal point to a null terminator
      freq[FrequencyPointer] = 0x00;
      break;
    }
    if (freq[FrequencyPointer] == 0x00) { // null terminator reached
      break;
    }
    FrequencyPointer++;
  }

  char tmpstr[12]; // will fit a long including sign and terminator
  if (PrecisionFrequency == false) {
    ultoa(HMC830_ChanStep, tmpstr, 10);
    BigNumber BN_freq = BigNumber(freq);
    // BigNumber has issues with modulus calculation which always results in 0
    BN_freq /= BigNumber(tmpstr);
    uint32_t ChanSteps = (uint32_t)((uint32_t) BN_freq); // round off the decimal - overflow is not an issue for the HMC830 frequency range
    ultoa(ChanSteps, tmpstr, 10);
    BN_freq -= BigNumber(tmpstr);
    if (BN_freq != BigNumber(0)) {
      BigNumber::finish();
      return HMC830_ERROR_RF_FREQUENCY_AND_STEP_FREQUENCY_HAS_REMAINDER;
    }
  }

  BigNumber BN_localosc_ratio = BigNumber("1500000000") / BigNumber(freq);
  uint8_t localosc_ratio = (uint8_t) ((uint32_t) BN_localosc_ratio);
  uint8_t HMC830_outdiv = 1;

  if (BigNumber(freq) != BigNumber("24193549")) {
    if (localosc_ratio > 0) {
      HMC830_outdiv = 0;
      while (HMC830_outdiv <= localosc_ratio && HMC830_outdiv <= 62) {
        HMC830_outdiv += 2;
      }
    }
  }
  else {
    HMC830_outdiv = 62;
  }

  ultoa(HMC830_reffreq, tmpstr, 10);
  word CurrentR = ReadR();
  BigNumber BN_HMC830_PFDFreq = (BigNumber(tmpstr) / BigNumber(CurrentR));
  BigNumber BN_HMC830_VCO = (BigNumber(freq) * BigNumber(HMC830_outdiv));

  ultoa(HMC830_ChanStep, tmpstr, 10);
  if ((BN_HMC830_VCO / BigNumber("20")) < BN_HMC830_PFDFreq) {
    BigNumber::finish();
    return HMC830_ERROR_PFD_VCO_RATIO_UNDER_20;
  }

  BigNumber BN_N = BN_HMC830_VCO / BN_HMC830_PFDFreq;
  uint32_t HMC830_N = (uint32_t)((uint32_t) BN_N); // remove the decimal
  ultoa(HMC830_N, tmpstr, 10);
  BigNumber BN_Frac = (BN_N - BigNumber(tmpstr)) * BigNumber("16777216");
  uint32_t HMC830_Frac = (uint32_t)((uint32_t) BN_Frac);

  /*
    // only required if VCO > ((2^32) - 1) Hz which cannot be attained in the HMC830
    BigNumber BN_t;
    BigNumber BN_HMC830_PFDFreq2 = BN_HMC830_PFDFreq; // a duplicate is required here
    while (true) {
      if (PrecisionFrequency == true && CalculationTimeout > 0) {
        uint32_t RunningTime = millis();
        RunningTime -= StartTime;
        if (RunningTime > CalculationTimeout) {
          GCDtimeout = true;
          break;
        }
      }
      if (BN_HMC830_VCO < BigNumber(1)) {
        BN_t = BN_HMC830_PFDFreq2;
        break;
      }
      if (BN_HMC830_PFDFreq2 < BigNumber(1)) {
        BN_t = BN_HMC830_VCO;
        break;
      }
      if (BN_HMC830_VCO == BN_HMC830_PFDFreq2) {
        BN_t = BN_HMC830_VCO;
        break;
      }
      if (BN_HMC830_VCO > BN_HMC830_PFDFreq2) {
        BN_HMC830_VCO -= BN_HMC830_PFDFreq2;
      }
      else {
        BN_HMC830_PFDFreq2 -= BN_HMC830_VCO;
      }
    }
    BigNumber BN_FFCR = ((BN_HMC830_PFDFreq / BN_t) + BigNumber("0.5")); // result should be 1 for 2 GHz VCO/10 MHz reference and 1536 for 2.002 GHz/61.44 MHz reference
    bool FrequencyErrorWarning = false;
    if (PrecisionFrequency == true && BN_FFCR > BigNumber("16383")) {
      FrequencyErrorWarning = true;
      while (BN_FFCR > BigNumber("16383")) {
        BN_FFCR /= BigNumber("2");
      }
      if (BN_FFCR <= BigNumber("16383.5")) { // round it if it will not overflow
        BN_FFCR += BigNumber("0.5");
      }
    }
    uint32_t HMC830_FFCR = (uint32_t)((uint32_t) BN_FFCR);
    BigNumber::finish();
  */

  // faster GCD routine if GCD_HMC830_VCO result is < (2^32) which is true for the HMC830
  uint32_t GCD_t;
  uint32_t GCD_HMC830_PFDFreq = (uint32_t)((uint32_t) BN_HMC830_PFDFreq);
  uint32_t GCD_HMC830_PFDFreq2 = GCD_HMC830_PFDFreq;
  uint32_t GCD_HMC830_VCO;
  char* tempVCO = BN_HMC830_VCO.toString();
  BigNumber::finish();
  GCD_HMC830_VCO = strtoul(tempVCO, NULL, 10);
  uint32_t VCO_MHz = GCD_HMC830_VCO;
  VCO_MHz /= 1000000; // convert to MHz
  free(tempVCO);
  bool GCDtimeout = false;
  uint32_t StartTime = millis();
  while (true) {
    if (PrecisionFrequency == true && CalculationTimeout > 0) {
      uint32_t RunningTime = millis();
      RunningTime -= StartTime;
      if (RunningTime > CalculationTimeout) {
        GCDtimeout = true;
        break;
      }
    }
    if (GCD_HMC830_VCO == 0) {
      GCD_t = GCD_HMC830_PFDFreq2;
      break;
    }
    if (GCD_HMC830_PFDFreq2 == 0) {
      GCD_t = GCD_HMC830_VCO;
      break;
    }
    if (GCD_HMC830_VCO == GCD_HMC830_PFDFreq2) {
      GCD_t = GCD_HMC830_VCO;
      break;
    }
    if (GCD_HMC830_VCO > GCD_HMC830_PFDFreq2) {
      GCD_HMC830_VCO -= GCD_HMC830_PFDFreq2;
    }
    else {
      GCD_HMC830_PFDFreq2 -= GCD_HMC830_VCO;
    }
  }

  if (GCDtimeout == true) {
    return HMC830_ERROR_PRECISION_FREQUENCY_CALCULATION_TIMEOUT;
  }

  uint32_t HMC830_FFCR = (GCD_HMC830_PFDFreq / GCD_t);
  bool FrequencyErrorWarning = false;
  if (PrecisionFrequency == true && HMC830_FFCR > 16383) {
    FrequencyErrorWarning = true;
    while (HMC830_FFCR > 16383) {
      HMC830_FFCR /= 2;
    }
  }

  if (HMC830_N < 20UL || HMC830_N > 524284UL) {
    return HMC830_ERROR_N_RANGE;
  }

  if ( HMC830_Frac < 0 || HMC830_Frac > 16777215UL) {
    return HMC830_ERROR_FRAC_RANGE;
  }

  if (HMC830_FFCR < 0 || HMC830_FFCR > 16383) {
    return HMC830_ERROR_FFCR_RANGE;
  }

  if (HMC830_Frac == 0) { // integer mode
    HMC830_R[0x06] = BitFieldManipulation.WriteBF_dword(1, 24, HMC830_R[0x06], 0x002003CA);
    HMC830_R[0x0B] = BitFieldManipulation.WriteBF_dword(1, 24, HMC830_R[0x0B], 0x0007C061);
  }
  else { // fractional mode
    HMC830_R[0x06] = BitFieldManipulation.WriteBF_dword(1, 24, HMC830_R[0x06], 0x00200B4A);
    HMC830_R[0x0B] = BitFieldManipulation.WriteBF_dword(1, 24, HMC830_R[0x0B], 0x0007C021);
  }

  // calculate charge pump current based on VCO frequency
  uint32_t fPFD = ReadPFDfreq();
  fPFD /= 1000000; // convert to MHz
  float Icp_offset = ((2.5 + 4.0 * (1000.0 / (float)VCO_MHz)) * (float)fPFD * HMC830_Icp); // uA
  if (Icp_offset > 635)
    Icp_offset = 635;
  if (Icp_offset < 0)
    Icp_offset = 0;
  HMC830_R[0x09] = BitFieldManipulation.WriteBF_dword((1 + 0), 7, HMC830_R[0x09], ((uint8_t)(HMC830_Icp * 1000.0 / 20.0 + 0.5))); // DN gain
  HMC830_R[0x09] = BitFieldManipulation.WriteBF_dword((1 + 7), 7, HMC830_R[0x09], ((uint8_t)(HMC830_Icp * 1000.0 / 20.0 + 0.5))); // UP gain
  HMC830_R[0x09] = BitFieldManipulation.WriteBF_dword((1 + 14), 7, HMC830_R[0x09], ((uint8_t)(Icp_offset / 5.0 + 0.5))); // Offset Magnitude
  HMC830_R[0x09] = BitFieldManipulation.WriteBF_dword((1 + 21), 1, HMC830_R[0x09], 0); // Offset UP Enable
  if (HMC830_Frac == 0) {
    HMC830_R[0x09] = BitFieldManipulation.WriteBF_dword((1 + 22), 1, HMC830_R[0x09], 0); // Offset DN Enable - Integer Mode
  }
  else {
    HMC830_R[0x09] = BitFieldManipulation.WriteBF_dword((1 + 22), 1, HMC830_R[0x09], 1); // Offset DN Enable - Fractional Mode
  }
  HMC830_R[0x09] = BitFieldManipulation.WriteBF_dword((1 + 23), 1, HMC830_R[0x09], 0); // HiKcp

  HMC830_R[0x03] = BitFieldManipulation.WriteBF_dword(1, 19, HMC830_R[0x03], HMC830_N);
  HMC830_R[0x04] = BitFieldManipulation.WriteBF_dword(1, 24, HMC830_R[0x04], HMC830_Frac);
  HMC830_R[0x0C] = BitFieldManipulation.WriteBF_dword(1, 14, HMC830_R[0x0C], HMC830_FFCR);
  HMC830_SubR5[0x02] = BitFieldManipulation.WriteBF_word((0 + 7), 6, HMC830_SubR5[0x02], HMC830_outdiv);
  if (Outputs > 0 && PowerLevel > 0) { // enable RF output
    PowerLevel--;
    HMC830_SubR5[0x02] = BitFieldManipulation.WriteBF_word((7 + 6), 2, HMC830_SubR5[0x02], PowerLevel);
    HMC830_SubR5[0x01] = BitFieldManipulation.WriteBF_dword((7 + 0), 5, HMC830_SubR5[0x01], 0x1F);
    if (Outputs == 2) {
      Outputs = 0; // differential mode (both outputs enabled) - 1 is single ended mode
    }
    HMC830_SubR5[0x03] = BitFieldManipulation.WriteBF_word((7 + 0), 1, HMC830_SubR5[0x03], Outputs);
    HMC830_SubR5[0x01] = 0x0F88;
  }
  else { // disable RF output
    HMC830_SubR5[0x01] = 0x0000;
  }
  WriteRegs();
  if (FrequencyErrorWarning == true) {
    return HMC830_WARNING_FREQUENCY_ERROR;
  }
  return HMC830_ERROR_NONE;  // ok
}

int HMC830::setrf(uint32_t f, uint16_t r)
{
  if (r > 16383 || r < 1) return HMC830_ERROR_R_RANGE;
  if (f < HMC830_REFIN_MIN || f > HMC830_REFIN_MAX) return HMC830_ERROR_REF_FREQUENCY;

  double newfreq  =  ((double) f) / r;  // check the loop freq

  if ( newfreq > HMC830_PFD_MAX || newfreq < HMC830_PFD_MIN ) return HMC830_ERROR_PFD_LIMITS;

  HMC830_reffreq = f ;
  HMC830_R[0x02] = BitFieldManipulation.WriteBF_dword(1, 14, HMC830_R[0x02], r);
  return HMC830_ERROR_NONE;
}

bool HMC830::CheckSignature() {
  digitalWrite(HMC830_PIN_SEN, HIGH);
  delayMicroseconds(1);
  ShiftX.out_Byte(HMC830_PIN_SDI, HMC830_PIN_SCK, MSBFIRST, 7, 1, 0x40); // Read from 0x00
  uint32_t SignatureRead = ShiftX.in_Dword(HMC830_PIN_SDO, HMC830_PIN_SCK, MSBFIRST, 25, 1);
  digitalWrite(HMC830_PIN_SEN, LOW);
  SignatureRead >>= 1;
  SignatureRead &= 0x00FFFFFF;
  if (SignatureRead == 0x000A7975) { // expected BIST value as per HMC830 Product & Operating Guide section 5.19
    digitalWrite(HMC830_PIN_SEN, HIGH);
    delayMicroseconds(1);
    ShiftX.out_Byte(HMC830_PIN_SDI, HMC830_PIN_SCK, MSBFIRST, 7, 1, 0x53); // Read from 0x13
    uint32_t SignatureRead = ShiftX.in_Dword(HMC830_PIN_SDO, HMC830_PIN_SCK, MSBFIRST, 25, 1);
    digitalWrite(HMC830_PIN_SEN, LOW);
    SignatureRead >>= 1;
    SignatureRead &= 0x0000FFFF;
    if (SignatureRead == 0x00001259) { // expected Chip ID value for wideband PLL/VCO including HMC830 as per HMC830 Product & Operating Guide section 5.1
      return true;
    }
    else {
      return false;
    }
  }
  else {
    return false;
  }
}