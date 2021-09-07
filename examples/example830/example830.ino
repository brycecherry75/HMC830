/*

  HMC830 demo by Bryce Cherry

  Commands:
  REF reference_frequency_in_Hz reference_divider - Set reference frequency and reference divider
  FREQ/FREQ_P frequency_in_Hz power_level(1-4) outputs_enabled(0-2) timeout_in_mS - set RF frequency (FREQ_P sets precision mode), power level and enable number of outputs, calculation timeout (precision mode only - 0 to disable)
  STEP frequency_in_Hz - set channel step
  SWEEP start_frequency stop_frequency step_in_mS(1-32767) power_level(1-4) outputs_enabled(1-2) - sweep RF frequency
  CE (ON/OFF) - enable/disable VFO via CE pin
  STATUS - view status of VFO

*/

#include <HMC830.h>
#include <BigNumber.h> // obtain at https://github.com/nickgammon/BigNumber

HMC830 vfo;

const byte CEpin = 9;
const byte SENpinUsed = 10;
const byte SDIpinUsed = 11;
const byte SDOpinUsed = 12;
const byte SCKpinUsed = 13;
const byte LockPin = SDOpinUsed;

const word SweepSteps = 14; // SweepSteps * ((4 * 6) + (2 * 3)) is the temporary memory calculation (remember to leave enough for BigNumber) - 14 is the limit which will not cause an ATmega328 based board to hang during a frequency sweep

const int CommandSize = 30;
char Command[CommandSize];

// ensures that the serial port is flushed fully on request
const unsigned long SerialPortRate = 9600;
const byte SerialPortRateTolerance = 5; // percent - increase to 50 for rates above 115200 up to 4000000
const byte SerialPortBits = 10; // start (1), data (8), stop (1)
const unsigned long TimePerByte = ((((1000000ULL * SerialPortBits) / SerialPortRate) * (100 + SerialPortRateTolerance)) / 100); // calculated on serial port rate + tolerance and rounded down to the nearest uS, long caters for even the slowest serial port of 75 bps

void FlushSerialBuffer() {
  while (true) {
    if (Serial.available() > 0) {
      byte dummy = Serial.read();
      while (Serial.available() > 0) { // flush additional bytes from serial buffer if present
        dummy = Serial.read();
      }
      if (TimePerByte <= 16383) {
        delayMicroseconds(TimePerByte); // delay in case another byte may be received via the serial port
      }
      else { // deal with delayMicroseconds limitation
        unsigned long DelayTime = TimePerByte;
        DelayTime /= 1000;
        if (DelayTime > 0) {
          delay(DelayTime);
        }
        DelayTime = TimePerByte;
        DelayTime %= 1000;
        if (DelayTime > 0) {
          delayMicroseconds(DelayTime);
        }
      }
    }
    else {
      break;
    }
  }
}

void getField (char* buffer, int index) {
  int CommandPos = 0;
  int FieldPos = 0;
  int SpaceCount = 0;
  while (CommandPos < CommandSize) {
    if (Command[CommandPos] == 0x20) {
      SpaceCount++;
      CommandPos++;
    }
    if (Command[CommandPos] == 0x0D || Command[CommandPos] == 0x0A) {
      break;
    }
    if (SpaceCount == index) {
      buffer[FieldPos] = Command[CommandPos];
      FieldPos++;
    }
    CommandPos++;
  }
  for (int ch = 0; ch < strlen(buffer); ch++) { // correct case of command
    buffer[ch] = toupper(buffer[ch]);
  }
  buffer[FieldPos] = '\0';
}

void PrintVFOstatus() {
  Serial.print(F("R: "));
  Serial.println(vfo.ReadR());
  Serial.print(F("N: "));
  Serial.println(vfo.ReadN());
  Serial.print(F("Fraction: "));
  Serial.println(vfo.ReadFraction());
  Serial.print(F("Fine Frequency Control Register: "));
  Serial.println(vfo.ReadFFCR());
  Serial.print(F("Output divider: "));
  Serial.println(vfo.ReadOutDivider());
  Serial.print(F("PFD frequency (Hz): "));
  Serial.println(vfo.ReadPFDfreq());
}

void PrintErrorCode(byte value) {
  switch (value) {
    case HMC830_ERROR_NONE:
      break;
    case HMC830_ERROR_RF_FREQUENCY:
      Serial.println(F("RF frequency out of range"));
      break;
    case HMC830_ERROR_POWER_LEVEL:
      Serial.println(F("Power level incorrect"));
      break;
    case HMC830_ERROR_OUTPUTS:
      Serial.println(F("Output configuration incorrect"));
      break;
    case HMC830_ERROR_ZERO_PFD_FREQUENCY:
      Serial.println(F("PFD frequency is zero"));
      break;
    case HMC830_ERROR_FRAC_RANGE:
      Serial.println(F("Fraction is out of range"));
      break;
    case HMC830_ERROR_N_RANGE:
      Serial.println(F("N is out of range"));
      break;
    case HMC830_ERROR_FFCR_RANGE:
      Serial.println(F("Fine Frequency Control Register is out of range"));
      break;
    case HMC830_ERROR_PFD_VCO_RATIO_UNDER_20:
      Serial.println(F("PFD to VCO frequency ratio is under 20"));
      break;
    case HMC830_ERROR_PFD_AND_STEP_FREQUENCY_HAS_REMAINDER:
      Serial.println(F("PFD and step frequency division has remainder"));
      break;
    case HMC830_ERROR_RF_FREQUENCY_AND_STEP_FREQUENCY_HAS_REMAINDER:
      Serial.println(F("RF frequency and step frequency division has remainder"));
      break;
    case HMC830_ERROR_PRECISION_FREQUENCY_CALCULATION_TIMEOUT:
      Serial.println(F("Precision frequency calculation timeout"));
      break;
    case HMC830_WARNING_FREQUENCY_ERROR:
      Serial.println(F("Actual frequency is different than desired"));
      break;
    case HMC830_ERROR_R_RANGE:
      Serial.println(F("R divider is out of range"));
      break;
    case HMC830_ERROR_REF_FREQUENCY:
      Serial.println(F("Reference frequency is out of range"));
      break;
    case HMC830_ERROR_PFD_LIMITS:
      Serial.println(F("PFD frequency is out of range"));
      break;
    case HMC830_ERROR_STEP_FREQUENCY_EXCEEDS_PFD:
      Serial.println(F("Step frequency exceeds PFD frequency"));
      break;
  }
}

void setup() {
  vfo.init(SENpinUsed, SDIpinUsed, SDOpinUsed, SCKpinUsed, LockPin, true, CEpin, true);
  Serial.begin(SerialPortRate);
  digitalWrite(CEpin, HIGH); // enable the HMC830
  if (vfo.CheckSignature() == true) {
    Serial.println(F("HMC830 is present"));
  }
  else {
    Serial.println(F("HMC830 not found"));
  }
}

void loop() {
  static int ByteCount = 0;
  if (Serial.available() > 0) {
    char value = Serial.read();
    if (value != '\n' && ByteCount < CommandSize) {
      Command[ByteCount] = value;
      ByteCount++;
    }
    else {
      ByteCount = 0;
      bool ValidField = true;
      char field[20];
      getField(field, 0);
      if (strcmp(field, "REF") == 0) {
        getField(field, 1);
        unsigned long ReferenceFreq = atol(field);
        getField(field, 2);
        word ReferenceDivider = atoi(field);
        byte ErrorCode = vfo.setrf(ReferenceFreq, ReferenceDivider);
        if (ErrorCode != HMC830_ERROR_NONE) {
          ValidField = false;
          PrintErrorCode(ErrorCode);
        }
      }
      else if (strcmp(field, "FREQ") == 0 || strcmp(field, "FREQ_P") == 0) {
        bool PrecisionRequired = false;
        unsigned long CalculationTimeout = 0;
        if (strcmp(field, "FREQ_P") == 0) {
          PrecisionRequired = true;
          getField(field, 4);
          CalculationTimeout = atol(field);
        }
        getField(field, 2);
        byte PowerLevel = atoi(field);
        getField(field, 3);
        byte OutputsEnabled = atoi(field);
        getField(field, 1);
        unsigned long FrequencyWriteTimeStart = millis();
        byte ErrorCode = vfo.setf(field, PowerLevel, OutputsEnabled, PrecisionRequired, CalculationTimeout);
        if (ErrorCode == HMC830_ERROR_NONE || ErrorCode == HMC830_WARNING_FREQUENCY_ERROR) {
          unsigned long FrequencyWriteTime = millis();
          FrequencyWriteTime -= FrequencyWriteTimeStart;
          Serial.print(F("Time measured during setf() with CPU speed of "));
          Serial.print((F_CPU / 1000000UL));
          Serial.print(F("."));
          Serial.print((F_CPU % 1000000UL));
          Serial.print(F(" MHz: "));
          Serial.print((FrequencyWriteTime / 1000));
          Serial.print(F("."));
          Serial.print((FrequencyWriteTime % 1000));
          Serial.println(F(" seconds"));
          PrintVFOstatus();
        }
        else {
          ValidField = false;
        }
        PrintErrorCode(ErrorCode);
      }
      else if (strcmp(field, "STEP") == 0) {
        getField(field, 1);
        unsigned long StepFrequency = atol(field);
        byte ErrorCode = vfo.SetStepFreq(StepFrequency);
        if (ErrorCode != HMC830_ERROR_NONE) {
          ValidField = false;
          PrintErrorCode(ErrorCode);
        }
      }
      else if (strcmp(field, "SWEEP") == 0) {
        BigNumber::begin(12); // will finish on setf()
        getField(field, 1);
        BigNumber BN_StartFrequency(field);
        getField(field, 2);
        BigNumber BN_StopFrequency(field);
        if (BN_StartFrequency < BN_StopFrequency) {
          getField(field, 3);
          word SweepStepTime = atoi(field);
          getField(field, 4);
          byte PowerLevel = atoi(field);
          getField(field, 5);
          byte OutputsEnabled = atoi(field);
          char tmpstr[12];
          ultoa(vfo.HMC830_ChanStep, tmpstr, 10);
          char tmpstr2[12];
          ultoa(SweepSteps, tmpstr2, 10);
          BigNumber BN_StepSize = ((BN_StopFrequency - BN_StartFrequency) / (BigNumber(tmpstr2)) - BigNumber("1"));
          if (BN_StepSize >= BigNumber(tmpstr)) {
            BigNumber BN_StepSizeRounding = (BN_StepSize / BigNumber(tmpstr));
            uint32_t StepSizeRounding = (uint32_t)((uint32_t) BN_StepSizeRounding);
            ultoa(StepSizeRounding, tmpstr2, 10);
            BN_StepSize = (BigNumber(tmpstr) * BigNumber(tmpstr2));
            char StepSize[14];
            char StartFrequency[14];
            char* tempstring1 = BN_StepSize.toString();
            for (int i = 0; i < 14; i++) {
              byte temp = tempstring1[i];
              if (temp == '.') {
                StepSize[i] = 0x00;
                break;
              }
              StepSize[i] = temp;
            }
            free(tempstring1);
            char* tempstring2 = BN_StartFrequency.toString();
            BigNumber::finish();
            for (int i = 0; i < 14; i++) {
              byte temp = tempstring2[i];
              if (temp == '.') {
                StepSize[i] = 0x00;
                break;
              }
              StartFrequency[i] = temp;
            }
            free(tempstring2);
            uint32_t regs[(HMC830_RegsToWrite * SweepSteps)];
            uint16_t subregs[(HMC830_SubregsToWrite * SweepSteps)];
            uint32_t reg_temp[HMC830_RegsToWrite];
            uint16_t subreg_temp[HMC830_SubregsToWrite];
            for (word SweepCount = 0; SweepCount < SweepSteps; SweepCount++) {
              Serial.print(F("Calculating step "));
              Serial.print(SweepCount);
              char CurrentFrequency[14];
              BigNumber::begin(12);
              BigNumber BN_CurrentFrequency = (BigNumber(StartFrequency) + (BigNumber(StepSize) * BigNumber(SweepCount)));
              char* tempstring3 = BN_CurrentFrequency.toString();
              BigNumber::finish();
              for (int y = 0; y < 14; y++) {
                byte temp = tempstring3[y];
                if (temp == '.') {
                  CurrentFrequency[y] = 0x00;
                  break;
                }
                CurrentFrequency[y] = temp;
              }
              free(tempstring3);
              Serial.print(F(" - frequency is now "));
              Serial.print(CurrentFrequency);
              Serial.println(F(" Hz"));
              byte ErrorCode = vfo.setf(CurrentFrequency, PowerLevel, OutputsEnabled, false, 0);
              if (ErrorCode != HMC830_ERROR_NONE) {
                ValidField = false;
                PrintErrorCode(ErrorCode);
                break;
              }
              PrintVFOstatus();
              vfo.ReadSweepValues(reg_temp, subreg_temp);
              for (int y = 0; y < HMC830_RegsToWrite; y++) {
                regs[(y + (HMC830_RegsToWrite * SweepCount))] = reg_temp[y];
              }
              for (int y = 0; y < HMC830_SubregsToWrite; y++) {
                subregs[(y + (HMC830_SubregsToWrite * SweepCount))] = subreg_temp[y];
              }
            }
            if (ValidField == true) {
              Serial.println(F("Now sweeping"));
              FlushSerialBuffer();
              while (true) {
                if (Serial.available() > 0) {
                  break;
                }
                for (word SweepCount = 0; SweepCount < SweepSteps; SweepCount++) {
                  if (Serial.available() > 0) {
                    break;
                  }
                  for (int y = 0; y < HMC830_RegsToWrite; y++) {
                    reg_temp[y] = regs[(y + (HMC830_RegsToWrite * SweepCount))];
                  }
                  for (int y = 0; y < HMC830_SubregsToWrite; y++) {
                    subreg_temp[y] = subregs[(y + (HMC830_SubregsToWrite * SweepCount))];
                  }
                  vfo.WriteSweepValues(reg_temp, subreg_temp);
                  delay(SweepStepTime);
                }
                Serial.print(F("*"));
              }
              Serial.print(F(""));
              Serial.println(F("End of sweep"));
            }
          }
          else {
            BigNumber::finish();
            Serial.println(F("Calculated frequency step is smaller than preset frequency step"));
            ValidField = false;
          }
        }
        else {
          BigNumber::finish();
          Serial.println(F("Stop frequency must be greater than start frequency"));
          ValidField = false;
        }
      }
      else if (strcmp(field, "STATUS") == 0) {
        PrintVFOstatus();
        if (digitalRead(LockPin) == LOW) {
          Serial.println(F("Lock pin LOW"));
        }
        else {
          Serial.println(F("Lock pin HIGH"));
        }
      }
      else if (strcmp(field, "CE") == 0) {
        getField(field, 1);
        if (strcmp(field, "ON") == 0) {
          digitalWrite(CEpin, HIGH);
        }
        else if (strcmp(field, "OFF") == 0) {
          digitalWrite(CEpin, LOW);
        }
        else {
          ValidField = false;
        }
      }
      else {
        ValidField = false;
      }
      FlushSerialBuffer();
      if (ValidField == true) {
        Serial.println(F("OK"));
      }
      else {
        Serial.println(F("ERROR"));
      }
    }
  }
}