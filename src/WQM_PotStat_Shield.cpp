//Libraries
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h> // for BlueTooth
#include "Adafruit_ADS1015.h"

//Project files
#include "WQM_PotStat_Shield.h"

//TODO: Define DPV parameters, set up experiment, find use for PS leds

/*
   PotStat command example:
   <R%SR:60%G:2%E:1%EP:100,100,0,0,0,-200,800,100,3,%/>

   '<' = Command start char
   'R' = Run experiment
   %SR:# = Sample rate, integer between MIN_SAMPLE_RATE and MAX_SAMPLE_RATE
   %G:# = Gain Setting (0-7): Determines TIA feedback resistance and ADC PGA setting:

   0: RG = 500, PGA = 4X, 2000uA
   1: RG = 500, PGA = 16X, 500uA
   2: RG = 10k, PGA = 4X, 100uA
   3: RG = 10k, PGA = 16X, 25uA
   4: RG = 200k, PGA = 4X, 5uA
   5: RG = 200k, PGA = 16X, 1.25uA
   6: RG = 4M, PGA = 4X, 250nA
   7: RG = 4M, PGA = 16X, 63nA

   %E:# = Experiment (1 or 2), 1 = CSV/LSV 2 = DPV (only CSV currently configured)

   %EP:#,#,...#, = Experiment parameters, varies by selected experiment

   CSV:
   P0 = Cleaning time
   P1 = Cleaning potential
   P2 = Deposition time
   P3 = Deposition potential
   P4 = Start Voltage, mV
   P5 = Vertex 1, mV
   P6 = Vertex 2, mV
   P7 = Slope, mV/S
   P8 = # of scans

   DPV:
   P0 = Cleaning time
   P1 = Cleaning potential
   P2 = Deposition time
   P3 = Deposition potential
   P4 = Start/Initial (mV)
   P5 = Stop/Final (mV)
   P6 = Step, mV
   P7 = Pulse Amplitude, mV
   P8 = Pulse Width
   P9 = Pulse Period
*/

/* IO
   Name Pin Function
   WQM:
   D4   4   O:Led (WQM_LED)
   D5   5   O:Free Cl Switch Enable (WQM_ClSwEn)
   D11  11  I:WQM Board present (WQM_BrdPresent)

   PotStat:
   D6   6   O:Led1 (PS_Led1)
   D7   7   O:Led2 (PS_Led2)
   D8   8   O:PS_MUX0, gain select multiplexer (PS_MUX0)
   D9   9   O:PS_MUX1, gain select multiplexer (PS_MUX1)
   D10  10  O:WE Switch Enable (PS_WE_SwEN)
   D12  12  I:WQM Board present (PS_BrdPresent)

   Main/Shared:
   D13  13  O:Led (MB_Led)
   A0   14  O:Ext. Led (if present)
   SDA  18  I2C SDA, comms to ADCs/DAC
   SCL  19  I2C SCL, comms to ADCs/DAC
   D0   0   RX1, Serial RX, HW Serial
   D1   1   TX1, Serial TX, HW Serial
   D2   2   RX2, Serial RX, SW Serial
   D3   3   TX2, Serial TX, SW Serial
*/

/*
 * GLOBAL VARIABLES
 */

//Board Vars
boolean WQM_Present = false; //WQM Shield Present
boolean PS_Present = false; //PotStat Shield Present

//unsigned long tScratch2 = 0;

// BlueTooth
// [BT] <-->  [Arduino]
// VCC  <-->  3.3V
// GND  <-->  GND
// TxD  <-->  pin D2
// RxD  <-->  pin D3
SoftwareSerial Serial_BT(2,3);

// WQM Variables
Adafruit_ADS1115 WQM_adc1(0x48);
Adafruit_ADS1115 WQM_adc2(0x49);

bool ClSwState = false;

float voltage_pH = 0.0; //Voltage in mV
float current_Cl = 0.0; //Current in nA
float temperature = 0.0; //Temperature in deg. C
float V_temp = 0.0; //Voltage for temperature calculation
float voltage_alkalinity = 0.0; //Voltage for temperature calculation

//Current time in seconds since start of free Cl measurement collection (milliseconds)
long switchTimeACC = 0;
//Total time of on and off phases
long switchTimePRE = CL_SW_ON_TIME + CL_MEASURE_TIME;

int16_t WQM_adc1_diff_0_1;  // pin0 - pin1, raw ADC val
int16_t WQM_adc1_diff_2_3;  // pin2 - pin3, raw ADC val
int16_t WQM_adc2_diff_0_1;  // pin0 - pin1, raw ADC val
int16_t WQM_adc2_diff_2_3;  // pin0 - pin1, raw ADC val

//end WQM vars

// PS ADC declaration
Adafruit_ADS1115 PS_adc1(0x4B);

int16_t PS_adc1_diff_0_1;  // pin0 - pin1, raw ADC val
int16_t PS_adc1_diff_2_3;  // pin2 - pin3, raw ADC val

//Number of parameters required per experiment, index 0 = null/not used, index 1 = CSV/LSV, index 2 = DPV
const int PARAMS_REQD[3] = {0, 9, 10};
//Experiment parameter limits: [Experiment][Parameter][Max/Min]
const long EXP_LIMITS[3][10][2] =  {
  {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}}, //null experiment / not used
  {{LIMS_CLEANT}, {LIMS_CLEANV}, {LIMS_CLEANT}, {LIMS_CLEANV}, {LIMS_CSV0}, {LIMS_CSV1}, {LIMS_CSV2}, {LIMS_CSV3}, {LIMS_CSV4}, {LIMS_CSV5}}, //CSV limits
  {{LIMS_CLEANT}, {LIMS_CLEANV}, {LIMS_CLEANT}, {LIMS_CLEANV}, {LIMS_DPV0}, {LIMS_DPV1}, {LIMS_DPV2}, {LIMS_DPV3}, {LIMS_DPV4}, {LIMS_DPV5}}  //DPV limits
};

unsigned int timer1_preload;
unsigned int timer2_preload;

unsigned long tExpStart = 0; // experiment start time
unsigned long tExp = 0; // current experiment time since start (total)
unsigned long tInt = 0; // current time since start of experiment interval

unsigned long tScratch = 0;

// current interval during experiment
byte currInterval = 0; // 0 = not started / NA, 1 = cleaning, 2 = deposition, 3 = 1st exp int., 4 = 2nd exp int., 5 = complete

// current cycle during experiment
int currCycle = 0;

// experiment started flag
uint8_t expStarted = 0;
// async. adc sampling started flag
boolean samplingStarted = false;

// start conversion flags set by ISR
boolean startDAC = false;
boolean PS_startADC = false;
boolean WQM_startADC = false;

//FWD and REV sampling completed for current cycle / period
boolean syncADCcompleteFWD = false;
boolean syncADCcompleteREV = false;

float vIn = 0.0;  //mV
float iIn = 0.0;  //uA
float vOut = 0.0; //V

uint16_t dacOut = DACVAL0; //Raw value for DAC output

//selected TIA feedback resistor value, in k ohm
float rGain;

//Structure for storing experiment config
struct Experiment {
  unsigned long tClean;
  float vClean;
  unsigned long tDep;
  float vDep;
  unsigned long tSwitch;    //start time of 2nd interval (us)
  unsigned long tOffset;    //offset time, used to get correct initial V
  unsigned long tCycle;     //total time per cycle, period (us)
  float vStart[2];          //start voltage of each interval (V)
  float vSlope[2];          //slope (V/us)
  float offset;             //voltage offset per cycle (V)
  int cycles;               //total cycles
  unsigned int sampRate;    //ADC sampling rate, if async sampling implemented
  boolean syncSamplingEN;     //Sync samping - false: ADC sampling based on Timer1 (CV, LSV) true: ADC samp. occurs twice per cycle (DPV, SWV)
  unsigned long tSyncSample;  //ADC start time for sync sampling
  /* Gain Setting (0-7): Determines TIA feedback resistance and ADC PGA setting:
     0: RG = 500, PGA = 4X, 2000uA
     1: RG = 500, PGA = 16X, 500uA
     2: RG = 10k, PGA = 4X, 100uA
     3: RG = 10k, PGA = 16X, 25uA
     4: RG = 200k, PGA = 4X, 5uA
     5: RG = 200k, PGA = 16X, 1.25uA
     6: RG = 4M, PGA = 4X, 250nA
     7: RG = 4M, PGA = 16X, 63nA
  */
  byte gain;
};

Experiment e; //current experiment config
char charRcvd;
/*
 * setup()
 *
 * Executes onnce board boot for setup
 *
 * Initializes IO, starts communications
 */
void setup() {
  //Initialize IO
  //main board (UNO) led pin
  pinMode(MB_LED, OUTPUT);
  //External led
  pinMode(EXT_LED, OUTPUT);

  //WQM board present input
  pinMode(WQM_BrdPresent, INPUT_PULLUP);

  //PotStat board present input
  pinMode(PS_BrdPresent, INPUT_PULLUP);

  delay(250);

  //Check for boards present
  WQM_Present = digitalRead(WQM_BrdPresent) ? false : true;
  PS_Present = digitalRead(PS_BrdPresent) ? false : true;

  delay(5000);
  //Initialize serial port - setup BLE shield
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Master Baud Rate: = 9600");
    Serial.println("Setting BLE shield comms settings, name/baud rate(115200)");
    delay(500);
    Serial.print("AT+NAMEIMWQMS"); //Set board name
    delay(250);
    Serial.print("AT+BAUD4"); //Set baud rate to 115200 on BLE Shield
    delay(250);
    Serial.println();
    Serial.println("Increasing MCU baud rate to 115200");
    delay(500);
    Serial.begin(115200);
    delay(200);
    Serial.println("Master Baud Rate: = 115200");

  //Initialize I2C
  Wire.begin(); //Start I2C
  Wire.setClock(400000L);


  if (PS_Present) {
    //Setup PotStat outputs
    //Gain select outputs
    pinMode(PS_MUX0, OUTPUT);
    pinMode(PS_MUX1, OUTPUT);
    //PotStat LEDs
    pinMode(PS_LED1, OUTPUT);
    pinMode(PS_LED2, OUTPUT);
    //WE digital switch enable
    pinMode(PS_WE_SwEn, OUTPUT);
    digitalWrite(PS_LED1, ON);
    digitalWrite(PS_LED2, ON);
    digitalWrite(PS_WE_SwEn, ON); //todo: update to only turn on during experiment
    PS_adc1.begin();

    //default to gain range 2 (10k, 4X PGA gain)
    setGain(2);

    // Reset DAC output
    writeDAC(DACVAL0); //MAX5217

    clearExp(); //clear experiment config
    defCVExp(); //set default exp config

    sendInfo("PotStat Setup complete");
  } else {
    sendInfo("No PotStat board detected");
  }
  if (WQM_Present) {
    //Setup WQM outputs
    //WQM LED
    pinMode(WQM_LED, OUTPUT);
    //Free Cl digital switch enable
    pinMode(WQM_ClSwEn, OUTPUT);
    wqm_led(ON);
    WQM_adc1.begin();
    WQM_adc2.begin();
    WQM_adc1.setGain(GAIN_TWO); // set PGA gain to 2 (LSB=0.0625 mV, FSR=2.048)
    WQM_adc2.setGain(GAIN_FOUR); // set PGA gain to 4 (LSB=0.03125 mV, FSR=1.024)
    sendInfo("WQM Setup complete");
    //Run WQM only
    delay(1000);
    startExperimentWQM(); //TODO: Delete when comms complete
  } else {
    sendInfo("No WQM board detected");
  }
  delay(100);
  digitalWrite(PS_LED1, OFF);
  digitalWrite(PS_LED2, OFF);
  wqm_led(OFF);

}
/*
 * Interrupt Service Routine
 * called by TMR2 overflow
 * raises flag in main loop to start DAC
 */
ISR(TIMER2_OVF_vect)
{
  TCNT2 = timer2_preload;   // preload timer
  startDAC = true;
}
/*
 * Interrupt Service Routine
 * called by TMR2 overflow
 * raises flag in main loop to start ADC
 */
ISR(TIMER1_OVF_vect)        // interrupt service routine
{
  TCNT1 = timer1_preload;   // preload timer
  if (expStarted == PS_EXP_RUNNING) {
    PS_startADC = true;
    WQM_startADC = false;
  } else if (expStarted == WQM_EXP_RUNNING) {
    PS_startADC = false;
    WQM_startADC = true;
    //Calculate time used for switched Cl measurements
    switchTimeACC = switchTimeACC + (1000 / WQM_SAMP_RATE);
    if (switchTimeACC > switchTimePRE) {
      switchTimeACC = switchTimeACC - switchTimePRE;
    }
  } else {
    PS_startADC = false;
    WQM_startADC = false;
    stopTimers();
  }

}

/*
 * Main program execution loop
 * 1. Calculate and start DAC conversion if flag set high (interrupt)
 * 2. Start potentiostat ADC conversion and process result if flag set high
 * 3. Start WQM ADC conversion and process resul if flag set high (interrupt)
 * 4. Receive and respond to exeternal serial comms (Start/Stop experiment)
 */
void loop()
{
  //startDAC flag set (set from interrupt)
  if (startDAC) {
    tScratch = micros(); //track execution time
    //calculate experiment time (time since exp. start)
    tExp = micros() - tExpStart;
    calcInterval(tExp); //calculate current interval, also currCycle and tInt

    if (!samplingStarted  && !e.syncSamplingEN && (currInterval > INTERVAL_DEP) && (currInterval < INTERVAL_DN)) {
      //start adc interrupt timer only after deposition period
      startTimerADC();
    }

    //Check if experiment not complete
    if (currInterval < INTERVAL_DN) {
      //calculate voltage output

      vOut = (float)calcOutput(tInt, currCycle);
      dacOut = scaleOutput(vOut);
      if (dacOut >= 0 && dacOut <= 65535) {
        long tdac = micros();
        writeDAC(dacOut); //MAX5217
        tdac = micros() - tdac;
        //Serial.print("Set DAC et: ");
        //Serial.println(tdac);
      } else {
        sendError("DAC out of range");
        //dac.setVoltage(DACVAL0, false);
        writeDAC(DACVAL0);
        programFail(4);
      }

      // Check sync sampling (DPV/SWV)
      // FWD sample takes place at end of 1st interval
      if (e.syncSamplingEN && !syncADCcompleteFWD && (tInt >= (e.tSwitch - SYNC_OFFSET)) && (tInt < e.tSwitch)) {
        //sample ADC
        PS_startADC = true;
        syncADCcompleteFWD = true;
      }
      // REV sample takes place at end of 2nd interval
      if (e.syncSamplingEN && !syncADCcompleteREV && (tInt >= (e.tCycle - SYNC_OFFSET))) {
        //sample ADC
        PS_startADC = true;
        syncADCcompleteREV = true;
      }

    } else {
      //experiment completed
      Serial.println("no");
      finishExperiment();
      sendInfo("Experiment Complete");
    }
    startDAC = false;

    //execution time:
    tScratch = micros() - tScratch;
    //Serial.print("Start DAC Total et: ");
    //Serial.println(tScratch);
  }
  //PS_startADC flag set  (set from interrupt (CSV) or after DAC(DPV))
  if (PS_startADC) {
    tScratch = micros();

    if (!PS_Present && MCU_ONLY) {
      iIn = vOut;
    } else {
      PS_adc1_diff_0_1 = PS_adc1.readADC_Differential_0_1();
      vIn = PS_adc1_diff_0_1 * 0.03125; // in mV
      iIn = vIn / rGain; // in uA
    }

    //**** Send new data message

    if (PS_STD_MSG){ /* Standard raw data msg */
      //Interface is expecting signed 32bit integer so data
      //must be padded with leading 0's or 1's depending on sign
      uint8_t fillBits = 0;
      if (PS_adc1_diff_0_1 < 0) fillBits = 0XFF;

      Serial.write('B'); //signify new data follows
      Serial.write(13); //cr
      Serial.flush();
      //Send Data

      //DAC output
      Serial.write((uint8_t)(dacOut & 0XFF));
      Serial.write((uint8_t)(dacOut >> 8));
      //Serial.print(dacOut);

      //ADC input
      Serial.write((uint8_t)(PS_adc1_diff_0_1 & 0XFF));
      Serial.write((uint8_t)(PS_adc1_diff_0_1 >> 8));
      Serial.write(fillBits);
      Serial.write(fillBits);
      Serial.write(13); //cr
      Serial.flush();
    }
    else /* debug / csv style msg */
    {
      Serial.print(dacOut);
      Serial.write(',');
      Serial.print(vOut);
      Serial.write(',');
      Serial.println(iIn);
    }

    PS_startADC = false;
    tScratch = micros() - tScratch;
    //Serial.println(tScratch);
  }
  //WQM_startADC flag set  (set from interrupt)
  if (WQM_startADC) {
    getMeasurementsWQM();
    sendValues();
    if (switchTimeACC >= CL_SW_ON_TIME && switchTimeACC < switchTimePRE) {
      ClSwState = true;
    } else {
      ClSwState = false;
    }
    setClSw(ClSwState);
    wqm_led(ClSwState);

    WQM_startADC = false;
    digitalWrite(EXT_LED,ClSwState);
  }

  /*
     Respond to serial communications
  */
  /*
  if (Serial.available() > 0) {
    charRcvd = Serial.read();
    //check if experiment started
    if (expStarted == 0 && charRcvd == '!') {
      //handshake received, reply and prepare to read command
      Serial.print("C");
      led(ON);
      //receive and parse command
      receiveCmd();
    } else if (expStarted == 0 && charRcvd == '?') {
      startExperimentWQM();
    } else if (expStarted == 0 && true && charRcvd == 'r') {//TODO: delete
      startExperiment();
    } else if (charRcvd == 'x') {
      finishExperiment();
      sendInfo("Experiment Stopped");
    }
  }
  */

}
/*
 * FUNCTIONS
 */
/*
 * Receives potentiostat experiment command from serial port
 * Parses data and starts experiment if valid
 */
void receiveCmd() {
  boolean receiving = true;
  boolean valid = false; //command has valid prefix/suffix chars
  unsigned long rxStart = millis();

  char cmd[MAX_CMD_LENGTH];
  memset(cmd, 0, sizeof(cmd));
  int nReceived = -1;
  Serial.setTimeout(2000);
  while (receiving && (millis() - rxStart) < 20000) {
    if (Serial.available() > 0) {
      //data has arrived, check for leading/start char ('<')
      if (Serial.read() == '<') {
        //leading char correct, check remaining chars
        nReceived = Serial.readBytesUntil('>', cmd, MAX_CMD_LENGTH);
        if (nReceived > 0 && cmd[nReceived - 1] == '/' ) {
          //command start/stop chars valid; parse cmd data
          //cmd[nReceived - 1] = 0;
          valid = true;
          //determine command type
          switch (cmd[0]) {
            case 'R':
              if (parseRunCmd(cmd, nReceived)) {
                startExperiment();
              } else {
                sendError("Could not parse command / command invalid");
              }
              break;

            default:
              //Command not recognized
              sendError("Command not recognized");
              break;

          }
          receiving = false;
        } else {
          //read timed out or invalid cmd stop char
          receiving = false;
        }

      } else {
        //leading char not correct
        receiving = false;
      }
    }
  }
  if (receiving) sendError("Command not received");
  else if (!valid) sendError("Received command not valid");
  led(OFF);
}

/* Extract parameters from run command char array

    References global current experiment config 'e'

    returns: success status - function is successful if parameters
    were extraced, converted to experiment config, and config is valid
*/
boolean parseRunCmd(char *cmd, int ncmd) {

  int iStart, iEnd;
  int iDelim, iDelimPrev;
  long value = 0;
  long params[10]; //array for holding parsed experiment parameters
  int nParams; //number of parsed parameters

  memset(params, 0, sizeof(params));


  //*** Sample Rate
  //look for "%SR:" in command (sample rate)
  iStart = findSubstring(0, "%SR:", 4, cmd, ncmd);
  if (iStart < 0) return false; // substring not found
  //find enclosing '%'
  iEnd = findSubstring(iStart, "%", 1, cmd, ncmd);
  if (iEnd < 0) return false; // substring not found
  //format chars cmd[i] for i = (iStart + 1) to (iEnd)  into sample rate (int)
  if (!convInt(&value, cmd, iStart + 1, iEnd - 1)) return false; //conversion not successful
  //check if extracted sample rate in valid range:
  if (value >= MIN_SAMPLE_RATE && value <= MAX_SAMPLE_RATE) {
    e.sampRate = value;
  } else {
    sendError("Sample Rate out of range");
    return false; //out of range
  }

  //*** Gain
  //look for "%G:" in command (gain)
  iStart = findSubstring(0, "%G:", 3, cmd, ncmd);
  if (iStart < 0) return false; // substring not found
  //find enclosing '%'
  iEnd = findSubstring(iStart, "%", 1, cmd, ncmd);
  if (iEnd < 0) return false; // substring not found
  //format chars cmd[i] for i = (iStart + 1) to (iEnd)  into value (long)
  if (!convInt(&value, cmd, iStart + 1, iEnd - 1)) return false; //conversion not successful
  //check if extracted gain in valid range:
  if (value >= MIN_GAIN && value <= MAX_GAIN) {
    e.gain = value;
    setGain(e.gain);
  } else {
    sendError("Gain out of range");
    return false; //out of range
  }

  //*** Experiment Parameters
  //look for "%EP:" in command (exp. params)
  iStart = findSubstring(0, "%EP:", 4, cmd, ncmd);
  if (iStart < 0) return false; // substring not found
  //find enclosing '%'
  iEnd = findSubstring(iStart, "%", 1, cmd, ncmd);
  if (iEnd < 0) return false; // substring not found
  nParams = 0;
  iDelimPrev = iStart;
  //extract individual parameters
  for (int i = 0; i < 10; i++) {
    iDelim = findSubstring(iDelimPrev + 1, ",", 1, cmd, ncmd);
    if (iDelim < 0) return false; //delimiter not found
    if (!convInt(&params[i], cmd, iDelimPrev + 1, iDelim - 1)) return false; //return if conversion not successful
    nParams++;
    if (iDelim == iEnd - 1) break; //last delimiter
    iDelimPrev = iDelim;
  }

  //*** Experiment
  //look for "%E:" in command (experiment type)
  iStart = findSubstring(0, "%E:", 3, cmd, ncmd);
  if (iStart < 0) return false; // substring not found
  //find enclosing '%'
  iEnd = findSubstring(iStart, "%", 1, cmd, ncmd);
  if (iEnd < 0) return false; // substring not found
  //format chars cmd[i] for i = (iStart + 1) to (iEnd)  into value (long)
  if (!convInt(&value, cmd, iStart + 1, iEnd - 1)) return false; //conversion not successful
  //check if extracted experiment is valid:
  if (value == EXP_CSV) {
    //valid experiment
    if (checkParams(value, nParams, params) && setConfig(value, params)) {
      //params valid and experiment configuration set
      return true;
    } else {
      return false;
    }
  } else if (value == EXP_DPV) {
    //valid experiment
    if (checkParams(value, nParams, params) && setConfig(value, params)) {
      //params valid and experiment configuration set
      return true;
    } else {
      return false;
    }
  } else {
    sendError("Selected experiment invalid/not supported");
    return false; //out of range
  }
  return true;
}

/* Finds substring within char array starting from a provided index
   inputs:
   @param   start   starting index to begin search
   @param   *sub    pointer to first element of substring char array
   @param   nsub    length of substring array
   @param   *str    pointer to first element of string to search char array
   @param   nstr    length of string array

   returns: index within string (searched) array
   corresponding to location of the END of substring
*/
int findSubstring(int start, char *sub, int nsub, char *str, int nstr) {

  // if either param <1 return unsuccseful
  if (nstr < 1 || nsub < 1) return -1;

  int index = -1;

  //iterate over string
  for (int i = start; i < (nstr - nsub) + 1;  i++ ) {
    //iterate over substring
    for (int j = 0; j < nsub; j++) {
      //break substring loop if not equal
      if ((sub[j] != str[i + j])) break;
      //check for complete match
      if (j == (nsub - 1)) {
        //substring found
        index = i + j;
        return index;
      }
    }
  }

  return -1;
}

/* Convert a series (array) of chars into an integer
 *
 *  returns: true if successful, false if invalid / non-numeric char is in array
 */
boolean convInt(long * vptr, char *arr, int startIndex, int stopIndex) {
  long scratch = 0;
  if (startIndex > stopIndex) return false;
  long multiplier = 1;
  //iterate in reverse order through array elements
  for (int i = stopIndex; i >= startIndex; i--) {
    //check if char is a number/digit 0-9
    if (isNum(arr[i])) {
      //convert to long, multiply by 10^iteration and add to total
      scratch = scratch + (long)(arr[i] - '0') * multiplier;
      multiplier = multiplier * 10;
    } else if (i == startIndex && arr[i] == '-') {
      //if leading char is minus sign, negate the total
      scratch = scratch * -1;
    } else {
      //invalid / non-numeric char
      return false;
    }
  }
  *vptr = scratch;
  return true;
}

//Check if char represents a number
boolean isNum(char c) {
  return (c >= 0x30 && c <= 0x39);
}

/*
 * Checks if experiment parameters are within max/min limits
 *
 * Checks if parameters are within experiment specific limits, if applicable
 *
 * returns: true if successful
 */
boolean checkParams (int e, int np, long * par) {
  if ((e != EXP_CSV) && (e != EXP_DPV)) return false; // invalid experiment
  if (np != PARAMS_REQD[e]) return false; // number of supplied parameters not equal to required parameters for selected exp

  //Check if supplied parameters within constant limits
  for (int i = 0; i < np; i++) {
    if (par[i] < EXP_LIMITS[e][i][0]) {
      sendError("Parameter out of range (below min)");
      return false;
    } else if (par[i] > EXP_LIMITS[e][i][1]) {
      sendError("Parameter out of range (above max)");
      return false;
    }
  }

  // Check if parameters within experiment specific limits

  switch (e) {
    case EXP_CSV:
      // TODO
      break;
    case EXP_DPV:
      // TODO Pulse period > pulse width, experiment < 70 min, stop voltage > start voltage, stop voltage + amplitude < 1500 mv
      break;
  }
  return true;
}

/*
   Sets experiment settings based on received parameters
   e.sampRate and e.gain set outside function

   CV/LSV:
   PAR#   DESC.           NOTES
   0      Clean T (us)    e.tClean=par[0];
   1      Clean V (mV)    e.vClean=par[1];
   2      Dep. T (us)     e.tDep=par[2];
   3      Dep. V (mV)     e.vDep=par[3];
   4      Start (mV)      determines tOffset from Vertex 1 and Slope: e.tOffset = 1e6*abs((par[5] - par[4])/par[7]);
   5      Vertex 1 (mV)   e.vStart[0] = par[5];
   6      Vertex 2 (mV)   e.vStart[1] = par[6];
   7      Slope (mV/s)    e.vSlope[] =  par[7]/1000000000; (sign based on vStart[0],vStart[1])
   8      Scans           e.cycles = par[8];

   tSwitch determined from Vertex 2 and Slope: e.tSwitch = 1e6*abs(par[6] - par[5])/par[7]; e.tCycle = 2*e.tSwitch;

   e.syncSamplingEN = false, e.tSyncSample = 0 (N/A), e.offset = 0 (N/A)

   DPV:
   PAR#   DESC.               NOTES
   0      Clean T (us)        e.tClean=par[0];
   1      Clean V (mV)        e.vClean=par[1]/1000;
   2      Dep. T (us)         e.tDep=par[2];
   3      Dep. V (mV)         e.vDep=par[3]/1000;
   4      Start (mV)          e.vStart[0] = par[4];
   5      Stop (mV)           determines # cycles from Start and Step: e.cycles = (par[4]-par[5])/par[6];
   6      Step (mV)
   7      Pulse Amp(mV)       e.vStart[1] = par[7] - e.vStart[0];
   8      Pulse Width (ms)    e.tSwitch = (par[9]-par[8])*1e3
   9      Pulse Period (ms)   e.tCycle = par[9]*1e3

   e.syncSamplingEN = true, e.tSyncSample = 0 (N/A), e.vSlope[] = 0

*/
boolean setConfig (int experiment, long * par) {

  switch (experiment) {
    case EXP_CSV:
      e.tClean = par[0];
      e.vClean = float(par[1] / 1000.0);
      e.tDep = par[2];
      e.vDep = float(par[3] / 1000.0);
      e.tOffset = abs((par[5] - par[4]) * 1e6 / par[7]);
      e.vStart[0] = float(par[5] / 1000.0);
      e.vStart[1] = float(par[6] / 1000.0);
      e.vSlope[0] = e.vStart[1] > e.vStart[0] ? float(par[7] * 1E-9) : float(par[7] * -1E-9);
      e.vSlope[1] = e.vSlope[0] * -1;
      e.tSwitch = abs(par[6] - par[5]) * 1e6 / par[7];
      e.tCycle = 2 * e.tSwitch;
      e.cycles = par[8];
      e.syncSamplingEN = false;
      e.offset = 0;
      break;

    case EXP_DPV:
      e.tClean = par[0];
      e.vClean = float(par[1] / 1000.0);
      e.tDep = par[2];
      e.vDep = float(par[3] / 1000.0);
      e.tOffset = 0.0;
      e.vStart[0] = float(par[4] / 1000.0);
      e.vStart[1] = float(par[7] / 1000.0) - e.vStart[0];
      e.vSlope[0] = 0.0;
      e.vSlope[1] = 0.0;
      e.tCycle = par[9] * 1e3;
      e.tSwitch = (par[9] - par[8]) * 1e3;
      e.cycles = (par[4] - par[5]) / par[6];
      e.syncSamplingEN = true;
      e.offset = float(par[6] / 1000.0);
      break;

    default:
      return false;
  }
  return true;
}
//Add error prefix text to message and send to user
size_t sendError(String s) {
  //return 0;
  return Serial.println(String("Error: " + s));
}

//Add info prefix text to message and send to user
size_t sendInfo(String s) {
  //return 0;
  return Serial.println(String("Info: " + s));
}


/* Calculate output voltage (float) based on current interval (global var),
    experiment interval time (ti), current cycle (c), and current
    experiment configuration (global var)
*/
float calcOutput(unsigned long ti, unsigned int c) {
  float vout;
  unsigned long timeEx = micros();
  if (currInterval == INTERVAL_CLEAN) {
    vout = e.vClean;
  } else if (currInterval == INTERVAL_DEP) {
    vout = e.vDep;
  } else if (currInterval == INTERVAL_EXP1) {
    //in active experiment region, 1st interval
    vout = e.vStart[0] + e.vSlope[0] * (float)ti + (float)c * e.offset;
  } else if (currInterval == INTERVAL_EXP2) {
    //in 2nd interval
    vout = e.vStart[1] + e.vSlope[1] * (float)(ti - e.tSwitch) + (float)c * e.offset;
  } else {
    vout = 0.0;
  }
  timeEx = micros() - timeEx;
  //Serial.println(timeEx);
  return vout;
}

/* Scales output from float (-1.5 <= in <= 1.5) to uint16_t for DAC output (0 to 65535)
*/
uint16_t scaleOutput(float in) {
  uint16_t scaled;
  unsigned long timeEx = micros();
  if (in >= 1.5) {
    //desired output out of range (>1.5V)
    //scaled = 4095;
    scaled = 65535;
  } else if (in <= -1.5) {
    //desired output out of range (<-1.5V)
    scaled = 0;
  } else {
    //desired output in range, scale
    long inVal = (long)((in + 1.5) * 21845000.0); // add offset, scale (*1635), convert to long
    scaled = inVal / 1000;
    if (inVal % 1000 >= 500) {
      //round up
      scaled += 1;
    }
  }
  timeEx = micros() - timeEx;
  //Serial.println(timeEx);
  return scaled;
}

/* Calculate current experiment interval and cycle based on experiment time
    if interval is a exp. cycle interval, set global var tInt
    reset syncADCcomplete on new cycle

    TODO this will probably need to be adjusted when DPV is added as there will be many
    cycles per scan, modify to check # of scans and cycles before marking experiment complete,
    and increment scans as appropriate (after every two cycles for CV)
*/
void calcInterval(unsigned long t) {
  byte prevInterval = currInterval;
  unsigned long timeEx = micros();
  if (t < e.tClean) {
    currInterval = INTERVAL_CLEAN;
    tInt = 0;
    currCycle = -1;
  }
  else if (t < (e.tClean + e.tDep)) {
    currInterval = INTERVAL_DEP;
    tInt = 0;
    currCycle = -1;
  }
  else {
    //in active experiment region
    //determine current cycle
    currCycle = (t - e.tClean - e.tDep) / e.tCycle;
    //calc interval time
    tInt = (t - (e.tClean + e.tDep) + e.tOffset) % e.tCycle;
    if (currCycle >= e.cycles) {
      //experiment complete
      currInterval = INTERVAL_DN;
    }
    else if (tInt < e.tSwitch) {
      //in 1st interval
      if (prevInterval != INTERVAL_EXP1) {
        //new interval, reset sync ADC
        syncADCcompleteFWD = false;
        syncADCcompleteREV = false;
        if (prevInterval == INTERVAL_EXP2) Serial.println("S"); //send new scan char, TODO: update when DPV added
      }
      currInterval = INTERVAL_EXP1;
    } else {
      //in 2nd interval
      currInterval = INTERVAL_EXP2;
    }
  }
  timeEx = micros() - timeEx;
  //Serial.println(timeEx);
}

/* Issue write command to DAC via I2C, return without writing if no shield (potentiostat) present */
void writeDAC(uint16_t value) {
  if (!PS_Present)
    return;
  Wire.beginTransmission(0x1C);
  Wire.write((uint8_t)0x01);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0XFF));
  Wire.endTransmission(0x1C);
}

/* Start timer used for to trigger interrupt for ADC conversion
    Timer preload/prescalers based on sample rate in current config (global var)
*/
void startTimerADC()
{
  // initialize timer1 (ADC interrupt)
  TCCR1A = 0;
  TCCR1B = 0;

  if (!e.syncSamplingEN) {
    // Set timer1_counter to the correct value for our interrupt interval
    timer1_preload = 131;

    if (e.sampRate > 30) {
      // 8x prescaler
      TCCR1B |= (1 << CS11);
      // preload timer 65536-16MHz/8/XXHz
      timer1_preload = 65536 - (2000000 / e.sampRate);
      //check if result should be rounded up
      if (2000000 % e.sampRate > e.sampRate / 2)
        timer1_preload += 1;
    } else {
      // 64x prescaler
      TCCR1B |= (1 << CS10);
      TCCR1B |= (1 << CS11);
      // preload timer 65536-16MHz/64/XXHz
      timer1_preload = 65536 - (250000 / e.sampRate);
      //check if result should be rounded up
      if (250000 % e.sampRate > e.sampRate / 2)
        timer1_preload += 1;
    }
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    TCNT1 = timer1_preload;   // preload timer
    samplingStarted = true;
  }
}

/* Start timer used for to trigger interrupt for DAC conversion
    Timer preload/prescalers based on desired DAC output rate
*/
void startTimerDAC()
{
  // initialize timer2 (DAC interrupt)
  TCCR2A = 0;
  TCCR2B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  timer2_preload = 131;   // preload timer 256-16MHz/256/500Hz

  TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);    // 256 prescaler
  TIMSK2 |= (1 << TOIE2);   // enable timer overflow interrupt
  TCNT2 = timer2_preload;   // preload timer

}

// Stops timer 1 and timer 2, disables interrupts
void stopTimers()
{
  // disable timer interrupts (DAC/ADC interrupt)
  TIMSK1 &= (0 << TOIE1);   // disable timer overflow interrupt
  TIMSK2 &= (0 << TOIE2);   // disable timer overflow interrupt
  //Stop timer
  TCCR1B = 0;
  TCCR2B = 0;
  samplingStarted = false;
}

//Turn Arduino board LED ON or OFF
void led(bool b) {
  digitalWrite(MB_LED, b);
}

//Flash Arduino board led n times with on/off time of d ms
void flashLed(byte n, unsigned int d) {
  for (byte i = 0; i < n * 2;  i++ ) {
    digitalWrite(MB_LED, digitalRead(MB_LED) ^ 1);
    digitalWrite(EXT_LED, digitalRead(MB_LED) ^ 1);
    delay(d);
  }
}


//Select feedback resistance based on gain selection (0-7)
void setGain(byte n) {
  //Set feedback resistance based on selection
  switch (n / 2)
  {
    case 0:
      //NO1, RG = RG1
      rGain = RGAIN1;
      digitalWrite(PS_MUX0, 0);
      digitalWrite(PS_MUX1, 0);
      break;
    case 1:
      //NO2, RG = RG2
      rGain = RGAIN2;
      digitalWrite(PS_MUX0, 1);
      digitalWrite(PS_MUX1, 0);
      break;
    case 2:
      //NO3, RG = RG3
      rGain = RGAIN3;
      digitalWrite(PS_MUX0, 0);
      digitalWrite(PS_MUX1, 1);
      break;
    case 3:
      //NO3, RG = RG4
      rGain = RGAIN4;
      digitalWrite(PS_MUX0, 1);
      digitalWrite(PS_MUX1, 1);
      break;
    default:
      //Invalid selection
      programFail(3);
      break;
  }
  if (n % 2) {
    //n = 1,3,5, or 7
    PS_adc1.setGain(GAIN_SIXTEEN);
    rGain = rGain * 4;
  } else {
    //n = 0,2,4 or 6
    PS_adc1.setGain(GAIN_FOUR);// set PGA gain to 4 (LSB=0.03125 mV, FSR=+/-1.048)
  }
}

void startExperiment() {
  printExp();//TODO: temp
  flashLed(4, 150);
  sendInfo("Starting Experiment");
  tExpStart = micros();
  samplingStarted = false;
  startTimerDAC();
  expStarted = PS_EXP_RUNNING;

}

/* Finish active experiment
    reset DAC to default, reset timers, reset expStarted status
*/
void finishExperiment() {
  writeDAC(DACVAL0);
  tExpStart = 0;
  currCycle = 0;
  stopTimers();
  expStarted = 0;
  switchTimeACC = 0; //Reset WQM switch time
  flashLed(2, 300);
}

// Action if unexpected error occurs
// lock program and flash led until board reset
void programFail(byte code) {
  stopTimers();
  while (1) {
    flashLed(code, 175);
    delay(2500);
  }
}
/* clear all elements of experiment structure
*/
void clearExp() {
  e.tClean = 0UL;
  e.vClean = 0.0;
  e.tDep = 0UL;
  e.vDep = 0.0;
  e.tSwitch = 0UL;
  e.tOffset = 0UL;
  e.vStart[0] = 0.0;
  e.vStart[1] = 0.0;
  e.vSlope[0] = 0.0;
  e.vSlope[1] = 0.0;
  e.tCycle = 0UL;
  e.offset = 0.0;
  e.cycles = 0;
  e.syncSamplingEN = false;
  e.tSyncSample = 0UL;
}

//default LSV experiment (debug)
void defLSVExp() {
  e.tClean = 000000UL;
  e.vClean = 0.0;
  e.tDep = 500000UL;
  e.vDep = -0.5;
  e.tSwitch = 40000000UL;
  e.tOffset = 00000000UL;
  e.vStart[0] = -1.0;
  e.vStart[1] = 1.0;
  e.vSlope[0] = 0.00000005;
  e.vSlope[1] = -0.00000005;
  e.tCycle = 40000000UL;
  e.offset = 0.0;
  e.cycles = 1;
  e.sampRate = 30;
  e.syncSamplingEN = false;
  e.tSyncSample = 0UL;
  e.gain = 0;
}
//default CV experiment (debug)
void defCVExp() {
  e.tClean = 000000UL;
  e.vClean = 0.0;
  e.tDep = 2000000UL;
  e.vDep = 0;
  e.tSwitch = 20000000UL;
  e.tOffset = 0UL;
  e.vStart[0] = -0.2;
  e.vStart[1] = 0.8;
  e.vSlope[0] = 0.00000005;
  e.vSlope[1] = -0.00000005;
  e.tCycle = 40000000UL;
  e.offset = 0.0;
  e.cycles = 2;
  e.sampRate = 10;
  e.syncSamplingEN = false;
  e.tSyncSample = 0UL;
  e.gain = 2;
}
//default DPV experiment (debug)
void defDPVExp() {
  e.tClean = 0UL;
  e.vClean = -0.5;
  e.tDep = 300000UL;
  e.vDep = 0.0;
  e.tSwitch = 40000UL;
  e.tOffset = 0UL;
  e.vStart[0] = 0.2;
  e.vStart[1] = 0.0;
  e.vSlope[0] = 0.0;
  e.vSlope[1] = 0.0;
  e.tCycle = 100000UL;
  e.offset = 0.1;
  e.cycles = 20;
  e.sampRate = 30;
  e.syncSamplingEN = true;
  e.tSyncSample = e.tCycle - SYNC_OFFSET;
  e.gain = 2;
}

//Print current experiment settings to serial port
void printExp() {
  Serial.println(String("tClean: " + String(e.tClean)));
  Serial.println(String("vClean: " + String(e.vClean)));
  Serial.println(String("tDep: " + String(e.tDep)));
  Serial.println(String("vDep: " + String(e.vDep)));
  Serial.println(String("tSwitch: " + String(e.tSwitch)));
  Serial.println(String("tOffset: " + String(e.tOffset)));
  Serial.println(String("vStart[0]: " + String(e.vStart[0])));
  Serial.println(String("vStart[1]: " + String(e.vStart[1])));
  Serial.println(String("vSlope[0]*1E9: " + String(e.vSlope[0]*1E9)));
  Serial.println(String("vSlope[1]*1E9: " + String(e.vSlope[1]*1E9)));
  Serial.println(String("tCycle: " + String(e.tCycle)));
  Serial.println(String("offset: " + String(e.offset)));
  Serial.println(String("cycles: " + String(e.cycles)));
  Serial.println(String("sampRate: " + String(e.sampRate)));
  Serial.println(String("syncSamplingEN: " + String(e.syncSamplingEN)));
  Serial.println(String("tSyncSample: " + String(e.tSyncSample)));
  Serial.println(String("gain: " + String(e.gain)));
}

//WQM FUNCTIONS:
  void startExperimentWQM() {
    flashLed(4, 150);
    sendInfo("Starting WQM Experiment");
    samplingStarted = false;
    e.syncSamplingEN = false;
    e.sampRate = WQM_SAMP_RATE;
    expStarted = WQM_EXP_RUNNING;
    startTimerADC();
  }
  void getMeasurementsWQM() {

    // read from the ADC, and obtain a sixteen bits integer as a result
    if (WQM_Present) {
      WQM_adc1_diff_2_3 = WQM_adc1.readADC_Differential_2_3();
      delay(5);
      WQM_adc2_diff_0_1 = WQM_adc2.readADC_Differential_0_1();
      delay(5);
      WQM_adc1_diff_0_1 = WQM_adc1.readADC_Differential_0_1();
      delay(5);
      WQM_adc2_diff_2_3 = WQM_adc2.readADC_Differential_2_3();


    } else {
      //Simulated ADC signals for when not connected to WQM board (temp) (fudges random data)
      WQM_adc1_diff_0_1 = 2000 + random(100);
      if (ClSwState) {
        WQM_adc1_diff_2_3 = -2000 + random(100);
      } else {
        WQM_adc1_diff_2_3 = 0;
      }
      WQM_adc2_diff_0_1 = 2000 + random(100);
      WQM_adc2_diff_2_3 = 2000 + random(100);

    }
    voltage_pH = WQM_adc1_diff_0_1 * 0.0625; // in mV
    current_Cl = -WQM_adc1_diff_2_3 * 0.0625 / 0.0255; // in nA, feedback resistor = 500k
    V_temp = WQM_adc2_diff_0_1 * 0.03125; // in mV
    voltage_alkalinity = WQM_adc2_diff_2_3 * 0.03125; // in mV
  }

  //Send WQM meas. values over serial port
  void sendValues() {

    // Send data to Serial port
    Serial.print(" ");
    Serial.print(V_temp, 4);
    Serial.print(" ");
    Serial.print(voltage_pH, 4);
    Serial.print(" ");
    Serial.print(voltage_alkalinity, 4);  //Make changes in app to read the proper order #TODO
    Serial.print(" ");
    Serial.print(current_Cl, 4);
    Serial.print(" ");
    Serial.print((float)switchTimeACC / 1000.0, 1);  //Turns off the switch for free chlorine
    Serial.print(" ");
    if (ClSwState) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
    Serial.print(" ");
    Serial.print("\n");
  }

  //Set free Cl switch ON or OFF
  void setClSw(boolean b) {
    digitalWrite(WQM_ClSwEn, b);
  }

  //Turn WQM board LED ON or OFF
  void wqm_led(boolean b) {
    digitalWrite(WQM_LED, b);
  }
