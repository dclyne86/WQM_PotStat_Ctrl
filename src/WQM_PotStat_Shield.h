#define MCU_ONLY true //for debugging purposes, will not issue commands to external shields TODO: define debugging
#define PS_STD_MSG true //true = standard msg format (raw data) for transmission to application, false = comma seperated format
//#define CONFIG_COMMS_USB true //true = communicate serial messages over USB and bluetooth


#define WQM_LED 4
#define WQM_ClSwEn 5
#define WQM_BrdPresent 11

#define PS_LED1 6
#define PS_LED2 7
#define PS_MUX0 8
#define PS_MUX1 9
#define PS_WE_SwEn 10
#define PS_BrdPresent 12

#define MB_LED 13
#define EXT_LED 14

//Constants
#define ON 1
#define OFF 0

//Experiment running / started status
#define PS_EXP_RUNNING 1
#define WQM_EXP_RUNNING 2

//Experiment Commands
#define MAX_CMD_LENGTH 64

#define MIN_SAMPLE_RATE 15
#define MAX_SAMPLE_RATE 120
#define MIN_GAIN 0
#define MAX_GAIN 7

//WQM
#define WQM_SAMP_RATE 5 //Sample freq (Hz)
#define CL_SW_ON_TIME 50000 //ms
#define CL_MEASURE_TIME 50000 //ms

// Experiment types
#define EXP_CSV 1
#define EXP_DPV 2

// Experiment intervals
#define INTERVAL_NA 0
#define INTERVAL_CLEAN 1
#define INTERVAL_DEP 2
#define INTERVAL_EXP1 3
#define INTERVAL_EXP2 4
#define INTERVAL_DN 5
// Offset to determine when current is sampled when using sync sampling (DPV/SWV)
// eg if cycle period is 100000 us, pulse width is 40000 us and SYNC_OFFSET is 2000
// then ADC sample gathered at tInt = 38000 (us) and tInt = 98000 (us)  for each cycle.
// SYNC_OFFSET should be greater than (1/fsample(DAC)) to ensure the sample is gathered before
// the output voltage changes
#define SYNC_OFFSET 2250

//dac value corresponding to 1.5V (VG/0V for analog cct)
#define DACVAL0 32767


//TIA feedback R values, in k ohm
#define RGAIN1 0.502 //499 ohm Rf + 3 ohm estimated switch resistance = 502 ohms
#define RGAIN2 10.0
#define RGAIN3 200.0
#define RGAIN4 4020.0


#define LONG_MIN -2147483648
#define LONG_MAX  2147483647

//String constants
#define STR_PRE0 "Cleaning time"
#define STR_PRE1 "Cleaning potential"
#define STR_PRE2 "Deposition time"
#define STR_PRE3 "Deposition potential"
#define STR_CSV0 "Start (mV)"
#define STR_CSV1 "Vertex 1 (mV)"
#define STR_CSV2 "Vertex 2 (mV)"
#define STR_CSV3 "Slope (mV/S)"
#define STR_CSV4 "# of scans"
#define STR_CSV5 "N/A"
#define STR_DPV0 "N/A"
#define STR_DPV1 "N/A"
#define STR_DPV2 "N/A"
#define STR_DPV3 "N/A"
#define STR_DPV4 "N/A"
#define STR_DPV5 "N/A"

//Pre-experiment parameter limits MIN,MAX

//Shared

//clean time, (us)
#define LIMS_CLEANT   0, 1800000000
//clean voltage (mV)
#define LIMS_CLEANV   -1500, 1500

//CSV Experiment parameter limits

//Start (mV)
#define LIMS_CSV0   -1500, 1500
//Vertex 1 (mV)
#define LIMS_CSV1   -1500, 1500
//Vertex 2 (mV)
#define LIMS_CSV2   -1500, 1500
//Slope (mV/S)
#define LIMS_CSV3   1, 500
//Scans
#define LIMS_CSV4   1, 100
//Not used
#define LIMS_CSV5   0, 0

//DPV Experiment parameter limits

//Start/Initial (mV)
#define LIMS_DPV0   -1500, 1500
//Stop/Final (mV)
#define LIMS_DPV1   -1500, 1500
//Step (mV)
#define LIMS_DPV2   1, 50
//Pulse Amplitude (mV)
#define LIMS_DPV3   5, 250
//Pulse Width (ms)
#define LIMS_DPV4   20, 1000
//Pulse Period (ms)
#define LIMS_DPV5   20, 5000

//Prototypes:
    void receiveCmd(void);
    boolean parseRunCmd(char *cmd, int ncmd);
    int findSubstring(int start, char *sub, int nsub, char *str, int nstr);
    boolean convInt(long * vptr, char *arr, int startIndex, int stopIndex);
    boolean isNum(char c);
    boolean checkParams (int e, int np, long * par);
    boolean setConfig (int experiment, long * par);
    size_t sendError(String s);
    size_t sendInfo(String s);
    float calcOutput(unsigned long ti, unsigned int c);
    uint16_t scaleOutput(float in);
    void calcInterval(unsigned long t);
    void writeDAC(uint16_t value);
    void startTimerADC(void);
    void startTimerDAC(void);
    void stopTimers(void);
    void led(bool b);
    void flashLed(byte n, unsigned int d);
    void setGain(byte n);
    void startExperiment(void);
    void finishExperiment(void);
    void programFail(byte code);
    void clearExp(void);
    void defLSVExp(void);
    void defCVExp(void);
    void defCVExp(void);
    void printExp(void);
    //Comms functions
    //void print(void);
    //void println(void);
    //WQM functions
    void startExperimentWQM(void);
    void getMeasurementsWQM(void);
    void sendValues(void);
    void setClSw(boolean b);
    void wqm_led(boolean b);