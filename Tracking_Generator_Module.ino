// =============================================================================
// Filename         : Tracking_Generator_Module.ino
// Version          : 
//
// Original Author  : Peter Metz
// Date Created     : 02-Jun-2020
//
// Revision History : 25-Jun-2020, Initial version
//                    30-Jun-2020, Added support for the ADS1115-based detector
//                        and frequency sweeping
//
// Purpose          : Implements a dual-channel tracking(ish) Signal Generator
//                    built around the (relatively common) AD9833-MCP41010 0 -- 
//                    12.5MHz signal generator and the ADS1015 multi-channel
//                    ADC modules; also implements an SCPI-like command int-
//                    erface.
//                     
// Licensing        : Copyright (C) 2020, Peter Metz
//
// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more 
// details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// ----------------------------------------------------------------------------
// Notes:
//   - the notion of an 'active' parser should be moved from here (and each
//     handler method) into an intermediate 'ModuleParser' subclass
//   - should probably change platform-dependent type declaration to fixed-
//     length ones
// =============================================================================
#include <EEPROM.h>
#include <AD9833-MCP41010.h>
#include <Adafruit_ADS1015.h>
#include <StateMachine.h>
#include <Parser.h>


// define DEBUG to enable logging of state changes, etc., to the Serial port;
// note that this approximately triples the size of the resulting code
//#define ENABLE_DEBUGGING

#ifdef ENABLE_DEBUGGING
#define LogToSerial(level, message) __LogToSerial(level, message)
#else
#define LogToSerial(level, message)
#endif

enum LogLevel { FATAL, ERROR, WARNING, INFO, VERBOSE, DEBUG };

LogLevel LoggingLevel = DEBUG;

void __LogToSerial(LogLevel level, String message) {
  if (level <= LoggingLevel)
    Serial.println(message);
}






// =============================================================================
// Class            : SweepController
//
// Purpose          : This class encapsulates the sweep state of an _individual_
//                    channel of the Tracking Generator
// =============================================================================
class SweepController : public StateMachine {
  public:
    enum SweepMode { MANUAL, AUTO, TRIGGERED };
    //enum SweepSpacing { SPACING_LINEAR , SPACING_LOGARITHMIC };

    SweepController();
    ~SweepController() { };

    // mode is represented by [this->state]
    bool          echo;
    double        startFrequency;
    double        stopFrequency;
    unsigned      steps;
    unsigned      step;
    double        timeStep;  // in seconds
    double        frequencyStep;
    unsigned long nextTime;  // in ms
};


SweepController::SweepController() {
  echo           = true;
  startFrequency = 1000.0;
  stopFrequency  = 10000.0;
  steps          = 10;
  step           = 0;
  timeStep       = 0.5;
  frequencyStep  = (stopFrequency - startFrequency) / (steps - 1);
  state          = AUTO;
}




// =============================================================================
// Class            : TrackingGeneratorModuleController
//
// Purpose          : 
// =============================================================================
class TrackingGeneratorModuleController {
  public:
    // plain-old-enums used in preference to enum class'es to minimise casts
    enum CouplingMode  { COUPLING_AC = HIGH, COUPLING_DC = LOW };
    enum TriggerSource { TRIGGER_SRC_EXTERNAL, TRIGGER_SRC_TIMER };
    enum TriggerState  { STANDBY, COUNTING_DOWN, TRIGGERED };

    // used to define defaults and for saving to, and recalling values from EEPROM
    struct SaveState {
      WaveformType waveform;
      double       frequency;
      double       phase;
      double       amplitude;
      byte         coupling;
      byte         sweepMode;
      double       sweepStartFrequency;
      double       sweepStopFrequency;
      unsigned     sweepSteps;
      double       sweepStepTime;
    };


    // not sure why we can't declare /and/ initialise these arrays here using
    // the static constexpr const <T> arrayname[] construct, but apparently
    // we can't, hence the two-part declaration...
    static const byte AddrSelectPin[];
    static const byte AD9833SelectPin[];
    static const byte MCP41010SelectPin[];
    static const byte CouplingModePin[];
    static const byte PWMDriverPin[];
    static const byte RunningLEDPin[];

    static const byte MainframeGoodPin = A7;  // HIGH => good

    static const byte ADS1115ModuleI2CAddress = 0x48;  // correct when the ADDR pin is tied to GND

    // Pins implicitly used for intra-board SPI / I2C communications --- note
    // that /here/, we're not actually using pins 10 (/SS) or 12 (MISO); also
    // note that if these pins /are/ used for something else, their modes will
    // be set/overwritten by SPI.Begin()
    // static const byte SPI_SS            = 10;
    // static const byte SPI_MOSI          = 11;
    // static const byte SPI_MISO          = 12;
    // static const byte SPI_SCK           = 13;
    // static const byte I2C_SDA           = A4;
    // static const byte I2C_SCL           = A5;

    TrackingGeneratorModuleController();
    ~TrackingGeneratorModuleController();

    void            tick();

    void            Recall();
    void            Reset();
    void            Save();
    void            Trigger();
    inline byte     UnitID() { return p_unitID; }

    // Trigger-related methods
    inline void     triggerSource(byte source)                     { p_triggerSource = source; }
    inline byte     triggerSource()                                { return p_triggerSource; }
    inline void     triggerTimer(unsigned long t)                  { p_triggerInterval = t; }
    inline unsigned triggerTimer()                                 { return p_triggerInterval; }

    // AD9833-MCP41010 signal generator-related setter- and getter-methods
    inline void     waveform(byte channel, WaveformType waveform)  { p_AD9833MCP41010Module[channel]->waveform(waveform); }
    inline double   frequency(byte channel, double frequency)      { return p_AD9833MCP41010Module[channel]->frequency(frequency); }
    inline double   phase(byte channel, double phase)              { return p_AD9833MCP41010Module[channel]->phase(phase); }
    inline double   amplitude(byte channel, double amplitude)      { return p_AD9833MCP41010Module[channel]->amplitude(amplitude); }
    inline void     enable(byte channel, bool enable)              { p_AD9833MCP41010Module[channel]->enable(enable); digitalWrite(RunningLEDPin[channel], enable); }
    inline void     coupling(byte channel, byte mode)              { p_couplingMode[channel] = mode; digitalWrite(CouplingModePin[channel], mode); }
    inline void     sweepMode(byte channel, byte mode)             { p_sweeper[channel]->state = mode; }
    inline void     sweepStartFrequency(byte channel, double freq) { p_sweeper[channel]->startFrequency = freq; }
    inline void     sweepStopFrequency(byte channel, double freq)  { p_sweeper[channel]->stopFrequency = freq; }
    inline void     sweepSteps(byte channel, unsigned steps)       { SweepController *s = p_sweeper[channel]; s->steps = steps; s->frequencyStep = (s->stopFrequency - s->startFrequency) / (steps - 1); }
    inline void     sweepStepTime(byte channel, double time)       { p_sweeper[channel]->timeStep = time; }

    inline WaveformType waveform(byte channel)                     { return p_AD9833MCP41010Module[channel]->waveform(); }
    inline double   frequency(byte channel)                        { return p_AD9833MCP41010Module[channel]->frequency(); }
    inline double   phase(byte channel)                            { return p_AD9833MCP41010Module[channel]->phase(); }
    inline double   amplitude(byte channel)                        { return p_AD9833MCP41010Module[channel]->amplitude(); }
    inline bool     enable(byte channel)                           { return p_AD9833MCP41010Module[channel]->enable(); }
    inline byte     coupling(byte channel)                         { return p_couplingMode[channel]; }
    inline byte     sweepMode(byte channel)                        { return p_sweeper[channel]->state; }
    inline double   sweepStartFrequency(byte channel)              { return p_sweeper[channel]->startFrequency; }
    inline double   sweepStopFrequency(byte channel)               { return p_sweeper[channel]->stopFrequency; }
    inline unsigned sweepSteps(byte channel)                       { return p_sweeper[channel]->steps; }
    inline double   sweepStepTime(byte channel)                    { return p_sweeper[channel]->timeStep; }

    // ADS1115 ADC-related methods
    double detector(byte channel);

    // Low-frequency analogue (i.e. PWM) output-related methods
    inline double x(byte channel, double x) { p_xOutput[channel] = x * 255; analogWrite(PWMDriverPin[channel], p_xOutput[channel]); return (double) p_xOutput[channel] / 255; }
    inline double x(byte channel) { return (double) p_xOutput[channel] / 255; }

  protected:
    void initialise(byte channel, SaveState s);

    static const SaveState Defaults;

    byte  p_unitID, p_couplingMode[2];
    double p_xOutput[2];

    byte p_triggerSource, p_triggerState;
    unsigned long p_triggerInterval, p_triggerTime;
    
    AD9833MCP41010ModuleController *p_AD9833MCP41010Module[2];
    SweepController                *p_sweeper[2];
    Adafruit_ADS1115                p_ads;
};


constexpr const byte TrackingGeneratorModuleController::AddrSelectPin[]     = { A2, A1, A0 };  // order should be reversed once backplane is corrected
constexpr const byte TrackingGeneratorModuleController::AD9833SelectPin[]   = { 2,  9  };      // aka FSYNC, LOW to enable
constexpr const byte TrackingGeneratorModuleController::MCP41010SelectPin[] = { 3,  10 };      // aka CS, LOW to enable
constexpr const byte TrackingGeneratorModuleController::CouplingModePin[]   = { 4,  7  };
constexpr const byte TrackingGeneratorModuleController::PWMDriverPin[]      = { 5,  6  };      // chosen for their higher frequency PWM capabilities
constexpr const byte TrackingGeneratorModuleController::RunningLEDPin[]     = { A3, A6 };


const TrackingGeneratorModuleController::SaveState TrackingGeneratorModuleController::Defaults = {
  SQUARE_WAVE,              // waveform
  10000,                    // frequency
  0,                        // phase
  1,                        // amplitude
  TrackingGeneratorModuleController::COUPLING_DC,
  SweepController::MANUAL,  // sweepMode
  10000,                    // sweepStartFrequency
  100000,                   // sweepStopFrequency
  91,                       // sweepSteps
  0.1                       // sweepStepTime
};




// =============================================================================
// Method           : TrackingGeneratorModuleController::TrackingGeneratorModuleController()
//
// Purpose          : 
//
// Inputs           : (none)
// =============================================================================
TrackingGeneratorModuleController::TrackingGeneratorModuleController() {
  // Set the I/O pin-modes
  pinMode(AddrSelectPin[0], INPUT_PULLUP);
  pinMode(AddrSelectPin[1], INPUT_PULLUP);
  pinMode(AddrSelectPin[2], INPUT_PULLUP);
  pinMode(MainframeGoodPin, INPUT);

  // Determine our UnitID
  p_unitID = (digitalRead(AddrSelectPin[0])) | (digitalRead(AddrSelectPin[1]) << 1) | (digitalRead(AddrSelectPin[2]) << 2);

  p_triggerState = STANDBY;
  p_triggerSource = TRIGGER_SRC_EXTERNAL;
  p_triggerInterval = 2;
  p_triggerTime = 0;

  //
  for (byte channel = 0; channel < 2; channel++) {
    p_AD9833MCP41010Module[channel] = new AD9833MCP41010ModuleController(AD9833SelectPin[channel], MCP41010SelectPin[channel]);
    p_sweeper[channel] = new SweepController();
    
    pinMode(PWMDriverPin[channel],       OUTPUT);
    pinMode(CouplingModePin[channel],    OUTPUT);
    pinMode(RunningLEDPin[channel],      OUTPUT);
    digitalWrite(RunningLEDPin[channel], LOW);
    coupling(channel, COUPLING_DC);
    x(channel, 0.0);

    initialise(channel, Defaults);
  }

  p_ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 0.125mV
  p_ads.begin();
}


// =============================================================================
// Method           : TrackingGeneratorModuleController::~TrackingGeneratorModuleController
//
// Purpose          : To manage the clean-shutdown of the physical module
// =============================================================================
TrackingGeneratorModuleController::~TrackingGeneratorModuleController() {
  for (byte channel = 0; channel < 2; channel++) {
    x(channel, 0.0);
    digitalWrite(RunningLEDPin[channel], LOW);
    delete p_AD9833MCP41010Module[channel];
    delete p_sweeper[channel];
  }
}


// =============================================================================
// Method           : TrackingGeneratorModuleController::initialise
//
// Purpose          : To 
//
// Inputs           : [channel]
//                    [s]
// =============================================================================
void TrackingGeneratorModuleController::initialise(byte channel, SaveState s) {
  waveform(channel, s.waveform);
  frequency(channel, s.frequency);
  phase(channel, s.phase);
  amplitude(channel, s.amplitude);
  coupling(channel, s.coupling);
  sweepMode(channel, s.sweepMode);
  sweepStartFrequency(channel, s.sweepStartFrequency);
  sweepStopFrequency(channel, s.sweepStopFrequency);
  sweepSteps(channel, s.sweepSteps);
  sweepStepTime(channel, s.sweepStepTime);
}


// =============================================================================
// Method           : TrackingGeneratorModuleController::Save
//
// Purpose          : To 
// =============================================================================
void TrackingGeneratorModuleController::Save() {
    SaveState s[2];

    for (byte channel = 0; channel < 2; channel++) {
      s[channel].waveform = waveform(channel);
      s[channel].frequency = frequency(channel);
      s[channel].phase = phase(channel);
      s[channel].amplitude = amplitude(channel);
      s[channel].coupling = coupling(channel);
      s[channel].sweepMode = sweepMode(channel);
      s[channel].sweepStartFrequency = sweepStartFrequency(channel);
      s[channel].sweepStopFrequency = sweepStopFrequency(channel);
      s[channel].sweepSteps = sweepSteps(channel);
      s[channel].sweepStepTime = sweepStepTime(channel);
    }

    EEPROM.put(0, s[0]);
    EEPROM.put(sizeof(s[0]), s[1]);
}


// =============================================================================
// Method           : TrackingGeneratorModuleController::Save
//
// Purpose          : To 
// =============================================================================
void TrackingGeneratorModuleController::Recall() {
  SaveState s[2];

  EEPROM.get(0, s)[0];
  EEPROM.get(sizeof(s[0]), s[1]);

  enable(0, false);
  enable(1, false);
  initialise(0, s[0]);
  initialise(1, s[1]);  
}


// =============================================================================
// Method           : TrackingGeneratorModuleController::Save
//
// Purpose          : 
// =============================================================================
void TrackingGeneratorModuleController::Reset() {
  enable(0, false);
  enable(1, false);

  initialise(0, Defaults);
  initialise(1, Defaults);
}


// =============================================================================
// Method           : TrackingGeneratorModuleController::Trigger
//
// Purpose          : 
// =============================================================================
void TrackingGeneratorModuleController::Trigger() {
  if (p_triggerSource == TRIGGER_SRC_TIMER) {
    p_triggerState = COUNTING_DOWN;
    p_triggerTime = millis() + 1000 * p_triggerInterval;
  } else
    p_triggerState = TRIGGERED;    
}


// =============================================================================
// Method           : TrackingGeneratorModuleController::tick
//
// Purpose          : To update the State Machine (currently this just affects /
//                    relates to our sweep-mode
// =============================================================================
void TrackingGeneratorModuleController::tick() {
  byte channel;
  String report = "";
  SweepController *s;
  
  if ((p_triggerState == COUNTING_DOWN) && (millis() >= p_triggerTime))
    p_triggerState = TRIGGERED;

  // check the p_sweeper's to see if one of their states have changed --- for
  // now, we're just going to support the [OFF] and [AUTO] sweep states
  for (channel = 0; channel < 2; channel++) {
    s = p_sweeper[channel];
    switch (s->state) {
      case SweepController::MANUAL:
      if (s->state.changed()) { }
      break;
  
      case SweepController::AUTO:
      if (p_triggerState == TRIGGERED) {
        enable(channel, true);
        frequency(channel, s->startFrequency);
        s->nextTime = millis() + (int) (1000 * s->timeStep);
        s->step = 1;
        if (s->echo)
          report += String(frequency(channel)) + "\t";

        s->state = SweepController::TRIGGERED;
      }
      break;
  
      case SweepController::TRIGGERED:
      if (s->step < s->steps) {
        if (millis() >= s->nextTime) {
          frequency(channel, frequency(channel) + s->frequencyStep);
          s->nextTime = millis() + (int) (1000 * s->timeStep);
          s->step++;

          if (s->echo)
            report += String(frequency(channel)) + "\t";
        }
      } else {
        // we've iterated [s->steps] number of times, so return to our 'standby' mode
        s->state = SweepController::AUTO;
      }

      break;
    }
  }

  if (p_triggerState == TRIGGERED)
    p_triggerState = STANDBY;

  // print out our sweep values if one or both [echo] flags are set
  if (report != "") {
    Serial.print(report);
    for (channel = 0; channel < 2; channel++)
      if ((p_sweeper[channel]->state == SweepController::TRIGGERED) && p_sweeper[channel]->echo)
        Serial.print(String(detector(channel), 4) + "\t");
    Serial.println("");
  }
}


// =============================================================================
// Method           : TrackingGeneratorModuleController::detector
//
// Purpose          : 
//
// Inputs           : [channel]
// =============================================================================
double TrackingGeneratorModuleController::detector(byte channel) {
  double d = 0.0;

  // average over two readings
  for (byte i = 0; i < 2; i++)
    d += (double) p_ads.readADC_SingleEnded(channel);  // channel is also zero indexed
    
  return d * 4.096 / 32768.0 / 2;
}




// =============================================================================
// Class            : TrackingGeneratorModuleParser
//
// Purpose          : This class implements the overarching controller's command
//                    parser
// =============================================================================
class TrackingGeneratorModuleParser : public Parser {
  public:
    TrackingGeneratorModuleParser(TrackingGeneratorModuleController *controller) {
      this->controller = controller;
      Parser::CommandHandlers = (Parser::Handler *)&(Handlers);

      if (controller->UnitID() == 7)
        p_active = true;
      else
        p_active = false;
    }
    inline bool active() { return p_active; }

  protected:
    TrackingGeneratorModuleController *controller;

    void                ADDRHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void                 IDNHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void                 SAVHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void                 RCLHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void                 RSTHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void                 TRGHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void                HELPHandler(const char *cmd, bool query, unsigned channel, const char *args);

    void          SystemAddrHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void              OutputHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void            CouplingHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void            FunctionHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void           FrequencyHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void               PhaseHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void           AmplitudeHandler(const char *cmd, bool query, unsigned channel, const char *args);

    void           SweepModeHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void SweepStartFrequencyHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void  SweepStopFrequencyHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void          SweepStepsHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void  SweepStepFrequencyHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void       SweepStepTimeHandler(const char *cmd, bool query, unsigned channel, const char *args);

    void       TriggerSourceHandler(const char *cmd, bool query, unsigned channel, const char *args);
    void        TriggerTimerHandler(const char *cmd, bool query, unsigned channel, const char *args);

    void            DetectorHandler(const char *cmd, bool query, unsigned channel, const char *args);

    static const char NOTHING[];
    static const char QUERY_ONLY[];
    static const char UNSIGNED[];
    static const char FLOAT[];

    static const Handler Handlers[];

    bool p_active;
    byte p_triggerMode;
    unsigned long p_triggerTime;
};


const char TrackingGeneratorModuleParser::NOTHING[]    = "";
const char TrackingGeneratorModuleParser::QUERY_ONLY[] = "(QUERY ONLY)";
const char TrackingGeneratorModuleParser::UNSIGNED[]   = "<UNSIGNED>";
const char TrackingGeneratorModuleParser::FLOAT[]      = "<FLOAT>";


const Parser::Handler TrackingGeneratorModuleParser::Handlers[] = {
    { "++ADDR",            (ParserHandler) &TrackingGeneratorModuleParser::ADDRHandler,                false, QUERY_ONLY },
    { "*IDN",              (ParserHandler) &TrackingGeneratorModuleParser::IDNHandler,                 false, QUERY_ONLY },
    { "ID",                (ParserHandler) &TrackingGeneratorModuleParser::IDNHandler,                 false, QUERY_ONLY },
    { "*SAV",              (ParserHandler) &TrackingGeneratorModuleParser::SAVHandler,                 false, NOTHING },
    { "*RCL",              (ParserHandler) &TrackingGeneratorModuleParser::RCLHandler,                 false, NOTHING },
    { "*RST",              (ParserHandler) &TrackingGeneratorModuleParser::RSTHandler,                 false, NOTHING },
    { "*TRG",              (ParserHandler) &TrackingGeneratorModuleParser::TRGHandler,                 false, NOTHING },
    { "HELP",              (ParserHandler) &TrackingGeneratorModuleParser::HELPHandler,                false, QUERY_ONLY },
    { "SYS:ADDR",          (ParserHandler) &TrackingGeneratorModuleParser::SystemAddrHandler,          false, QUERY_ONLY },
    { "OUT0",              (ParserHandler) &TrackingGeneratorModuleParser::OutputHandler,              true,  "{ 0 | 1 }" },
    { "OUT0:STAT",         (ParserHandler) &TrackingGeneratorModuleParser::OutputHandler,              true,  "{ 0 | 1 }" },
    { "OUT0:COUP",         (ParserHandler) &TrackingGeneratorModuleParser::CouplingHandler,            true,  "{ DC | AC }" },
    { "SOUR0:AMP",         (ParserHandler) &TrackingGeneratorModuleParser::AmplitudeHandler,           true,  "[ 0 .. 1]" },
    { "SOUR0:FUNC",        (ParserHandler) &TrackingGeneratorModuleParser::FunctionHandler,            true,  "{ SINe | TRIangle | SQUare }" },
    { "SOUR0:FREQ",        (ParserHandler) &TrackingGeneratorModuleParser::FrequencyHandler,           true,  FLOAT },
    { "SOUR0:FREQ:STAR",   (ParserHandler) &TrackingGeneratorModuleParser::SweepStartFrequencyHandler, true,  FLOAT },
    { "SOUR0:FREQ:STOP",   (ParserHandler) &TrackingGeneratorModuleParser::SweepStopFrequencyHandler,  true,  FLOAT },
    { "SOUR0:PHAS",        (ParserHandler) &TrackingGeneratorModuleParser::PhaseHandler,               true,  FLOAT },
    { "SOUR0:SWEEP:MODE",  (ParserHandler) &TrackingGeneratorModuleParser::SweepModeHandler,           true,  "{ MANual | AUTo }" },
    { "SOUR0:SWEEP:NSTEP", (ParserHandler) &TrackingGeneratorModuleParser::SweepStepsHandler,          true,  UNSIGNED },
    { "SOUR0:SWEEP:FSTEP", (ParserHandler) &TrackingGeneratorModuleParser::SweepStepFrequencyHandler,  true,  FLOAT },
    { "SOUR0:SWEEP:DWELL", (ParserHandler) &TrackingGeneratorModuleParser::SweepStepTimeHandler,       true,  FLOAT },
    { "TRIG:IMM",          (ParserHandler) &TrackingGeneratorModuleParser::TRGHandler,                 false, NOTHING },
    { "TRIG:SOUR",         (ParserHandler) &TrackingGeneratorModuleParser::TriggerSourceHandler,       false, "{ EXTernal | TIMer }" },
    { "TRIG:TIM",          (ParserHandler) &TrackingGeneratorModuleParser::TriggerTimerHandler,        false, FLOAT },
    { "DET0",              (ParserHandler) &TrackingGeneratorModuleParser::DetectorHandler,            true,  QUERY_ONLY },
    END_OF_HANDLERS
};




void TrackingGeneratorModuleParser::ADDRHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (!query) {
    p_active = (controller->UnitID() == (byte) strtol(args, NULL, 10));

    if (p_active)
      process("*IDN?");
  }
}


void TrackingGeneratorModuleParser::IDNHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active && query)
    Serial.println("ENGINUITY.DE,SGM-125-2,000000,0.2-20200629");
}


void TrackingGeneratorModuleParser::SAVHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active && !query)
    controller->Save();
}


void TrackingGeneratorModuleParser::RCLHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active && !query)
    controller->Recall();
}


void TrackingGeneratorModuleParser::RSTHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active && !query)
    controller->Reset();
}


// Note - triggers are not address-specific!
void TrackingGeneratorModuleParser::TRGHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  controller->Trigger();
}


void TrackingGeneratorModuleParser::HELPHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  byte i = 0;
  
  if (p_active && query)
    while (Handlers[i].cmd) {
      Serial.println(Handlers[i].cmd + String(" ") + Handlers[i].help);
      i++;
    }
}


void TrackingGeneratorModuleParser::SystemAddrHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active)
    Serial.println(controller->UnitID());
}


void TrackingGeneratorModuleParser::OutputHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active && ((channel == 1) || (channel == 2)))
    if (query) {
      Serial.println(controller->enable(channel - 1));  
  
    } else {
      if (!strcasecmp(args, "OFF") || !strncasecmp(args, "DIS", 3) || !strcasecmp(args, "0"))
        controller->enable(channel - 1, false);
      else if (!strcasecmp(args, "ON") || !strncasecmp(args, "EN", 2) || !strcasecmp(args, "1"))
        controller->enable(channel - 1, true);
    }
}


void TrackingGeneratorModuleParser::CouplingHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active && ((channel == 1) || (channel == 2)))
    if (query) {
      Serial.println((controller->coupling(channel - 1) == TrackingGeneratorModuleController::COUPLING_AC) ? "AC" : "DC");  
  
    } else {
      if (!strcasecmp(args, "AC"))
        controller->coupling(channel - 1, TrackingGeneratorModuleController::COUPLING_AC);
      else if (!strcasecmp(args, "DC"))
        controller->coupling(channel - 1, TrackingGeneratorModuleController::COUPLING_DC);
    }
}


void TrackingGeneratorModuleParser::FunctionHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active && ((channel == 1) || (channel == 2)))
    if (query) {
      switch(controller->waveform(channel - 1)) {
        case SINE_WAVE:
          Serial.println("SIN");
          break;
  
        case TRIANGLE_WAVE:
          Serial.println("TRI");
          break;
        
        case SQUARE_WAVE:
          Serial.println("SQU");
          break;
      }
    } else {
      if (!strncasecmp(args, "SIN", 3))
        controller->waveform(channel - 1, SINE_WAVE);
        
      else if (!strncasecmp(args, "TRI", 3))
        controller->waveform(channel - 1, TRIANGLE_WAVE);
        
      else if (!strncasecmp(args, "SQU", 3))
        controller->waveform(channel - 1, SQUARE_WAVE);
    }
}


void TrackingGeneratorModuleParser::FrequencyHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  double d;

  if (p_active && ((channel == 1) || (channel == 2)))
    if (query)
      Serial.println(controller->frequency(channel - 1));  
    else {
      d = strtod(args, NULL);
      if (d > 0)
        controller->frequency(channel - 1, d);
    }
}


void TrackingGeneratorModuleParser::PhaseHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  double d;

  if (p_active && ((channel == 1) || (channel == 2)))
    if (!query)
      Serial.println(controller->phase(channel - 1), 3);  
    else {
      d = strtod(args, NULL);
      if (d > 0)
        controller->phase(channel - 1, d);
    }
}


void TrackingGeneratorModuleParser::AmplitudeHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  double d;

  if (p_active && ((channel == 1) || (channel == 2)))
    if (query)
      Serial.println(controller->amplitude(channel - 1), 3);  
    else {
      d = strtod(args, NULL);
      if (d > 0)
        controller->amplitude(channel - 1, d);
    }
}


void TrackingGeneratorModuleParser::SweepModeHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active && ((channel == 1) || (channel == 2)))
    if (query) {
      switch(controller->sweepMode(channel - 1)) {  
        case SweepController::MANUAL:
          Serial.println("MANUAL");
          break;
        
        case SweepController::AUTO:
          Serial.println("AUTO");
          break;
          
        case SweepController::TRIGGERED:
          Serial.println("TRIGGERED");
          break;
        
      }
    } else {
      if (!strncasecmp(args, "MAN", 3))
        controller->sweepMode(channel - 1, SweepController::MANUAL);
        
      else if (!strncasecmp(args, "AUT", 3))
        controller->sweepMode(channel - 1, SweepController::AUTO);

      else if (!strncasecmp(args, "TRIG", 4))  // only exposed for initial testing purposes
        controller->sweepMode(channel - 1, SweepController::TRIGGERED);
    }
}


void TrackingGeneratorModuleParser::SweepStartFrequencyHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  double d;

  if (p_active && ((channel == 1) || (channel == 2)))
    if (query)
      Serial.println(controller->sweepStartFrequency(channel - 1));  
    else {
      d = strtod(args, NULL);
      if (d > 0)
        controller->sweepStartFrequency(channel - 1, d);
    }
}


void TrackingGeneratorModuleParser::SweepStopFrequencyHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  double d;

  if (p_active && ((channel == 1) || (channel == 2)))
    if (query)
      Serial.println(controller->sweepStopFrequency(channel - 1));  
    else {
      d = strtod(args, NULL);
      if (d > 0)
        controller->sweepStopFrequency(channel - 1, d);
    }
}


void TrackingGeneratorModuleParser::SweepStepsHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  unsigned steps;

  if (p_active && ((channel == 1) || (channel == 2)))
    if (query)
      Serial.println(controller->sweepSteps(channel - 1));  
    else {
      steps = atoi(args);
      if (steps == 0)
        steps = 10;
      controller->sweepSteps(channel - 1, steps);
    }  
}


// should probably extract this out into a helper function which is called whenever the other sweep-freq methods are called
void TrackingGeneratorModuleParser::SweepStepFrequencyHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  double d;

  if (p_active && ((channel == 1) || (channel == 2)))
    if (query)
      Serial.println((controller->sweepStopFrequency(channel - 1) - controller->sweepStartFrequency(channel - 1)) / controller->sweepSteps(channel - 1) + 1);  
    else {
      d = strtod(args, NULL);
      if (d > 0)
        controller->sweepSteps(channel - 1, (controller->sweepStopFrequency(channel - 1) - controller->sweepStartFrequency(channel - 1)) / d + 1);
    }  
}


void TrackingGeneratorModuleParser::SweepStepTimeHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  double d;

  if (p_active && ((channel == 1) || (channel == 2)))
    if (query)
      Serial.println(controller->sweepStepTime(channel - 1));  
    else {
      d = strtod(args, NULL);
      if (d > 0)
        controller->sweepStepTime(channel - 1, d);
    }
}


void TrackingGeneratorModuleParser::TriggerSourceHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active) {
    if (query) {
      switch(controller->triggerSource()) {
        case TrackingGeneratorModuleController::TRIGGER_SRC_EXTERNAL:
          Serial.println("EXT");
          break;
  
        case TrackingGeneratorModuleController::TRIGGER_SRC_TIMER:
          Serial.println("TIMER");
          break;        
      }

    } else {
      if (!strncasecmp(args, "EXT", 3))
        controller->triggerSource(TrackingGeneratorModuleController::TRIGGER_SRC_EXTERNAL);
        
      else if (!strncasecmp(args, "TIM", 3))
        controller->triggerSource(TrackingGeneratorModuleController::TRIGGER_SRC_TIMER);
    }
  }
}


void TrackingGeneratorModuleParser::TriggerTimerHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  double d;

  if (p_active) {
    if (query) {
      Serial.println(controller->triggerTimer());
    } else {
      d = strtod(args, NULL);
      if (d > 0)
        controller->triggerTimer(d);
    }
  }
}


void TrackingGeneratorModuleParser::DetectorHandler(const char *cmd, bool query, unsigned channel, const char *args) {
  if (p_active && query && ((channel == 1) || (channel == 2)))
      Serial.println(controller->detector(channel - 1), 5);  
}






// =============================================================================
// Arduino-specific code starts here
// =============================================================================
TrackingGeneratorModuleController *controller;
TrackingGeneratorModuleParser *parser;
bool cmdReady;
char next;
unsigned long lastTick, duration;
String cmd;

const unsigned long TickInterval = 50;  // in ms


// =============================================================================
// Function         : setup()
//
// Note             : This is the standard Arduino-defined setup function
// =============================================================================
void setup() {
  Serial.begin(38400);
  while (!Serial) {
  }

  cmd.reserve(100);
  cmdReady = false;
  
  controller = new TrackingGeneratorModuleController();
  parser = new TrackingGeneratorModuleParser(controller);
}


// =============================================================================
// Function         : serialEvent()
// =============================================================================
void serialEvent() {
  while (Serial.available()) {
    next = (char)Serial.read();
    if (next == '\b')                      // a backspace
      cmd.remove(cmd.length() - 1, 1);
    else
      cmd += next;
    
    if ((next == '\r') || (next == '\n'))  // a CR/LF
      cmdReady = true;
    else if (parser->active())
      Serial.write(next);                  // echo the received character
  }
}


// =============================================================================
// Function         : loop()
//
// Note             : This is the standard Arduino-defined loop function
// =============================================================================
void loop() {
  lastTick = millis();
  
  if (cmdReady) {
    cmd.trim();
    if (parser->active())
      Serial.println("");
      
    parser->process(cmd.c_str());
    
    cmd = "";
    cmdReady = false;  
  }

  controller->tick();

  // we need the intermediate [duration], as some serial-heavy outputs might
  // take longer than [TickInterval], and we don't want a negative (which would
  // actually turn out to be a _very_ large positive) delay
  duration = millis() - lastTick;
  if (duration < TickInterval)
    delay(TickInterval - duration);
}
