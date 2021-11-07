# ate-tracking-generator-module

Implements a dual-channel tracking(ish) Signal Generator built around the (relatively common) AD9833-MCP41010 0 -- 12.5MHz signal generator and the ADS1015 multi-channel ADC modules; also implements an SCPI-like command interface. In lieu of more complete documentation, the following excerpt defines the full set of supported commands; in (interactive) use, substitute the '0' in the multi-channel command definitions with the actual intended channel number.

```c++
const char TrackingGeneratorModuleParser::NOTHING[]    = "";
const char TrackingGeneratorModuleParser::QUERY_ONLY[] = "(QUERY ONLY)";
const char TrackingGeneratorModuleParser::UNSIGNED[]   = "<UNSIGNED>";
const char TrackingGeneratorModuleParser::FLOAT[]      = "<FLOAT>";


const Parser::Handler TrackingGeneratorModuleParser::Handlers[] = {
//    Command              Command handler                                                    Multi-channel?  Allowed values
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

```

Currently tested just on the ATmega-based Arduino platform.
