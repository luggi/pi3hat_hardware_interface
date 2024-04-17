#ifndef MoteusEnums_h
#define MoteusEnums_h


enum class MoteusAxisState{
  kStopped = 0,
  kFault = 1,
  kEnabling = 2,
  kCalibrating = 3,
  kCalibrationComplete = 4,
  kPwm = 5,
  kVoltage = 6,
  kVoltageFoc = 7,
  kVoltageDq = 8,
  kCurrent = 9,
  kPosition = 10,
  kPositionTimeout = 11,
  kZeroVelocity = 12,
  kStayWithin = 13,
  kMeasureInd = 14,
  kBrake = 15,
  kNumModes,
};

enum class HomeState {
  kRelative = 0,
  kRotor = 1,
  kOutput = 2,
};


#endif