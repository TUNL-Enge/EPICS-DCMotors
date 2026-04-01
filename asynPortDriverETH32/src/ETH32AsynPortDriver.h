/*
 * ETH32AsynPortDriver.h
 *
 * Asyn driver that inherits from the asynPortDriver class. This is
 * the EPICS driver that will talk to the Winfield ETH32 device,
 * reading analog inputs and controlling digital outputs that will
 * drive some DC motors
 *
 * Author: Mark Rivers, Richard Longland
 *
 * Created Mar. 24, 2026
 */

#include "asynPortDriver.h"

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_RunString "SIM_RUN"                  /* asynInt32,    r/w */
#define P_Motor1ForwardString "MOTOR_1_FORWARD" /* asynInt32,    r/w */
#define P_Motor1BackwardString "MOTOR_1_BACKWARD" /* asynInt32,    r/w */
#define P_Motor1PositionString "MOTOR_1_POS"      /* asynFloat64,  r/o */
#define P_UpdateTimeString "SIM_UPDATE_TIME"   /* asynFloat64,  r/w */

/** Class that demonstrates the use of the asynPortDriver base class to greatly
 * simplify the task of writing an asyn port driver. This class does a simple
 * simulation of a motor being driven in and out. I have made the methods of
 * this class public in order to generate doxygen documentation for them, but
 * they should really all be private. */
class ETH32AsynPortDriver : public asynPortDriver {
public:
  ETH32AsynPortDriver(const char *portName);

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

  /* These are the methods that are new to this class */
  void simTask(void);

protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library.
   */
  int P_Run;
  int P_Motor1Forward;
  int P_Motor1Backward;
  int P_Motor1Position;
  int P_UpdateTime;

private:
  /* Our data */
  epicsEventId eventId_;
  epicsFloat64 *pData_;
  epicsFloat64 *pTimeBase_;
  // Functions that actually call the ETH32 library to turn on/off a digital pin
  void setMotor1Forward();    // On Port 0, bit 0
  void setMotor1Backward();   // On Port 0, bit 1
};
