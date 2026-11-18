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

/* Number of motors. Each motor is one asyn address (0 to NUM_MOTORS-1).
 * The ETH32 has 8 analog inputs (one position readback per motor, on
 * logical channel = address), and each motor uses 2 digital outputs,
 * 4 motors per port, on ports 0 and 1. */
#define NUM_MOTORS 8
#define MOTORS_PER_PORT 4

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support.
 * The MOTOR_* parameters exist once per motor (asyn address). */
#define P_MotorRunString "MOTOR_RUN"           /* asynInt32,    r/w */
#define P_MotorForwardString "MOTOR_FORWARD"   /* asynInt32,    r/w */
#define P_MotorBackwardString "MOTOR_BACKWARD" /* asynInt32,    r/w */
#define P_MotorPositionString "MOTOR_POS"      /* asynFloat64,  r/o */
#define P_UpdateTimeString "SIM_UPDATE_TIME"   /* asynFloat64,  r/w (addr 0) */

/** Class that demonstrates the use of the asynPortDriver base class to greatly
 * simplify the task of writing an asyn port driver. This class does a simple
 * simulation of motors being driven in and out. I have made the methods of
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
  int P_MotorRun;
  int P_MotorForward;
  int P_MotorBackward;
  int P_MotorPosition;
  int P_UpdateTime;

private:
  constexpr static const double voltage_reference = 5.0;
  /* Our data */
  epicsEventId eventId_;
  // Functions that actually call the ETH32 library to turn on/off a digital pin
  void setMotorForward(int motor);  // Port motor/4, bit 2*(motor%4)
  void setMotorBackward(int motor); // Port motor/4, bit 2*(motor%4)+1
  void stopMotor(int motor);        // Both directions off
  void setMotorPin(int port, int bit, epicsInt32 setting); // Turn on and off an arbitrary pin
  void getMotorPos(int motor); // Read the motor's analog channel into MOTOR_POS
};
