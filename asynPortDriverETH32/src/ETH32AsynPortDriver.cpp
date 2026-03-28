/*
 * ETH32AsynPortDriver.cpp
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

#include <errno.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <epicsTypes.h>
#include <iocsh.h>

#include "ETH32AsynPortDriver.h"
#include <epicsExport.h>
#include <unistd.h>

#include "eth32.h"

#define MIN_UPDATE_TIME                                                        \
  0.1 /* Minimum update time, to prevent CPU saturation                        \
       */

#define MAX_ENUM_STRING_SIZE 20

#define HAVE_ETH32

#ifdef HAVE_ETH32
// Some parameters needed for connection with eth32
char hostname[] = "host";
// char *result;
int eth32result;
// eth32_handler event_handler_config={0}; // Initialize contents to all
// zeroes
eth32 handle;
#endif

static const char *driverName = "ETH32AsynPortDriver";
void simTask(void *drvPvt);

/** Constructor for the ETH32AsynPortDriver class.
 * Calls constructor for the asynPortDriver base class.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] maxPoints The maximum  number of points in the volt and time
 * arrays */
ETH32AsynPortDriver::ETH32AsynPortDriver(const char *portName)
    : asynPortDriver(portName, 1, /* maxAddr */
                     asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask |
                         asynEnumMask | asynDrvUserMask, /* Interface mask */
                     asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask |
                         asynEnumMask, /* Interrupt mask */
                     0, /* asynFlags.  This driver does not block and it is not
                           multi-device, so flag is 0 */
                     1, /* Autoconnect */
                     0, /* Default priority */
                     0) /* Default stack size*/
{
  asynStatus status;
  const char *functionName = "ETH32AsynPortDriver";

  eventId_ = epicsEventCreate(epicsEventEmpty);
  createParam(P_RunString, asynParamInt32, &P_Run);
  createParam(P_Motor1ForwardString, asynParamInt32, &P_Motor1Forward);
  createParam(P_Motor1BackwardString, asynParamInt32, &P_Motor1Backward);
  createParam(P_Motor1PositionString, asynParamFloat64, &P_Motor1Position);
  createParam(P_UpdateTimeString, asynParamFloat64, &P_UpdateTime);

  /* Set the initial values of some parameters */
  setIntegerParam(P_Run, 0);
  setIntegerParam(P_Motor1Forward, 0);
  setIntegerParam(P_Motor1Backward, 0);
  setDoubleParam(P_Motor1Position, 0.0);
  setDoubleParam(P_UpdateTime, 0.5);

  /* Create the thread that computes a random voltage in the background */
  status =
      (asynStatus)(epicsThreadCreate(
                       "ETH32AsynPortDriverTask", epicsThreadPriorityMedium,
                       epicsThreadGetStackSize(epicsThreadStackMedium),
                       (EPICSTHREADFUNC)::simTask, this) == NULL);
  if (status) {
    printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
    return;
  }

#ifdef HAVE_ETH32
  // Finally, connect with the ETH32
  printf("\n\nHi! I'm about to open the eth32\n");
  handle = eth32_open(hostname, ETH32_PORT, 5000, &eth32result);
  if (handle == 0) {
    printf("Error connecting to ETH32: %s\n", eth32_error_string(eth32result));
    // return;
  }

  // Try turning on both LEDs just to make sure it's working
  eth32_set_led(handle, 0, 1);
  eth32_set_led(handle, 1, 1);
  eth32_get_led(handle, 0, &eth32result);
  printf("LED 0 is set to %d; ", eth32result);
  eth32_get_led(handle, 0, &eth32result);
  printf("LED 1 is set to %d\n", eth32result);
  // Sleep, then try turning off both LEDs
  sleep(10);
  eth32_set_led(handle, 0, 0);
  eth32_set_led(handle, 1, 0);
  eth32_get_led(handle, 0, &eth32result);
  printf("LED 0 is set to %d; ", eth32result);
  eth32_get_led(handle, 0, &eth32result);
  printf("LED 1 is set to %d\n", eth32result);

  // Set Port A to be an output port
  eth32result = eth32_set_direction(handle, ETH32_PORT, 0);

#endif
}

// Start the simulation
void simTask(void *drvPvt) {
  ETH32AsynPortDriver *pPvt = (ETH32AsynPortDriver *)drvPvt;

  pPvt->simTask();
}

// Simulation task that runs as a separate thread. When the P_Run
// parameter is set to 1, it calculates a slow triangle wave

void ETH32AsynPortDriver::simTask(void) {
  /* This thread computes the voltage as if the motor is driving in or out */
  double updateTime, Voltage = 0.0;
  double maxV = 10.0, minV = 0.0;
  epicsInt32 MotorForward, MotorBackward, DirectionSign;
  epicsInt32 run;
  double dVdt = 0.1; // volts per second

  lock();
  /* Loop forever */
  while (1) {
    getIntegerParam(P_Run, &run);
    getIntegerParam(P_Motor1Forward, &MotorForward);
    getIntegerParam(P_Motor1Backward, &MotorBackward);
    getDoubleParam(P_UpdateTime, &updateTime);
    // Get the current position
    getDoubleParam(P_Motor1Position, &Voltage);
    // std::cout << run << " " << MotorForward << " " << MotorBackward << " "
    //           << Voltage << std::endl;

    // Release the lock while we wait for a command to start or wait for
    // updateTime
    unlock();
    if (run)
      epicsEventWaitWithTimeout(eventId_, updateTime);
    else
      (void)epicsEventWait(eventId_);
    // Take the lock again
    lock();

    /* run could have changed while we were waiting */
    getIntegerParam(P_Run, &run);
    if (!run)
      continue;

    // Get direction of motor. Default is no movement so simulation
    // can "run" with no movement
    DirectionSign = 0;
    if (MotorForward == 1)
      DirectionSign = 1;
    else if (MotorBackward == 1)
      DirectionSign = -1;

    // Increment the voltage on analog pin 0
    Voltage += DirectionSign * dVdt * updateTime;
    // std::cout << DirectionSign << " " << dVdt << " " << updateTime << " => "
    //           << Voltage << std::endl;

    // Check for bounds
    if (Voltage > maxV)
      Voltage = maxV;
    if (Voltage < minV)
      Voltage = minV;

    updateTimeStamp();
    setDoubleParam(P_Motor1Position, Voltage);
    callParamCallbacks();
  }
}

/** Called when asyn clients call pasynInt32->write().
 * This function sends a signal to the simTask thread if the value of P_Run has
 * changed. For all parameters it sets the value in the parameter library and
 * calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus ETH32AsynPortDriver::writeInt32(asynUser *pasynUser,
                                           epicsInt32 value) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char *paramName;
  const char *functionName = "writeInt32";

  /* Set the parameter in the parameter library. */
  status = (asynStatus)setIntegerParam(function, value);

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  if (function == P_Run) {
    /* If run was set then wake up the simulation task */
    if (value)
      epicsEventSignal(eventId_);
    else if (function == P_Motor1Forward) {
      setMotor1Forward();
    } else {
      /* All other parameters just get set in parameter list, no need to
       * act on them here */
    }
  }

  /* Do callbacks so higher layers see any changes */
  status = (asynStatus)callParamCallbacks();

  if (status)
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d, name=%s, value=%d",
                  driverName, functionName, status, function, paramName, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, value=%d\n", driverName,
              functionName, function, paramName, value);
  return status;
}

/** Called when asyn clients call pasynFloat64->write().
 * This function sends a signal to the simTask thread if the value of
 * P_UpdateTime has changed. For all  parameters it  sets the value in the
 * parameter library and calls any registered callbacks.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus ETH32AsynPortDriver::writeFloat64(asynUser *pasynUser,
                                             epicsFloat64 value) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  epicsInt32 run;
  const char *paramName;
  const char *functionName = "writeFloat64";

  /* Set the parameter in the parameter library. */
  status = (asynStatus)setDoubleParam(function, value);

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  if (function == P_UpdateTime) {
    /* Make sure the update time is valid. If not change it and put back in
     * parameter library */
    if (value < MIN_UPDATE_TIME) {
      asynPrint(
          pasynUser, ASYN_TRACE_WARNING,
          "%s:%s: warning, update time too small, changed from %f to %f\n",
          driverName, functionName, value, MIN_UPDATE_TIME);
      value = MIN_UPDATE_TIME;
      setDoubleParam(P_UpdateTime, value);
    }
    /* If the update time has changed and we are running then wake up the
     * simulation task */
    getIntegerParam(P_Run, &run);
    if (run)
      epicsEventSignal(eventId_);
  } else {
    /* All other parameters just get set in parameter list, no need to
     * act on them here */
  }

  /* Do callbacks so higher layers see any changes */
  status = (asynStatus)callParamCallbacks();

  if (status)
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d, name=%s, value=%f",
                  driverName, functionName, status, function, paramName, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, value=%f\n", driverName,
              functionName, function, paramName, value);
  return status;
}

// Turn on/off digital channel 0
void ETH32AsynPortDriver::setMotor1Forward() {

  epicsInt32 setting;

  // Get the setting
  getIntegerParam(P_Motor1Forward, &setting);

#ifdef HAVE_ETH32
  // Call the ETH32 library to actually turn on or off the pin
  int result = eth32_output_byte(handle, ETH32_PORT, 85);
  if (result) {
    printf("Some kind of error happened when writing\n");
  }
  // Print the output
  // asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
  //           "%s:%s: function=%d, name=%s, value=%f\n", driverName,
  //           functionName, function, paramName, value);
#endif
}

/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C" {

/** EPICS iocsh callable function to call constructor for the
 * ETH32AsynPortDriver class.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] maxPoints The maximum  number of points in the volt and time
 * arrays */
int ETH32AsynPortDriverConfigure(const char *portName) {
  new ETH32AsynPortDriver(portName);
  return (asynSuccess);
}

/* EPICS iocsh shell commands */

static const iocshArg initArg0 = {"portName", iocshArgString};
static const iocshArg *const initArgs[] = {&initArg0};
static const iocshFuncDef initFuncDef = {"ETH32AsynPortDriverConfigure", 1,
                                         initArgs};
static void initCallFunc(const iocshArgBuf *args) {
  ETH32AsynPortDriverConfigure(args[0].sval);
}

void ETH32AsynPortDriverRegister(void) {
  iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(ETH32AsynPortDriverRegister);
}
