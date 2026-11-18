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

// #define HAVE_ETH32

#ifdef HAVE_ETH32
// Some parameters needed for connection with eth32
char hostname[] = "192.168.2.100";
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
 * Each motor is one asyn address, 0 to NUM_MOTORS-1. */
ETH32AsynPortDriver::ETH32AsynPortDriver(const char *portName)
    : asynPortDriver(portName, NUM_MOTORS, /* maxAddr */
                     asynInt32Mask | asynFloat64Mask | asynEnumMask |
                         asynDrvUserMask, /* Interface mask */
                     asynInt32Mask | asynFloat64Mask |
                         asynEnumMask, /* Interrupt mask */
                     ASYN_MULTIDEVICE, /* asynFlags. One address per motor;
                                          this driver does not block */
                     1, /* Autoconnect */
                     0, /* Default priority */
                     0) /* Default stack size*/
{
  asynStatus status;
  const char *functionName = "ETH32AsynPortDriver";

  eventId_ = epicsEventCreate(epicsEventEmpty);
  createParam(P_MotorRunString, asynParamInt32, &P_MotorRun);
  createParam(P_MotorForwardString, asynParamInt32, &P_MotorForward);
  createParam(P_MotorBackwardString, asynParamInt32, &P_MotorBackward);
  createParam(P_MotorPositionString, asynParamFloat64, &P_MotorPosition);
  createParam(P_UpdateTimeString, asynParamFloat64, &P_UpdateTime);

  /* Set the initial values of some parameters */
  for (int motor = 0; motor < NUM_MOTORS; motor++) {
    setIntegerParam(motor, P_MotorRun, 0);
    setIntegerParam(motor, P_MotorForward, 0);
    setIntegerParam(motor, P_MotorBackward, 0);
    setDoubleParam(motor, P_MotorPosition, 0.0);
  }
  setDoubleParam(0, P_UpdateTime, 0.5);

#ifdef HAVE_ETH32
  // Connect with the ETH32 before the polling thread starts using it
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

  // Setup the analog input channels
  eth32result = eth32_set_analog_state(handle, ADC_ENABLED);
  // Configure logical channel n to read physical channel n relative to
  // ground (single-ended), one channel per motor. This is the power-on
  // default anyway, but is shown here for completeness:
  for (int motor = 0; motor < NUM_MOTORS; motor++)
    eth32_set_analog_assignment(handle, motor, ANALOG_SE0 + motor);
  // Configure the analog voltage reference to be the internal 5V source
  eth32_set_analog_reference(handle, REF_INTERNAL);

  // Set every pin on the motor ports to be an output (8 pins per port)
  for (int port = 0; port < (NUM_MOTORS + MOTORS_PER_PORT - 1) / MOTORS_PER_PORT;
       port++)
    eth32result = eth32_set_direction(handle, port, DIR_OUTPUT);
#endif

  /* Create the thread that simulates (or reads) the motor positions */
  status =
      (asynStatus)(epicsThreadCreate(
                       "ETH32AsynPortDriverTask", epicsThreadPriorityMedium,
                       epicsThreadGetStackSize(epicsThreadStackMedium),
                       (EPICSTHREADFUNC)::simTask, this) == NULL);
  if (status) {
    printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
    return;
  }
}

// Start the simulation
void simTask(void *drvPvt) {
  ETH32AsynPortDriver *pPvt = (ETH32AsynPortDriver *)drvPvt;

  pPvt->simTask();
}

// Task that runs as a separate thread. With the ETH32 it reads every
// motor's position each update. In simulation, each motor whose run
// parameter is set moves in its selected direction at dVdt volts per second.
void ETH32AsynPortDriver::simTask(void) {
  double updateTime, Voltage;
  double maxV = 10.0, minV = 0.0;
  epicsInt32 MotorForward, MotorBackward, DirectionSign;
  epicsInt32 run;
  bool wasRunning[NUM_MOTORS];
  bool anyRunning;
  double dVdt = 0.1; // volts per second
  epicsTimeStamp lastTick, now;

  lock();
  epicsTimeGetCurrent(&lastTick);
  /* Loop forever */
  while (1) {
    getDoubleParam(0, P_UpdateTime, &updateTime);
    // Note which motors are running before we wait, so a motor that is
    // started while we wait does not get credited with the time it was
    // stopped
    anyRunning = false;
    for (int motor = 0; motor < NUM_MOTORS; motor++) {
      getIntegerParam(motor, P_MotorRun, &run);
      wasRunning[motor] = run;
      anyRunning = anyRunning || run;
    }

    // Release the lock while we wait for a command to start or wait for
    // updateTime
    unlock();
#ifdef HAVE_ETH32
    // Always poll the hardware so the positions stay current
    epicsEventWaitWithTimeout(eventId_, updateTime);
#else
    if (anyRunning)
      epicsEventWaitWithTimeout(eventId_, updateTime);
    else
      (void)epicsEventWait(eventId_);
#endif
    // Take the lock again
    lock();

    // Move by the time that has actually passed, so an early wake-up
    // (e.g. another motor being started) does not speed a motor up
    epicsTimeGetCurrent(&now);
    double dt = epicsTimeDiffInSeconds(&now, &lastTick);
    lastTick = now;
    updateTimeStamp();

    for (int motor = 0; motor < NUM_MOTORS; motor++) {
#ifdef HAVE_ETH32
      getMotorPos(motor);
#else
      /* run could have changed while we were waiting */
      getIntegerParam(motor, P_MotorRun, &run);
      if (!run || !wasRunning[motor])
        continue;

      // Read the direction and position only now, after the wait, so a
      // direction set (or cleared) while we were waiting is not missed.
      getIntegerParam(motor, P_MotorForward, &MotorForward);
      getIntegerParam(motor, P_MotorBackward, &MotorBackward);
      getDoubleParam(motor, P_MotorPosition, &Voltage);

      // Get direction of motor. Default is no movement so simulation
      // can "run" with no movement
      DirectionSign = 0;
      if (MotorForward == 1)
        DirectionSign = 1;
      else if (MotorBackward == 1)
        DirectionSign = -1;
      if (DirectionSign == 0)
        continue;

      Voltage += DirectionSign * dVdt * dt;

      // Check for bounds
      if (Voltage > maxV)
        Voltage = maxV;
      if (Voltage < minV)
        Voltage = minV;

      setDoubleParam(motor, P_MotorPosition, Voltage);
#endif
      callParamCallbacks(motor);
    }
  }
}

/** Called when asyn clients call pasynInt32->write().
 * This function sends a signal to the simTask thread if a motor's run
 * parameter was set. For all parameters it sets the value in the parameter
 * library and calls any registered callbacks.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus ETH32AsynPortDriver::writeInt32(asynUser *pasynUser,
                                           epicsInt32 value) {
  int function = pasynUser->reason;
  int motor;
  asynStatus status = asynSuccess;
  const char *paramName;
  const char *functionName = "writeInt32";

  /* Which motor (asyn address) is this for? */
  status = getAddress(pasynUser, &motor);
  if (status != asynSuccess)
    return status;

  /* Set the parameter in the parameter library. */
  status = (asynStatus)setIntegerParam(motor, function, value);

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  if (function == P_MotorRun) {
    if (value) {
      /* If run was set then wake up the simulation task */
      epicsEventSignal(eventId_);
    } else {
      /* Turning a motor off also stops it */
      stopMotor(motor);
    }
  } else if (function == P_MotorForward) {
    setMotorForward(motor);
  } else if (function == P_MotorBackward) {
    setMotorBackward(motor);
  } else {
      /* All other parameters just get set in parameter list, no need to
       * act on them here */
  }

  /* Do callbacks so higher layers see any changes */
  status = (asynStatus)callParamCallbacks(motor);

  if (status)
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d, name=%s, motor=%d, value=%d",
                  driverName, functionName, status, function, paramName,
                  motor + 1, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, motor=%d, value=%d\n", driverName,
              functionName, function, paramName, motor + 1, value);
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
  int addr;
  asynStatus status = asynSuccess;
  const char *paramName;
  const char *functionName = "writeFloat64";

  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess)
    return status;

  /* Set the parameter in the parameter library. */
  status = (asynStatus)setDoubleParam(addr, function, value);

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
      setDoubleParam(addr, P_UpdateTime, value);
    }
    /* Wake up the task so it uses the new update time */
    epicsEventSignal(eventId_);
  } else {
    /* All other parameters just get set in parameter list, no need to
     * act on them here */
  }

  /* Do callbacks so higher layers see any changes */
  status = (asynStatus)callParamCallbacks(addr);

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

// Generic function to switch on a motor pin
void ETH32AsynPortDriver::setMotorPin(int port, int bit, epicsInt32 setting) {

#ifdef HAVE_ETH32
  int led, bit_corresponding, result;

  // based on the port and bit, we'll check the corresponding neighbor
  // and make sure it's off before turning a pin on
  //
  // Forward pins are ALWAYS even
  // Backward pins are ALWAYS odd
  if (bit % 2 == 0) {
    bit_corresponding = bit + 1;
  } else {
    bit_corresponding = bit - 1;
  }

  // Turn off corresponding bit in all circumstances
  result = eth32_output_bit(handle, port, bit_corresponding, 0);
  if (result) {
    printf("Some kind of error happened when writing: %d to port %d, bit %d\n returned %d",
           0, port, bit_corresponding,result);
  }
  // Toggle the LED to indicate which direction a motor is moving
  led = !(bit % 2); //std::max(0,bit_corresponding - bit);
  eth32_set_led(handle, led, 0);

  // Now do whatever you intended with the bit you wanted
  result = eth32_output_bit(handle, port, bit, setting);
  if (result) {
    printf("Some kind of error happened when writing: %d to port %d, bit %d\n returned %d",
           setting, port, bit, result);
  }
  // Toggle the LED to indicate which direction a motor is moving
  led = bit % 2;
  eth32_set_led(handle, led, setting);
#endif
}


// Turn on/off a motor's forward pin: port motor/4, even bit 2*(motor%4)
void ETH32AsynPortDriver::setMotorForward(int motor) {

  epicsInt32 setting;

  // Get the setting
  getIntegerParam(motor, P_MotorForward, &setting);
  setMotorPin(motor / MOTORS_PER_PORT, 2 * (motor % MOTORS_PER_PORT), setting);
  setIntegerParam(motor, P_MotorBackward, 0); // Turn off the other direction
}

// Turn on/off a motor's backward pin: port motor/4, odd bit 2*(motor%4)+1
void ETH32AsynPortDriver::setMotorBackward(int motor) {

  epicsInt32 setting;

  // Get the setting
  getIntegerParam(motor, P_MotorBackward, &setting);
  setMotorPin(motor / MOTORS_PER_PORT, 2 * (motor % MOTORS_PER_PORT) + 1,
              setting);
  setIntegerParam(motor, P_MotorForward, 0); // Turn off the other direction
}

// Turn off both of a motor's direction pins
void ETH32AsynPortDriver::stopMotor(int motor) {
  // Writing 0 to the forward pin also turns off the backward pin
  setIntegerParam(motor, P_MotorForward, 0);
  setMotorForward(motor);
}

// Get the current position of a motor from its analog channel
void ETH32AsynPortDriver::getMotorPos(int motor) {

  int value = 0;

#ifdef HAVE_ETH32
  // Get the voltage on the motor's analog input
  int result = eth32_input_analog(handle, motor, &value);
  if (result) {
    printf("Some kind of error happened when reading analog channel %d\n",
           motor);
  }
#endif

  double Voltage = voltage_reference*(double)value/1024.0;
  setDoubleParam(motor, P_MotorPosition, Voltage);
}

/*----------------------------------------------------------------------
  Standard AsynPortDriver stuff below here
  ----------------------------------------------------------------------*/

/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C" {

  /** EPICS iocsh callable function to call constructor for the
   * ETH32AsynPortDriver class.
   * \param[in] portName The name of the asyn port driver to be created.
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
