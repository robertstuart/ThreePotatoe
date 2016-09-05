
//const int BATTERY_WARNING = 1090;  // about 10% capacity (centivolts)
//const int BATTERY_CRITICAL = 1000; // about 1% cap (centivolts)
const int BATTERY_WARNING = 726;  // about 10% capacity (centivolts)
const int BATTERY_CRITICAL = 666; // about 1% cap (centivolts)

int beepCycleCount = 0;
boolean beepStat = false;
int *beepSequence;
int beepPtr = 0;

#define SONAR_BUF_SIZE 6
int sonarBufRight[SONAR_BUF_SIZE];
int sonarBufPtrRight = 0;
int sonarBufLeft[SONAR_BUF_SIZE];
int sonarBufPtrLeft = 0;

boolean flip = false;
int warningCount = 0;
int criticalCount = 0;
int addFlip = 0;
boolean redLedState = false;
boolean greenLedState = false;
unsigned long taskMilliseconds = 0L;
unsigned long gravityTrigger = 0L;
//unsigned long audioTrigger = 0L;
unsigned long errorTrigger = 0L;
unsigned int taskPtr = 0;
unsigned int pingTpHCCount = 0;
unsigned long onGroundTime = 0L;
unsigned long warningTrigger = 0;
unsigned long batteryLastGood = 0;


/**************************************************************************.
 *
 * commonTasks()
 *
 *    Execute all tasks common to every algorithm.  This includes:
 *      1. Reading the XBee for commands.
 *      2. Flushing the serial output buffer
 *      3. Setting the time variables.
 *      4. Dumping any data if we are in "dump data" mode.
 *      5. Turning the power off if the motors have been idle.
 *      6. Flash the led.
 *
 **************************************************************************/
void commonTasks() {
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  readXBee();  // Read commands from PC or Hand Controller
  readBluetooth();
  readSonar();
  switches();
  blink();
  battery();
  beacon();
  //  hoverPower();
  controllerConnected();
}



/*********************************************************
 *
 * safeAngle()
 *
 *     Check to see if we have fallen sidways or forwards.
 *     If so, unset the STATE_UPRIGHT bit.
 *     Otherwise, set the bit.
 *
 *********************************************************/
void safeAngle() {
  static unsigned long tTime = 0UL; // time of last state change
  static boolean tState = false;  // Timed state. true = upright

  boolean cState = ((abs(gaFullPitch) < 45.0) && ((abs(gaRoll) < 45))); // Current real state
  if (cState != tState) {
    tTime = timeMilliseconds; // Start the timer for a state change.
    tState = cState;
  }
  else {
    if ((timeMilliseconds - tTime) > 50) {
      isUpright = cState;
    }
  }
}  // End safeAngle().


/**************************************************************************.
 *
 * hoverPower()
 *              Disconnect and reconnect interrupts whenever
 *              the hoverboard is powered up or down.
 *
 **************************************************************************/
void hoverPower(boolean isOn) {
  //  int pwr = analogRead(HOVER_POWER_PIN); // 948 indicates power is on
  if (isOn) {
    attachInterrupt(MOT_RIGHT_ENCA,  encoderIsrRightA, CHANGE);
    attachInterrupt(MOT_RIGHT_ENCB, encoderIsrRightB, CHANGE);
    attachInterrupt(MOT_RIGHT_ENCC, encoderIsrRightC, CHANGE);
    attachInterrupt(MOT_LEFT_ENCA, encoderIsrLeftA, CHANGE);
    attachInterrupt(MOT_LEFT_ENCB, encoderIsrLeftB, CHANGE);
    attachInterrupt(MOT_LEFT_ENCC, encoderIsrLeftC, CHANGE);
  } else {
    detachInterrupt(MOT_RIGHT_ENCA);
    detachInterrupt(MOT_RIGHT_ENCB);
    detachInterrupt(MOT_RIGHT_ENCC);
    detachInterrupt(MOT_LEFT_ENCA);
    detachInterrupt(MOT_LEFT_ENCB);
    detachInterrupt(MOT_LEFT_ENCC);
  }
}


/**************************************************************************.
 *
 * controllerConnected()
 *
 *********************************************************/
void controllerConnected() {
  isHcActive =  ((tHc + 1000) > timeMilliseconds) ? true : false;
  isPcActive =  ((tPc + 1000) > timeMilliseconds) ? true : false;
}



/**************************************************************************.
 * battery()
 ***************************************************************/
void battery() {
  static unsigned long batteryTrigger = 0L;
  if (timeMilliseconds > batteryTrigger) {
    batteryTrigger = timeMilliseconds + 1000;  // 1 per second
    battAVolt = ((float) analogRead(BATT_A_PIN)) * 0.00922;
    battBVolt = ((float) analogRead(BATT_B_PIN)) * 0.0513;
//Serial.print(battAVolt); Serial.print("\t"); Serial.println(battBVolt);
  }
}



/**************************************************************************.
 * beacon()
 *      Send TwoPotatoe an XY beacon.  If the hc is active, send it 30ms
 *      after the hc message.  If it is not active, send the beacon every
 *      100ms.
 *      
 *      Also, manage the state of the ??Active variables.
 **************************************************************************/
void beacon() {
  static unsigned int beaconTime = 0;
  if ((hcMsgTime + 110) < timeMilliseconds) isHcActive = false;   
  if ((pcMsgTime + 110) < timeMilliseconds) isPcActive = false;   

  if (isHcActive) {
    if((!isBeaconTransmitted) && ((hcMsgTime + 30) > timeMilliseconds)) {
      isBeaconTransmitted = true;
      sendBeacon();
    }
  } else {
    if ((beaconTime + 100) < timeMilliseconds) {
      beaconTime = timeMilliseconds;
      sendBeacon();
    }
  }
}



/**************************************************************************.
 * switches()
 *      Toggle TP_STATE_RUN_READY on yellow switch.  1 sec dead period.
 **************************************************************************/
void switches() {
  static int gnTimer = 0;
  static boolean gnState = false;
  static boolean oldGnState = false;
  static int gnHoldTimer = 0;

  static int rTimer = 0;
  static boolean rState = false;
  static boolean oldRState = false;

  static int lTimer = 0;
  static boolean lState = false;
  static boolean oldLState = false;

  // Debounce green
  boolean gn = digitalRead(SW_GN_PIN) == LOW;
  if (gn) gnTimer = timeMilliseconds;
  if ((timeMilliseconds - gnTimer) > 50) gnState = false;
  else gnState = true;

  // Debounce right
  boolean r = digitalRead(SW_R_PIN) == LOW;
  if (r) rTimer = timeMilliseconds;
  if ((timeMilliseconds - rTimer) > 50) rState = false;
  else rState = true;

  // Debounce left
  boolean (l) = digitalRead(SW_L_PIN) == LOW;
  if (l) lTimer = timeMilliseconds;
  if ((timeMilliseconds - lTimer) > 50) lState = false;
  else lState = true;
  
  // Green - all actions on release
  if ((gnState) && (!oldGnState))   gnHoldTimer = timeMilliseconds; // Press green
  if ((!gnState) && (oldGnState)) {                                 // Release greeen
    if (!isRouteInProgress && ((timeMilliseconds - gnHoldTimer) > 1000)) {
      startRoute();
    } else if (isRouteInProgress) {
      stopRoute(); 
    } else {
      setHeading(0.0D);
      run(!isRunReady);
    }
  }

  // Turn right or left if runready, increment route if not
  if (isRunReady) {
    isRPressed = rState;
    isLPressed = lState;
  } else {
    isRPressed = isLPressed = false;
    if (rState && !oldRState) setRoute(true);
  }

  oldGnState = gnState;
  oldRState = rState;
  oldLState = lState;
}



/**************************************************************************.
 * readSonar() 
 **************************************************************************/
void readSonar() {
  const int S_BUFFER_SIZE = 20;
  static char msgStr[S_BUFFER_SIZE + 1];
  static boolean isMsgInProgress = false;
  static int msgPtr = 0;
  static boolean isRight = false;
  
  float distance = 0.0;
 
  while (SONAR_SER.available()) {
    byte b = SONAR_SER.read();
    if (isMsgInProgress) {
      if (b == 13) {
        msgStr[msgPtr] = 0;
        int e = sscanf(msgStr, "%f", &distance);
        if ((e > 0) && (distance < 30.0)) {
          doSonar(distance, isRight);
        }
        isMsgInProgress = false;
      }
      else if (msgPtr >= S_BUFFER_SIZE) {
        isMsgInProgress = false;
      } else {
        msgStr[msgPtr++] = b;
      }
    } else {
      if (b == 'R') isRight = true;
      else if (b == 'L') isRight = false;
      else continue; 
      msgPtr = 0;
      isMsgInProgress = true;         
    }
  }
}



/**************************************************************************.
 * doSonar() Process received distance from sonar.
 *           Called for every sonar reading.
 **************************************************************************/
void doSonar(float distance, int isRight) {
  float d = distance - 0.32;
    
  // Collect data for Charted Object measurements. 
  if (isRight) {     
    sonarRightArray[sonarRightArrayPtr] = (distance * 100.0);
    sonarRightArrayPtr = ++sonarRightArrayPtr % SONAR_ARRAY_SIZE;
    sonarRight = distance;
  } else {
    sonarLeftArray[sonarLeftArrayPtr] = (int) (distance * 100.0);
    sonarLeftArrayPtr = ++sonarLeftArrayPtr % SONAR_ARRAY_SIZE;
    sonarLeft = distance;
  }
  if (isRouteInProgress) {
    addLog(
        (long) timeMilliseconds,
        (short) (currentMapLoc.x * 100.0),
        (short) (currentMapLoc.y * 100.0),
        (short) (sonarRight * 100.0),
        (short) (sonarLeft * 100.0),
        (short) (0),
        (short) (routeStepPtr)
    );
  }
}



/**************************************************************************.
 *  sonarMode() Set sonars on/off
 **************************************************************************/
void setSonar(int mode) {
  int b = 3;
  if      (mode == SONAR_NONE)  b = 0;
  else if (mode == SONAR_LEFT)  b = 1;
  else if (mode == SONAR_RIGHT) b = 2;
  else if (mode == SONAR_BOTH)  b = 3;
  SONAR_SER.print(b);
}



/**************************************************************************.
 *  rangeAngle() Set angle value between -180 and +180
 **************************************************************************/
double rangeAngle(double head) {
  while (head > 180.0D) head -= 360.0D;
  while (head <= -180.0D) head += 360.0D;
  return head;
}

/**************************************************************************.
 *  blink()
 **************************************************************************/
void blink() {
  static int routeCycle = 0; //
  static int routeOffCount = 0;
  static unsigned int blinkTrigger = 0;
  static boolean toggle = false;
  if (timeMilliseconds > blinkTrigger) {
    blinkTrigger = timeMilliseconds + 100;  // 10 per second
    toggle = !toggle;
   
    if(isRouteInProgress) {
      digitalWrite(LED_GN_PIN, toggle);
      digitalWrite(LED_PIN, toggle);
    } else {
      // Blink route number
      if (++routeOffCount >=5) {
        routeOffCount = 0;
        if (routeCycle <= routeTablePtr) {
          digitalWrite(LED_GN_PIN, HIGH);
          digitalWrite(LED_PIN, HIGH);
        }
        routeCycle++;
        if (routeCycle >= (routeTablePtr + 3)) {
          routeCycle = 0;
        }
      } else if (routeOffCount == 2) {
        digitalWrite(LED_GN_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
      }
    }
  }  
}

void run(boolean b) {
  isRunReady = b;
  motors(b);
}


/**************************************************************************.
 *  motors() Turn the motors on and off by turning the
 *           LEDs on and off in the foot switch.
 **************************************************************************/
void motors(boolean b) {
  if (b) {
    digitalWrite(IR_R_PIN, HIGH);
    digitalWrite(IR_L_PIN, HIGH);
  } else {
    digitalWrite(IR_R_PIN, LOW);
    digitalWrite(IR_L_PIN, LOW);
  }
}




/**************************************************************************.
 *  addLog() Put values in the dump arrays.
 **************************************************************************/
void addLog(long aVal, short bVal, short cVal, short dVal, short eVal, short fVal, short gVal) {
  if (!isDumpingData) {
    if (aVal == 0L) aVal = 1L; // don't indicate end
    aArray[dataArrayPtr] = aVal;
    bArray[dataArrayPtr] = bVal;
    cArray[dataArrayPtr] = cVal;
    dArray[dataArrayPtr] = dVal;
    eArray[dataArrayPtr] = eVal;
    fArray[dataArrayPtr] = fVal;
    gArray[dataArrayPtr] = gVal;
    dataArrayPtr++;
    dataArrayPtr = dataArrayPtr %  DATA_ARRAY_SIZE;
  }
}

void encoderIsrRightA() {
  static unsigned int lastTime = 0;
  int dir;
  unsigned int t = micros();
  unsigned int ti = t - lastTime;
  if (ti < 100) return;
  lastTime = t;
  boolean encA = (!!(g_APinDescription[MOT_RIGHT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_RIGHT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCB].ulPin)) ? true : false;
  boolean encC = (!!(g_APinDescription[MOT_RIGHT_ENCC].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCC].ulPin)) ? true : false;

  if      ( encA && !encB &&  encC) dir = BKWD;
  else if (!encA &&  encB && !encC) dir = BKWD;
  else if ( encA &&  encB && !encC) dir = FWD;
  else if (!encA && !encB &&  encC) dir = FWD;
  else dir = DIR_ERROR;

  noInterrupts();
  encoderRightDir = dir;
  encoderRightPeriod = t - encoderRightTime;
  encoderRightTime = t;
  interrupts();

  if (dir == FWD) tickPositionRight++;
  else if (dir == BKWD) tickPositionRight--;
}

void encoderIsrRightB() {
  static unsigned int lastTime = 0;
  int dir;
  unsigned int t = micros();
  unsigned int ti = t - lastTime;
  if (ti < 100) return;
  lastTime = t;
  boolean encA = (!!(g_APinDescription[MOT_RIGHT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_RIGHT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCB].ulPin)) ? true : false;
  boolean encC = (!!(g_APinDescription[MOT_RIGHT_ENCC].pPort ->
                     PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCC].ulPin)) ? true : false;

  if (      encA &&  encB && !encC) dir = BKWD;
  else if (!encA && !encB &&  encC) dir = BKWD;
  else if (!encA &&  encB &&  encC) dir = FWD;
  else if ( encA && !encB && !encC) dir = FWD;
  else dir = DIR_ERROR;

  noInterrupts();
  encoderRightDir = dir;
  encoderRightPeriod = t - encoderRightTime;
  encoderRightTime = t;
  interrupts();

  if (dir == FWD) tickPositionRight++;
  else if (dir == BKWD) tickPositionRight--;
}


void encoderIsrRightC() {
  static unsigned int lastTime = 0;
  int dir;
  unsigned int t = micros();
  unsigned int ti = t - lastTime;
  if (ti < 100) return;
  lastTime = t;
  boolean encA = (!!(g_APinDescription[MOT_RIGHT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_RIGHT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCB].ulPin)) ? true : false;
  boolean encC = (!!(g_APinDescription[MOT_RIGHT_ENCC].pPort -> PIO_PDSR & g_APinDescription[MOT_RIGHT_ENCC].ulPin)) ? true : false;

  if (     !encA &&   encB &&  encC) dir = BKWD;
  else if ( encA &&  !encB && !encC) dir = BKWD;
  else if ( encA &&  !encB &&  encC) dir = FWD;
  else if (!encA &&   encB && !encC) dir = FWD;
  else  dir = DIR_ERROR;

  noInterrupts();
  encoderRightDir = dir;
  encoderRightPeriod = t - encoderRightTime;;
  encoderRightTime = t;
  interrupts();

  if (dir == FWD) tickPositionRight++;
  else if (dir == BKWD) tickPositionRight--;
}

void encoderIsrLeftA() {
  static unsigned int lastTime = 0;
  int dir;
  unsigned int t = micros();
  unsigned int ti = t - lastTime;
  if (ti < 100) return;
  lastTime = t;
  boolean encA = (!!(g_APinDescription[MOT_LEFT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_LEFT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCB].ulPin)) ? true : false;
  boolean encC = (!!(g_APinDescription[MOT_LEFT_ENCC].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCC].ulPin)) ? true : false;

  if      ( encA && !encB &&  encC) dir = FWD;
  else if (!encA &&  encB && !encC) dir = FWD;
  else if ( encA &&  encB && !encC) dir = BKWD;
  else if (!encA && !encB &&  encC) dir = BKWD;
  else dir = DIR_ERROR;

  noInterrupts();
  encoderLeftDir = dir;
  encoderLeftPeriod = t - encoderLeftTime;
  encoderLeftTime = t;
  interrupts();

  if (dir == FWD) tickPositionLeft++;
  else if (dir == BKWD) tickPositionLeft--;
}

void encoderIsrLeftB() {
  static unsigned int lastTime = 0;
  int dir;
  unsigned int t = micros();
  unsigned int ti = t - lastTime;
  if (ti < 100) return;
  lastTime = t;
  boolean encA = (!!(g_APinDescription[MOT_LEFT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_LEFT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCB].ulPin)) ? true : false;
  boolean encC = (!!(g_APinDescription[MOT_LEFT_ENCC].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCC].ulPin)) ? true : false;

  if (      encA &&  encB && !encC) dir = FWD;
  else if (!encA && !encB &&  encC) dir = FWD;
  else if (!encA &&  encB &&  encC) dir = BKWD;
  else if ( encA && !encB && !encC) dir = BKWD;
  else dir = DIR_ERROR;

  noInterrupts();
  encoderLeftDir = dir;
  encoderLeftPeriod = t - encoderLeftTime;
  encoderLeftTime = t;
  interrupts();

  if (dir == FWD) tickPositionLeft++;
  else if (dir == BKWD) tickPositionLeft--;
}

void encoderIsrLeftC() {
  static unsigned int lastTime = 0;
  int dir;
  unsigned int t = micros();
  unsigned int ti = t - lastTime;
  if (ti < 100) return;
  lastTime = t;
  boolean encA = (!!(g_APinDescription[MOT_LEFT_ENCA].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCA].ulPin)) ? true : false;
  boolean encB = (!!(g_APinDescription[MOT_LEFT_ENCB].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCB].ulPin)) ? true : false;
  boolean encC = (!!(g_APinDescription[MOT_LEFT_ENCC].pPort -> PIO_PDSR & g_APinDescription[MOT_LEFT_ENCC].ulPin)) ? true : false;

  if (     !encA &&   encB &&  encC) dir = FWD;
  else if ( encA &&  !encB && !encC) dir = FWD;
  else if ( encA &&  !encB &&  encC) dir = BKWD;
  else if (!encA &&   encB && !encC) dir = BKWD;
  else  dir = DIR_ERROR;

  noInterrupts();
  encoderLeftDir = dir;
  encoderLeftPeriod = t - encoderLeftTime;;
  encoderLeftTime = t;
  interrupts();

  if (dir == FWD) tickPositionLeft++;
  else if (dir == BKWD) tickPositionLeft--;
}

void dumpEncoders() {
  unsigned int e = dataArrayPtr;
  for (int i = 0; i < e; i++) {
    Serial.print(aArray[i]); Serial.print("  \t");
    Serial.print(bArray[i]); Serial.print("\t");
    Serial.print(cArray[i]); Serial.print("\t");
    Serial.print(dArray[i]); Serial.print("\t");
    Serial.print(eArray[i]); Serial.print("\t");
    Serial.print(fArray[i]); Serial.print("\t");
    Serial.println(gArray[i]);
  }
  dataArrayPtr = 0;
}

void readSpeed() {
  unsigned int t = micros();
  tickPosition = tickPositionRight + tickPositionLeft;
  tickPositionDiff = tickPositionLeft - tickPositionRight;

  //Right
  if (encoderRightDir == DIR_ERROR) {
    fpsRight == 0.0D;
  } else if ((t - encoderRightTime) > 100000) {
    fpsRight = 0.0D;
  } else {
    fpsRight = (ENC_FACTOR * 1000.0) / ((double) encoderRightPeriod);
    if (fpsRight > 10.0D) fpsRight = 0.0D;
  }
  if (encoderRightDir == BKWD) fpsRight = -fpsRight;

  //Left
  if (encoderLeftDir == DIR_ERROR) {
    fpsLeft = 0.0D;
  } else if ((t - encoderLeftTime) > 100000) {
    fpsLeft = 0.0D;
  } else {
    fpsLeft = (ENC_FACTOR * 1000.0) / ((double) encoderLeftPeriod);
    if (fpsLeft > 10.0D) fpsLeft = 0.0D;
  }
  if (encoderLeftDir == BKWD) fpsLeft = -fpsLeft;

  mFpsRight = (int) (fpsRight * 1000.0);
  mFpsLeft = (int) (fpsLeft * 1000.0);
  wheelSpeedFps = (fpsLeft + fpsRight) / 2.0D;
  mWheelSpeedFps = (mFpsRight + mFpsLeft) / 2;
}
//
//void readSpeed() {
//  static unsigned int lastRight = 0;
//  static unsigned int lastLeft = 0;
//  noInterrupts();
//  for (int i = 0; i < encoderRightPtr; i++)
//  interrupts();
//}

