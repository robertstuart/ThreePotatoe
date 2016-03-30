
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
//  readSonar();
//  battery();
  controllerConnected();
//  liftJump();
//  gyroTemperature();
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
    battVolt = (1000 * analogRead(BATT_LOGIC_PIN)) / 451;
  }
}


/**************************************************************************.
 * switches()
 *      Toggle TP_STATE_RUN_READY on yellow switch.  1 sec dead period.
 **************************************************************************/
void switches() {
//  static int yeTrigger = 0;
//  static int buTrigger = 0;
//  static boolean buState = false;
//  String s = "";
//  if (digitalRead(YE_SW_PIN) == LOW) {
//    if (timeMilliseconds > yeTrigger) {
//      yeTrigger = timeMilliseconds + 1000;
//      isRunReady = isRunReady ? false : true;
//    }
//  }
//  if (digitalRead(BU_SW_PIN) == LOW) {
//    if (timeMilliseconds > buTrigger) {
//      buTrigger = timeMilliseconds + 1000;
//      buState = !buState;
//    }
//  }
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
    
  // Collect data for least squares calculation.
  if (isGXAxis) {
    lsXArray[lsPtr] = (float) currentMapLoc.x;
    lsYLArray[lsPtr] = (float) currentMapLoc.y;
  } else {
    lsXArray[lsPtr] = (float) currentMapLoc.y;
    lsYLArray[lsPtr] = (float) currentMapLoc.x;
  }
  lsSArray[lsPtr] = (float) distance;
  if (lsPtr < (LS_ARRAY_SIZE - 1)) lsPtr++;

  // Collect data for Charted Object measurements. 
  if (isRight) {     
    sonarRightArray[sonarRightArrayPtr] = sonarRight;
    sonarRightArrayPtr = ++sonarRightArrayPtr % SONAR_ARRAY_SIZE;
    sonarRight = distance;
  } else {
    sonarLeftArray[sonarLeftArrayPtr] = sonarLeft;
    sonarLeftArrayPtr = ++sonarLeftArrayPtr % SONAR_ARRAY_SIZE;
    sonarLeft = distance;
  }

  if (isRouteInProgress) routeLog();
}

void setSonar(int mode) {
  int r, l;
  sonarMode = mode;
  if (mode == SONAR_RIGHT) {
    r = HIGH;
    l = LOW;
  } else if (mode == SONAR_LEFT) {
    r = LOW;
    l = HIGH;
  } else if (mode == SONAR_BOTH) {
    r = HIGH;
    l = HIGH;
  } else {
    r = LOW;
    l = LOW;
  }
  digitalWrite(SONAR_RIGHT_PIN, r);
  digitalWrite(SONAR_LEFT_PIN, l);
}

void setTargetSpeedRight(float speed) {
  
}
void setTargetSpeedLeft(float speed) {
  
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
   encoderRightdir = dir;
  encoderRightPeriod = encoderRightTime - t;;
  encoderRightTime = t;
  interrupts();
  ti = ti / 1000;
  int bin = encA ? 881 : 111;
//  addLog(t, ti, bin, 0, 0, 0, 0);  
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
  encoderRightdir = dir;
  encoderRightPeriod = encoderRightTime - t;;
  encoderRightTime = t;
  interrupts();
  ti = ti / 1000;
  int bin = encB ? 882 : 112;
//  addLog(t, 0, 0, ti, bin, 0, 0);  
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
  encoderRightdir = dir;
  encoderRightPeriod = encoderRightTime - t;;
  encoderRightTime = t;
  interrupts();
  ti = ti / 1000;
  int bin = encC ? 883 : 113;
//  addLog(t, 0, 0, 0, 0, ti, bin);  
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
  encoderLeftdir = dir;
  encoderLeftPeriod = encoderLeftTime - t;;
  encoderLeftTime = t;
  interrupts();
  ti = ti / 1000;
  int bin = encA ? 881 : 111;
  addLog(t, ti, bin, 0, 0, 0, 0);  
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
  encoderLeftdir = dir;
  encoderLeftPeriod = encoderLeftTime - t;;
  encoderLeftTime = t;
  interrupts();
  ti = ti / 1000;
  int bin = encB ? 882 : 112;
  addLog(t, 0, 0, ti, bin, 0, 0);  
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
  encoderLeftdir = dir;
  encoderLeftPeriod = encoderLeftTime - t;;
  encoderLeftTime = t;
  interrupts();
  ti = ti / 1000;
  int bin = encC ? 883 : 113;
  addLog(t, 0, 0, 0, 0, ti, bin);  
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
  //Right
  if ((t - encoderRightTime) > 100000) {
    fpsRight = 0.0D;
  } else {
    fpsRight = (ENC_FACTOR * 1000.0) / ((double) encoderRightPeriod);
  }
  if (encoderRightdir == BKWD) fpsRight = -fpsRight;

  //Left
  if ((t - encoderLeftTime) > 100000) {
    fpsLeft = 0.0D;
  } else {
    fpsLeft = (ENC_FACTOR * 1000.0) / ((double) encoderLeftPeriod);
  }
  if (encoderLeftdir == BKWD) fpsLeft = -fpsLeft;
  
  mFpsRight = (int) (fpsRight * 1000.0);
  mFpsLeft = (int) (fpsLeft * 1000.0);
  wheelSpeedFps = (fpsLeft + fpsRight) / 2.0D;
  mWheelSpeedFps = (mFpsRight + mFpsLeft) / 2;
}

