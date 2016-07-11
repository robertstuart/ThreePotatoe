/***********************************************************************.
 * -----  ------
 ***********************************************************************/
double tp6SpeedError = 0.0;
//double tp6Fps = 0.0f;
double fpsCorrection = 0.0f;
//double tp6LoopSec = 0.0f;
double tp6LpfCosOld = 0.0;
double fpsLpfCorrectionOld = 0.0;
double fpsLpfCorrection = 0.0;
double tp6AngleError = 0.0;
double tp6TargetAngle = 0.0;
double tp6Cos = 0.0; 
double tp6Rotation = 0.0;
double rotationCorrection = 0.0;
int servoMsR, servoMsL;

/***********************************************************************.
 *  aThreeRun() 
 ***********************************************************************/
void aThreeRun() {
  unsigned int subCycle = 0;
  
  timeMicroseconds = gyroTrigger = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickPositionRight = tickPositionLeft = tickPosition = 0L;
  hoverPower(true);
  delay(20);
  readGyro();
  setHeading(0.0D);
  setNavigation();
  delay(200);
  servoRight.writeMicroseconds(servoCenterR);
  servoLeft.writeMicroseconds(servoCenterL);
  targetHeading = gyroHeading;
  while(true) { // main loop
    commonTasks();
    // Do the timed loop
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    if (digitalRead(GYRO_INTR_PIN) == HIGH) {   
      subCycle++;
      readGyro();                                    // 104/sec
      readAccel();
      setNavigation();
      aThreePotatoe();
      sendLog();
    } // end timed loop
  }
}



/***********************************************************************.
 *  aThreePotatoe() 
 ***********************************************************************/
 #define LPF_SE 0.05  // damping this doesn't seem to reduce oscillation
void aThreePotatoe() {
  static double lpfSpeedErrorOld = 0;
  double lpfSpeedError = 0;
  readSpeed();
  tp6Rotation = 0.75 * gyroPitchDelta; // Original
  tp6Cos = wheelSpeedFps - tp6Rotation; // subtract rotation 
  tp6LpfCos = (tp6LpfCosOld * (1.0 - 0.2))  + (tp6Cos  * 0.2); // smooth it out a little (0.2) ********************
  tp6LpfCosOld = tp6LpfCos;

  if (isRouteInProgress) {
    tp6ControllerSpeed = routeFps;
  } else {
    tp6ControllerSpeed = hcY * 10.0;
  }
  
  tp6SpeedError = tp6LpfCos - tp6ControllerSpeed;

  // LPF the target angle to reduce seek oscillation
  lpfSpeedError = (lpfSpeedErrorOld * (1.0 - LPF_SE)) + (tp6SpeedError * LPF_SE);
  lpfSpeedErrorOld = lpfSpeedError;
  
  tp6TargetAngle = lpfSpeedError * 2.5; //************ Speed error to angle *******************

  if (isRouteInProgress) {
    route();
  } else {
    steer();
  }

  setServos(servoMsR, servoMsL);
} // end aTp6() 



/***********************************************************************.
 *  sendLog() Called 104 time/sec.
 ***********************************************************************/
void sendLog() {
  static unsigned int logLoop = 0;

  logLoop++;
  
  if (isDumpingData) {  // 
    if ((logLoop % 4) == 0) {
      dumpData();
    }
  }

  if ((logLoop % 52) == 0) {
    log2PerSec();
  }
//  if ((logLoop % 10) == 0) {  // 10/sec 
//    Serial.print(targetHeading); Serial.print("\t"); Serial.println(gyroHeading);  
//  }
  if ((logLoop % 5) == 2) { // 20/sec
    routeLog();
//    log20PerSec();
  }
}

void log20PerSec() {
  addLog(
        (long) (timeMilliseconds),
        (short) (targetHeading * 100.0),
        (short) (gyroHeading * 100.0),
        (short) (0 * 100.0),
        (short) (wheelSpeedFps * 100.0),
        (short) (0 * 100.0),
        (short) (0 * 100.0)
   );
}

void log400PerSec() {
  addLog(
        (long) (0),
        (short) (routeTargetBearing * 100.0),
        (short) (routeTargetBearing * 100.0),
        (short) (gaPitch * 100.0),
        (short) (wheelSpeedFps * 100.0),
        (short) (tp6LpfCos * 100.0),
        (short) (tp6TargetAngle * 100.0)
   );
}
        
void log2PerSec() {
  static unsigned int t1 = 0;
  unsigned int t2 = millis();
    sprintf(message, "Time: %d", t2 - t1);
    sendBMsg(SEND_MESSAGE, message);
    t1 = t2;
}

/***********************************************************************.
 *  steer() 
 ***********************************************************************/
void steer() {
//  targetTickPositionDiff += controllerX * 0.3;
//  if (digitalRead(SW_R_PIN) == LOW) targetTickPositionDiff += 0.1;
//  if (digitalRead(SW_L_PIN) == LOW) targetTickPositionDiff -= 0.1;
//  float  steerVal = ((float) (((int) targetTickPositionDiff) - tickPositionDiff)) * 0.05;
 
  if (digitalRead(SW_R_PIN) == LOW) targetHeading += 0.25;
  if (digitalRead(SW_L_PIN) == LOW) targetHeading -= 0.25;
  targetHeading += controllerX * 1.0;
  double aDiff = rangeAngle(targetHeading - gyroHeading);
  float steerVal = aDiff * 5.0;
//  steerVal = constrain(steerVal, -400.0, 400.0);

  servoMsR = ((int) ((-tp6TargetAngle * 50.0) + steerVal)) + servoCenterR;
  servoMsL = ((int) ((tp6TargetAngle * 50.0) + steerVal)) + servoCenterL;  

//  static int loop = 0;
//  loop++;
//  if ((loop %100) == 0) {
//    Serial.print(targetHeading); Serial.print("\t"); Serial.println(gyroHeading);
//  }
}



/***********************************************************************.
 *  setServos() Prevent rapic changes in servo
 ***********************************************************************/
#define SERVO_MAX 2.0
void setServos(int newR, int newL) {
  static float r = servoCenterR;
  static float l = servoCenterL;

//  // Right value
//  if ((newR - r) > SERVO_MAX)       r += SERVO_MAX;
//  else if ((r - newR) > SERVO_MAX)  r -= SERVO_MAX;
//  else r = newR;
//
//  // Left value
//  if ((newL - l) > SERVO_MAX)       l += SERVO_MAX;
//  else if ((l - newL) > SERVO_MAX)  l -= SERVO_MAX;
//  else l = newL;
r = newR;
l = newL;
  if (isRunReady) {
    servoRight.writeMicroseconds((int) r);
    servoLeft.writeMicroseconds((int) l);
  }
//Serial.print(r); Serial.print("\t"); Serial.println(newR);
}

/************************************************************************
 *  stand() Keep position from base tickPosition
 ************************************************************************/
double standFps() {
//  float joyY = (abs(pcY) > abs(hcY)) ? pcY : hcY;
//  routeFps = 0.0;
//  
//  if (abs(joyY) > 0.05) {
//    standTPRight = tickPositionRight;
//    standTPLeft = tickPositionLeft;
//    return(joyY * 1.0);
//  } 
//  else {
//    int targetPos = standTPRight + standTPLeft;
//    int currentPos = tickPositionRight + tickPositionLeft;
//    return((float) ((targetPos - currentPos)) * 0.0005);
//  }
//}
//
//void standSteer() {
//  float headingSpeedAdjustment = 0.0;
//  float joyX = (abs(pcX) > abs(hcX)) ? pcX : hcX;
//  
//  if (abs(joyX) > 0.05) {
//    headingSpeedAdjustment = joyX * 0.3;
//    standTPRight = tickPositionRight;
//    standTPLeft = tickPositionLeft;
//  }
//  else {
//    int targetTD = standTPRight - standTPLeft;
//    int currentTD = tickPositionRight - tickPositionLeft;
//    headingSpeedAdjustment = ((float) (currentTD - targetTD)) * 0.01;
//  }
//
//  tp6FpsRight = tp6Fps - headingSpeedAdjustment;
//  tp6FpsLeft = tp6Fps + headingSpeedAdjustment;
}


