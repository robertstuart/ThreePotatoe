/***********************************************************************.
 *  Angle6
 ***********************************************************************/

int yawTempComp = 0;

/***********************************************************************.
 *  angleInit6()
 ***********************************************************************/
void angleInit6() {
  Wire.begin();
  if (!lsm6.init()) Serial.println("IMU initialize failed!");
  else Serial.println("IMU Initialized!****************************");
  lsm6.enableDefault();
  lsm6.writeReg(LSM6::INT1_CTRL, 0X02); // Gyro data on INT1
  lsm6.writeReg(LSM6::CTRL2_G, 0X4C); // 2000fs, 104hz
  lsm6.writeReg(LSM6::CTRL1_XL, 0X40); // 104hz, 2g
//  zeroGyro();
}

#define TG_PITCH_TC 0.90D

/***********************************************************************.
 * readGyro()
 ***********************************************************************/
boolean readGyro() {
  static int temperatureLoop = 0;

  lsm6.readGyro();

  // Pitch
  gyroPitchRaw = (double) lsm6.g.x - timeDriftPitch;
  gyroPitchRate = -((double) gyroPitchRaw) * GYRO_SENS;  // Rate in degreesChange/sec
  gyroPitchDelta = gyroPitchRate / 104.0; // degrees changed during period
  gPitch = gPitch + gyroPitchDelta;   // Used by tgPitch & debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle

  // Yaw
  gyroYawRaw = ((double) lsm6.g.z) - timeDriftYaw; 
  gyroYawRate = -(((double) gyroYawRaw) * GYRO_SENS);  // Rate in degreesChange/sec
  gyroYawDelta = gyroYawRate / 104.0; // degrees changed during period
//  gYaw += gyroYawDelta;
  gyroCumHeading += gyroYawDelta;   //
  double tc = (gyroCumHeading > 0.0) ? 180.0 : -180.0;
  int rotations = (int) ((gyroCumHeading + tc) / 360.0);
  gyroHeading = gyroCumHeading - (((double) rotations) * 360.0);

  // Yaw for gmHeading
  gmCumHeading += gyroYawDelta;
  tc = (gmCumHeading > 0.0) ? 180 : - 180;
  rotations = (int) ((gmCumHeading + tc) / 360.0);
  gmHeading = gmCumHeading - (((double) rotations) * 360.0);

  // Rotation rate: complementary filtered gPitch and tPitch
  tPitch = (double) tickPosition / TICKS_PER_PITCH_DEGREE;
  double qDiff = tPitch - oldTPitch;
  double q = tgPitch + qDiff;
  oldTPitch = tPitch;
  tgPitch = (q * TG_PITCH_TC) + (gPitch * (1.0D - TG_PITCH_TC));
  tgPitchDelta = tgPitch - oldTgPitch;
  oldTgPitch = tgPitch;

  // tgaPitch:
  q = tgaPitch + qDiff;
  tgaPitch = (q * TG_PITCH_TC) + (gaPitch * (1.0D - TG_PITCH_TC));

  return true;
}


/***********************************************************************.
 *  readAccel()
 ***********************************************************************/
void readAccel() {
  lsm6.readAcc();

  // Pitch
  aPitch = (atan2(-lsm6.a.y, lsm6.a.z)) * RAD_TO_DEG;
  gaFullPitch = (gaFullPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
  if (          ((lsm6.a.z < 25000) && (lsm6.a.z > 0))
             && ((lsm6.a.y > -9000) && (lsm6.a.y < 9000))
             && ((aPitch > -45.0) && (aPitch < 45.0))) {
      gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
    }

}


//#define GM_HEADING_TC 0.95D
#define GM_HEADING_TC 0.98D
#define TM_HEADING_TC 0.999D




/***********************************************************************.
 *  readTemperature()  Read the gyro temperature and set the yawTempComp
 *                     variable 
 ***********************************************************************/
#define YAW_TEMP_FACTOR 0.84  // 
float readTemperature() {
  Wire.beginTransmission(DS33_SA0_HIGH_ADDRESS);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  Wire.write(LSM6::OUT_TEMP_L);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t) DS33_SA0_HIGH_ADDRESS, (uint8_t) 2);

//  uint16_t millis_start = millis();
  while (Wire.available() < 2) {
//    if (io_timeout > 0 && ((uint16_t)millis() - millis_start) > io_timeout)
//    {
//      did_timeout = true;
//      return;
//    }
  }

  uint8_t tl = Wire.read();
  uint8_t th = Wire.read();

  // combine high and low bytes
  int16_t out = (int16_t)(th << 8 | tl);
  float ret = ((float) out) / 16;
  ret += 25.0;
  ret = ((ret * 9.0) / 5) + 32;
  return ret;
}



/***********************************************************************.
 *  setNavigation() Set gmHeading, tmHeading, tickHeading, currentLoc
 *                  Called 400/sec (every read of gyro).
 ***********************************************************************/
void setNavigation() {

  // Tick heading.
  tickCumHeading =  ((double) (tickOffset + tickPositionLeft - tickPositionRight)) / TICKS_PER_DEGREE_YAW;
  double c = (tickCumHeading > 0.0) ? 180.0 : -180.0;
  int rotations = (int) ((tickCumHeading + c) / 360.0); 
  tickHeading = tickCumHeading - (((double) rotations) * 360.0);
  
  // tmHeading.  Complementary filter tick and mag headings.
  tmCumHeading += tickCumHeading - oldTickCumHeading;
  oldTickCumHeading = tickCumHeading;
  tmCumHeading = (tmCumHeading * TM_HEADING_TC) + (gridCumHeading * (1.0 - TM_HEADING_TC));
  c = (tmCumHeading > 0.0) ? 180.0 : -180.0;
  rotations = (int) ((tmCumHeading + c) / 360.0);
  tmHeading = tmCumHeading - (((double) rotations) * 360.0);

  // Map heading
  switch (headingSource) {
    case HEADING_SOURCE_G: // gyro
      currentMapCumHeading = gyroCumHeading;
      currentMapHeading = gyroHeading;
      break;
    case HEADING_SOURCE_T: // ticks
      currentMapCumHeading = tickCumHeading;
      currentMapHeading = tickHeading;
      break;
    case HEADING_SOURCE_M: // mag
      currentMapCumHeading = gridCumHeading;
      currentMapHeading = magHeading;
      break;
    case HEADING_SOURCE_GM: // gyro & mag complementary filtered
      currentMapCumHeading = gmCumHeading;
      currentMapHeading = gmHeading;
      break;
  }
 
 // compute the Center of Oscillation Tick Position
//  coTickPosition = tickPosition - ((long) (sin(gaPitch * DEG_TO_RAD) * 4000.0));
  coTickPosition = tickPosition;  // For ThreePotatoe

  // Compute the new co position
  double dist = ((double) (coTickPosition - navOldTickPosition)) / TICKS_PER_FOOT;
  navOldTickPosition = coTickPosition;
  currentMapLoc.x += sin(currentMapHeading * DEG_TO_RAD) * dist;
  currentMapLoc.y += cos(currentMapHeading * DEG_TO_RAD) * dist;

  currentAccelLoc();

  
}




//double accelFpsSelfX = 0.0;
//double accelFpsSelfY = 0.0;
//double accelFpsMapX = 0.0;
//double accelFpsMapY = 0.0;
//struct loc currentAccelSelfLoc;
//struct loc currentAccelMapLoc;

const double A_FACTOR = .000001D;
/***********************************************************************.
 *  setAccelLoc() Set currentAccelLoc
 ***********************************************************************/
void currentAccelLoc() {
//  currentAccelMapLoc.y += ((double) compass.a.y) * A_FACTOR;
//  currentAccelMapLoc.x += ((double) compass.a.x) * A_FACTOR;
  //  currentAccelMapLoc.x += accelFpsSelfX * .0025;
  //  currentAccelMapLoc.y += accelFpsSelfY * .0025;
  //  accelFpsMapX = (sin(currentMapHeading * DEG_TO_RAD) * accelFpsSelfX) + (cos(currentMapHeading * DEG_TO_RAD) * accelFpsSelfY);
  //  accelFpsMapY = (cos(currentMapHeading * DEG_TO_RAD) * accelFpsSelfX) + (sin(currentMapHeading * DEG_TO_RAD) * accelFpsSelfY);
  //  currentAccelMapLoc.x += accelFpsMapX * 0.0025;
  //  currentAccelMapLoc.y += accelFpsMapY * 0.0025;
}






/**************************************************************************.
 * setHeading() Sets the bearing to the new value.  The the gridOffset
 *              value will be set so that the gridBearing is an
 *              offset from magHeading.  All of cumulative rotations 
 *              be lost.
 **************************************************************************/
void setHeading(double newHeading) {
  newHeading = rangeAngle(newHeading);
  tickPosition = tickPositionRight = tickPositionLeft = navOldTickPosition = coTickPosition = 0;
  oldTPitch = 0.0D;
  gmCumHeading = tmCumHeading = gyroCumHeading = tickCumHeading = gridCumHeading = newHeading;
  oldGyroCumHeading = oldTickCumHeading = newHeading;
  tickHeading = gyroHeading = newHeading;
  targetHeading = newHeading;
  tickOffset = newHeading * TICKS_PER_DEGREE_YAW;
  gridRotations = 0.0;
}



/**************************************************************************.
 * zeroGyro()  Take z values for ? second.  Compute the value to
 *             cancel out drift.
 **************************************************************************/
#define GLOOPS 500  // Number of ~100/sec reads
void zeroGyro() {
  int loopCount = GLOOPS;
  double sumPitch = 0;
  double gyroPitchMin, gyroPitchMax;
  double sumYaw = 0;
  double gyroYawMin, gyroYawMax;
  unsigned long endTime = millis() + (GLOOPS * 3);  // Bailout time in case of bad I2C.

  for (int i = 0; i < 20; i++) {
      lsm6.readGyro();
    delay(10);
  }

  gyroPitchMin = gyroPitchMax = lsm6.g.x;
  gyroYawMin = gyroYawMax = lsm6.g.z;
  
  while(true) {
    if (digitalRead(GYRO_INTR_PIN) == HIGH) {   
      lsm6.readGyro();
      
      gyroPitchRaw = (double) lsm6.g.x; 
      sumPitch += (double) gyroPitchRaw;
      if (gyroPitchRaw > gyroPitchMax) gyroPitchMax = gyroPitchRaw;
      if (gyroPitchRaw < gyroPitchMin) gyroPitchMin = gyroPitchRaw;
      
      gyroYawRaw = ((double) lsm6.g.z) - timeDriftYaw; 
      sumYaw += (double) gyroYawRaw;
      if (gyroYawRaw > gyroYawMax) gyroYawMax = gyroYawRaw;
      if (gyroYawRaw < gyroYawMin) gyroYawMin = gyroYawRaw;

      
      if (--loopCount <= 0) break;
    }
  }
  
  timeDriftPitch = (sumPitch / ((double) GLOOPS));
//  if ((gyroPitchMax - gyroPitchMin) > 15) blinkMs = 500;
  
  timeDriftYaw = (sumYaw / ((double) GLOOPS));
//  if ((gyroYawMax - gyroYawMin) > 15) blinkMs = 500;

  Serial.println();
  Serial.print("pitchDrift: "); Serial.print(timeDriftPitch); Serial.println();
  Serial.print("Pitch min: "); Serial.print(gyroPitchMin,0); Serial.print("\t");
  Serial.print("Pitch Max: "); Serial.print(gyroPitchMax,0); Serial.print("\t");
  Serial.print("Pitch Range: "); Serial.print(gyroPitchMax - gyroPitchMin,0); Serial.println();
   
  Serial.print("yawDrift: "); Serial.print(timeDriftYaw); Serial.println();
  Serial.print("Yaw min: "); Serial.print(gyroYawMin,0); Serial.print("\t");
  Serial.print("Yaw Max: "); Serial.print(gyroYawMax,0); Serial.print("\t");
  Serial.print("Yaw Range: "); Serial.print(gyroYawMax - gyroYawMin,0); Serial.println(); 
  
  gPitch = 0.0;
  gyroCumHeading = 0.0; 
}

/**************************************************************************.
 * resetIMU()  From: https://forum.arduino.cc/index.php?topic=386269.0
 *             I2C clocks to make sure no slaves are hung in a read
 *             at startup
 **************************************************************************/
void resetIMU() {
  // Issue 20 I2C clocks to make sure no slaves are hung in a read
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(70, OUTPUT);
  pinMode(71, OUTPUT);
  digitalWrite(20, LOW);
  digitalWrite(70, LOW);
  for (int i = 0; i < 1000; i++)
  {
    digitalWrite(21, LOW);
    digitalWrite(71, LOW);
    delayMicroseconds(10);
    digitalWrite(21, HIGH);
    digitalWrite(71, HIGH);
    delayMicroseconds(10);
  }
}

