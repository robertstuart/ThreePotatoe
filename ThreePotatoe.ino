/* ---------------------- ThreePotatoe ----------------------- */

#include "Common.h"
//#include <DueTimer.h>
#include <Servo.h>
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

#define XBEE_SER Serial3
#define BLUE_SER Serial1
#define SONAR_SER Serial2

#define TICKS_PER_FOOT 2222.0D // For Losi DB XL 1/5 scale
//#define TICKS_PER_CIRCLE_YAW  11900.0  // For Pro-Line Masher 2.8" PRO1192-12
#define TICKS_PER_CIRCLE_YAW  7816.0  // For Losi DB XL 1/5 scale

const int SERVO_CENTER_RIGHT = 1650;
const int SERVO_CENTER_LEFT = 1550;

L3G gyro;
LSM303 compass;

// defines for motor pins
// connections are reversed here to produce correct forward motion in both motors
const int MOT_RIGHT_ENCA =  35;  
const int MOT_RIGHT_ENCB =  37; 
const int MOT_RIGHT_ENCC =  39; 
const int MOT_LEFT_ENCA =   45;   
const int MOT_LEFT_ENCB =   47; 
const int MOT_LEFT_ENCC =   49; 

const double SPEED_MULTIPLIER = 12.0;
const unsigned int TP_PWM_FREQUENCY = 10000;
//const unsigned int TP_PWM_FREQUENCY = 1000;

const int HEADING_SOURCE_G =  0;
const int HEADING_SOURCE_M =  1;
const int HEADING_SOURCE_T =  2;
const int HEADING_SOURCE_GM = 3;

#define BRAKE 2
#define FWD 1
#define COAST 0
#define BKWD -1
#define STOP -2  // for target direction
#define DIR_ERROR -99
#define MOTOR_RIGHT 1
#define MOTOR_LEFT 2

#define SONAR_NONE 0
#define SONAR_RIGHT 1
#define SONAR_LEFT 2
#define SONAR_BOTH 3

#define LED_PIN 13 // LED connected to digital pin 13

#define SERVO_R_PIN 26
#define SERVO_L_PIN 24

//#define SW1_PIN 46
//#define SW2_PIN 47
//#define SW3_PIN 48
//#define SW4_PIN 49

//#define FOOTSW_PIN 52

#define BATT_LOGIC_PIN A0
#define BATT_MOTOR_PIN A1

#define SONAR_RIGHT_PIN 43
#define SONAR_LEFT_PIN 45

#define A_LIM 20.0 // degrees at which the speedAdjustment starts reducing.
#define S_LIM 1.0  // maximum speedAdjustment;

//Encoder factor
//const double ENC_FACTOR = 1329.0f;  // Change pulse width to fps speed, 1/29 gear
//const long ENC_FACTOR_M = 1329000L;  // Change pulse width to milli-fps speed, 1/29 gear
const double ENC_FACTOR = 17.0f;  // Change pulse width to fps speed, 1/29 gear
const long ENC_FACTOR_M = 17000L;  // Change pulse width to milli-fps speed
const double FPS_TO_TPCS = 7.52f;   // Convert foot/sec to tics/centisecond
const double ENC_BRAKE_FACTOR = ENC_FACTOR * 0.95f;

// Max int/long values
#define UNSIGNED_LONG_MAX 4294967295UL 
#define LONG_MAX  2147483647L
#define LONG_MIN -2147483648L

//#define TICKS_PER_FOOT 1536.0D
//#define TICKS_PER_RADIAN_YAW (TICKS_PER_CIRCLE_YAW / TWO_PI)
#define TICKS_PER_DEGREE_YAW (TICKS_PER_CIRCLE_YAW / 360.0)
//#define TICKS_PER_PITCH_DEGREE 20.0
#define TICKS_PER_PITCH_DEGREE 54.0D
#define GYRO_WEIGHT 0.98    // Weight for gyro compared to accelerometer
#define DEFAULT_GRID_OFFSET 0.0
#define SONAR_SENS 0.0385

// Decrease this value to get greater turn for a given angle
//#define GYRO_SENS 0.0660     // Multiplier to get degree. subtract 1.8662% * 8 for 2000d/sec
#define GYRO_SENS 0.0664     // Multiplier to get degree. subtract 1.8662% * 8 for 2000d/sec
//#define GYRO_SENS 0.0665     // Multiplier to get degree. subtract 1.8662% * 8 for 2000d/sec
//#define GYRO_SENS 0.0670     // Multiplier to get degree. subtract 1.8662% * 8 for 2000d/sec

#define INVALID_VAL -123456.78D

// Due has 96 kbytes sram
#define DATA_ARRAY_SIZE 2500
//#define DATA_ARRAY_SIZE 20
// Arrays to save data to be dumped in blocks.
long  aArray[ DATA_ARRAY_SIZE];
short bArray[ DATA_ARRAY_SIZE];
short cArray[ DATA_ARRAY_SIZE];
short dArray[ DATA_ARRAY_SIZE];
short eArray[ DATA_ARRAY_SIZE];
short fArray[ DATA_ARRAY_SIZE];
short gArray[ DATA_ARRAY_SIZE];
unsigned int dataArrayPtr = 0;

//#define TICK_ARRAY_SIZE 10300
#define TICK_ARRAY_SIZE 1030
short tArray[TICK_ARRAY_SIZE]; // Period in usec
short uArray[TICK_ARRAY_SIZE]; // State of rotation tick
unsigned int tickArrayPtr = 0;

Servo servoRight, servoLeft;

struct valSet {
  double t;
  double u;
  double v;
  double w;
  double x;
  double y;
  double z;
};

int bbb = 42;
 

struct loc {
  double x;
  double y;
};

struct loc currentMapLoc;
struct loc routeTargetLoc;
//boolean isFixLoc = false;
struct loc coSetLoc;
boolean isFixHeading = false;
boolean isAngleControl = false;
boolean decelStopped = false;

struct chartedObject {
  boolean isRightSonar;
  char type;            // 'S', 'H', or ' ',
  double trigger;
  double surface;
};

#define SONAR_ARRAY_SIZE 20
double sonarRightArray[SONAR_ARRAY_SIZE];
double sonarLeftArray[SONAR_ARRAY_SIZE];
int sonarRightArrayPtr = 0;
int sonarLeftArrayPtr = 0;

#define CO_SIZE 10
struct chartedObject chartedObjects[CO_SIZE];
int coPtr = 0;
int coEnd = 0;

#define LS_ARRAY_SIZE 1000
unsigned int lsPtr = 0;
float lsXArray[LS_ARRAY_SIZE] = {5,7,9,12,15};
float lsYLArray[LS_ARRAY_SIZE] = {2,2.2,2.5,2.6,3};
float lsSArray[LS_ARRAY_SIZE] = {1.9,2.1,2.6,2.7,3.2};
double lsXYSurface = 0;
double hugXYRhumb = 0.0D;
double hugXYSurface = 0.0D;
double hugSonarDistance = 0.0D;
double hugBearing = 0.0D;
char hugDirection = 'N';
boolean isHug = false;

int routeStepPtr = 0;
String routeTitle = "No route";

boolean isTurnDegrees = false;
double turnTargetCumHeading = 0.0;
struct loc pivotLoc;

char routeCurrentAction = 0;
double routeTargetBearing = 0.0;
//double phantomTargetBearing = 0.0;
double routeMagTargetBearing = 0.0;
boolean isReachedMagHeading = false;
boolean routeIsRightTurn = true;
long routeTargetTickPosition = 0L;
double targetDistance = 0.0D;
double pirouetteFps = 0.0;
double routeFps = 0.0;
double routeRadius = 0.0;
int routeWaitTime = 0L;
boolean isEsReceived = false;
boolean isRouteTargetIncreasing = false;
boolean isGXAxis = false;
double routeTargetXY = 0.0;
long navOldTickPosition = 0L;
double currentMapHeading = 0.0;
double currentMapCumHeading = 0.0;
double routeTargetXYDistance = 0.0;
int originalAction = 0;

int gyroTempCompX = 200;
int gyroTempCompY = 0;
int gyroTempCompZ = 105;
double sumX = 0.0D;
double sumY = 0.0D;
double sumZ = 0.0D;
double meanX = 0.0D;
double meanY = 0.0D;
double meanZ = 0.0D;

double gaPitch = 0.0;
double gaFullPitch = 0.0;
double tgaPitch = 0.0;
double gaRoll = 0.0;

double aPitch = 0.0;
double aRoll = 0.0;

double pitchDrift = 0.0D;
double rollDrift = 0.0D;
double yawDrift = 0.0D;
double gPitch = 0.0;
double gRoll = 0.0;
double gYaw = 0.0;
double gyroCumHeading = 0.0;
double gyroHeading = 0;

double tPitch = 0.0D;
double oldTPitch = 0.0D;
double tgPitch = 0.0D;
double tgPitchDelta = 0.0D;
double oldTgPitch = 0.0D;

double accelFpsSelfX = 0.0;
double accelFpsSelfY = 0.0;
double accelFpsMapX = 0.0;
double accelFpsMapY = 0.0;

double rotation2 = 0.0D;
double cos2 = 0.0D;
double lpfCos2 = 0.0D;
double lpfCosOld2 = 0.0D;
double oldGyroCumHeading = 0.0D;
double oldTickCumHeading = 0.0D;

double headX, headY;

int16_t mX, mY, mZ;

double xVec, yVec, zVec;
boolean isMagAdjust = true;
double magHeading = 0.0;
double gridOffset = 0.0;
double gridHeading = 0.0;
double gridCumHeading = 0.0;
double gridRotations = 0.0;
double tickHeading = 0.0;
double tickCumHeading = 0.0;
int tickOffset = 0;
double tmHeading = 0.0;
double tmCumHeading = 0.0;
double gmHeading = 0.0;
double gmCumHeading = 0.0;
double currentX = 0.0;
double currentY = 0.0;
int fixPosition = 0;
double fixHeading = 0.0;

// Speed and position variables
long tickPositionRight = 0L;
long tickPositionLeft = 0L;
long tpDistanceDiff = 0L;
long tickPosition;
long coTickPosition;
double tp6LpfCos = 0.0;
double startDecelSpeed = 0.0;

// System status (used to be status bits
boolean isRunReady = false;   // Reflects the Run command
boolean isRunning = false;
boolean isUpright = false;
boolean isLifted = true;
boolean isOnGround = false;
boolean isHcActive = false; // Hand controller connected.
boolean isPcActive = false; // PC connected
boolean isRouteInProgress  = false; // Route in progress
boolean isDumpingData = false; // Dumping data
boolean isHoldHeading = false; // 
boolean isStand = false; // 

int standTPRight = 0;
int standTPLeft = 0;

unsigned long tickTimeRight = 0UL;  // time for the last interrupt
unsigned long tickTimeLeft = 0UL;
long tickPeriodRight = 0L;     // last period. Minus for reverse.
long tickPeriodLeft = 0L;
double targetSpeedRight = 0.0;
double targetSpeedLeft = 0.0;
long targetTickPeriodRight = 0L;
long targetMFpsRight = 0L;
long targetBrakeMFpsRight = 0L;
long targetRevMFpsRight = 0L;
long targetMFpsLeft = 0L;
long targetBrakeMFpsLeft = 0L;
long targetRevMFpsLeft = 0L;

double fpsRight = 0.0f; // right feet per second
double fpsLeft = 0.0f;  // left feet per second TODO rename!
double wheelSpeedFps = 0.0f;
int mWheelSpeedFps = 0;

unsigned long gyroTrigger = 0UL;
unsigned long statusTrigger = 0UL;
unsigned long oldTimeTrigger = 0UL;
unsigned long timeMicroseconds = 0UL; // Set in main loop.  Used by several routines.
unsigned long timeMilliseconds = 0UL; // Set in main loop from above.  Used by several routines.

int tpState = 0; // contains system state bits
int cmdState = 0;  // READY, PWR, & HOME command bits

unsigned int forceRight = 0; // force sensor value
unsigned int forceLeft = 0; // force sensor value
double sonarRight = 0.0;
double sonarRightMin = 0.0;
double sonarLeft = 0.0;
double sonarLeftMin = 0.0;
int sonarMode = SONAR_BOTH;
double sonarMin = 0.0D;
double sonarMax = 100.0D;

unsigned int actualLoopTime; // Time since the last
double hcX = 0.0;
double hcY = 0.0;
double pcX = 0.0;
double pcY = 0.0;
//double controllerX = 0.0; // +1.0 to -1.0 from controller
//double controllerY = 0.0;  // Y value set by message from controller
boolean isNewMessage = false;
char message[100] = "";

int gyroFahrenheit = 0;
double gyroPitchRaw;  // Vertical plane parallel to wheels
double gyroPitchRate;
long gyroPitchRawSum;
double oldGaPitch = 0.0;
double gyroPitchDelta = 0.0;
double gyroPitch = 0.0; // not needed
double temperatureDriftPitch = 0.0D;
double timeDriftPitch = 0.0;

double gyroRollRaw = 0;
double gyroRollRate;
double gyroRoll = 0.0f;
double accelRoll = 0.0f;
double temperatureDriftRoll = 0.0D;
double timeDriftRoll = 0.0;

double gyroYawRaw = 0.0f;
double gyroYawRate = 0.0f;
double gyroYawAngle = 0.0f;
double gyroYawRawSum = 0.0;
double temperatureDriftYaw = 0.0D;
double timeDriftYaw = 0.0;

int gyroErrorX = 0;
int gyroErrorY = 0;
int gyroErrorZ = 0;

int battVolt = 0; // battery 
int tpDebug = 4241;

unsigned long tHc = 0L;  // Time of last Hc packet
unsigned long tPc = 0L;  // Time of last Pc packet

const int MAX_PACKET_SIZE = 100;
byte sendArray[MAX_PACKET_SIZE + 1];
unsigned int packetByteCount = 0;
unsigned int dataPtr = 0;
int packetValue;   // int value
boolean isPacketInProgress = false;
unsigned int packetSource;
unsigned int packetSignal;
boolean isTxStatusMessage = false;
int txAckFrame = 0;
boolean isBluePassthrough = false;

boolean isHcCommand = false;

/* roll pitch and yaw angles computed by iecompass */
int iPhi, iThe, iPsi; 
/* magnetic field readings corrected for hard iron effects and PCB orientation */
int iBfx, iBfy, iBfz;
/* hard iron estimate */
int iVx, iVy, iVz;

double magCorrection = 0.0;

// Sequence variables
int sequenceCount = 0;
boolean sequenceIsRunning = false;
int runSequenceCount = 0;
boolean isSequence = false;
int seqDur = 0;
double seqWs = 0.0;
int seqPw = 0;
double wsArray[30];
int wsDurArray[30];
int pulseIndex = 0;
int pulseCount = 0;
int motorArray[30];
int pwArray[30];
unsigned long pulseTrigger = 0;
boolean isReceivingBlock = false;
boolean isPwData = false; // Send data after a sequence?

int actionRight = 99;
int actionLeft = 99;

int tVal = 0;
int uVal = 0;
int vVal = 0;
int wVal = 0;
int xVal = 0;
int yVal = 0;
int zVal = 0;

int wsMFpsRight = 0;
int wsMFpsLeft = 0;
int mFpsRight = 0;
int mFpsLeft = 0;
long wsMFpsRightSum = 0;
long wsMFpsLeftSum = 0;
int wsMFpsRightCount = 0;;
int wsMFpsLeftCount = 0;;
double airFps = 0.0;
unsigned long airTrigger = 0L;

int ackFrameNumber = 0;
int ackFailure = 0;
int ccaFailure = 0;;
int purgeFailure = 0;

double tp5LpfCosAccel = 0.0;
double tp6LpfCosAccel = 0.0;

double tp5FpsLeft = 0.0f;
double tp5FpsRight = 0.0f;
double tp6FpsLeft = 0.0f;
double tp6FpsRight = 0.0f;

int interruptErrorsRight = 0;
int interruptErrorsLeft = 0;

double stopADiff = 0;
double stopDist = 0;

unsigned int xBeeCount = 0;
int rightK = 0;
int leftK = 0;
unsigned int tr;
unsigned int stopTimeRight = UNSIGNED_LONG_MAX;
unsigned int stopTimeLeft = UNSIGNED_LONG_MAX;

int motorRightAction;
int headingSource = HEADING_SOURCE_G;
boolean isRouteWait = false;
long standPos = 0;
int msgCmdX = 0;
boolean isDiagnostics = false;

boolean isStepFall = false;
boolean isOldStepFall = false;
double tp6ControllerSpeed = 0;
double jumpTarget;
double jumpFallXY;

double routeTargetMagHeading = 0.0;
char pBuf[100];

unsigned int encoderRightdir = FWD;
int encoderRightPeriod;
unsigned int encoderRightTime = 0;
//double fpsRight = 0.0D;
unsigned int encoderLeftdir = FWD;
int encoderLeftPeriod;
unsigned int encoderLeftTime = 0;
//double fpsLeft = 0.0D;

/*********************************************************
 *
 * setup()
 *
 *     Required by the Arduino system.  Called once at boot time.
 *
 *********************************************************/
void setup() {
  XBEE_SER.begin(57600);  // XBee, See bottom of this page for settings.
  BLUE_SER.begin(115200);  // Bluetooth 
  SONAR_SER.begin(9600);   // Mini-pro sonar controller
  Serial.begin(115200); // for debugging output

  servoRight.attach(SERVO_R_PIN);
  servoLeft.attach(SERVO_L_PIN);

   
  pinMode(LED_PIN,OUTPUT);  // Status LED
   
  pinMode(BATT_LOGIC_PIN, INPUT);
  pinMode(BATT_MOTOR_PIN, INPUT);

  pinMode(SONAR_RIGHT_PIN, OUTPUT);
  pinMode(SONAR_LEFT_PIN, OUTPUT);

//  pinMode(FOOTSW_PIN, OUTPUT);
//  digitalWrite(FOOTSW_PIN, HIGH); // Motors off
  
//  pinMode(SW1_PIN, INPUT_PULLUP);
//  pinMode(SW2_PIN, INPUT_PULLUP);
//  pinMode(SW3_PIN, INPUT_PULLUP);
//  pinMode(SW4_PIN, INPUT_PULLUP);

  pinMode(MOT_RIGHT_ENCA, INPUT);
  pinMode(MOT_RIGHT_ENCB, INPUT);
  pinMode(MOT_RIGHT_ENCC, INPUT);
  pinMode(MOT_LEFT_ENCA, INPUT);
  pinMode(MOT_LEFT_ENCB, INPUT);
  pinMode(MOT_LEFT_ENCC, INPUT);

  attachInterrupt(MOT_RIGHT_ENCA,  encoderIsrRightA, CHANGE);
  attachInterrupt(MOT_RIGHT_ENCB, encoderIsrRightB, CHANGE);
  attachInterrupt(MOT_RIGHT_ENCC, encoderIsrRightC, CHANGE);
  attachInterrupt(MOT_LEFT_ENCA, encoderIsrLeftA, CHANGE);
  attachInterrupt(MOT_LEFT_ENCB, encoderIsrLeftB, CHANGE);
  attachInterrupt(MOT_LEFT_ENCC, encoderIsrLeftC, CHANGE);


  digitalWrite(LED_PIN, HIGH);

//  setSonar(SONAR_BOTH);
  
  Serial.println("Serial & pins initialized.");
  angleInit6();
  Serial.println("Navigation initialized");
  gyroTrigger = micros();
  delay(100);
  for (int i = 0; i < DATA_ARRAY_SIZE; i++) {
    aArray[i] = 42;
    bArray[i] = 9;
    cArray[i] = 4242;
    dArray[i] = i;
  }
//  zeroGyro();
//  Serial.println("Gyro zeroed out.");
//  diagnostics();
  Serial.println("Diagnostics ignored.");
} // end setup()



/************************************************************************** *
 *
 * loop()
 *
 *     Used only to change modes.  All algorithms should
 *     run in an infinite loop and then exit their loop
 *     whenever the mode changes.
 *
 *********************************************************/
void loop() { //Main Loop
//  bluePass();
  aThreeRun();
//  testServos();
} // End loop().  

void testServos() {
  boolean toggle = false;
  double oldX = 0.0D;
  double oldY = 0.0D;
  int centerRight = 1650;
  int centerLeft = 1550;
  unsigned int lTrigger = millis();
  servoRight.writeMicroseconds(centerRight);
  servoLeft.writeMicroseconds(centerLeft);
  while(true) { // main loop
    readXBee();
    timeMilliseconds = millis();
    if (timeMilliseconds > lTrigger) {
      lTrigger = timeMilliseconds + 50; // 20/sec
      sendXMsg(SEND_STATE, 42); // MUST BE LAST MESSAGE!
      if ((oldX != hcX) || (oldY != hcY)) {
        oldX = hcX;
        oldY = hcY;
        int rightVal = ((int) ((hcY * 200.0) + (hcX * 100.0))) + centerRight;
        int leftVal = ((int) ((-hcY * 200.0) + (hcX * 100.0))) + centerLeft;
        servoRight.writeMicroseconds(rightVal);
        servoLeft.writeMicroseconds(leftVal);
      }
      toggle = !toggle;
      if (toggle) {
        digitalWrite(LED_PIN, HIGH);
//        printSpeed();
      }
      else digitalWrite(LED_PIN, LOW);
    } // end timed loop
  }
}



/************************************************************************** *
 *
 * bluePass()
 * 
 *********************************************************/
void bluePass() {
  if (BLUE_SER.available()) Serial.write(BLUE_SER.read());
  if (Serial.available())   {
    int b = Serial.read();
    Serial.write(b);
    BLUE_SER.write(b);
    
  }
}



/***********************************************************************.
 *  diagnostics()  If the yellow switch is pressed at startup, 
 *                 run diagnosics
 *                 every second which give a printout of the status of
 *                 all systems.
 ***********************************************************************/
void diagnostics() {
  unsigned long dTime1, dTime2;
//  Serial.println(digitalRead(SW1_PIN));
//  if (digitalRead(SW1_PIN) != LOW) return;
  isDiagnostics = true;
  Serial.println("Start diagnositics.");
  
  while (true) {
    delay(1000);

    // Accelerometer
    dTime1 = micros();
    readAccel(); 
    Serial.print("uSec:"); Serial.print(micros() - dTime1); Serial.print("\t");
    Serial.print("aPitch: "); Serial.print(aPitch); Serial.print("\t");
    Serial.print("aRoll: "); Serial.print(aRoll); Serial.println();

    // Gyro
    dTime1 = micros();
    readGyro();
    Serial.print("uSec:"); Serial.print(micros() - dTime1); Serial.print("\t");
    Serial.print("gyroPitchRate: "); Serial.print(gyroPitchRate); Serial.print("\t");
    Serial.print("gyroRollRate: "); Serial.print(gyroRollRate); Serial.print("\t");
    Serial.print("gyroYawRate: "); Serial.print(gyroYawRate); Serial.println();

    Serial.println();
  }
}


/*********************************************************
 *
 * Xbee settings
 *
 *     The XBee 900 hz modules are left in their 
 *     default state except that the baud rate is set to 
 *     57k and the destination addresses are set 
 *     to point to the other module.
 *
 *********************************************************/











