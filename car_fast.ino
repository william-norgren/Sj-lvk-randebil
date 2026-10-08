// =====================================================================
//  Self-driving car: obstacle avoidance while moving + garage detection
//  The car never stops to "think". It drives continuously, the distance
//  sensor sweeps front/right/front/left on its own, and the steering
//  servo steers away from walls and cans while moving.
// =====================================================================


// =====================================================================
//  >>> MEASUREMENTS: fill these in from the real car <<<
//  All in centimetres unless noted. The numbers below are PLACEHOLDER
//  GUESSES so the code compiles. Replace every one marked MEASURE.
// =====================================================================

// [M1] MEASURE: Turning radius, RIGHT.
//      Car standing still, wheels already at full right lock (SteerRightAngle),
//      then drive at TurnSpeed until the car has turned 90 degrees.
//      Measure how far the MIDDLE OF THE FRONT AXLE moved SIDEWAYS.
float RadiusRight_cm = 40;

// [M2] MEASURE: Turning radius, LEFT. Same as M1 but full left lock.
float RadiusLeft_cm = 60;

// [M3] MEASURE: Car width at its WIDEST point
//      (wheels, chassis, battery pack: whatever sticks out furthest).
float CarWidth_cm = 16;

// [M4] MEASURE: How far the ultrasonic sensor's front face is BEHIND the
//      front-most point of the car. Use 0 if the sensor is the front-most part.
float SensorSetback_cm = 0;

// [M5] OPTIONAL MEASURE: Time for one FULL circle at TurnSpeed and full lock,
//      in milliseconds (e.g. 3.2 s = 3200). Leave at 0 if not measured.
unsigned long LapTimeRight_ms = 0;
unsigned long LapTimeLeft_ms  = 0;

// Known / adjustable, no measuring needed:
float CanRadius_cm     = 3.3;  // a standard 33 cl can is about 6.6 cm wide
float SafetyMargin_cm  = 4;    // extra room everywhere. Raise if it clips things
float ReactionDist_cm  = 8;    // distance covered while servo + sensor react. Raise if it reacts too late at high speed
float SideClearance_cm = 6;    // how close things beside the car may get (beyond half the car width)


// =====================================================================
//  >>> CALIBRATION: found by testing (set SteeringTest = true) <<<
// =====================================================================

// [C1] Steering servo angle where the car drives STRAIGHT.
//      Find it by driving, not by looking at the axle (the linkage has play).
int SteerCenterAngle = 90;

// [C2] Steering servo angle that makes the car turn LEFT when driving forward,
//      at full lock. (Rear steering: the wheels will point the OTHER way.)
//      Use a value a few degrees short of where the servo starts buzzing.
int SteerLeftAngle = 150;

// [C3] Same as C2, for turning RIGHT.
int SteerRightAngle = 40;

// [C4] Distance-sensor servo angle where the sensor points straight ahead.
int SensorCenterAngle = 110;

// [C5] Photoresistor: cover it with your hand. Reading goes DOWN -> true, UP -> false.
bool DarkerMeansLower = false;

// [C6] Photoresistor: fraction of room brightness that counts as "in the garage".
//      Check the reading inside the actual box (test mode prints it).
float DarkFraction = 0.55;


// =====================================================================
//  OTHER SETTINGS (fine to leave as they are at first)
// =====================================================================

bool SteeringTest = false;     // true = run the test routine instead of driving
bool Debug        = false;     // true = print sensor readings while driving (slows it slightly)

int PanAngle    = 35;          // how far the sensor looks right/left while sweeping
int PanStepTime = 90;          // ms per sweep position

int FullSpeed    = 220;        // open road (0-255)
int TurnSpeed    = 180;        // while avoiding. Measure M1/M2/M5 at THIS speed
int ReverseSpeed = 200;
int GarageSpeed  = 150;
int KickTime     = 80;         // ms of full power when starting to move

float ReverseDist = 10;        // cm in front of the car: closer than this = back up
float RightBias   = 0.8;       // prefer right unless left is clearly more open
int   ReverseTime = 450;       // ms of reversing

int   LightPin         = A2;   // photoresistor (A0/A1 are used by the ultrasonic sensor)
float DimFraction      = 0.80; // "approaching the garage"
unsigned long DarkConfirmTime  = 150;
unsigned long GarageIgnoreTime = 500;
float GarageWallDistance = 10; // cm: dim + wall this close = inside the garage


// =====================================================================
//  CALCULATED FROM THE MEASUREMENTS (don't edit, see computeGeometry())
// =====================================================================
float needRight;               // distance ahead needed to dodge a can by turning right
float needLeft;                // ... by turning left
float AvoidDist;               // start dodging here (the larger of the two)
float ClearDist;               // slow down within this (room to turn 90 degrees away from a wall)
float SideDist;                // diagonal reading that triggers a nudge away
unsigned long MaxTurnRight;    // ms for a 90 degree turn
unsigned long MaxTurnLeft;


#include <Servo.h>

int pinLB = 6;
int pinLF = 7;
int pinRB = 8;
int pinRF = 9;
int pinMotor = 11;   // ENB, speed (PWM)
int echoPin  = A0;
int trigPin  = A1;

Servo panServo;      // pin 5, distance sensor
Servo steerServo;    // pin 10, rear axle steering

// Latest distances (cm from the FRONT of the car)
float distF = 300, distL = 300, distR = 300;

const int panPattern[4] = {0, -1, 0, 1};   // 0 front, -1 right, +1 left
int panIndex = 0;
unsigned long lastPanStep = 0;

int  turnDir = 0;            // 0 none, +1 right, -1 left
unsigned long turnStart = 0;
bool reversing = false;
unsigned long reverseStart = 0;
int  motorState = 0;         // 1 forward, -1 backward, 0 stopped

float lightBaseline = 0;
unsigned long darkSince = 0;
unsigned long startTime = 0;
unsigned long lastLightSample = 0;


// ======================= Geometry ====================================

// How far ahead of a can the car must start a full-lock turn to get past it.
// On a circle of radius R, moving forward d shifts you sideways by s,
// where d = sqrt(2*R*s - s*s).
float dodgeDistance(float R)
{
    float s = CanRadius_cm + CarWidth_cm / 2 + SafetyMargin_cm;   // sideways shift needed
    float d = (s < R) ? sqrt(2 * R * s - s * s) : R;
    return d + ReactionDist_cm;
}

void computeGeometry()
{
    needRight = dodgeDistance(RadiusRight_cm);
    needLeft  = dodgeDistance(RadiusLeft_cm);
    AvoidDist = max(needRight, needLeft);

    // Turning 90 degrees away from a wall straight ahead needs about R + half the width
    float bigR = max(RadiusRight_cm, RadiusLeft_cm);
    ClearDist = bigR + CarWidth_cm / 2 + SafetyMargin_cm + ReactionDist_cm;
    if (ClearDist < AvoidDist + 10) ClearDist = AvoidDist + 10;

    // A diagonal reading d at angle PanAngle is d*sin(PanAngle) to the side
    SideDist = (CarWidth_cm / 2 + SideClearance_cm) / sin(PanAngle * PI / 180.0);

    // A quarter of a full circle = 90 degrees
    MaxTurnRight = LapTimeRight_ms ? LapTimeRight_ms / 4 : 1200;
    MaxTurnLeft  = LapTimeLeft_ms  ? LapTimeLeft_ms  / 4 : 1200;
}

void printGeometry()
{
    Serial.println("--- Calculated values ---");
    Serial.print("Dodge distance right: "); Serial.println(needRight);
    Serial.print("Dodge distance left:  "); Serial.println(needLeft);
    Serial.print("AvoidDist:  "); Serial.println(AvoidDist);
    Serial.print("ClearDist:  "); Serial.println(ClearDist);
    Serial.print("SideDist:   "); Serial.println(SideDist);
    Serial.print("90 deg turn right (ms): "); Serial.println(MaxTurnRight);
    Serial.print("90 deg turn left  (ms): "); Serial.println(MaxTurnLeft);
}


// ======================= Motor & steering ============================

void brake()
{
    digitalWrite(pinRB, HIGH);
    digitalWrite(pinRF, HIGH);
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, HIGH);
    digitalWrite(pinMotor, HIGH);
    motorState = 0;
}

void forward(int speed)
{
    digitalWrite(pinRB, LOW);
    digitalWrite(pinRF, HIGH);
    digitalWrite(pinLB, LOW);
    digitalWrite(pinLF, HIGH);
    if (motorState != 1) {
        analogWrite(pinMotor, 255);
        motorState = 1;
        smartDelay(KickTime);
    }
    analogWrite(pinMotor, speed);
}

void backward(int speed)
{
    if (motorState == 1) { brake(); smartDelay(60); }
    digitalWrite(pinRB, HIGH);
    digitalWrite(pinRF, LOW);
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, LOW);
    if (motorState != -1) {
        analogWrite(pinMotor, 255);
        motorState = -1;
        smartDelay(KickTime);
    }
    analogWrite(pinMotor, speed);
}

// s = -1: car turns fully LEFT (going forward), 0 straight, +1 fully RIGHT
void steer(float s)
{
    s = constrain(s, -1.0, 1.0);
    int angle;
    if (s < 0) angle = SteerCenterAngle + (int)(-s * (SteerLeftAngle  - SteerCenterAngle));
    else       angle = SteerCenterAngle + (int)( s * (SteerRightAngle - SteerCenterAngle));
    steerServo.write(angle);
}


// ======================= Distance sensor =============================

// Returns the distance from the FRONT of the car (sensor setback subtracted)
float ping()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long d = pulseIn(echoPin, HIGH, 25000);
    if (d == 0) return 300;                       // no echo = nothing in range
    float cm = d / 58.0 - SensorSetback_cm;
    return cm < 0 ? 0 : cm;
}

void updateSensor()
{
    if (millis() - lastPanStep < PanStepTime) return;

    float d = ping();
    int p = panPattern[panIndex];
    if (p == 0)      distF = d;
    else if (p < 0)  distR = d;
    else             distL = d;

    if (Debug) {
        Serial.print("L "); Serial.print(distL);
        Serial.print("  F "); Serial.print(distF);
        Serial.print("  R "); Serial.println(distR);
    }

    panIndex = (panIndex + 1) % 4;
    panServo.write(SensorCenterAngle + panPattern[panIndex] * PanAngle);  // higher angle = left
    lastPanStep = millis();
}


// ======================= Garage detection ============================

int readLight()
{
    int v = analogRead(LightPin);
    return DarkerMeansLower ? v : 1023 - v;
}

void parkInGarage()
{
    brake();
    steer(0);
    Serial.println("GARAGE - STOPPED");
    while (true) { }
}

void checkGarage()
{
    if (millis() - lastLightSample < 10) return;
    lastLightSample = millis();

    int light = readLight();
    if (light >= lightBaseline * 0.85) {
        lightBaseline = lightBaseline * 0.995 + light * 0.005;
    }

    bool dark = light < lightBaseline * DarkFraction;
    if (!dark) { darkSince = 0; return; }
    if (millis() - startTime < GarageIgnoreTime) return;

    if (darkSince == 0) darkSince = millis();
    if (millis() - darkSince >= DarkConfirmTime) parkInGarage();
}

bool isDim()
{
    return readLight() < lightBaseline * DimFraction;
}

void smartDelay(unsigned long ms)
{
    unsigned long t0 = millis();
    while (millis() - t0 < ms) checkGarage();
}


// ======================= Decisions ===================================

// +1 right, -1 left. Prefers right (the car's strong side), but switches
// if the preferred side can't be made in the distance left and the other can.
int chooseSide()
{
    int side = (distR >= distL * RightBias) ? 1 : -1;
    bool leftFits  = distF >= needLeft  && distL > SideDist;
    bool rightFits = distF >= needRight && distR > SideDist;

    if (side == -1 && !leftFits && rightFits) side = 1;
    if (side == 1 && !rightFits && leftFits)  side = -1;
    return side;   // if neither fits it still tries; the reverse logic catches it if it gets too close
}

float sideNudge()
{
    float n = 0;
    if (distL < SideDist) n += (SideDist - distL) / SideDist;   // something left -> steer right
    if (distR < SideDist) n -= (SideDist - distR) / SideDist;   // something right -> steer left
    return constrain(n, -1.0, 1.0);
}


// ======================= Test mode ===================================

void runSteeringTest()
{
    delay(1000);
    printGeometry();
    Serial.println("Steering: car should turn LEFT");  steer(-1); delay(1500);
    Serial.println("Steering: center");                steer(0);  delay(1000);
    Serial.println("Steering: car should turn RIGHT"); steer(1);  delay(1500);
    steer(0);
    Serial.println("Sensor LEFT");  panServo.write(SensorCenterAngle + PanAngle); delay(1500);
    Serial.println("Sensor RIGHT"); panServo.write(SensorCenterAngle - PanAngle); delay(1500);
    panServo.write(SensorCenterAngle);
    Serial.println("Motor FORWARD"); forward(FullSpeed); delay(1000);
    brake();
    while (true) {
        Serial.print("Distance from front: "); Serial.print(ping());
        Serial.print("   Light: "); Serial.println(readLight());
        delay(200);
    }
}


// ======================= Setup & loop ================================

void setup()
{
    Serial.begin(115200);
    pinMode(pinLB, OUTPUT);
    pinMode(pinLF, OUTPUT);
    pinMode(pinRB, OUTPUT);
    pinMode(pinRF, OUTPUT);
    pinMode(pinMotor, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);

    panServo.attach(5);
    steerServo.attach(10);
    panServo.write(SensorCenterAngle);
    steer(0);

    computeGeometry();
    if (SteeringTest) runSteeringTest();
    if (Debug) printGeometry();

    long sum = 0;
    for (int i = 0; i < 10; i++) sum += readLight();
    lightBaseline = sum / 10.0;
    startTime = millis();
    lastPanStep = millis();

    forward(FullSpeed);
}

void loop()
{
    checkGarage();
    updateSensor();
    unsigned long now = millis();

    // 1. Finishing a reverse maneuver
    if (reversing) {
        if (now - reverseStart < ReverseTime) return;
        reversing = false;
        turnStart = now;
        distF = AvoidDist;           // stale reading: stay in "turning" until a fresh one arrives
    }

    // 2. Approaching the garage: don't avoid its walls, creep in and stop
    if (isDim()) {
        if (distF < GarageWallDistance) parkInGarage();
        turnDir = 0;
        steer(sideNudge());
        forward(GarageSpeed);
        return;
    }

    // 3. About to hit something: back up while swinging the nose toward the open side
    if (distF < ReverseDist) {
        turnDir = chooseSide();
        steer(-turnDir);             // reversing with "left" steering swings the nose right (and vice versa)
        backward(ReverseSpeed);
        reversing = true;
        reverseStart = now;
        return;
    }

    // 4. In the middle of a committed turn
    if (turnDir != 0) {
        steer(turnDir);
        forward(TurnSpeed);
        unsigned long maxTurn = (turnDir == 1) ? MaxTurnRight : MaxTurnLeft;
        if (distF > ClearDist || now - turnStart > maxTurn) turnDir = 0;
        return;
    }

    // 5. Obstacle ahead within dodging distance: pick a side and commit
    if (distF < AvoidDist) {
        turnDir = chooseSide();
        turnStart = now;
        steer(turnDir);
        forward(TurnSpeed);
        return;
    }

    // 6. Open road: drive, nudging away from things on the diagonals.
    //    Slow down when a wall is closer than the room needed to turn away from it.
    steer(sideNudge());
    forward(distF < ClearDist ? TurnSpeed : FullSpeed);
}
