// User-defined variables:
// OBS: men mounting servo during robot assembly, use SensorCenterAngle = 90;

int SensorCenterAngle  = 110;    //Angle when "distance sensor" points straight ahead. OBS: between 80-100 !
int SensorTurningAngle = 70;    //Angle the "distance sensor" turns relative to center

int AlarmDistanceClose = 25;    // Very close!
int AlarmDistanceFar   = 40;    // Close but no cigar!
int CheckRouteTime     = 2000;  //How often the Robot should stop and check if any obstacle has arrived from right or left

int ForwardWheelSpeed  = 120;   // min-max = 0-255 Choose an even number = possible to divide by 2
int ForwardSteerServoAngle  = 90;   // Fine-tune the steering heading = 80-100 Choose an even number

int TurnWheelSpeed     = ForwardWheelSpeed-0; // min-max = 0-255 Choose an even number = possible to divide by 2
int BackwardWheelSpeed = ForwardWheelSpeed-0; // min-max = 0-255 Choose an even number = possible to divide by 2
int TurnTime           = 4;     // Time for turning => turning angle of vehicle. Use ~0-10
    

    test





// -----------------------------------------------------------------------------------
// Here starts the main code:
#include <Servo.h>
int pinLB=6;          // define pin6 as left back connect with IN1
int pinLF=7;          // define pin9 as left forward connect with IN2
int pinRB=8;          // define pin10 as right back connect with IN3
int pinRF=9;          // define pin11 as right back connect with IN4
int pinMotor = 11;    // Pin that controls the motor: ENB - Green cord
int inputPin = A0;    // define ultrasonic receive pin (Echo)
int outputPin =A1;    // define ultrasonic send pin(Trig)
int Fspeedd = 0;      // forward distance
int Rspeedd = 0;      // right distance
int Lspeedd = 0;      // left distance
int directionn = 0;   //


#define SERVOS 2      // Define the number of servos
//#define STATES 7    // Define the number of states
Servo myservo[SERVOS];// new myservo
int servo_pins[SERVOS] = {5,10}; // means that two servos are controlled on port 5 and 10
                                 // myservo[0] will be on port 5, and myservo[1] will be on port 10, 
                                 // (and so on if more servos will be attached...)
int delay_time = 250; // set stable time
int Fgo = 8;          // forward
int Rgo = 6;          // turn right
int Lgo = 4;          // turn left
int Bgo = 2;          // back
unsigned long DriveTime;    // Timevariables to control how often to check for directions
unsigned long DriveTimeOld; // Timevariables to control how often to check for directions
int RightSensorAngle   = SensorCenterAngle-SensorTurningAngle; //Angle the distance sensor turns to the right (90 is straight ahead)
int LeftSensorAngle    = SensorCenterAngle+SensorTurningAngle; //Angle the distance sensor turns to the left  (90 is straight ahead)

void setup()
{
    Serial.begin(9600);
    pinMode(pinLB,OUTPUT);
    pinMode(pinLF,OUTPUT);
    pinMode(pinRB,OUTPUT);
    pinMode(pinRF,OUTPUT);
    pinMode(pinMotor,OUTPUT);
    pinMode(inputPin, INPUT);
    pinMode(outputPin, OUTPUT);
    //myservo[0].attach(5); // define the servo pin(PWM) for ultrasonic sensor
    //myservo[1].attach(10); // define the servo pin(PWM) for ultrasonic sensor
        for(int i = 0; i < SERVOS; i++) {
            myservo[i].attach(servo_pins[i]); // Attach the servo to the servo object 
            delay(500);
         }  
    DriveTimeOld = millis(); // initiate start time
    CheckRouteTime         = CheckRouteTime+1000; //Adding one second because it takes roughly one second to check left and right and continue to drive

    //Forward motion directly from the start:
    digitalWrite(pinRB,LOW);
    digitalWrite(pinRF,HIGH);
    digitalWrite(pinLB,LOW);
    digitalWrite(pinLF,HIGH);
    analogWrite(pinMotor,ForwardWheelSpeed);
    delay(150);
   

}

void advance(int a) // forward
{
    myservo[1].write(ForwardSteerServoAngle);
    delay(250);
    digitalWrite(pinRB,LOW);
    digitalWrite(pinRF,HIGH);
    digitalWrite(pinLB,LOW);
    digitalWrite(pinLF,HIGH);
    analogWrite(pinMotor,ForwardWheelSpeed);
    delay(a * 15);
}

void turnR(int d) //turn right
{
    myservo[1].write(150);
    delay(250);        
    digitalWrite(pinRB,HIGH);
    digitalWrite(pinRF,LOW);
    digitalWrite(pinLB,HIGH);
    digitalWrite(pinLF,LOW);
    analogWrite(pinMotor,TurnWheelSpeed);
    delay(d * 50);
}

void turnL(int e) //turn left
{
    myservo[1].write(40);
    delay(250);   
    digitalWrite(pinRB,HIGH);
    digitalWrite(pinRF,LOW);
    digitalWrite(pinLB,HIGH);
    digitalWrite(pinLF,LOW);
    analogWrite(pinMotor,TurnWheelSpeed);
    delay(e * 50);
}

void stopp(int f) //stop
{
    myservo[1].write(ForwardSteerServoAngle);
    delay(250);       
    digitalWrite(pinRB,HIGH);
    digitalWrite(pinRF,HIGH);
    digitalWrite(pinLB,HIGH);
    digitalWrite(pinLF,HIGH);
    digitalWrite(pinMotor,HIGH);
    delay(f * 100);
}

void back(int g) //back
{
    myservo[1].write(ForwardSteerServoAngle);
    delay(250);       
    analogWrite(pinMotor,BackwardWheelSpeed);
    digitalWrite(pinRB,HIGH);
    digitalWrite(pinRF,LOW);
    digitalWrite(pinLB,HIGH);
    digitalWrite(pinLF,LOW);
    delay(g * 300  /2);
}

void detection() //test the distance in different directions
{
          int delay_time = 250; // An delay for the detection of obstacles to the sides..
          
       // This is an add-on detection algorithm:
       // "Stop now and then and check right and left, 
       // to ensure that no obstacle has been emerging to
       // the left or right
       delay(5);
       DriveTime = millis();
       if (DriveTime > DriveTimeOld + CheckRouteTime)
          {       
          Serial.println("");  //Line break...
          stopp(1);
          ask_pin_L();
          delay(delay_time);
          ask_pin_R();
          delay(delay_time);

          if(Lspeedd < AlarmDistanceClose && Rspeedd > AlarmDistanceClose) //if left distance is "Too close"
                {
                turnL(TurnTime*2);
                //back(1);
                Serial.println(" Turning Right ");
                }
            if(Rspeedd < AlarmDistanceClose && Lspeedd > AlarmDistanceClose)//if left distance is "Too close"
                {
                turnR(TurnTime*2);
                //back(1);
                Serial.println(" Turning Left ");
                } 
             if (Lspeedd < AlarmDistanceClose && Rspeedd < AlarmDistanceClose)//if left distance and right distance both less than 1
                {
                back(3);
                Serial.println(" Reverse ");
                }
             else
                 {
                 directionn = Fgo; // forward go
                 }
          DriveTimeOld = DriveTime;
          }
        
        // This is the ~original detection algorithm: 
        // "Drive forward until you reach an obstacle in front of you"
        delay(200);
        ask_pin_F(); // read forward distance
        if(Fspeedd < AlarmDistanceClose) // if distance less then "AlarmDistanceClose"
            {
            stopp(1);
            back(2);
            }
        if(Fspeedd < AlarmDistanceFar) // if distance less then "AlarmDistanceFar"
            {
            stopp(1);
            ask_pin_L();
            delay(delay_time);
            ask_pin_R();
            delay(delay_time);
            
            if(Lspeedd > Rspeedd) //if left distance more than right distance
                {
                directionn = Lgo;
                DriveTimeOld = DriveTime;
                }
            if(Lspeedd <= Rspeedd)//if left distance not more than right distance
                {
                directionn = Rgo;
                DriveTimeOld = DriveTime;
                } 
             if (Lspeedd < AlarmDistanceClose && Rspeedd < AlarmDistanceClose)//if left distance and right distance both less than 1
                {
                directionn = Bgo;
                DriveTimeOld = DriveTime;
                }
              }
        else
            {
            directionn = Fgo; // forward go
            }
}


void ask_pin_F() // test forward distance
{
    myservo[0].write(SensorCenterAngle);
    digitalWrite(outputPin, LOW);
    delayMicroseconds(2);
    digitalWrite(outputPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(outputPin, LOW);
    float Fdistance = pulseIn(inputPin, HIGH);
    Fdistance= Fdistance/5.8/10;
    Serial.print(" F distance= ");
    Serial.println(Fdistance);
    Fspeedd = Fdistance;
}


void ask_pin_L() // test left distance
{
    myservo[0].write(LeftSensorAngle);
    delay(delay_time);
    digitalWrite(outputPin, LOW);
    delayMicroseconds(2);
    digitalWrite(outputPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(outputPin, LOW);
    float Ldistance = pulseIn(inputPin, HIGH);
    Ldistance= Ldistance/5.8/10;
    Serial.print(" L distance= ");
    Serial.println(Ldistance);
    Lspeedd = Ldistance;
}


void ask_pin_R() // test right distance
{
    myservo[0].write(RightSensorAngle);
    delay(delay_time);
    digitalWrite(outputPin, LOW);
    delayMicroseconds(2);
    digitalWrite(outputPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(outputPin, LOW);
    float Rdistance = pulseIn(inputPin, HIGH);
    Rdistance= Rdistance/5.8/10;
    Serial.print(" R distance= ");
    Serial.println(Rdistance);
    Rspeedd = Rdistance;
}



void loop()  // Main loop from where the program is run. 
{   
    myservo[0].write(SensorCenterAngle);
    detection();
    if(directionn == 2)
         {
         back(3);
         turnR(2);
         Serial.print(" Reverse ");
         }
    if(directionn == 6)
        {
        back(1);
        turnL(TurnTime);
        Serial.print(" Turning Right ");
        }
    if(directionn == 4)
        {
        back(1);
        turnR(TurnTime);
        Serial.print(" Turning Left ");
        }
    if(directionn == 8)
        {
        advance(1);
        Serial.print(" Forward ");
        Serial.print(" ");
        }
}
