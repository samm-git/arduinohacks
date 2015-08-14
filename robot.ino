/* code for the robotic toy from 
   http://www.dx.com/cs/p/diy-intelligent-tortoise-smart-wheel-robot-module-173668
   Based on the source code from the CD, license and author unknown. 
   Some modification by Alex Samorukov
*/

#include <IRremote.h>
#include <Servo.h>
// Motor pins
const int MotorRight1 = 5;
const int MotorRight2 = 6;
const int MotorLeft1 = 10;
const int MotorLeft2 = 11;

int counter = 0;
const int irReceiverPin = 2; // OUTPUT signal of the infrared receiver is connected to the pin 2

// Remote control scan codes
const long IRfront = 0x511DBB; // forward code
const long IRback =  0xA3C8EDDB; // Back
const long IRturnright = 0x20FE4DBB; // turn right
const long IRturnleft = 0x52A3D41F; // turn left
const long IRstop = 0xD7E84B1B; // Stop, button "OK"
const long IRAutorun = 0xC101E57B; // ultrasound self-driven mode, button "1"
const long IRcny70 = 0x97483BFB; // CNY70 self-driven mode, button "2"
const long IRturnsmallleft = 0x00FF22DD; // not implemented?
IRrecv irrecv(irReceiverPin); // the definition IRrecv object to receive infrared signals

// Optical Reflex Sensors pins
const int SensorLeft = 7; // left sensor input pin
const int SensorMiddle = 4; //  middle sensor input pin
const int SensorRight = 3; // right sensor input pin
int SL; // left sensor status
int SM; // in the sensor state
int SR; // right sensor status

decode_results results; // decoding results on decode_results structure result variable
// ************************* Definition ultrasound pin ****************** ************
const int inputPin = 15; // define the ultrasound signal reception pin rx
const int outputPin = 14; // define the ultrasonic signal transmitter pin tx
int Fspeedd = 0; // in front of the distance
int Rspeedd = 0; // the right distance
int Lspeedd = 0; // left distance
int direction = 0; // = 8 = 2 Left = Right = 6

Servo myservo; // set myservo
int delay_time = 250; // servo motor steering stability time
const int Fgo = 8; // forward
const int Rgo = 6; // turn right
const int Lgo = 4; // turn left
const int Bgo = 2; // reversing
// ************************************************ ******************** (SETUP)
void setup()
{
  Serial.begin(9600);
  // set motor pins mode
  pinMode(MotorRight1, OUTPUT); // pin 8 (PWM)
  pinMode(MotorRight2, OUTPUT); // pin 9 (PWM)
  pinMode(MotorLeft1, OUTPUT); // pin 10 (PWM)
  pinMode(MotorLeft2, OUTPUT); // pin 11 (PWM)

  irrecv.enableIRIn(); // start infrared decoding
  pinMode(SensorLeft, INPUT); // define left sensor mode
  pinMode(SensorMiddle, INPUT); // define middle sensor mode
  pinMode(SensorRight, INPUT); // define right sensor mode
  digitalWrite(2, HIGH);
  pinMode(inputPin, INPUT); // definition ultrasound input pin
  pinMode(outputPin, OUTPUT); // definition ultrasound output pin
  myservo.attach(9); // define the servo motor output 5th pin (PWM)
}

void advance(int a) // forward
{
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, HIGH);
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, HIGH);
  delay(a * 100);
}

void right(int b) // turn right (single wheel)
{
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, HIGH);
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, LOW);
  delay(b * 100);
}
void left(int c) // turn left (single wheel)
{
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, HIGH);
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, LOW);
  delay(c * 100);
}
void turnR(int d) // turn right (two-wheeled)
{
  digitalWrite(MotorRight1, HIGH);
  digitalWrite(MotorRight2, LOW);
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, HIGH);
  delay(d * 100);
}
void turnL(int e) // turn left (two-wheeled)
{
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, HIGH);
  digitalWrite(MotorLeft1, HIGH);
  digitalWrite(MotorLeft2, LOW);
  delay(e * 100);
}
void stop(int f) // stop
{
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, LOW);
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, LOW);
  delay(f * 100);
}
void back(int g) // Back
{
  digitalWrite(MotorRight1, HIGH);
  digitalWrite(MotorRight2, LOW);
  digitalWrite(MotorLeft1, HIGH);
  digitalWrite(MotorLeft2, LOW);;
  delay(g * 100);
}


void detection() // measuring three angles(front left right)
{
  int delay_time = 250; // servo motor steering stability time
  ask_pin_F(); // read in front of the distance

  if(Fspeedd < 10) // If the front of the distance is less than 10 cm
  {
    stop(1); // Clear the output data
    back(2); // back 0.2 seconds
  }
  if(Fspeedd < 25) // If the front of the distance is less than 25 cm
  {
    stop(1); // Clear the output data
    ask_pin_L(); // read the left distance
    delay(delay_time); // wait for the servo motor stable
    ask_pin_R(); // read the right distance
    delay(delay_time); // wait for the servo motor stable

    if(Lspeedd > Rspeedd) // if the left distance is greater than the right distance
    {
      direction = Lgo; // go left
    }

    if(Lspeedd <= Rspeedd) // if the left distance is less than or equal to the right side of the distance
    {
      direction = Rgo; // right away
    }

    if(Lspeedd < 15 && Rspeedd < 15) // if the left distance and right distance are less than 10 cm
    {
      direction = Bgo; // walk backward
    }
  }
  else // add, such as in front of more than 25 cm
  {
    direction = Fgo; // forward
  }
}

// ************************************************ *********************************
void ask_pin_F() // measure out the front of the distance
{
  myservo.write(90);
  digitalWrite(outputPin, LOW); // let the ultrasonic transmitter Low Voltage 2¶Ãs
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH); // ultrasonic transmitting high voltage 10¶Ãs, at least 10¶Ãs
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW); // maintain ultrasonic transmitter low voltage
  float Fdistance = pulseIn(inputPin, HIGH); // read differential difference
  Fdistance = Fdistance / 5.8 / 10; // time to distance distance (unit: cm)
  Serial.print("F distance:"); // output distance (unit: cm)
  Serial.println(Fdistance); // display distance
  Fspeedd = Fdistance; // distance read Fspeedd (speed)
}

// ************************************************ ********************************
void ask_pin_L() // measure out the left distance
{
  myservo.write(177);
  delay(delay_time);
  digitalWrite(outputPin, LOW); // let the ultrasonic transmitter Low Voltage 2¶Ãs
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH); // ultrasonic transmitting high voltage 10¶Ãs, at least 10¶Ãs
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW); // maintain ultrasonic transmitter low voltage
  float Ldistance = pulseIn(inputPin, HIGH); // read differential difference
  Ldistance = Ldistance / 5.8 / 10; // time to distance distance (unit: cm)
  Serial.print("L distance:"); // output distance (unit: cm)
  Serial.println(Ldistance); // display distance
  Lspeedd = Ldistance; // distance reads Lspeedd (left-speed)
}
// ************************************************ ******************************
void ask_pin_R() // measure out the right side of the distance
{
  myservo.write(5);
  delay(delay_time);
  digitalWrite(outputPin, LOW); // let the ultrasonic transmitter Low Voltage 2¶Ãs
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH); // ultrasonic transmitting high voltage 10¶Ãs, at least 10¶Ãs
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW); // maintain ultrasonic transmitter low voltage
  float Rdistance = pulseIn(inputPin, HIGH); // read differential difference
  Rdistance = Rdistance / 5.8 / 10; // time to distance distance (unit: cm)
  Serial.print("R distance: "); // output distance (unit: cm)
  Serial.println(Rdistance); // display distance
  Rspeedd = Rdistance; // distance read into Rspeedd (right speed)
}
void loop()
{
  SL = digitalRead(SensorLeft);
  SM = digitalRead(SensorMiddle);
  SR = digitalRead(SensorRight);
  // ************************************************ *************************** normal remote mode
  if(irrecv.decode(& results))
  { // Decoding successful, receive a set of infrared signals
    // ************************************************* ********************** /
    Serial.println(results.value, HEX);
    if(results.value == IRfront) // forward
    {
      advance(10);// forward
    }
    // ************************************************* ********************** /
    if(results.value == IRback) // Back
    {
      back(10);// Back
    }
    // ************************************************* ********************** /
    if(results.value == IRturnright) // right turn
    {
      right(6); // turn right
    }
    // ************************************************* ********************** /
    if(results.value == IRturnleft) // left turn
    {
      left(6); // turn left);
    }
    // ************************************************* ********************** /
    if(results.value == IRstop) // stop
    {
      digitalWrite(MotorRight1, LOW);
      digitalWrite(MotorRight2, LOW);
      digitalWrite(MotorLeft1, LOW);
      digitalWrite(MotorLeft2, LOW);
    }
    // ************************************************ black self-propelled mode the *********************** cny70 mode: LOW white:
    if(results.value == IRcny70)
    {
      while(IRcny70)
      {
        SL = digitalRead(SensorLeft);
        SM = digitalRead(SensorMiddle);
        SR = digitalRead(SensorRight);

        if(SM == HIGH) // in the sensor in the black areas
        {
          if(SL == LOW & SR == HIGH) // left black right white to turn left
          {
            digitalWrite(MotorRight1, LOW);
            digitalWrite(MotorRight2, HIGH);
            analogWrite(MotorLeft1, 0);
            analogWrite(MotorLeft2, 80);
          }
          else if(SR == LOW & SL == HIGH) // left white right black, turn right
          {
            analogWrite(MotorRight1, 0); // turn right
            analogWrite(MotorRight2, 80);
            digitalWrite(MotorLeft1, LOW);
            digitalWrite(MotorLeft2, HIGH);
          }
          else // both sides are white, straight
          {
            digitalWrite(MotorRight1, LOW);
            digitalWrite(MotorRight2, HIGH);
            digitalWrite(MotorLeft1, LOW);
            digitalWrite(MotorLeft2, HIGH);
            analogWrite(MotorLeft1, 200);
            analogWrite(MotorLeft2, 200);
            analogWrite(MotorRight1, 200);
            analogWrite(MotorRight2, 200);
          }
        }
        else // in the sensor in the white areas
        {
          if(SL == LOW & SR == HIGH) // white left black right, quick left turn
          {
            digitalWrite(MotorRight1, LOW);
            digitalWrite(MotorRight2, HIGH);
            digitalWrite(MotorLeft1, LOW);
            digitalWrite(MotorLeft2, LOW);
          }
          else if(SR == LOW & SL == HIGH) // left white right black, quick right turn
          {
            digitalWrite(MotorRight1, LOW);
            digitalWrite(MotorRight2, LOW);
            digitalWrite(MotorLeft1, LOW);
            digitalWrite(MotorLeft2, HIGH);
          }
          else // are white, stop
          {
            digitalWrite(MotorRight1, HIGH);
            digitalWrite(MotorRight2, LOW);
            digitalWrite(MotorLeft1, HIGH);
            digitalWrite(MotorLeft2, LOW);;
          }
        }
        if(irrecv.decode(& results))
        {
          irrecv.resume();
          Serial.println(results.value, HEX);
          if(results.value == IRstop)
          {
            digitalWrite(MotorRight1, HIGH);
            digitalWrite(MotorRight2, HIGH);
            digitalWrite(MotorLeft1, HIGH);
            digitalWrite(MotorLeft2, HIGH);
            break;
          }
        }
      }
      results.value = 0;
    }
    // ************************************************ the *********************** ultrasound self-propelled mode
    if(results.value == IRAutorun)
    {
      while(IRAutorun)
      {
        myservo.write(90); // the servo motors regression ready position ready for the next time measurement
        detection(); // measure the angle and the judgment to where to move in one direction
        if(direction == 8) // if direction(direction) = 8 (forward)
        {
          if(irrecv.decode(& results))
          {
            irrecv.resume();
            Serial.println(results.value, HEX);
            if(results.value == IRstop)
            {
              digitalWrite(MotorRight1, LOW);
              digitalWrite(MotorRight2, LOW);
              digitalWrite(MotorLeft1, LOW);
              digitalWrite(MotorLeft2, LOW);
              break;
            }
          }
          results.value = 0;
          advance(1); // normal ahead
          Serial.print("Advance"); // display direction(forward)
          Serial.print("");
        }
        if(direction == 2) // if direction (direction) = 2 (reverse)
        {
          if(irrecv.decode(& results))
          {
            irrecv.resume();
            Serial.println(results.value, HEX);
            if(results.value == IRstop)
            {
              digitalWrite(MotorRight1, LOW);
              digitalWrite(MotorRight2, LOW);
              digitalWrite(MotorLeft1, LOW);
              digitalWrite(MotorLeft2, LOW);
              break;
            }
          }
          results.value = 0;
          back(8); // reverse (car)
          turnL(3); // move slightly to the left (to prevent stuck in a dead alley)
          Serial.print("Reverse"); // display direction (reverse)
        }
        if(direction == 6) // if direction (direction) = 6 (turn right)
        {
          if(irrecv.decode(& results))
          {
            irrecv.resume();
            Serial.println(results.value, HEX);
            if(results.value == IRstop)
            {
              digitalWrite(MotorRight1, LOW);
              digitalWrite(MotorRight2, LOW);
              digitalWrite(MotorLeft1, LOW);
              digitalWrite(MotorLeft2, LOW);
              break;
            }
          }
          results.value = 0;
          back(1);
          turnR(6); // turn right
          Serial.print("Right"); // display direction (turn left)
        }
        if(direction == 4) // if direction (direction) = 4 (turn left)
        {
          if(irrecv.decode(& results))
          {
            irrecv.resume();
            Serial.println(results.value, HEX);
            if(results.value == IRstop)
            {
              digitalWrite(MotorRight1, LOW);
              digitalWrite(MotorRight2, LOW);
              digitalWrite(MotorLeft1, LOW);
              digitalWrite(MotorLeft2, LOW);
              break;
            }
          }
          results.value = 0;
          back(1);
          turnL(6); // turn left
          Serial.print("Left"); // display direction (turn right)
        }

        if(irrecv.decode(& results))
        {
          irrecv.resume();
          Serial.println(results.value, HEX);
          if(results.value == IRstop)
          {
            digitalWrite(MotorRight1, LOW);
            digitalWrite(MotorRight2, LOW);
            digitalWrite(MotorLeft1, LOW);
            digitalWrite(MotorLeft2, LOW);
            break;
          }
        }
      }
      results.value = 0;
    }
    // ************************************************* ********************** /
    else
    {
      digitalWrite(MotorRight1, LOW);
      digitalWrite(MotorRight2, LOW);
      digitalWrite(MotorLeft1, LOW);
      digitalWrite(MotorLeft2, LOW);
    }
    irrecv.resume(); // continue to the next set of infrared signals received
  }
}