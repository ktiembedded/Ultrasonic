#include <Arduino.h>

#define Baud_rate 115200          //communication speed

char Char;
// ----- HC-SR04 ultrasonic transducer
#define Trig 5
#define Echo 6
#define RawEcho 7                 //extra wire connected to pin 10 (see text)
#define buzzer 3
// ----- motor controller definitions
/*
   Connect your motor controller pins IN1..IN4 to the following Arduino pins.
   The Arduino "talks" directly to controller pins IN1..IN4 via PORTB.
*/
#define IN4  11
#define IN3  10
#define IN2  9
#define IN1  8



// ----- motor pattern
byte Motor[8] =                  //half-stepping
{ 
  B00001000,
  B00001100,
  B00000100,
  B00000110,
  B00000010,
  B00000011,
  B00000001,
  B00001001
};

uint8_t Index = 0;                  //Motor[] array index
uint16_t Step_counter = 0;           //180 degrees requires 2048 steps
unsigned long Delay = 2;        //give motor shaft time to move
byte Pattern;                   //Motor[] pattern
boolean outputTone = false; 
// ----- acoustic "radar" display data
int Azimuth = 0;                //Azimuth (PI/128 radians) measured CCW from reference
uint16_t Distance1 = 0;
uint16_t Distance2 = 0;
uint8_t Direction = 0;          //counter-clockwise=0, clockwise=1
unsigned long previousMillis = 0;
uint8_t Speed_of_rotation = 30;         //controls beam rotation: 1 = fastest
long currentMillis;

// ===============================
// connect to graphics display
// ===============================
void connect_to_display()
{
  while (Serial.available() <= 0)
  {
    // ----- keep sending synch ('S') until the display responds
    Serial.println("S");
    
    delay(250);
  }
}

// ===============================
// measure distances
// ===============================
void measure()
{
  // ----- locals
  unsigned long start_time;           //microseconds
  unsigned long finish_time;          //microseconds
  unsigned long time_taken;           //microseconds
  unsigned long timeout;              //microseconds
  unsigned long pause;                //microseconds
  boolean flag;

  // ----- generate 10uS start pulse
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  // ----- wait for pulse(s) to be sent
  while (!digitalRead(Echo));                 //wait for high
  start_time = micros();

  // ----- set timeout radius
  timeout = start_time + 12000;               //set timeout radius to 2 meters

  // ----- measure first object distance
  flag = false;
  while (!flag)
  {
    if (!digitalRead(Echo)) flag = true;      //exit loop if object detected
    if (timeout < micros()) flag = true;      //exit loop if timeout exceeded
  }
  finish_time = micros();

  // ----- calculate first object distance(cm)
  time_taken = finish_time - start_time;
  Distance1 = ((float)time_taken) / 59;

  // ----- wait for first object echo to finish
  pause = finish_time + 1000;                 //1000uS means 17cm closest object spacing
  while (pause > micros());                   //wait 1000uS

  // ----- measure second object distance
  flag = false;
  while (!flag)                               //wait for high
  {
    if (digitalRead(RawEcho)) flag = true;    //exit loop if object dectected
    if (timeout < micros()) flag = true;      //exit loop if timeout exceeded
  }
  finish_time = micros();

  // ----- calculate second object distance (cm)
  time_taken = finish_time - start_time;
  Distance2 = ((float)time_taken) / 59;
}

// ===============================
// rotate motor to next ping position
// ===============================
void rotate()
{
  // ----- counter-clockwise scan
  if (Direction == 0)
  {
    for (int i = 0; i < 8; i++)
    {
      // ----- rotate motor to next ping position
      Index = Step_counter % 8;                 //calculate array index
      Pattern = PORTB;                          //get current motor pattern
      Pattern = Pattern & B11110000;            //preserve MSN
      Pattern = Pattern | Motor[Index];         //create new motor pattern
      PORTB = Pattern;                          //send new pattern to motor
      Step_counter++;
      delay(Delay);                             //controls motor speed (fastest=1)
    }

    // ----- loop control
    Azimuth++;
    if (Azimuth > 256)
    {
      Azimuth = 256;
      Direction = 1;
      Step_counter = 2048;
    }
  }
  else
  {
    // ----- clockwise scan
    for (int i = 0; i < 8; i++)
    {
      // ----- rotate motor to next ping position
      Index = Step_counter % 8;                 //calculate array index
      Pattern = PORTB;                          //get current motor pattern
      Pattern = Pattern & B11110000;            //preserve MSN
      Pattern = Pattern | Motor[Index];         //create new motor pattern
      PORTB = Pattern;                          //send new pattern to motor
      Step_counter--;
      delay(Delay);                             //controls motor speed (fastest=1)
    }

    // ----- loop control
    Azimuth--;                                  //decrement Azimuth every 8 steps
    if (Azimuth < 0)
    {
      Azimuth = 0;
      Direction = 0;
      Step_counter = 0;
    }
  }
}

void startBuzzer() {
  if (outputTone) {
    if (currentMillis - previousMillis >= Distance1) {
      previousMillis = currentMillis;
      noTone(buzzer);
      outputTone = false;
    }
  } 
  else {
    if (currentMillis - previousMillis >= Distance1*4) {
      previousMillis = currentMillis;
      tone(buzzer, 1000);
      outputTone = true;
    }
  }
}

void setup()
{
  // ----- configure serial port
  Serial.begin(Baud_rate);

  // ----- configure arduino pinouts
  pinMode(Echo, INPUT);               //make Echo pin an input
  pinMode(RawEcho, INPUT);            //make RawEcho pin an input
  pinMode(Trig, OUTPUT);              //set Trig pin LOW
  digitalWrite(Trig, LOW);

  // ----- configure stepper motor
  Pattern = DDRB;                       // get PORTB data directions
  Pattern = Pattern | B00001111;        // preserve MSN data direction &
  DDRB = Pattern;                       // make pins 8,9,10,11 outputs

  // ----- rotate beam to start-up position
  // ----- attach the graphics display
  pinMode(buzzer,OUTPUT);
  connect_to_display();                 //connect to the display
}

// ======================
// loop
// ======================
void loop()
{
  // delay(cm*3);
  // ----- has the display asked for data
  if (Serial.available() > 0)
  {
    Char = Serial.read();               // read character

    // ----- send data to display whenever a send character ('S') is received
    if (Char == 'S')
    {
      currentMillis = millis();
      // ----- measure distances
      measure();

      // ----- rotate beam to next ping position
      rotate();

      // ----- send the results to the display
      Serial.print(Azimuth);
      Serial.print(',');
      Serial.print(Distance1);
      Serial.print(',');
      Serial.print(Distance2);
      Serial.print(',');
      Serial.println(Direction);
      startBuzzer();
      delay(Speed_of_rotation);        //slows rotational speed
    }
  }
}
