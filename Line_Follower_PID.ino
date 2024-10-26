#define F_CPU 16000000UL  // Set to 16 MHz

#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12
#define IR_SENSOR_CENTER 4
#define LOW_MOTOR_SPEED 190
#define BASE_MOTOR_SPEED 220

//Right motor
int enableRightMotor=5;
int rightMotorPin1=9;
int rightMotorPin2=10;

//Left motor
int enableLeftMotor=6;
int leftMotorPin1=7;
int leftMotorPin2=8;

float Kp=50;
float Ki=0.5;
float Kd=10;
int lastError = 0;
float integral = 0;

int L2 = 0;
int L3 = 0;
int L4 = 0; 
int L5 = 0; 
int R2 = 0;
int R3 = 0;
int R4 = 0;
int R5 = 0;
int errorcount=0;
int maxcount=3;
int maxtime=500;
unsigned long lasterrortime = 0;

void setup()
{
  //The problem with TT gear motors is that, at very low pwm value it does not even rotate.
  //If we increase the PWM value then it rotates faster and our robot is not controlled in that speed and goes out of line.
  //For that we need to increase the frequency of analogWrite.
  //Below line is important to change the frequency of PWM signal on pin D5 and D6
  //Because of this, motor runs in controlled manner (lower speed) at high PWM value.
  //This sets frequecny as 7812.5 hz.
  TCCR0B = TCCR0B & B11111000 | B00000001; // Set prescaler to 8 
  
  // new lines from here

  
  TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1); // Fast PWM, clear OC0A on compare match

  // Configure Timer 1 for CTC mode
  TCCR1A = 0; // Normal operation
  TCCR1B = (1 << WGM12) | (1 << CS12); // CTC mode, prescaler = 256
  
  // Set compare match value for 1-second interval
  OCR1A = 625; // (16MHz / 256) = 625 counts for 1 second
  
  // Enable Timer 1 Compare Match A interrupt
  TIMSK1 |= (1 << OCIE1A);

  
  DDRB |= (1 << PD7);  // Set pin 5 as output
  DDRB |= (1 << PD6);  // Set pin 6 as output
  DDRB |= (1 << PD7);  // Set pin 7 as output
  DDRB |= (1 << PB0);  // Set pin 8 as output
  DDRB |= (1 << PB1);  // Set pin 9 as output
  DDRB |= (1 << PB2);  // Set pin 10 as output

  DDRD &= ~((1 << PD4) | (1 << PB3) | (1 << PB4)); // sets pins 4,11,12 as input pins

  // new lines till here


  
  // put your setup code here, to run once:
//  pinMode(enableRightMotor, OUTPUT);
//  pinMode(rightMotorPin1, OUTPUT);
//  pinMode(rightMotorPin2, OUTPUT);
//  
//  pinMode(enableLeftMotor, OUTPUT);
//  pinMode(leftMotorPin1, OUTPUT);
//  pinMode(leftMotorPin2, OUTPUT);
//
//  pinMode(IR_SENSOR_RIGHT, INPUT);
//  pinMode(IR_SENSOR_LEFT, INPUT);
//  pinMode(IR_SENSOR_CENTER, INPUT);
 
    rotateMotor(0,0);   
}


void loop()
{ 
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
  int centerIRSensorValue = digitalRead(IR_SENSOR_CENTER);
  
  int error = (leftIRSensorValue && !rightIRSensorValue) || (!leftIRSensorValue && rightIRSensorValue);
  int propotional = error; 
  integral += error; 
  int derivative = error - lastError;
  
  float correction = constrain((Kp* propotional + Ki* integral + Kd* derivative),0,80) ;
  lastError = error;


  //DIRECTING BOT USING LAST 5 ERRORS
  int E1 = (L5 - R5);  
  
  L5 = L4;
  L4 = L3;
  L3 = L2;
  L2 = leftIRSensorValue;

  R5 = R4;
  R4 = R3;
  R3 = R2;
  R2 = rightIRSensorValue;

  int MOTOR_SPEED = constrain(BASE_MOTOR_SPEED - correction , 160 , 190);
  
  if ((millis() - lasterrortime < maxtime) && (error = lastError) && error != 0){
    errorcount++; 
  }else{
    errorcount = 0;
  }

  
 if (errorcount > maxcount)
  { MOTOR_SPEED = MOTOR_SPEED;
    delay(800); 
  }
  
  else {
  //If all of the sensors detects black line, then go straight
  if (rightIRSensorValue == HIGH && leftIRSensorValue == HIGH && centerIRSensorValue == HIGH)
  {
    rotateMotor(BASE_MOTOR_SPEED, BASE_MOTOR_SPEED);
  }
  else if (rightIRSensorValue == LOW && leftIRSensorValue == LOW && centerIRSensorValue == HIGH)
  {
    rotateMotor(BASE_MOTOR_SPEED, BASE_MOTOR_SPEED);
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW && centerIRSensorValue == LOW)
  {
      rotateMotor(-MOTOR_SPEED , MOTOR_SPEED); 
  }
  //If left sensor detects black line, then turn left  
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH && centerIRSensorValue == LOW)
  {
      rotateMotor(MOTOR_SPEED , -MOTOR_SPEED); 
  } 
  //If both the side sensors detect black line, then stop 
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW && centerIRSensorValue == HIGH)
  {
    rotateMotor(-MOTOR_SPEED , MOTOR_SPEED);
  }
   else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH && centerIRSensorValue == HIGH)
  {
    rotateMotor(MOTOR_SPEED , -MOTOR_SPEED);
  }
   else if (rightIRSensorValue == HIGH && leftIRSensorValue == HIGH && centerIRSensorValue == LOW)
  {
    rotateMotor(MOTOR_SPEED*E1 , -MOTOR_SPEED*E1);
  }
}
  // Update the last error time
  lasterrortime = millis();
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}
