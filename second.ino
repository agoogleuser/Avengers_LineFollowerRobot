#include <Wire.h>
//==============MD==============//
#define Rspeed 10
#define in4 9
#define in3 8
#define in2 7
#define in1 6
#define Lspeed 5
//==============Ir==============//
#define IR_RR 13 // S1
#define IR_R 3   // S2
#define IR_C 4   // S3
#define IR_L 11  // S4
#define IR_LL A0 // S5

//============MPU==============//
#define MPU_ADDRESS 0b1101000

// Gyroscope modes

#define pm250 0b00000
#define pm500 0b01000
#define pm1000 0b10000
#define pm2000 0b11000

#define gyroMode pm1000

#if gyroMode == pm2000
#define converter 131
#elif gyroMode == pm1000
#define converter 65.5
#elif gyroMode == pm500
#define converter 32.8
#else
#define converter 16.4
#endif

// needed Registers
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define GYRO_CONFIG 0x1B

// Other Registers
#define MPU_POWER 0x6B
//==============================//
int flag = 0;

#define leftInitSpeed 120
#define rightInitSpeed 100
#define SWITCH  12

//==============Pid==============//
int Kp = 70;
int Kd = 110;
float Ki = .005;
int PIDvalue = 0;
float milliOld;
float milliNew;
float dt;
float milliOld_mpu;
float milliNew_mpu;
float dt_mpu;
int error = 0; // ?? Why there are so many global variables here while we can use them in functions only 
float previousError = 0;
float ErrorChange;
float Slope;
float Area = 0;
int pid;
int counter = 0;// ?? What is the usage of this counter
//char arr[5] = {'f', 'f', 'r', 'r', 'r'}; // ?? this array shouldn't be here it should be in void loop
int d[5]; // ?? what does this array stand for 
void setup()
{
  Wire.begin();
  Serial.begin(9600);
  MPU_reg_write(MPU_POWER, 0x00); // MPU Reset
  MPU_reg_write(GYRO_CONFIG, gyroMode);
  for (int i = 5; i <= 10; i++)
  {
    pinMode(i, OUTPUT);
  }
  pinMode(IR_LL, INPUT);
  pinMode(IR_L, INPUT);
  pinMode(IR_C, INPUT);
  pinMode(IR_R, INPUT);
  pinMode(IR_RR, INPUT);

  pinMode (SWITCH, INPUT);
}

void loop()
{ char path[40];

  if (digitalRead(SWITCH)==LOW){
    ProcessingPath(path,40);
    second_t(path,40,readSensor());}
  else
    followLine();
}

//==============read sensor==============//
int readSensor()
{

  // static int data[5];
  //  Read in Data
  int data = 0;

  data = digitalRead(IR_LL);
  data = data << 1 + digitalRead(IR_C);
  data = data << 1 + digitalRead(IR_L);
  data = data << 1 + digitalRead(IR_R);
  data = data << 1 + digitalRead(IR_RR);

  return data;
}
//==============MPU FUNCTIONS============//
inline int16_t gyroZ_raw()
{
  int16_t data = 0;
  Wire.beginTransmission(MPU_ADDRESS); // Orders MPU to communicate to send data
  Wire.write(GYRO_ZOUT_H);             // Sends Register Address to MPU, then the MPU responds.
  Wire.endTransmission(false);         // Doesn't send a stop bit at the end of the transmission

  Wire.requestFrom(MPU_ADDRESS, 2); // Request 2 bytes from MPU. GYRO_ZOUT_L
  // The MPU will send the data in GYRO_ZOUT_H then GYRO_ZOUT_L
  data = Wire.read() << 8 | Wire.read();

  return data;
}

void MPU_reg_write(byte address, byte data) // inline means the function is inserted at the call place when compiled,
{                                           //  and not sent to this function address as normal functions.
  Wire.beginTransmission(MPU_ADDRESS);      // Start communication with MPU6050 // MPU=0x68
  Wire.write(address);                      // Talk to the register 6B
  Wire.write(data);                         // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);               // end the transmission
}

float readYaw()
{ // gyroZ angle
  milliOld_mpu = milliNew_mpu;
  milliNew_mpu = millis();
  dt_mpu = (milliOld_mpu - milliNew_mpu) / 1000;
  float reading = (float)gyroZ_raw();
  static float yaw = 0;
  reading /= converter;
  yaw += reading * (dt_mpu);
  return yaw * 2;
}

void rotate_180()
{
  brake();
  // delay(200);
  float yaw1 = readYaw();
  // start rotation in a certian direction
  turnLeft(rightInitSpeed, rightInitSpeed);

  // spins until the following condition is false
  while (readYaw() <= yaw1 + 170)
  {
    Serial.println(readYaw());
  }

  brake(); // end rotation
}

void rotate_90(float yaw1, char dir)
{
  // brake();
  //  delay(200);

  // start rotation in a certian direction
  if (dir == 'l')
  {
    turnLeft(rightInitSpeed, leftInitSpeed);
    while (readYaw() <= yaw1 + 65)
      ;
  }
  else if (dir == 'r')
  {
    turnRight(rightInitSpeed, leftInitSpeed);
    while (readYaw() >= yaw1 - 65)
      ;
  }
  // spins until the following condition is false

  brake(); // end rotation

  // delay(3000);
}
//==============right motor==============//
void setRightMotor(bool dir, int motorSpeed)
{
  // setting speed
  analogWrite(Rspeed, motorSpeed); // x between 0-255

  // sets direction as forward
  if (dir == 1)
  { // left dir set to forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }

  // sets direction to reverse
  else if (dir == 0)
  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
}

//==============left motor==============//
void setLeftMotor(bool dir, int motorSpeed)
{
  // Sets the speed of the
  analogWrite(Lspeed, motorSpeed); // x between 0-255

  // sets direction as forward for left motor
  if (dir == 1)
  { // sets direction as forward for right motor
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
  }

  else if (dir == 0)
  { // sets direction as forwoard for right motor
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
  }
};

//==============forward==============//
void forward(int rightSpeed, int leftSpeed)
{
  setRightMotor(1, rightSpeed);
  setLeftMotor(1, leftSpeed);
};

//==============stop==============//
void brake()
{
  flag = 1;
  digitalWrite(in4, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in1, LOW);
  delay(1000);
};

//==============reverse==============//
void reverse(int rightSpeed, int leftSpeed)
{
  setRightMotor(0, rightSpeed);
  setLeftMotor(0, leftSpeed);
};

// Turn left using Encoder
void turnLeft(int rightSpeed, int leftSpeed)
{
  setRightMotor(1, rightSpeed - 10);
  setLeftMotor(0, leftSpeed - 10);
}
// Turn right using Encoder
void turnRight(int rightSpeed, int leftSpeed)
{
  setRightMotor(0, rightSpeed - 10);
  setLeftMotor(1, leftSpeed - 10);
}

//==============calc error==============//
int returnError(int LFSensor)
{
  static int err = 0;
  switch (LFSensor)
  {
    case 0b00111:
    case 0b00011:
    //90deg right
      err=-14;
      break;
    case 0b11100:
    case 0b11000:
    //90deg left
      err=-14;
      break;
    case 0b10100:
      err=-1;
      break;
    case 0b00101:
      err=1;
      break;
    case 0b01000:
      err=2;
      break;
    case 0b00010:
      err=-2;
      break;
    case 0b00100:
      err=0;
      break;
    case 0b01100:
      err=4;
      break;
    case 0b00110:
      err=-4;
      break;
    case 0b01110:
      err=5;
      break;
    case 0b00000:
      err=6;
      break;
  }
  return err;

}

//==============calculates and returns PID error==============//
int calcPID(int error, int previousError)
{
  milliNew = millis();
  dt = milliNew - milliOld;
  milliOld = milliNew;
  ErrorChange = error - previousError;
  Slope = ErrorChange / dt;
  Area = Area + (error * dt);
  pid = (Kp * error) + (Kd * Slope) + (Ki * Area);
  return pid;
}

//==============set motor speed based on PID error==============//
void setLeftRightSpeed(int PIDvalue, int error) // This virable int error is defined as globale above :)
{
  int rightSpeed = rightInitSpeed + PIDvalue;
  int leftSpeed = leftInitSpeed - PIDvalue;

  // The motor speed should not exceed the max PWM value
  constrain(leftSpeed, 0, 255);
  constrain(rightSpeed, 0, 255);

  if (error == 0)
  {

    forward(rightSpeed, leftSpeed);
  }

  else if (error == 1)
  {
    brake();
    forward(60, 80);
    delay(100);
    turnLeft(rightSpeed, leftSpeed);
  }

  else if (error == -1)
  {
    brake();
    forward(100, 120);
    delay(300);
    // turnRight(rightSpeed, leftSpeed);
    rotate_90(readYaw(), 'r');
  }

  else if (error == -4)
  {
    brake();
    forward(100, 120);
    delay(300);
    // turnRight(rightSpeed, leftSpeed);
    rotate_90(readYaw(), 'r');
  }

  else if (error == 4)
  {
    brake();
    forward(60, 80);
    delay(100);
    turnLeft(rightSpeed, leftSpeed);
  }
  else if (error == 14)
  {
    // rotate_90(readYaw(), 'l');
    brake();
    forward(60, 80);
    delay(100);
    turnLeft(160, 160);
  }
  else if (error == -14)
  {
    // rotate_90(readYaw(), 'r');
    brake();
    forward(100, 120);
    delay(300);
    // turnRight(160, 160);
    rotate_90(readYaw(), 'r');
  }
}

//==============follows straight line==============//
void followLine()
{ // Read Sensor
  int LFSensor = readSensor();
  // Error for PID
  error = returnError(LFSensor);
  // PID Calculation
  PIDvalue = calcPID(error, previousError);
  previousError = error;
  // Set motor speeds based on error
  setLeftRightSpeed(PIDvalue, error);
} // end followLine()

void second_t(char arr[],int Size,int LFSensor)
{
  int i = 0;
  while (i<Size){
  if (LFSensor==0b01100 || LFSensor==0b11100 ||
     LFSensor==0b00110 || LFSensor==0b00111 ||
     LFSensor==0b01110 || LFSensor==0b11111 ||
     LFSensor==0b11011 || LFSensor==0b01010 )
  {
    forward(100, 80);
    delay(200);
    if (arr[i] == 'l')
    {
      brake();
      forward(rightInitSpeed, leftInitSpeed);
      delay(100);
      rotate_90(readYaw(), 'l');
      i++;
    }
    else if (arr[i] == 'f')
    {
      // constrain(leftSpeed, 0, 255);
      // constrain(rightSpeed, 0, 255);
      forward(100, 80);
      delay(200);
      i++;
    }
    else if (arr[i] == 'r')
    {
      brake();
      forward(100, 80);
      delay(300);
      rotate_90(readYaw(), 'r');
      i++;
    }
  }
  else
  {
    followLine();
  }}
}

void Delete(char arr[],int Size, int index){
    for(int j=0;j<2;j++){
        for(int i=index;i<Size ;i++){
            arr[i]=0;
            arr[i]= arr[i]+arr[i+1]- (arr[i+1]=arr[i]);
        }}
}

void ProcessingPath(char path[],int Size){
bool any_end = true;
int Bcount = 0;
while (any_end){
    for(int i=0; i<=Size;i++){
        if(path[i]=='b'){
            if (path[i-1]=='l'&&path[i+1]=='r'||path[i-1]=='r'&&path[i+1]=='l'||path[i-1]=='f'&&path[i+1]=='f'){
                path[i-1]='b';
                Delete(path,Size,i);   }
            else if (path[i-1]=='f'&&path[i+1]=='r'||path[i-1]=='r'&&path[i+1]=='f'){
                path[i-1]='l';
                Delete(path,Size,i);   }
            else if (path[i-1]=='f'&&path[i+1]=='l'||path[i-1]=='l'&&path[i+1]=='f'){
                path[i-1]='r';
                Delete(path,Size,i);   }
            else if (path[i-1]=='l'&&path[i+1]=='l'||path[i-1]=='r'&&path[i+1]=='r'){
                path[i-1]='f';
                Delete(path,Size,i);   }}}

    for(int i=0; i<=Size;i++){
        if(path[i]=='b'){
            Bcount++;   }    }
    if (Bcount==0){any_end=false;}
    else{Bcount=0;}
}}
