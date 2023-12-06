include <SPI.h>  
#include <Pixy.h>
#include <AFMotor.h>
AF_DCMotor Lmotor3(3, MOTOR12_64KHZ);
AF_DCMotor Rmotor4(4, MOTOR12_64KHZ);

#define X_CENTER    135L
#define Y_CENTER    95L
#define RCS_MIN_POS     50L
#define RCS_MAX_POS     950L
#define RCS_MIN_POS_1     200L
#define RCS_MAX_POS_1     500L
#define RCS_CENTER_POS  ((RCS_MAX_POS-RCS_MIN_POS)/2)

//---------------------------------------
// Servo Loop Class
// A Proportional/Derivative feedback
// loop for pan/tilt servo tracking of
// blocks.
// (Based on Pixy CMUcam5 example code)
//---------------------------------------
class ServoLoop
{
public:
  ServoLoop(int32_t proportionalGain, int32_t derivativeGain);

  void update(int32_t error);

  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_proportionalGain;
  int32_t m_derivativeGain;
};

ServoLoop panLoop(350, 600);  // Servo loop for pan
ServoLoop tiltLoop(500, 700); // Servo loop for tilt

// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
  m_pos = RCS_CENTER_POS;  //RCservo
  m_proportionalGain = proportionalGain;
  m_derivativeGain = derivativeGain;
  m_prevError = 0x80000000L;
}

// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error)
{
  long int velocity;
  char buf[32];
  if (m_prevError!=0x80000000)
  { 
    velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;

    m_pos += velocity;
    if (m_pos>RCS_MAX_POS) 
    {
      m_pos = RCS_MAX_POS; 
    }
    else if (m_pos<RCS_MIN_POS) 
    {
      m_pos = RCS_MIN_POS;
    }
  }
  m_prevError = error;
}
// End Servo Loop Class
//---------------------------------------

Pixy pixy;  // Declare the camera object


//---------------------------------------
// Setup - runs once at startup
//---------------------------------------
void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();
}

uint32_t lastBlockTime = 0;

//---------------------------------------
// Main loop - runs continuously after setup
//---------------------------------------
void loop()
{ 
  uint16_t blocks;
  blocks = pixy.getBlocks();

  // If we have blocks in sight, track and follow them
  if (blocks)
  {
    int trackedBlock = TrackBlock(blocks);
    FollowBlock(trackedBlock);
    lastBlockTime = millis();  //計算秒數
  }  
  else if (millis() - lastBlockTime > 100)
  {
    Lmotor3.run(FORWARD);
    Rmotor4.run(FORWARD);
    ScanForBlocks();
  }
}

int oldX, oldY, oldSignature;

//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int TrackBlock(int blockCount)
{
  int trackedBlock = 0;
  long maxSize = 0;

  Serial.print("blocks =");
  Serial.println(blockCount);

  for (int i = 0; i < blockCount; i++)
  {
    if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
    {
      long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
      if (newSize > maxSize)
      {
        trackedBlock = i;
        maxSize = newSize;
      }
    }
  }

  int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
 
  int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;

  panLoop.update(panError);
  tiltLoop.update(tiltError);

  pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

  oldX = pixy.blocks[trackedBlock].x;
  oldY = pixy.blocks[trackedBlock].y;
  oldSignature = pixy.blocks[trackedBlock].signature;
  return trackedBlock;
}

//---------------------------------------
// Follow blocks via the our drive

// This code makes the robot base turn 
// and move to follow the pan/tilt tracking
// of the head.
//---------------------------------------
int32_t size = 400;
void FollowBlock(int trackedBlock)
{

  int32_t followError = RCS_CENTER_POS - panLoop.m_pos; 
   followError+=52;
  // How far off-center are we looking now?

  // Size is the area of the object.
  // We keep a running average of the last 8.
  size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
  size -= size >> 3;

  // Forward speed decreases as we approach the object (size is larger)
  int forwardSpeed = constrain(150 - (size/256), -100, 180);  
  // limits range of sensor values to between -100 and 400
  // Steering differential is proportional to the error times the forward speed
  int32_t differential = (followError + (followError * forwardSpeed))>>8;
          differential*=0.4;
  // Adjust the left and right speeds by the steering differential.
  int leftSpeed = constrain(forwardSpeed + differential, -255, 255);
  int rightSpeed = constrain(forwardSpeed - differential, -255, 255);
  delay(200);

Serial.print("fS:");
Serial.println(forwardSpeed);
Serial.print("Diff:");
Serial.println(differential);
Serial.print("L:");
Serial.println(leftSpeed);
Serial.print("R:");
Serial.println(rightSpeed);
Serial.print("followError:");
Serial.println(followError);
Serial.print("RCS_CENTER_POS:");
Serial.println(RCS_CENTER_POS);
Serial.print("panLoop.m_pos:");
Serial.println(panLoop.m_pos);
Serial.print("pixy.blocks[trackedBlock].x");
Serial.println(pixy.blocks[trackedBlock].x);




Serial.println();


  
// Set the motor speeds,and tell the speed
//  motors.setLeftSpeed(leftSpeed);
//  motors.setRightSpeed(rightSpeed);
  if(leftSpeed>0 && rightSpeed>0)
  {
    Lmotor3.setSpeed(leftSpeed);
    Rmotor4.setSpeed(rightSpeed);
    Lmotor3.run(FORWARD);
    Rmotor4.run(FORWARD);
  }
  else if(leftSpeed>0 && rightSpeed<0)
   {
    Lmotor3.setSpeed(leftSpeed);
    Rmotor4.setSpeed(abs(rightSpeed));
    Lmotor3.run(FORWARD);
    Rmotor4.run(BACKWARD);
  }  
  else if(leftSpeed<0 && rightSpeed>0)
   {
    Lmotor3.setSpeed(abs(leftSpeed));
    Rmotor4.setSpeed(rightSpeed);
    Lmotor3.run(BACKWARD);
    Rmotor4.run(FORWARD);
  }
  else
   {
    Lmotor3.setSpeed(abs(leftSpeed));
    Rmotor4.setSpeed(abs(rightSpeed));
    Lmotor3.run(BACKWARD);
    Rmotor4.run(BACKWARD);
  }

}

//---------------------------------------
// Random search for blocks
//
// This code pans back and forth randomly
// until a block is detected
//---------------------------------------
int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 50;
uint32_t lastMove = 0;

void ScanForBlocks()
{
  if (millis() - lastMove > 20)
  {
    lastMove = millis();
    panLoop.m_pos += scanIncrement;
    if ((panLoop.m_pos >= RCS_MAX_POS)||(panLoop.m_pos <= RCS_MIN_POS))
    {
      tiltLoop.m_pos = random(RCS_MAX_POS_1 * 0.6, RCS_MAX_POS_1);
      scanIncrement = -scanIncrement;
      if (scanIncrement < 0)
      {

        Lmotor3.setSpeed(0);
        Rmotor4.setSpeed(0);
        Lmotor3.run(BACKWARD);
        Rmotor4.run(FORWARD);
     // motors.setLeftSpeed(-250);
     // motors.setRightSpeed(250);
      }
      else
      {
        Lmotor3.setSpeed(0);
        Rmotor4.setSpeed(0);
        Lmotor3.run(FORWARD);
        Rmotor4.run(BACKWARD);
    //  motors.setLeftSpeed(+180);
    //  motors.setRightSpeed(-180);
      }
      delay(random(250, 500));
    }

    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
  }
}
