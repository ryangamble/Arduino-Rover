//TO DO        Line sensors readings debug.      Correction algorithm debug.    Drive Algorithm Debug.        Double Check that distance sensors work.       



#include <math.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <Servo.h>


//Functions////////////////////////////////////////////////////////////////////////////////////////////////////////
void stopMotors();                 //A function to stop the motors completely.
void getCorrection();              //Calculates a steering correction speed based on line sensor readings and the PD calculation.
void transmit();                   //Sends a byte to the arm arduino telling the position of the obstacle.
void checkObstacle();              //Checks to see if there is an obstacle in the way of the rover and if so how far away it is. 
void drive();                      //Writes a speed to the 4 servos to make the rover move forward turn left or turn right, depending on the correciton output.
void readSensors();

//SERVOS AND MOTOR VARIABLES////////////////////////////////////////////////////////////////////////////////////////
Servo driveLeft;                   //Left wheel steering servo
Servo driveLeftRear;               //Left rear wheel steering servo
Servo driveRight;                  //Right wheel steering servo
Servo driveRightRear;              //Right rear wheel steering servo
Servo armRotator;                  //Arm rotating servo
int lStopSpeed = 90;               //Calibrated Value to be written to the left servo to stop. (These values may need calibrated for each servo independently.)
int rStopSpeed = 90;               //Calibrated Value to be written to the right servo to stop.
int rRStopSpeed = 90;              //Calibrated Value to be written to the left rear servo to stop.
int lRStopSpeed = 90;              ////Calibrated Value to be written to the right rear servo to stop.
int lSpeed =  180;                   //Number to be written to the left servo if the correction is 0. (to go straight)
int lRSpeed = 180;                   //Number to be written to the left rear servo if the correction is 0.
int rSpeed =  0;                 //Number to be written to the right servo if the correction is 0.
int rRSpeed = 0;                 //Number to be written to the right rear servo if the correction is 0.

//Debug/////////////////////////////////////////////////////////////////////////////////////////////////////////////
int debug = 0;                    
int debugTransmit = 0;       
int debugSensors = 1;     
int debugObstacle = 0;       
int debugDrive = 1;       
int debugMain = 0;
int debugCorrection = 0;
int lSpeedActual;                  //Prints the actual current speed being written to the left servo.
int lRSpeedActual;                 //Prints the actual current speed being written to the left rear servo.
int rSpeedActual;                  //Prints the actual current speed being written to the right servo.
int rRSpeedActual;                 //Prints the actual current speed being written to the right rear servo.

//Distance Sensor Variables/////////////////////////////////////////////////////////////////////////////////////////
float duration;                                   //The amount of time the sound took to return to the echo pin.
float inches = 999;                               //Variable for the number of inches the object is from the sensor.
float distanceArray[5] = {10,10,10,10,10};        //Stores the distance read from each distance sensor in sequential order, left to right. 
float height = 3;                                 //A variable for the height of sensor off of the ground. This will be used to calculate the horizontal distance of obstacle from the rover base. 
int echoArray[5] = {23,24,25,26,27};              //Stores the values of the pins the echo pins will be connected to.
int trigPin = 22;                                 //The distance sensor trigger output pin.      

//Wried Comm Variables//////////////////////////////////////////////////////////////////////////////////////////////////////
int transmitPosition = 0;                         //Stores the position of obstacle from rover to be sent to the arm.
int transmitDistance = 0;                         //Stores the distance of obstacle from rover to be sent to the arm.
bool obstacle = false;

//Line Sensor Variables (See Polulu Example Sketch)//////////////////////////////////////////////////////////////////////////////////////////////////////
#define NUM_SENSORS   8                           // number of sensors used
#define TIMEOUT       2500                        // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   10                          // emitter is controlled by digital pin 2
QTRSensorsRC qtrrc((unsigned char[]) {2, 3, 4, 5, 6, 7, 8, 9}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];                      
unsigned int position;
int oldLineTime;                                //Records when the line position reading was taken.
int newLineTime;                              //Records when the next line position reading was taken. The startLineTime can then be deducted from this value to give the time elapsed between readings.
int oldError;                                     //
int ledPin = 50;                                  //The pin the LED calibration light is to be connected to. This light will signal when the rover is in claibration mode.
int turnLeft = 0;                                     //Is set to 0 or 1 in the getCcorrection loop. This will be a conditional variable telling the drive loop which servos to adjust to turn left.
int turnRight = 0;                                    //Is set to 0 or 1 in the getCcorrection loop. This will be a conditional variable telling the drive loop which servos to adjust to turn right.

// PID variables (see https://en.wikipedia.org/wiki/PID_controller)                   (This program only utilizes the PD portion)
float Kp = .1;                                    //Multiplier for the PD calculation to adjust the correction speed. This variable can be adjusted to determine how extreme of a correction the PD algorithm will output.
float Kd = .1;                                    //Multiplier for the PD calculation to adjust the correction speed. This variable can be adjusted to determine how extreme of a correction the PD algorithm will output.
float P;                                          //Proportional error
float D;                                          //Rate change of the errror
float correction = 0;                             //Output of the PD algorithm
int i = 0;                                        //Counter Variable

//Other Variables////////////////////////////////////////////////////////////////////////////////////////////////////////////
int x;                                            //Counter Variable.
int timeOut = 2000;                               //TODO______________________________________
bool obstacle = false;                            //checks to see if there is an obstacle
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Wire.begin();
  Serial.begin(9600);                        //Open the serial port at 9600 bps for debugging
  for(x=0; x<5; x++)                         //Loop to assign the entire array as inputs.
  {
    pinMode(echoArray[x],INPUT);             //Assign Inputs
    distanceArray[x] = 999;                  //Sets distance array to some arbitrary value.
  } 
  pinMode(ledPin, OUTPUT);                   //Set led pin to output.
  pinMode(trigPin,OUTPUT);                   //Set trigger pin to output.
  
  driveLeft.attach(28);                      // attach servo object
  driveRight.attach(29);                     // attach servo object
  driveLeftRear.attach(30);                  // attach servo object
  driveRightRear.attach(31);                 // attach servo object
  armRotator.attach(13);
  
  driveLeft.write(90);
  driveRight.write(90);
  driveLeftRear.write(90);
  driveRightRear.write(90);
  
  digitalWrite(ledPin, HIGH);                //Turn on LED to signal calibration start.
  for (int i = 0; i < 400; i++)              // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();                       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(ledPin, LOW);                 //Turn Off LED. (calibration is over)  
}

///////////////////////////////////////////////////////////////////////////

void loop()
{
  readSensors();                       
  if(debugMain)                           //Debug Statements will only execute if the debug Main is set to a non Zero number.
  {
    for(x=0; x<8; x++)
    {
    
    }
    Serial.println(); 
  }
  getCorrection();
  if(debugMain)
  {
    Serial.print("correction = ");
    Serial.println(correction);
    Serial.println();
  }
//  checkObstacle();
  if(debugMain)
  {
    Serial.print("Transmit Position = ");
    Serial.println(transmitPosition);
    Serial.print("Transmit Distance = ");
    Serial.println(transmitDistance);
    Serial.println();
  }
  drive();
  if(debugMain)
  {
    Serial.print("Left Motor Speed = ");
    Serial.println(lSpeedActual);
    Serial.print("Left Rear Motor Speed = ");
    Serial.println(lRSpeedActual);
    Serial.print("Right Motor Speed = ");
    Serial.println(rSpeedActual);
    Serial.print("Right Rear Motor Speed = ");
    Serial.println(rRSpeedActual);
    Serial.println();
    
  }
  
}
//END MAIN/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readSensors()                                               //For information on sensors and the sensor library see https://www.pololu.com/docs/pdf/0J12/QTR-8x.pdf
{                                                  
  oldError = position;                                           //Sets the previously recorded position value to oldError so the value is not lost. It will be used in the PD calculation.
  oldLineTime = newLineTime;                                     //Sets the previous line time to old lone time, so the value is not lost when new line time is reassigned.
  position = qtrrc.readLine(sensorValues);                       //Reads all sensors.
  if(debugSensors)
  {
    for (unsigned char i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println(); 
    Serial.print("Position = ");
    Serial.println(position); 
    newLineTime = micros();                                     //Records the time the line sensor readings were taken.
  } 
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getCorrection()                                              
{
  P = position;                                                   //Proportional value for the PD calculation.
  D = (position - oldError) / (newLineTime - oldLineTime);        //Derivative value for the PD calculation.
  if(position < 3000)                                             //The position call to the line sensors returns a value of 0 - 7000 based one which sensor the detects the line. 3500 is estimated to be over the center of the line. If position value is less than 3000 the loop will execute setting turn left to 1.
  {
    P = P - 3000;                                   //A position value of 0 corresponds to being off completely to the right. In order to use this number in the correction calculation we need the absolute value of the supposed middle value (3500) - the position vslue given. 
    turnLeft = 1;   
    P = P * -1;                                     //Correction value was set to middle value - position. Since position is less than 3000 this will return a negative number. The absolute Value is needed so multiplying by -1 will get poisition value back to a positive number.
    D = (P - oldError) / (newLineTime - oldLineTime);      //Derivative value for the PD calculation.
    correction = (P*Kp) + (Kd*D);                                 //PD calculation.
  }
  else if(position > 4000 )                                            //If position is greater than 4000 then it is off center and needs to be corrected to the left.
  {                             
    P = P - 3000;
    D = (P - oldError) / (newLineTime - oldLineTime);      //Derivative value for the PD calculation.
    correction = (P*Kp) + (Kd*D);                                 //Calculation
    
    turnRight = 1;                                                
  }
   
  else                                                            //If the previous conditions were not met then the rover is centered on the line and does not need to make a correction.      
  {
    correction = 0;                                               //Sets correction to 0 so the rover will go straight.
    turnRight = 0;                             //Assign the turn variables to 0.
    turnLeft = 0;
  }
  if(correction > 90)                                             //If the correction algorithm attempts to assign a correction value greater than 90 then this will assign correction to 90. (90 is the maximum correction value that can be given to the servos.)
  {
    correction = 90;
  }
  if(debugCorrection)
    {
      //Serial.print("P = ");
      //Serial.println(P);
      //Serial.print("D = ");
      //Serial.println(D);
      //Serial.print("PID = ");
      //Serial.print("Correction In Function = ");
      //Serial.println(correction);
      ///Serial.println();
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Precondition: sonar distance sensors are hooked up
//Postcondition: obstacles are detected and removed after
//communication with the arm
void checkObstacle() 
{
  int temp;
  int minValue;
  byte transmitPosition;
  byte transmitDistance;
  for(x=0; x<5; x++)
  {
    digitalWrite(trigPin, LOW);                           //Assigns trigPin to low to be sure that when it goes high there is no ambient previous voltage.
    delayMicroseconds(2);                                 //Delay time.
    digitalWrite(trigPin, HIGH);                          //Set trig Pin to high.
    delayMicroseconds(15);                                //Wait to ensure high pulse to trig pin is sufficiently long.
    digitalWrite(trigPin, LOW);                           //Set trig pin back to low to end the trigger cycle.
    duration = pulseIn(echoArray[x], HIGH);               //Counts the amount of time the given echo pin stays high. Returns microseconds.
    inches = (duration/74)/2;                             //Converts inches from duration.
    if(inches < 8 && inches > 1)
    {
      //Serial.println("Standby");
      distanceArray[x] = inches;                            //Assign the distance array values.
      //distanceArray[x] = pow((inches*inches - height*height),1.0/2.0);        //Calcaulate the actual distance horizontally from rover. 
      transmitPosition = x;                                //Indicates which Sensor detected the obstacle.
      transmitDistance = distanceArray[x];                 //The distance to be transmitted.
      minValue = distanceArray[x];                         
      for(x = 1; x < 5; x++)                               //Sorts the array and reassigns the transmitPosition variable to the sensor that is closest to the object. Then reassigns the transmitPosition variable to the corresponding distance.
      {
        if(distanceArray[x] < minValue)
        {
          minValue = distanceArray[x];
        }
      }
      if(minValue < 8 && minValue > 1)                     //If Obstacle is in the range 0 to 8 the rover will stop and attempt to remove obstacle. (Values Below 1 can be garbage values and may not be reliable and values above 8 are not to be dealt with.)
      {    
          stopMotors();
          obstacle = true;
         
          transmitPosition = x;
          transmitDistance = minValue;

          if(transmitPosition == 0){       //Rotates the arm to the correct angle
            armRotator.write(0);
          }
          else if(transmitPosition == 1){
            armRotator.write(45);
          }
          else if(transmitPosition == 2){
            armRotator.write(90);
          }
          else if(transmitPosition == 3){
            armRotator.write(135);
          }
          else if(transmitPosition == 4){
            armRotator.write(180);
          }
           
          transmit(transmitPosition);  //Tells the arm which position the obstacle is in
          while(obstacle){             //While there is an obstacle, continue requesting
            request();
            delay(100);
          }
       }
     }
   }
}

//Precondition: transmitPosition is known
//Postcondition: transmitPosition is sent to arm
void transmit(byte transmission){    
  Wire.beginTransmission(8);              // transmit to device #8
  Wire.write(transmission);               // sends one byte
  Serial.print(transmission);
  Serial.println("Transmitted");
  Wire.endTransmission();                 // stop transmitting
}

//Precondition: boolean obstacle is true
//Postcondition: base reacts to the actions of the arm
void request(){                   
  int response;                   // variable used to keep track of what the arm is doing
  
  Wire.requestFrom(8, 1);         // request 1 byte from slave device #8
  while (Wire.available()) {      // slave may send less than requested
    response = Wire.read();       // receive a byte as an integer
    Serial.print(response);       // print the integer for debugging 
    Serial.println(" Received");
  }
  if(response == 0){
   Serial.println("Claw is moving to object");
  }
  else if(response == 1){
    Serial.println("Object is in claw");
    armRotator.write(90);
    delay(500);
    transmit(5);
  }
  else if(response == 2){
    Serial.println("Claw is moving to bucket");
  }
  else if(response == 3){
    Serial.println("Obstacle removed");
    transmit(6);
    obstacle = false;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drive()
{
  if(turnRight)                               //Variable assigned 0 or 1 in the previous readSensors function.
  {
    driveRightRear.write(180-correction);     //If the rover needs to turn right the left servos will continue to go full speed forwards, and the right servos will be slowed down by the amount predetermined in the PD calculation.
    rRSpeedActual = 180 - correction;          
    driveRight.write(180-correction);
    rSpeedActual = 180 - correction;
    driveLeftRear.write(lRSpeed);
    lRSpeedActual = lRSpeed;
    driveLeft.write(lSpeed);
    lSpeedActual = lSpeed;
  }
  else if(turnLeft)
  {
    driveRightRear.write(rRSpeed);            //If the rover needs to turn left the right servos will continue to go full speed forwards, and the left servos will be slowed down by the amount predetermined in the PD calculation.
    rRSpeedActual = rRSpeed;
    driveRight.write(rSpeed);
    rSpeedActual = rSpeed;
    driveLeftRear.write(correction);
    lRSpeedActual = correction;
    driveLeft.write(correction);
    lSpeedActual = correction;
  }
  else                                       //If neither variable, turnLeft or turnRight is set to 1 then the rover will go straight.
  { 
    driveRightRear.write(rRSpeed);
    rRSpeedActual = rRSpeed;
    driveRight.write(rSpeed);
    rSpeedActual = rSpeed;
    driveLeftRear.write(lRSpeed);
    lRSpeedActual = lRSpeed;
    driveLeft.write(lSpeed);
    lSpeedActual = lSpeed;
  }
  turnRight = 0;                             //Assign the turn variables to 0.
  turnLeft = 0;
  if(debugDrive)
  {
    Serial.print("Left Motor Speed = ");
    Serial.println(lSpeedActual);
    Serial.print("Left Rear Motor Speed = ");
    Serial.println(lRSpeedActual);
    Serial.print("Right Motor Speed = ");
    Serial.println(rSpeedActual);
    Serial.print("Right Rear Motor Speed = ");
    Serial.println(rRSpeedActual);
    Serial.println();
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void stopMotors()                       
{
  driveLeft.write(lStopSpeed);
  driveRight.write(rStopSpeed);
  driveLeft.write(lRStopSpeed);
  driveRight.write(rRStopSpeed);
}






