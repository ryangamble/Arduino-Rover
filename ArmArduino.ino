#include <Servo.h>
#include <Wire.h>

Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo clawServo;

int shoulderServoPin = 8;
int elbowServoPin = 9;
int wristServoPin = 10;
int clawServoPin = 11;

int shoulderFeedbackPin = A0;
int elbowFeedbackPin = A1;
int wristFeedbackPin = A2;
int clawFeedbackPin = A3;

int minDegrees = 0;
int maxDegrees = 180;
int shoulderMinFeedback;
int shoulderMaxFeedback;
int elbowMinFeedback;
int elbowMaxFeedback;
int wristMinFeedback;
int wristMaxFeedback;
int clawMinFeedback;
int clawMaxFeedback;

int target;
int tolerance = 2;

int shoulderPos;
int elbowPos;
int wristPos;
int clawPos;

int clawActiveAngle = 180; //TODO determine correct angles
int clawInactiveAngle = 0;
int shoulderDefaultAngle = 0;
int elbowDefaultAngle = 0;
int wristDefaultAngle = 0;
int shoulderBucketAngle = 180;
int elbowBucketAngle = 180;
int wristBucketAngle = 180;

byte removalState = 0;
int obstaclePosition = 5;
int LED = 13;

bool armToCenter = false;

//////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW); 
  
  shoulderServo.attach(shoulderServoPin);
  elbowServo.attach(elbowServoPin);
  wristServo.attach(wristServoPin);
  clawServo.attach(clawServoPin);

  calibrate_servos();
  default_servos();
  
  Wire.begin(8);
  Wire.onReceive(receiveEvent); // When transmission is received, run receiveEvent
  Wire.onRequest(requestEvent); // When request is received, run requestEvent
}

/////////////////////////////////////////////////////////////////
//Functions
/////////////////////////////////////////////////////////////////

void calibrate_shoulder_servo(){
  Serial.println("Calibrating shoulder.");
  shoulderServo.write(minDegrees);
  delay(1000);
  shoulderMinFeedback=analogRead(shoulderFeedbackPin);
  
  shoulderServo.write(maxDegrees);
  delay(1000);
  shoulderMaxFeedback=analogRead(shoulderFeedbackPin);
}

void calibrate_elbow_servo(){
  Serial.println("Calibrating elbow.");
  elbowServo.write(minDegrees);
  delay(1000);
  elbowMinFeedback=analogRead(elbowFeedbackPin);
  
  elbowServo.write(maxDegrees);
  delay(1000);
  elbowMaxFeedback=analogRead(elbowFeedbackPin);
}

void calibrate_wrist_servo(){
  Serial.println("Calibrating wrist.");
  wristServo.write(minDegrees);
  delay(1000);
  wristMinFeedback=analogRead(wristFeedbackPin);
  
  wristServo.write(maxDegrees);
  delay(1000);
  wristMaxFeedback=analogRead(wristFeedbackPin);
}

void calibrate_claw_servo(){
  Serial.println("Calibrating claw.");
  clawServo.write(minDegrees);
  delay(1000);
  clawMinFeedback=analogRead(clawFeedbackPin);
  
  clawServo.write(maxDegrees);
  delay(1000);
  clawMaxFeedback=analogRead(clawFeedbackPin);
}

void calibrate_servos(){
  Serial.println("Beginning calibration.");
  calibrate_shoulder_servo();
  calibrate_elbow_servo();
  calibrate_wrist_servo();
  calibrate_claw_servo();
  Serial.println("Calibration complete.");
}

void seek(Servo servo, char activeServo, int analogPin, int pos){
  servo.write(pos);
  
  // Calculate the target feedback value for the final position
  if(activeServo == 1){
    Serial.println("Seeking shoulder angle.");
    target = map(pos, minDegrees, maxDegrees, shoulderMinFeedback, shoulderMaxFeedback);
  }
  else if(activeServo == 2){
    Serial.println("Seeking elbow angle.");
    target = map(pos, minDegrees, maxDegrees, elbowMinFeedback, elbowMaxFeedback);
  }
  else if(activeServo == 3){
    Serial.println("Seeking wrist angle.");
    target = map(pos, minDegrees, maxDegrees, wristMinFeedback, wristMaxFeedback);
  }
  else if(activeServo == 4){
    Serial.println("Seeking claw angle.");
    target = map(pos, minDegrees, maxDegrees, clawMinFeedback, clawMaxFeedback);
  }
  else{
    Serial.println("Could not seek.");
  }
  
  // Wait until it reaches the target
  while(abs(analogRead(analogPin) - target) > tolerance){} // wait...
}

void arm_position(int shoulderAngle, int elbowAngle, int wristAngle){
  Serial.println("arm_position for shoulder initialized.");
  seek(shoulderServo, 1, shoulderFeedbackPin, shoulderAngle);
  Serial.println("arm_position for elbow initialized.");
  seek(elbowServo, 2, elbowFeedbackPin, elbowAngle);
  Serial.println("arm_position for wrist initialized.");
  seek(wristServo, 3, wristFeedbackPin, wristAngle);
}

void claw_position(char clawState){
  if(clawState == 1){
    Serial.println("claw_position initialized for active.");
    seek(clawServo, 4, clawFeedbackPin, clawActiveAngle);
  }
  else if(clawState == 0){
    Serial.println("claw_position initialized for inactive.");
    seek(clawServo, 4, clawFeedbackPin, clawInactiveAngle);
  }
}

void default_servos(){
  Serial.println("Setting servos to default.");
  arm_position(shoulderDefaultAngle, elbowDefaultAngle, wristDefaultAngle);
  claw_position(0);
}

void receiveEvent(int howMany){
  int transmission = Wire.read();    // receive byte as an integer

  if(transmission == 5){
    armToCenter = true;
  }
  else if(transmission == 6){
    removalState = 0;
  }
  else{
    obstaclePosition = transmission;
  }
  
  Serial.print("Transmission received: ");
  Serial.println(transmission);
}

void requestEvent(){
  Wire.write(removalState);
}

void remove_object(){
  claw_position(1);
  removalState = 1;
  while(!armToCenter){
    Serial.print("Removal state is: ");
    Serial.println(removalState);
    delay(100);
  }
  armToCenter = false;
  removalState = 2;
  arm_position(shoulderBucketAngle, elbowBucketAngle, wristBucketAngle);
  claw_position(0);
  default_servos();
  removalState = 3;
  obstaclePosition = 5;
}

////////////////////////////////////////////////////////////////////
  
void loop() {
  digitalWrite(LED, HIGH);
  if(obstaclePosition == 5){
    Serial.println("Standing by for transmission.");
  }
  else if(obstaclePosition == 0){
    Serial.println("Registered 0");
    arm_position(30, 30, 30);
    remove_object();
  }
  else if(obstaclePosition == 1){
    Serial.println("Registered 1");
    arm_position(60, 60, 60);
    remove_object();
  }
  else if(obstaclePosition == 2){
    Serial.println("Registered 2");
    arm_position(90, 90, 90);
    remove_object();
  }
  else if(obstaclePosition == 3){
    Serial.println("Registered 3");
    arm_position(120, 120, 120);
    remove_object();
  }
  else if(obstaclePosition == 4){
    Serial.println("Registered 4");
    arm_position(150, 150, 150);
    remove_object();
  }
  delay(500);
}
