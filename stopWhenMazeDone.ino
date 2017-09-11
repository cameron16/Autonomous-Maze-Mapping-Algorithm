#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

/*Treasure and FFT Stuff*/
#define LIN_OUT 1 // use the log output function
#define FFT_N 128 // set to 256 point fft

#include <FFT.h>
#include <avr/interrupt.h>

#include <avr/io.h>
#include <stdio.h>
#include <inttypes.h>

int signal = 0;                                                                                 //the maximum index of the fft
int consistency = 0;                                                                            //consistency factor of the signal. 
int start_execution = 0;

#include <StackList.h>
//#include <StackArray.h>
StackList<int> myStack;

int adjacentUnexploredCoordinate[2];
boolean noAdjacentUnexplored; //true iff there are no adjacent unexplored coordinates to current coordinate
int frontCoordinate[2];
int rightCoordinate[2];
int leftCoordinate[2];

int nodesToVisit;
int oldNodesToVisit;
boolean wasVisited;


RF24 radio(9,10);   //initializing setup data
const uint64_t pipes[2] = { 0x000000002LL, 0x0000000003LL };
typedef enum { role_ping_out = 1, role_pong_back } role_e;
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};
role_e role = role_ping_out;

unsigned char temp1[32];       
unsigned char temp2[32];

Servo myServoLeft;
Servo myServoRight;
int rightServo = 4;
int leftServo = 5;
//convention: right and left are from 1st person perspective 
//(so if you look at the robot from behind, your right is the robots right in this code)


int finished = 0; //0 means not finished, 1 means finished

//configure the mux
int selectA = 2;
int selectB = 3;
int selectC = 8;
int muxRead = A2; //in order (from 0-6): left wall, center wall, right wall, OL, FL, FR, OR

//configure the line sensors
boolean FR; //true iff front right sensor is ON black line
boolean FL; //true iff front left sensor is ON black line
boolean OR; //true iff outside right sensor is ON black line
boolean OL; //true iff outside left sensor is ON black line
const int lineThres = 4;


boolean frontWall;
boolean leftWall;
boolean rightWall;

double frontWallThreshold = 15.5;//16.57475;//15.7475; //was 11.7475
double rightWallThreshold = 10.5;//9.5; //was 7.3025;
double leftWallThreshold = 10.5;//9.5; //7.3025;



//for debugging sensors
int frontLight =0;
int rightLight = A5;
int leftLight = A4;
int flagLight = 1;


/*******************STATE VARIBLES***************************************/
int orientationState = 0;
int lineState = 0;
int intersectionState = 0;

/*
lineState definitions:
0: initial point. both front sensors on line 
1: robot is slightly right of line
2: robot is slightly left of line
3: robot is far right of line
4: robot is far left of line
*/

/*
intersectionState definitions: (yes, the ordering sucks and is stupid... my bad)
0: initial point. all 3 sensors do not detect wall
1: only both side sensors detect wall
2: only front and right side sensor detect wall
3: only front and left side sensor detect wall
4: only front side sensor detects wall
5: all 3 sensors do detect wall
6: only right sensor detects wall
7: only left sensor detects wall
*/

/*
orientationState definitions:
0: robot is facing south
1: robot is facing west
2: robot is facing north
3: robos is facing east
*/
/*
xCoordinate at top left is 0, increments upwards as you walk down the matrix column
yCoordinate at top left is 0, increments upwards as you walk across the matrix row
*/

/************************************************************/

/***********************THE MATRIX********************************************************/

int xCoordinate = 1; //this is the xCoordinate that we are GOING to. depending on HOW the robot is orientaed at the start,
                    //does the robot start just before or jst after the intersection (we can't have it start at the intersection)
                    //if robot starts just before the first intersection, then the xCoordinate should be 0.
                    //if the robot starts just after the first intersection, then the xCoordinate should be 1
         
                    //UPDATE: robot starts just after the intersection                    
int yCoordinate = 0;

boolean intersectionMatrix[5][4][6] = 
{
  { {true, true, false, false, true,false}, {false, true, false, false, false,false}, {false, true, false, false, false,false}, {false, true, false, true, false,false} },
  { {false, false, false, false, true,false}, {false, false, false, false, false,false}, {false, false, false, false, false,false}, {false, false, false, true, false,false} },
  { {false, false, false, false, true,false}, {false, false, false, false, false,false}, {false, false, false, false, false,false}, {false, false, false, true, false,false} },
  { {false, false, false, false, true,false}, {false, false, false, false, false,false}, {false, false, false, false, false,false}, {false, false, false, true, false,false} },
  { {false, false, true, false, true,false}, {false, false, true, false, false,false}, {false, false, true, false, false,false}, {false, false, true, true, false,false} }
};

//notation: {explored, top, bottom, right, left} => {explored, north, south, east, west, unexplorable}
//false indicates not explored,..  no wall at that position
//matrix is preset to account for outer walls
//false indicates not explored,..  no wall at that position, not yet deemed unexplorable
//true indicates explored,.. wall at that position, deemed unexplorable


/*****************************************************************************************/

volatile int T1capture = 0;
volatile int period = 0;
volatile int lastT1capture = 0;
volatile int treasure_consistancy = 0;
volatile int found_treasure = 0;
volatile int count =0;
volatile int ISRin = 0;
int TrY = -1;
int TrX = -1;

ISR (ANALOG_COMP_vect)
{
      period = TCNT2;
      TCNT2 = 0;
      if(period > 75 && period < 87) 
      {
        treasure_consistancy++;
           if(treasure_consistancy == 7)
           {
             found_treasure = 1;
             TrX = xCoordinate;
             TrY = yCoordinate;
             ACSR -= (1 << ACIE);        
           }
      }
      else
      {
        treasure_consistancy = 0;
      }
    
}

void setup() {
  // put your setup code here, to run once:
    ACSR =  (1<<ACIE) |  (1<<ACIS1) |  (1<<ACIS0); 
    TCCR2B = 2; // running with clock divided by 8 
    TCCR2A=0;
    while(start_execution == 0){
     check_for_start();
     }  
   printf_begin();
   myStack.push(0);
   myStack.push(0); //push coordinates (0,0) onto stack
   pinMode(rightServo, OUTPUT);
   pinMode(leftServo,OUTPUT);
   pinMode(selectA,OUTPUT);
   pinMode(selectB,OUTPUT);
   pinMode(selectC,OUTPUT);
   myServoRight.attach(rightServo);
   myServoLeft.attach(leftServo);
   pinMode(13,OUTPUT);
   pinMode(frontLight,OUTPUT);
   pinMode(flagLight,OUTPUT);
   digitalWrite(frontLight,LOW);
   digitalWrite(flagLight,LOW);
   analogWrite(rightLight,0);
   analogWrite(leftLight,0);
   
   //channel 0. look for wall on left at very start
   digitalWrite(selectA,LOW);
   digitalWrite(selectB,LOW);
   digitalWrite(selectC,LOW);
   double leftSensorValue = analogRead(muxRead);
   double leftSensorDistance = convertToDistance(leftSensorValue);  //leftSensorDistance is distance between left sensor and wall in centimeters
   leftWall = leftSensorDistance <= leftWallThreshold;
   if (leftWall){
    intersectionMatrix[0][0][3]=true;
    intersectionMatrix[0][1][4]=true;
    myStack.push(1); //push (1,0) onto stack
    myStack.push(0);
    nodesToVisit = 1;
   }
   else{// if (!leftWall){
    digitalWrite(selectA, HIGH); //channel 1, read front sensor
    double frontSensorValue = analogRead(muxRead);
    double frontSensorDistance = convertToDistance(frontSensorValue); //frontSensorDistance is distance between front sensor and wall in centimeters
    frontWall = frontSensorDistance <= frontWallThreshold;
    if (frontWall){
      turn_left();
      orientationState =3; //going east
      xCoordinate = 0;
      yCoordinate =1;
      intersectionMatrix[0][0][2]=true;
      intersectionMatrix[1][0][1]=true;
      myStack.push(0); //push (0,1) onto stack
      myStack.push(1);
      nodesToVisit = 1;
    }
    else{ //no left wall and no front wall
      myStack.push(1); //push (1,0)
      myStack.push(0);
      nodesToVisit = 2;
  } 
 }
  radio.begin();                 //sets up radio
  radio.setRetries(15,15);
  radio.setAutoAck(true);
  radio.setChannel(0x50);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
      
   if ( role == role_ping_out )             //sets reading and writing pipes
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
  }
  else
  {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
  }
  
  
  radio.startListening();             //open to receiving
  radio.printDetails();

}

void check_for_start()
   {
      check_override();
      if(start_execution == 0)
     {
         check_660();
      }
   }
     void check_override()
  {
    int temp;
    
    temp = analogRead(A1);
    if( 1000 < temp )
    {
      start_execution = 1;
    }
  } 
  
  void check_660()
   {
     for( int i = 0; i < 256; i +=2)                                                             //get input data
     {
        fft_input[i] = analogRead(A0);
        fft_input[i+1] = 0;
     }  
    
    fft_window();                                                                                //does the reordering algorithm and the fft
    fft_reorder();
    fft_run();
    fft_mag_lin();
    
    
    signal = maximum();                                                                              //calculates the maximum in the signal  
    if(((signal == 9) || (signal == 10) || (signal == 11)|| (signal==12)) && (fft_lin_out[signal] > 10))           //checks for consistency in the maximum and if its around 660 Hz
    {
      consistency = consistency + 1;
      if(consistency >= 20)
         start_execution = 1;                                                          //says that the input frequency is 660 Hz. 
    }
    else
    {
      consistency = 0;
      start_execution = 0;
    }
   }
  
   
   int maximum()                                                                                      //computes the maximum of the fft_out[]
   {
     int i;
     int index = 5;
     for(i = 5; i < 50; i++)
     {
        if(fft_lin_out[index] < fft_lin_out[i])
          index = i;
     }
     
     return index;
   }
    
boolean at_intersection(){
  if (FR && FL && OR && OL){ //all sensors are on the line{
    return true;
  }
  else{
    return false;
  }
}

void stay_straight() //go straight
{  
  myServoLeft.write(180);
  myServoRight.write(0);
}

void veer_right()
{
  //right wheel slows down
  myServoLeft.write(170);
  myServoRight.write(85);
  
}

void veer_left()
{
  //left wheel slows down
  myServoLeft.write(95);
  myServoRight.write(10);
}


void stopRobot(){
  while (finished){
    myServoLeft.write(90);
    myServoRight.write(90);
    //radio_transmit();
  }
}

void turn_left()
{
  //stop left wheel
  //keep going with right wheel
  myServoLeft.write(90);
  myServoRight.write(0);
  //delay(1200); //initial turning delay was 1200
  delay(300);
  
  //set to channel 5 - read front right sensor
  digitalWrite(selectA, HIGH);
  digitalWrite(selectB,LOW);
  digitalWrite(selectC,HIGH);
  

  while (analogRead(muxRead)*.0049<4){ //while both front sensors are off line
     myServoLeft.write(90);
     myServoRight.write(0);
  }
  stay_straight(); 
}

void turn_right()
{
  //stop right wheel
  //keep going with left wheel
  myServoLeft.write(180);
  myServoRight.write(90);
  //delay(1200); //initial turning delay used to be 1200
  delay(300);
  
  //set to channel 4 - read front left sensor
  digitalWrite(selectA, LOW);
  digitalWrite(selectB,LOW);
  digitalWrite(selectC,HIGH);
  
  while (analogRead(muxRead)*.0049<4){ //while both front sensors are off line
     myServoLeft.write(180);
     myServoRight.write(90);
  }
  stay_straight();
}

void turn_180degrees(){
  myServoLeft.write(180);
  myServoRight.write(180);
  delay(250); //delay so that robot gets past intersection
  //channel 6 - outside right sensor
  digitalWrite(selectA,LOW);
  digitalWrite(selectB,HIGH);
  digitalWrite(selectC,HIGH);
  
  while (analogRead(muxRead)*.0049>4){ //while outside right sensor is ON black line
    //this is just so that we don't count the line that we are currently on
  }
  //start turning around clockwise
 // myServoLeft.write(180);
  //myServoRight.write(180);
  
  //channel 6 - outside right sensor
  digitalWrite(selectA,LOW);
  digitalWrite(selectB,HIGH);
  digitalWrite(selectC,HIGH);
  
  while (analogRead(muxRead)*.0049 <4){ //while outside right sensor is NOT on black line
  }
  
  //channel 3 - outside left sensor
  digitalWrite(selectA,HIGH);
  digitalWrite(selectB,HIGH);
  digitalWrite(selectC,LOW);
  delay(75);
  while (analogRead(muxRead)*.0049<4){ //while outside left sensor is not on black line
  }
 
  //channel 6 - outside right sensor
  digitalWrite(selectA,LOW);
  digitalWrite(selectB,HIGH);
  digitalWrite(selectC,HIGH);
  
  //slow down
  myServoLeft.write(150);
  myServoRight.write(135);
  while (analogRead(muxRead)*.0049<4){ //while outside right sensor is not on black line
  }
  //slow down more
  myServoLeft.write(140);
  myServoRight.write(120);
  while (analogRead(muxRead)*.0049>4){ //while outside right sensor IS ON black line
  }
  
   //set to channel 5 - read front right sensor
  digitalWrite(selectA, HIGH);
  digitalWrite(selectB,LOW);
  digitalWrite(selectC,HIGH);
  while (analogRead(muxRead)*.0049<4){ //while front right sensor is not on black line
  }
  //set to channel 4 - read front left sensor
  digitalWrite(selectA, LOW);
  digitalWrite(selectB,LOW);
  digitalWrite(selectC,HIGH);
  if (analogRead(muxRead)*.0049>4){ //if front left sensor is on black line
     myServoLeft.write(90);
     myServoRight.write(90);
  }
  myServoLeft.write(90);
  myServoRight.write(90);
  delay(100);
  stay_straight();
}

void turn_180degrees2(){
  //turn clockwise
  //keep moving forward with left wheel
  //reverse right wheel
  myServoLeft.write(90);
  myServoRight.write(90);
  myServoLeft.write(180); //
  myServoRight.write(180);
  delay(150);

  //set to channel 4 - read front left sensor
  digitalWrite(selectA, LOW);
  digitalWrite(selectB,LOW);
  digitalWrite(selectC,HIGH);
  
  int howManyLines=0;
  while (analogRead(muxRead)*.0049<4){//while left front sensor is off line
  }
  howManyLines = 1;
  
  delay(200);
  while (howManyLines == 1){
     myServoLeft.write(180);
     myServoRight.write(180);
     if (analogRead(muxRead)*.0049>4){//left front sensor is on line
       howManyLines=2; //break out of while loop, and method ends
     }
  }
  //digitalWrite(flagLight,HIGH);

  stay_straight();
}

double convertToDistance(double sensorValue){
  /*
  input: analog value on sensor pin [0-1024)
  output: analog value converted to centimeters
  this is meant exclusively for the wall sensors
  */
  double newValue =sensorValue * 0.0049; //converts value of 0-1024 to 0-5Volts
  double distanceFromWall = (1/newValue);
  distanceFromWall = distanceFromWall*12.75;
  distanceFromWall = distanceFromWall-0.4; //uses line of best fit to calculate distance in centimeters
  return distanceFromWall; 
}

int intersectionStateNumberGenerator(){
  //turn based on wall information - basically, this won't hit walls
  if (intersectionState == 0){ //all 3 sensors do not detect wall
    return 0;// random(0,3);
  }
  else if (intersectionState ==1){ //1: only both side sensors detect wall
    return 0; //go straight
  }
  else if (intersectionState ==2){ //2: only front and right side sensor detect wall
    return 1; //turn left
  }
  else if (intersectionState == 3){ //3: only front and left side sensor detect wall
    return 2; //turn right
  }
  else if (intersectionState==4){ //4: only front sensor detects wall
    //digitalWrite(13,HIGH);
    return 2; //turn right
  }
  else if (intersectionState ==5){ //5: all 3 sensors do detect wall
  //must turn around
    return 3;
  //turnaround
  }
  else if (intersectionState ==6){ //6: only right sensor detects wall
    return 0; //go straight
  }
  else if (intersectionState==7){ //7: only left sensor detects wall
    return 0; //go straight
  }
}

void updateIntersection(){ //sensor distances are all GLOBAL variables
 
   if (!frontWall && !rightWall && !leftWall){ //no sensor detects a wall
     intersectionState = 0; 
   }  
   else if (!frontWall && rightWall && leftWall){ //only right and left sensor detects wall
     intersectionState = 1;
   }
   else if (frontWall && rightWall && !leftWall){ //only right and front sensor detects wall
     intersectionState = 2;
   }
   else if (frontWall && !rightWall && leftWall){ //only front and left sensor detect wall
     intersectionState = 3;
   }
   else if (frontWall && !rightWall && !leftWall){ //only front sensor detecs wall
     intersectionState = 4;
   }
   else if (frontWall && rightWall && leftWall){ //all 3 sensors detect wall
     intersectionState = 5;
   }
   else if(!frontWall && rightWall && !leftWall){ //only right sensor detects wall
     intersectionState = 6;
   }
   else if(!frontWall && !rightWall && leftWall){ //only left sensor detects wall
     intersectionState = 7;
   }
}
void updateOuterWalls(){
  if (xCoordinate ==4 && orientationState == 0){ //going south into southern wall
     frontWall=true;
   }
   else if(xCoordinate==0 && orientationState == 2){ //going north into northern wall
     frontWall=true;
   }
   else if (yCoordinate == 0 && orientationState == 1){ //going west into western wall
     frontWall = true;
   }
   else if (yCoordinate == 3 && orientationState == 3){ //going east into eastern wall 
     frontWall=true;
   }
   if (yCoordinate ==3 && orientationState == 0){ //going south on east edge of maze
     leftWall = true;
   }
   else if (yCoordinate == 3 && orientationState == 2){ //going north on east edge of maze
     rightWall = true;
   }
   else if (yCoordinate == 0 && orientationState == 0){ //going south on west edge of maze
     rightWall = true;
   }
   else if (yCoordinate == 0 && orientationState == 2){ //going north on west edge of maze
     leftWall = true;
   }
   else if (xCoordinate == 0 && orientationState == 1){ //going west on top wall
     rightWall = true;
   } 
   else if (xCoordinate ==0 && orientationState == 3){ //going east on top wall
     leftWall = true;
   }
   else if(xCoordinate == 4 && orientationState == 1){ //going west on bottom wall
     leftWall=true;
   }
   else if (xCoordinate == 4 && orientationState == 3){ //going east on bottom wall
     rightWall = true;
   }

}

String myFunct(){
  return "hi";

}

void turnOrGoStraight(int whichDirection){
    //Serial.println(whichDirection); 
    if (whichDirection == 0)
    {
      stay_straight();
      delay(200); //give sensors time to pass the intersection
    }
    else if (whichDirection == 1)
    {
      delay(50); //allow robot to go a bit past intersection 
      turn_left();
    }
    //whichDirection must have value of 2
    else if (whichDirection == 2){
      delay(50); //allow robot to go a bit past intersection 
      turn_right();
    }
    else if (whichDirection == 4){
      stopRobot();
    }
    else{ //whichDirection must have a value of 3
      //need to turn 180 degrees
      turn_180degrees();
    
    }
  }
  
  
  
  
/***********************FUNCTION TO UPDATE THE MATRIX************************/
void populateIntersectionMatrix(){ 
  /*
  all of these variables are GLOBAL - int intersectionState,int orientationState, int xCoordinate, int yCoordinate)
  this method serves to update the intersectionMatrix's wall variables and the explored field of its current position (it doesn't update the xCoordinate, yCoordinate)
  robot just hit intersection now. it has not yet determined which direction it will turn in
  orientationState and xCoordinate and yCoordinate represent the robot's CURRENT status
  intersectionState is updated (as in, it reflects the CURRENT intersection)
  populate the intersectoinMatrix accoridngly using these global variables
  */

  intersectionMatrix[xCoordinate][yCoordinate][0] = true; //set this current position to EXPLORED regardless of orientation
  if (intersectionState == 0){ //all 3 sensor do NOT detect wall
    //nothing to update  
  }
  else if (intersectionState == 1){ //only both side sensors detect wall
    if (orientationState == 0 || orientationState == 2){ //robot is facing south OR north
      intersectionMatrix[xCoordinate][yCoordinate][3] = true; //set right wall true
      intersectionMatrix[xCoordinate][yCoordinate][4]= true; //set left wall true
      
      if (yCoordinate!=0){
        intersectionMatrix[xCoordinate][yCoordinate-1][3] = true; //set right wall of left coordinate true
      }
      if (yCoordinate!=3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4] = true; //set left wall of right coordinate true
      }
    } 
    else{ //orientationState must be 1 or 3 //robot is facing east OR west
      intersectionMatrix[xCoordinate][yCoordinate][1]= true;
      intersectionMatrix[xCoordinate][yCoordinate][2]= true;
      
      if (xCoordinate !=4){
        intersectionMatrix[xCoordinate+1][yCoordinate][1]=true;
      }
      if (xCoordinate != 0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2]=true;
      }
    }
  }
  else if (intersectionState == 2){ //only front and right side sensor detect wall
    if (orientationState == 0){ //facing south
      intersectionMatrix[xCoordinate][yCoordinate][2]=true; //bottom
      intersectionMatrix[xCoordinate][yCoordinate][4]=true; //left
      if (xCoordinate != 4){
        intersectionMatrix[xCoordinate+1][yCoordinate][1]=true; // set top wall of lower coordinate
      }
      if (yCoordinate != 0){
        intersectionMatrix[xCoordinate][yCoordinate-1][3]=true; //set right wall of left coordinate
      }
    }
    else if (orientationState == 1){ //facing west
      intersectionMatrix[xCoordinate][yCoordinate][1]=true; //top
      intersectionMatrix[xCoordinate][yCoordinate][4]= true; //left
      if (yCoordinate!=0){
        intersectionMatrix[xCoordinate][yCoordinate-1][3]=true; //set right wall of left coordinate
      }
      if (xCoordinate != 0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2]=true; //set bottom wall of upper coordinate
      }
    }
    else if (orientationState ==2){ //robot is facing north
      intersectionMatrix[xCoordinate][yCoordinate][1]=true; //top
      intersectionMatrix[xCoordinate][yCoordinate][3]=true; //right
      if (yCoordinate !=3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4] = true; //set left wall of right coordinate
      }
      if (xCoordinate !=0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2] = true; //set bottom wall of upper coordinate
      }
    }
    else if (orientationState == 3){ //robot is facing east
      intersectionMatrix[xCoordinate][yCoordinate][3] = true; //right 
      intersectionMatrix[xCoordinate][yCoordinate][2] = true; //bottom
      if (yCoordinate !=3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4] = true; //set left wall of right coordinate
      }
      if (xCoordinate != 4){
        intersectionMatrix[xCoordinate+1][yCoordinate][1] = true; //set top wall of lower coordinate
      }
    }
  }
  
  else if (intersectionState == 3){ //only front and left side sensor detect wall

    if (orientationState == 0){ //robot is facing south
      intersectionMatrix[xCoordinate][yCoordinate][2]=true; //bottom
      intersectionMatrix[xCoordinate][yCoordinate][3]=true; //right
      
      if (xCoordinate != 4){
        intersectionMatrix[xCoordinate+1][yCoordinate][1]=true; //set top wall of lower coordinate
      }
      if (yCoordinate != 3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4]; //set left wall of right coordinate
      }
    }
    else if (orientationState == 1){ //facing west
      intersectionMatrix[xCoordinate][yCoordinate][4]=true; //left
      intersectionMatrix[xCoordinate][yCoordinate][2]=true; //bottom
      if (xCoordinate !=4){
        intersectionMatrix[xCoordinate+1][yCoordinate][1]=true; //set top wall of lower coordinate
      }
      if (yCoordinate !=0){
        intersectionMatrix[xCoordinate][yCoordinate-1][3]=true; //set right wall of left coordinate
      }
    
    }
    else if (orientationState ==2){ //robot facing north
      intersectionMatrix[xCoordinate][yCoordinate][1]=true; //top
      intersectionMatrix[xCoordinate][yCoordinate][4]=true; //left
      if (xCoordinate != 0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2]=true; //set bottom wall of upper coordinate
      }
      if (yCoordinate !=0){
        intersectionMatrix[xCoordinate][yCoordinate-1][3] = true; //set right wall of left coordinate
      }
      
    }
    else if (orientationState == 3){//robot is facing east
      intersectionMatrix[xCoordinate][yCoordinate][3]=true; //right
      intersectionMatrix[xCoordinate][yCoordinate][1]=true;
      if (xCoordinate != 0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2]=true; //bottom wall of upper coordinate
      }  
      if (yCoordinate !=3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4]= true; //left wall of right coordinate
      }  
    }
  }
  
  else if (intersectionState == 4){ //only front sensor detects wall
    if (orientationState == 0){ //robot is facing south
     intersectionMatrix[xCoordinate][yCoordinate][2]=true; //bottom
     if (xCoordinate != 4){
       intersectionMatrix[xCoordinate+1][yCoordinate][1] = true; //top wall of lower coordinate
     }
    }
    else if (orientationState == 1){ //robot is facing west
      intersectionMatrix[xCoordinate][yCoordinate][4]=true; //left
      if (yCoordinate!=0){
        intersectionMatrix[xCoordinate][yCoordinate-1][3]=true; //set right wall of left coordinate
      }
    }
    else if (orientationState ==2){//robot is facing north
     intersectionMatrix[xCoordinate][yCoordinate][1]=true; //top
      if (xCoordinate!=0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2] = true; //bottom wall of upper coordinate
      }
    }
    else if (orientationState == 3){ //robot is facing east
      intersectionMatrix[xCoordinate][yCoordinate][3]=true; //right
      if (yCoordinate!=3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4]=true; //set left wall of right coordinate
      }
    }
  }
  else if (intersectionState == 5){//all 3 sensors detect wall
    if (orientationState == 0){ //robot facing south
      intersectionMatrix[xCoordinate][yCoordinate][2] = true; //bottom
      intersectionMatrix[xCoordinate][yCoordinate][3] = true; //right
      intersectionMatrix[xCoordinate][yCoordinate][4] = true;//left
      
      if (xCoordinate!=4){
        intersectionMatrix[xCoordinate+1][yCoordinate][1] = true; //top wall of lower coordinate
      }
      if (yCoordinate!=0){
        intersectionMatrix[xCoordinate][yCoordinate-1][3] = true; //right wall of left coordinate
      }
      if (yCoordinate!=3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4] = true; //left wall of right coordinate
      }
    }
    else if (orientationState == 1){ //facing west
      intersectionMatrix[xCoordinate][yCoordinate][2] = true; //bottom
      intersectionMatrix[xCoordinate][yCoordinate][1] = true; //top
      intersectionMatrix[xCoordinate][yCoordinate][4] = true;//left
       if (xCoordinate!=0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2] = true; //bottom wall of upper coordinate
      }
      if (xCoordinate!=4){
        intersectionMatrix[xCoordinate+1][yCoordinate][1] = true; //top wall of lower coordinate
      }
      if (yCoordinate!=0){
        intersectionMatrix[xCoordinate][yCoordinate-1][3] = true; //right wall of left coordinate
      }
    }
    else if (orientationState ==2){ //facing north
      intersectionMatrix[xCoordinate][yCoordinate][1] = true; //top
      intersectionMatrix[xCoordinate][yCoordinate][3] = true; //right
      intersectionMatrix[xCoordinate][yCoordinate][4] = true;//left
       if (xCoordinate!=0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2] = true; //bottom wall of upper coordinate
      }
      if (yCoordinate!=0){
        intersectionMatrix[xCoordinate][yCoordinate-1][3] = true; //right wall of left coordinate
      }
      if (yCoordinate!=3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4] = true; //left wall of right coordinate
      }
    }
    else if (orientationState == 3){ //facing east
      intersectionMatrix[xCoordinate][yCoordinate][2] = true; //bottom
      intersectionMatrix[xCoordinate][yCoordinate][3] = true; //right
      intersectionMatrix[xCoordinate][yCoordinate][1] = true;//top
       if (xCoordinate!=0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2] = true; //bottom wall of upper coordinate
      }
      if (xCoordinate!=4){
        intersectionMatrix[xCoordinate+1][yCoordinate][1] = true; //top wall of lower coordinate
      }
      if (yCoordinate!=3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4] = true; //left wall of right coordinate
      }
    }
  }
  else if (intersectionState == 6){ //only right sensor detects wall 
     if (orientationState ==0){ //facing south
       intersectionMatrix[xCoordinate][yCoordinate][4]=true; //left
       if (yCoordinate!=0){
         intersectionMatrix[xCoordinate][yCoordinate-1][3]==true; //right wall of left coordinate
       }
     }
    else if (orientationState ==1){ //facing west
      intersectionMatrix[xCoordinate][yCoordinate][1]=true; //top 
      if (xCoordinate!=0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2]=true; //bottom wall of top coordinate
      }
    }
    else if(orientationState ==2){ //facing north
      intersectionMatrix[xCoordinate][yCoordinate][3]=true; //right
      if (yCoordinate!=3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4]=true; //left wall of right coordinate
      }
    }
    else if (orientationState == 3){ //facing east
      intersectionMatrix[xCoordinate][yCoordinate][2]=true; //bottom
      if (xCoordinate!=4){
        intersectionMatrix[xCoordinate+1][yCoordinate][1]=true; //top wall of bottom coordinate
      }
    }
  }
  else if (intersectionState ==7){ //only left sensor detects wall
    if (orientationState ==0){ //facing south
      intersectionMatrix[xCoordinate][yCoordinate][3]=true; //right 
      if (yCoordinate!=3){
        intersectionMatrix[xCoordinate][yCoordinate+1][4] = true; //left wall of right coordinate
      }
    }
    else if(orientationState==1){ //facing west
      intersectionMatrix[xCoordinate][yCoordinate][2]=true; //bottom
    
      if (xCoordinate!=4){
        intersectionMatrix[xCoordinate+1][yCoordinate][1] = true; //top wall of bottom coordinate
      }
    }
    else if(orientationState == 2){ //facing north
      intersectionMatrix[xCoordinate][yCoordinate][4]=true; //left
      if (yCoordinate!=0){
        intersectionMatrix[xCoordinate][yCoordinate-1][3]=true; //right wall of left coordinate
      }
    }
    else if(orientationState == 3){ //facing east
      intersectionMatrix[xCoordinate][yCoordinate][1]=true; //top
      if (xCoordinate!=0){
        intersectionMatrix[xCoordinate-1][yCoordinate][2]=true; //bottom wall of top coordinate
      }
    } 
  }  
}

/******************************/

  
/**************UPDATE ORIENTATION AND COORDINATES************/

void updateOrientation(int whichDirection){
  /**
    updates orientationState of robot according to whichDirection, which determines which direction the robot
    is about to go (straight, right, left, or turnaround)
    for example, if robot WAS moving north and IS NOW turning left, then the robot will 
    be going west. this method updates orienatationState to reflect that properly 
    */
  if (whichDirection == 0){ //going straight
    orientationState = orientationState;
  }
  else if (whichDirection == 1){ //turning left
    if (orientationState !=0){
        orientationState = orientationState - 1;
    }
    else{
       orientationState = 3;
    }
  }
  else if (whichDirection == 2){ //turning right
   if (orientationState != 3){
      orientationState = orientationState +1;
   }
   else{
      orientationState=0;
   }
  }
  else if (whichDirection == 3){ //making a 180 degree turn
    if (orientationState == 1){
      orientationState = 3;
    }
    else if(orientationState == 0){
      orientationState = 2;
    }
    else{
      orientationState = orientationState -2;
    }
  
  }
}
  
  void updateCoordinates(){
    /**
     updates x and y coordinates of robot according to ONLY the newly updated orientationState
     sets x and y coordinates of robot to the x and y coordinates that IT IS NOW SET TO MOVE TOWARDS
     example: if robot is at (3,2), and it's orientationState is 1, then the robot is moving west so this
     updates the coordinates to (3,1) 
     */
    if (orientationState == 0){ //going south
      xCoordinate += 1;
      yCoordinate = yCoordinate;
    }
    else if (orientationState == 1){ //going west
      xCoordinate = xCoordinate;
      yCoordinate -=1;
    }
    else if (orientationState == 2){ //going north
      xCoordinate -= 1;
      yCoordinate = yCoordinate;
    }
    else{ //orientationState == 3 -- going east
      xCoordinate = xCoordinate;
      yCoordinate += 1;
    }
 }
 
 
 void radio_transmit(){   
  ImplementProtocol();

  if(role == role_ping_out )
  {
    bool fail_1 = true;
    bool fail_2 = true;

      fail_1= TransmitData(temp1);

      fail_2= TransmitData(temp2);

     };
  
}

bool TransmitData(unsigned char data[32])
{
  radio.stopListening();
  
   printf("Now sending  data...\n");                    //sending data
   bool ok = radio.write( data, sizeof(unsigned char)*32 );

   if (ok)                                                //printing if data is sent
     printf("Sent ok...\n");
   else
     printf("Sent failed.\n\r");
     
     
   radio.startListening();                              //set to reveice confirmation
  
  
    unsigned long started_waiting_at = millis();           // confirm sent message 
    bool timeout = false;
    while ( ! radio.available() && !timeout )
    {  
      if (millis() - started_waiting_at > 30 )
        timeout = true; 
    }
  
   if ( timeout )    
    {
      printf("Failed, response timed out. Not received confirmation. \n\r");
    }
    else
    {
      unsigned char got_data[32];
      radio.read( got_data, sizeof(unsigned char)*32);
      printf("Confirmed sent \n");
    }
     return timeout;
}

void ImplementProtocol()
{
  //open space
  temp1[0] = intersectionMatrix[0][3][0] ? 1: (intersectionMatrix[0][3][5] ? 4:0);
  temp1[2] = intersectionMatrix[1][3][0] ? 1: (intersectionMatrix[1][3][5] ? 4:0);
  temp1[4] = intersectionMatrix[2][3][0] ? 1: (intersectionMatrix[2][3][5] ? 4:0);
  temp1[6] = intersectionMatrix[3][3][0] ? 1: (intersectionMatrix[3][3][5] ? 4:0);
  temp1[8] = intersectionMatrix[4][3][0] ? 1: (intersectionMatrix[4][3][5] ? 4:0);
  
  temp1[18] = intersectionMatrix[0][2][0] ? 1: (intersectionMatrix[0][2][5] ? 4:0);
  temp1[20] = intersectionMatrix[1][2][0] ? 1: (intersectionMatrix[1][2][5] ? 4:0);
  temp1[22] = intersectionMatrix[2][2][0] ? 1: (intersectionMatrix[2][2][5] ? 4:0);
  temp1[24] = intersectionMatrix[3][2][0] ? 1: (intersectionMatrix[3][2][5] ? 4:0);
  temp1[26] = intersectionMatrix[4][2][0] ? 1: (intersectionMatrix[4][2][5] ? 4:0);
  
  temp2[5] = intersectionMatrix[0][1][0] ? 1: (intersectionMatrix[0][1][5] ? 4:0);
  temp2[7] = intersectionMatrix[1][1][0] ? 1: (intersectionMatrix[1][1][5] ? 4:0);
  temp2[9] = intersectionMatrix[2][1][0] ? 1: (intersectionMatrix[2][1][5] ? 4:0);
  temp2[11] = intersectionMatrix[3][1][0] ? 1: (intersectionMatrix[3][1][5] ? 4:0);
  temp2[13] = intersectionMatrix[4][1][0] ? 1: (intersectionMatrix[4][1][5] ? 4:0);
  
  temp2[23] = intersectionMatrix[0][0][0] ? 1: (intersectionMatrix[0][0][5] ? 4:0);
  temp2[25] = intersectionMatrix[1][0][0] ? 1: (intersectionMatrix[1][0][5] ? 4:0);
  temp2[27] = intersectionMatrix[2][0][0] ? 1: (intersectionMatrix[2][0][5] ? 4:0);
  temp2[29] = intersectionMatrix[3][0][0] ? 1: (intersectionMatrix[3][0][5] ? 4:0);
  temp2[31] = intersectionMatrix[4][0][0] ? 1: (intersectionMatrix[4][0][5] ? 4:0);
  
  //walls
  temp1[1] = (!intersectionMatrix[0][3][0] && !intersectionMatrix[1][3][0]) ? 0 : (intersectionMatrix[0][3][2] ? 2:1);
  temp1[3] = (!intersectionMatrix[1][3][0] && !intersectionMatrix[2][3][0]) ? 0 : (intersectionMatrix[1][3][2] ? 2:1);
  temp1[5] = (!intersectionMatrix[2][3][0] && !intersectionMatrix[3][3][0]) ? 0 : (intersectionMatrix[2][3][2] ? 2:1);
  temp1[7] = (!intersectionMatrix[3][3][0] && !intersectionMatrix[4][3][0]) ? 0 : (intersectionMatrix[3][3][2] ? 2:1);

  temp1[9] = (!intersectionMatrix[0][3][0] && !intersectionMatrix[0][2][0]) ? 0 : (intersectionMatrix[0][3][4] ? 2:1);
  temp1[11] = (!intersectionMatrix[1][3][0] && !intersectionMatrix[1][2][0]) ? 0 : (intersectionMatrix[1][3][4] ? 2:1);
  temp1[13] = (!intersectionMatrix[2][3][0] && !intersectionMatrix[2][2][0]) ? 0 : (intersectionMatrix[2][3][4] ? 2:1);
  temp1[15] = (!intersectionMatrix[3][3][0] && !intersectionMatrix[3][2][0]) ? 0 : (intersectionMatrix[3][3][4] ? 2:1);
  temp1[17] = (!intersectionMatrix[4][3][0] && !intersectionMatrix[4][2][0]) ? 0 : (intersectionMatrix[4][3][4] ? 2:1);
  
  temp1[19] = (!intersectionMatrix[0][2][0] && !intersectionMatrix[1][2][0]) ? 0 : (intersectionMatrix[0][2][2] ? 2:1);
  temp1[21] = (!intersectionMatrix[1][2][0] && !intersectionMatrix[2][2][0]) ? 0 : (intersectionMatrix[1][2][2] ? 2:1);
  temp1[23] = (!intersectionMatrix[2][2][0] && !intersectionMatrix[3][2][0]) ? 0 : (intersectionMatrix[2][2][2] ? 2:1);
  temp1[25] = (!intersectionMatrix[3][2][0] && !intersectionMatrix[4][2][0]) ? 0 : (intersectionMatrix[3][2][2] ? 2:1);

  temp1[27] = (!intersectionMatrix[0][2][0] && !intersectionMatrix[0][1][0]) ? 0 : (intersectionMatrix[0][2][4] ? 2:1);
  temp1[29] = (!intersectionMatrix[1][2][0] && !intersectionMatrix[1][1][0]) ? 0 : (intersectionMatrix[1][2][4] ? 2:1);
  temp1[31] = (!intersectionMatrix[2][2][0] && !intersectionMatrix[2][1][0]) ? 0 : (intersectionMatrix[2][2][4] ? 2:1);
  temp2[2] = (!intersectionMatrix[3][2][0] && !intersectionMatrix[3][1][0]) ? 0 : (intersectionMatrix[3][2][4] ? 2:1);
  temp2[4] = (!intersectionMatrix[4][2][0] && !intersectionMatrix[4][1][0]) ? 0 : (intersectionMatrix[4][2][4] ? 2:1);
  
  temp2[6] = (!intersectionMatrix[0][1][0] && !intersectionMatrix[1][1][0]) ? 0 : (intersectionMatrix[0][1][2] ? 2:1);
  temp2[8] = (!intersectionMatrix[1][1][0] && !intersectionMatrix[2][1][0]) ? 0 : (intersectionMatrix[1][1][2] ? 2:1);
  temp2[10] = (!intersectionMatrix[2][1][0] && !intersectionMatrix[3][1][0]) ? 0 : (intersectionMatrix[2][1][2] ? 2:1);
  temp2[12] = (!intersectionMatrix[3][1][0] && !intersectionMatrix[4][1][0]) ? 0 : (intersectionMatrix[3][1][2] ? 2:1);

  temp2[14] = (!intersectionMatrix[0][0][0] && !intersectionMatrix[0][1][0]) ? 0 : (intersectionMatrix[0][1][4] ? 2:1);
  temp2[16] = (!intersectionMatrix[1][0][0] && !intersectionMatrix[1][1][0]) ? 0 : (intersectionMatrix[1][1][4] ? 2:1);
  temp2[18] = (!intersectionMatrix[2][0][0] && !intersectionMatrix[2][1][0]) ? 0 : (intersectionMatrix[2][1][4] ? 2:1);
  temp2[20] = (!intersectionMatrix[3][0][0] && !intersectionMatrix[3][1][0]) ? 0 : (intersectionMatrix[3][1][4] ? 2:1);
  temp2[22] = (!intersectionMatrix[4][0][0] && !intersectionMatrix[4][1][0]) ? 0 : (intersectionMatrix[4][1][4] ? 2:1);
  
  temp2[24] = (!intersectionMatrix[0][0][0] && !intersectionMatrix[1][0][0]) ? 0 : (intersectionMatrix[0][0][2] ? 2:1);
  temp2[26] = (!intersectionMatrix[1][0][0] && !intersectionMatrix[2][0][0]) ? 0 : (intersectionMatrix[1][0][2] ? 2:1);
  temp2[28] = (!intersectionMatrix[2][0][0] && !intersectionMatrix[3][0][0]) ? 0 : (intersectionMatrix[2][0][2] ? 2:1);
  temp2[30] = (!intersectionMatrix[3][0][0] && !intersectionMatrix[4][0][0]) ? 0 : (intersectionMatrix[3][0][2] ? 2:1);
  
  //Don't cares -- Signal DONE or not
  for(int i=10; i<17; i+=2) {
    temp1[i] = (finished == 0) ? 0:5;
  }
  for(int i=17; i<23; i+=2) {
    temp2[i] = (finished == 0) ? 0:5;
  }
  temp1[28] = (finished == 0) ? 0:5;
  temp1[30] = (finished == 0) ? 0:5;
  temp2[1] = (finished == 0) ? 0:5;
  temp2[3] = (finished == 0) ? 0:5;
 
  //PUT TREASURE IN
  
 if(TrY == 3) {
    temp1[2*TrX] = 7;
  }
  else if (TrY == 2){
    temp1[18+2*TrX] = 7;
  }
  else if(TrY ==1) {
    temp2[5 + 2*TrX] = 7;
  }
  else if(TrY ==0) {
    temp2[23+ 2*TrX] = 7;
  }
  //PUT CURRENT LOCATION
  if(yCoordinate == 3) {
    temp1[2*xCoordinate] = 6;
  }
  else if (yCoordinate == 2){
    temp1[18+2*xCoordinate] = 6;
  }
  else if(yCoordinate ==1) {
    temp2[5 + 2*xCoordinate] = 6;
  }
  else {//yCoord = 0
    temp2[23+ 2*xCoordinate] = 6;
  }
  temp2[0] = 10;
} 

int checkNodesToVisit(){
  if (nodesToVisit == 0){ //finished
    finished = 1;
    for (int i =0; i<5; i++){
      for (int j = 0; j<4; j++){
        if (!intersectionMatrix[i][j][0]){ //if the intersection is still unexplored
          intersectionMatrix[i][j][5]=true; //set it to unexplorable
        }
      }
    }
    analogWrite(rightLight,0);
    analogWrite(leftLight,0);
    digitalWrite(frontLight,LOW);
    digitalWrite(flagLight,HIGH);
    myServoLeft.write(90);
    myServoRight.write(90);
    for (int i =0; i<50; i++){
      radio_transmit();
    }
    stopRobot();
    return 4; //sets whichDirection to 4. stops robot
  }
}

int checkEmptyStack(){
  if (myStack.isEmpty()){
    //set unepxlored spots to unexplorable
    //robot stops
    //turn on light
    finished = 1;
    for (int i =0; i<5; i++){
      for (int j = 0; j<4; j++){
        if (!intersectionMatrix[i][j][0]){ //if the intersection is still unexplored
          intersectionMatrix[i][j][5]=true; //set it to unexplorable
        }
      }
    }
    //digitalWrite(,LOW);
    digitalWrite(frontLight,LOW);
    analogWrite(rightLight,0);
    analogWrite(leftLight,0);
    digitalWrite(flagLight,HIGH);
    myServoLeft.write(90);
    myServoRight.write(90);
    for (int i =0; i<50; i++){
      radio_transmit();
    }
    stopRobot();
    return 4; //sets whichDirection to 4. stops robot
  }
}

int depthFirstSearch(){
  findAdjacentUnexploredCoordinate(); //find an adjacent unexplored coordinate of robot's current location
  if (wasVisited){ //if wasViisted == true, then this node was already visited before the robot got there
    nodesToVisit = oldNodesToVisit;
  }
  checkNodesToVisit(); //should robot stop?
  updateStack();
  checkEmptyStack(); //should robot stop? this is a safety net!
  else{
    int nextY = myStack.pop();
    int nextX = myStack.peek();
    myStack.push(nextY);
    return goToNumber(nextX,nextY); //go to coordinates (nextX,nextY)
  }  
}

void findAdjacentUnexploredCoordinate(){
  //find the adjacent unexplored coordinates of the robot's current location
  //populatie adjacentUnexploredCoordinate[2] global variable
  noAdjacentUnexplored = true; //true if there are no adjacent unexplored coordinates
  if (intersectionState == 0){ //all 3 sensors do not detect wall
    if (frontOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (rightOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (leftOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (frontOpen()){ //add front
      adjacentUnexploredCoordinate[0]=frontCoordinate[0]; //xCoordinate
      adjacentUnexploredCoordinate[1]=frontCoordinate[1]; //yCoordinate
      noAdjacentUnexplored = false; 
    }
    else{
      int randomizer = random(0,2);
      if (randomizer == 0){
        if (rightOpen()){
          adjacentUnexploredCoordinate[0]=rightCoordinate[0];
          adjacentUnexploredCoordinate[1]=rightCoordinate[1];
          noAdjacentUnexplored = false;
        }
        else if (leftOpen()){
          adjacentUnexploredCoordinate[0]=leftCoordinate[0];
          adjacentUnexploredCoordinate[1]=leftCoordinate[1];
          noAdjacentUnexplored = false;
        }
      }
      else{
       if (leftOpen()){
         adjacentUnexploredCoordinate[0]=leftCoordinate[0];
         adjacentUnexploredCoordinate[1]=leftCoordinate[1];
         noAdjacentUnexplored = false;
       }
       else if (rightOpen()){
          adjacentUnexploredCoordinate[0]=rightCoordinate[0];
          adjacentUnexploredCoordinate[1]=rightCoordinate[1];
          noAdjacentUnexplored = false;
       }
      }
    }
  }
  else if (intersectionState == 1){ //only both side sensors detect wall
   if (frontOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (frontOpen()){ //add front
      adjacentUnexploredCoordinate[0]=frontCoordinate[0];
      adjacentUnexploredCoordinate[1]=frontCoordinate[1];
      noAdjacentUnexplored = false; 
    }
  }
  else if (intersectionState == 2){ //only front and right side sensor detects wall
    if (leftOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (leftOpen()){ //add left
       adjacentUnexploredCoordinate[0]=leftCoordinate[0];
       adjacentUnexploredCoordinate[1]=leftCoordinate[1];
       noAdjacentUnexplored = false;
    }
  }
  else if (intersectionState == 3){ //only front and left side sensor detect wall
    if (rightOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (rightOpen()){ //add right
      adjacentUnexploredCoordinate[0]=rightCoordinate[0];
      adjacentUnexploredCoordinate[1]=rightCoordinate[1];
      noAdjacentUnexplored = false;
    }
    
  }
  else if (intersectionState == 4){ //only front sensor detects wall
    if (rightOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (leftOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    int randomizer = random(0,2);
    if (randomizer == 0){
      if (rightOpen()){ //add right
        adjacentUnexploredCoordinate[0]=rightCoordinate[0];
        adjacentUnexploredCoordinate[1]=rightCoordinate[1];
        noAdjacentUnexplored = false;
      }
      else if (leftOpen()){ //add left
        adjacentUnexploredCoordinate[0]=leftCoordinate[0];
        adjacentUnexploredCoordinate[1]=leftCoordinate[1];
        noAdjacentUnexplored = false;
      }
    }
    else{
     if (leftOpen()){ //add left
       adjacentUnexploredCoordinate[0]=leftCoordinate[0];
       adjacentUnexploredCoordinate[1]=leftCoordinate[1];
       noAdjacentUnexplored = false;
     }
     else if (rightOpen()){ //add right
        adjacentUnexploredCoordinate[0]=rightCoordinate[0];
        adjacentUnexploredCoordinate[1]=rightCoordinate[1];
        noAdjacentUnexplored = false;
     }
    }
  
  }
  else if (intersectionState == 5){ //all 3 sensors do detect wall
    noAdjacentUnexplored = true;
  }
  else if (intersectionState == 6){ //only right sensor detects wall
   if (frontOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (leftOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (frontOpen()){ //add front
      adjacentUnexploredCoordinate[0]=frontCoordinate[0];
      adjacentUnexploredCoordinate[1]=frontCoordinate[1];
      noAdjacentUnexplored = false; 
    }
    else if (leftOpen()){ //add left
      adjacentUnexploredCoordinate[0]=leftCoordinate[0];
      adjacentUnexploredCoordinate[1]=leftCoordinate[1];
      noAdjacentUnexplored = false;
    }
  }
  else{ //intersectionState == 7 //only left sensor detects wall
   if (frontOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (rightOpen()){
      nodesToVisit++;
    }
    else{
      nodesToVisit--;
    }
    if (frontOpen()){ //add front
      adjacentUnexploredCoordinate[0]=frontCoordinate[0];
      adjacentUnexploredCoordinate[1]=frontCoordinate[1];
      noAdjacentUnexplored = false; 
    }
    else if (rightOpen()){ //add right
       adjacentUnexploredCoordinate[0]=rightCoordinate[0];
       adjacentUnexploredCoordinate[1]=rightCoordinate[1];
       noAdjacentUnexplored = false;
    }
  }
  
}

void updateStack(){
  if (noAdjacentUnexplored){ //if there are no adjacent unexplored coordinates
    myStack.pop();
    myStack.pop();
  }
  else{
    myStack.push(adjacentUnexploredCoordinate[0]); //push xCoordinate first
    myStack.push(adjacentUnexploredCoordinate[1]);   
  }
}

boolean frontOpen(){
  //use global variables to determine if the intersection infront of the robot has been explored
  //return true iff intersection infront has NOT been explored OR if outer wall is NOT directly infront
  //also update frontCoordinate iff outer wall is NOT directly infront
  if (orientationState==0){//robot going south
    if (xCoordinate==4){
      return false;
    }
    else{
      frontCoordinate[0] = xCoordinate+1;
      frontCoordinate[1]=yCoordinate;
      return !(intersectionMatrix[xCoordinate+1][yCoordinate][0]);
    }
  }
  else if (orientationState == 1){ //going west
    if (yCoordinate==0){
      return false;
    }
    else{
      frontCoordinate[0] = xCoordinate;
      frontCoordinate[1]=yCoordinate-1;
      return !(intersectionMatrix[xCoordinate][yCoordinate-1][0]);
    }
  }
  else if (orientationState ==2){//going north
    if (xCoordinate ==0){
      return false;
    }    
    else{
      frontCoordinate[0] = xCoordinate-1;
      frontCoordinate[1]=yCoordinate;
      return !(intersectionMatrix[xCoordinate-1][yCoordinate][0]);
    }
  }
  else{  // if (orientationState ==3){ //robot going east
    if (yCoordinate ==3){
      return false;
    }
    else{
      frontCoordinate[0] = xCoordinate;
      frontCoordinate[1]=yCoordinate+1;
      return !(intersectionMatrix[xCoordinate][yCoordinate+1][0]);
    }
  }
}

boolean rightOpen(){
  if (orientationState ==0){ //robot going south
    if (yCoordinate ==0){
      return false;
    }
    else{
      rightCoordinate[0] = xCoordinate;
      rightCoordinate[1]=yCoordinate-1;
      return !(intersectionMatrix[xCoordinate][yCoordinate-1][0]);
    }
  }
  else if (orientationState == 1){ //robot going west
    if (xCoordinate ==0){
      return false;
    }
    else{
      rightCoordinate[0] = xCoordinate-1;
      rightCoordinate[1]=yCoordinate;
      return !(intersectionMatrix[xCoordinate-1][yCoordinate][0]);
    }
  }
  else if (orientationState == 2){ //robot going north
    if (yCoordinate == 3){
      return false;
    }
    else{ 
      rightCoordinate[0] = xCoordinate;
      rightCoordinate[1]=yCoordinate+1;
      return !(intersectionMatrix[xCoordinate][yCoordinate+1][0]);
    }
  }
  else{ //(if (orientationState ==3){ //going east
    if (xCoordinate==4){
      return false;
    }
    else{
      rightCoordinate[0] = xCoordinate+1;
      rightCoordinate[1]=yCoordinate;
      return !(intersectionMatrix[xCoordinate+1][yCoordinate][0]);
    }
  }  
}


boolean leftOpen(){
  if (orientationState == 0){ //going south
    if (yCoordinate==3){
      return false;
    }
    else{
      leftCoordinate[0] =xCoordinate;
      leftCoordinate[1]=yCoordinate+1;
      return !(intersectionMatrix[xCoordinate][yCoordinate+1][0]);
    }
  }
  else if (orientationState ==1){//going west
    if (xCoordinate ==4){
      return false;
    }
    else{
      leftCoordinate[0] = xCoordinate+1;
      leftCoordinate[1]=yCoordinate;
      return !(intersectionMatrix[xCoordinate+1][yCoordinate][0]);
    }
  }
  else if (orientationState == 2){//going north
    if (yCoordinate ==0){
      return false;
    }
    else{
      leftCoordinate[0] = xCoordinate;
      leftCoordinate[1]=yCoordinate-1;
      return !(intersectionMatrix[xCoordinate][yCoordinate-1][0]);
    }
  }
  else{ //if (orientationState == 3) //going east
    if (xCoordinate==0){
      return false;
    }
    else{
      leftCoordinate[0] = xCoordinate-1;
      leftCoordinate[1]=yCoordinate;
      return !(intersectionMatrix[xCoordinate-1][yCoordinate][0]);
    }
  }

}


   
int goToNumber(int goToX,int goToY){
  //depends on global variables xCoordinate and yCoordinate (which represent current location)
  //depends on global variable orientationState (which represents current orientation)
  //depends on the parameters, which represent the coordinates that we want to get to
  //return the number that is then stored in whichDirection
  boolean east;
  boolean south;
  boolean west;
  boolean north;
  if (goToX > xCoordinate && goToY == yCoordinate){ //south
      east = false;
      south = true;
      west =false;
      north =false;
  }
  else if (goToX == xCoordinate && goToY > yCoordinate){ //east
      east = true;
      south = false;
      west =false;
      north =false;
  }
  else if (goToX < xCoordinate && goToY == yCoordinate){//north
      east = false;
      south = false;
      west =false;
      north =true;
  }
  else{//west
      east = false;
      south = false;
      west =true;
      north =false;
  }
  if (east){ //need to go east
    if (orientationState == 0){//robot facing south, need to turn left
      return 1;
    }
    else if(orientationState == 1){//robot facing west, turn 180
      return 3;
    }
    else if(orientationState == 2){//robot facing north, turn right
      return 2;
    }
    else{ //robot facing east, go straight
      return 0;
    }
  }
  else if(south){ //need to go south
    if (orientationState == 0){//robot facing south, go straight
      return 0;
    }
    else if(orientationState == 1){//robot facing west, turn left
      return 1;
    }
    else if(orientationState == 2){//robot facing north, turn 180 degrees
      return 3;
    }
    else{ //robot facing east, turn right
      return 2;
    }
  }
  else if (west){ //need to go west
   if (orientationState == 0){//robot facing south, turn right
      return 2;
    }
    else if(orientationState == 1){//robot facing west, go straight
      return 0;
    }
    else if(orientationState == 2){//robot facing north, turn left
      return 1;
    }
    else{ //robot facing east, turn 180
      return 3;
    }
  }
  else{ //need to go north
     if (orientationState == 0){//robot facing south, turn 180
      return 3;
    }
    else if(orientationState == 1){//robot facing west, turn right
      return 2;
    }
    else if(orientationState == 2){//robot facing north, go straight
      return 0;
    }
    else{ //robot facing east, turn left
      return 1;
    } 
  }
}  




void loop() {
  
  //channel 0
  digitalWrite(selectA, LOW);
  digitalWrite(selectB,LOW);
  digitalWrite(selectC,LOW);
  double leftSensorValue = analogRead(muxRead);
  
  //channel 1
  digitalWrite(selectA, HIGH);
  double frontSensorValue = analogRead(muxRead);
  
  //channel 2
  digitalWrite(selectA,LOW);
  digitalWrite(selectB,HIGH);
  double rightSensorValue = analogRead(muxRead);
  
  //channel 3
  digitalWrite(selectA,HIGH);
  double mappedOL = analogRead(muxRead)*.0049;
  OL = mappedOL > lineThres;

  
  //channel 4
  digitalWrite(selectA,LOW);
  digitalWrite(selectB,LOW);
  digitalWrite(selectC,HIGH);
  double mappedFL = analogRead(muxRead)*.0049;
  FL = mappedFL > lineThres;

  
  //channel 5
  digitalWrite(selectA,HIGH);
  double mappedFR = analogRead(muxRead)*.0049;
  FR = mappedFR > lineThres;

  
  //channel 6
  digitalWrite(selectA,LOW);
  digitalWrite(selectB,HIGH);
  double mappedOR = analogRead(muxRead)*.0049;
  OR = mappedOR > lineThres;

  
   double frontSensorDistance = convertToDistance(frontSensorValue); //frontSensorDistance is distance between front sensor and wall in centimeters
   double rightSensorDistance = convertToDistance(rightSensorValue);//rightSensorDistance is distance between right sensor and wall in centimeters
   double leftSensorDistance = convertToDistance(leftSensorValue);  //leftSensorDistance is distance between left sensor and wall in centimeters
   
   frontWall = frontSensorDistance <= frontWallThreshold;
   rightWall = rightSensorDistance <= rightWallThreshold;
   leftWall = leftSensorDistance <= leftWallThreshold;
   
   
   /*move that up tomorrow*/

updateOuterWalls();
if (frontWall){
  digitalWrite(frontLight,HIGH);
}
else{
  digitalWrite(frontLight,LOW);
}
if (leftWall){
  analogWrite(leftLight,255);
}
else{
  analogWrite(leftLight,0);
}
if (rightWall){
  analogWrite(rightLight,255);
}
else{
  analogWrite(rightLight,0);
}




  if (at_intersection()==true){
    oldNodesToVisit= nodesToVisit;
    if (!intersectionMatrix[xCoordinate][yCoordinate][0]){ //if it has not yet been set as explored
      nodesToVisit--;
      wasVisited = false;  
    }
    else{
      wasVisited=true;
    }
    updateOuterWalls(); //make sure outer wall are always accounted for by wall sensors
    updateIntersection(); //determine type of intersection robot is at
    populateIntersectionMatrix(); //update intersection matrrix
    int whichDirection = depthFirstSearch(); //use DFS algorithm to determine direction to go in
    turnOrGoStraight(whichDirection); //go a certain direction based on whichDirectoin value
    radio_transmit(); //transmit data 
    updateOrientation(whichDirection); //depends on direction you chose to go
    updateCoordinates(); //depends only on updated orientation, which is a global variable   
  }
  
  else //not at an intersection, so let's LINE FOLLOW
  {
     
    if (FR && FL) // front right and front left BOTH on line
      {
        lineState = 0;
        stay_straight();
      }   
     else if(!FR && FL) //front right off line, front left on line
    {
      lineState = 1;
      veer_left();
    }
    else if(FR && !FL)//front right on line, front left off line
    {
     lineState = 2;
     veer_right();
    }
   
    //front right and front left must be off line
    //only 1 of 2 outside lines should be on the line at a time
    else if (!OR && OL) //outside left sensor on line
    {
      lineState = 3;
      veer_left();
    }
    else if(OR && !OL) //outside right sensor on line 
    {
      lineState = 4;
      veer_right();
    }
    else if (!FR && !FL && !OR && !OL){ //no sensor is on line
      if (lineState==0){
        //stopRobot();
      }
      else if (lineState == 1){
        veer_left();
      }
      else if (lineState == 2){
        veer_right();
      }
      else if (lineState == 3){
        veer_left();
      }
      else if (lineState == 4){
        veer_right();
      }  
    } 
  }
}  


