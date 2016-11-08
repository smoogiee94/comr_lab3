#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Servo.h>
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

const int SFSensor = A0;  // Analog input pin that the potentiometer is attached to
const int SLSensor = A1;  // Analog input pin that the potentiometer is attached to
const int SRSensor = A2;  // Analog input pin that the potentiometer is attached to
const int LFSensor = A3;  // Analog input pin that the potentiometer is attached to

char dir = 'E';

float SFValue = 0;    
float SLValue = 0;    
float SRValue = 0;    
float LFValue = 0;   

int turns = 0;
int x = 0;
int y = 0;
char maze[4][4] = 
  {
    {'X', 'X', 'X', 'X'},
    {'X', 'X', 'X', 'X'},
    {'X', 'X', 'X', 'X'},
    {'X', 'X', 'X', 'X'}
  };
Servo LServo;
Servo RServo;
void setup() {
  // initialize serial communications at 9600 bps:
  LServo.attach(2);
  RServo.attach(3);
  Serial.begin(9600);
  lcd.begin(16, 2);
  maze[x][y] = 'O';
    lcd.setCursor(0, 0);
  lcd.write(maze[3][0]);

  lcd.setCursor(1, 0);
  lcd.write(maze[3][1]);

  lcd.setCursor(2, 0);
  lcd.write(maze[3][2]);
  
  lcd.setCursor(3, 0);
  lcd.write(maze[3][3]);


  lcd.setCursor(0, 1);
  lcd.write(maze[2][0]);

  lcd.setCursor(1, 1);
  lcd.write(maze[2][1]);

  lcd.setCursor(2, 1);
  lcd.write(maze[2][2]);
  
  lcd.setCursor(3, 1);
  lcd.write(maze[2][3]);


  lcd.setCursor(5, 0);
  lcd.write(maze[1][0]);

  lcd.setCursor(6, 0);
  lcd.write(maze[1][1]);

  lcd.setCursor(7, 0);
  lcd.write(maze[1][2]);
  
  lcd.setCursor(8, 0);
  lcd.write(maze[1][3]);


  lcd.setCursor(5, 1);
  lcd.write(maze[0][0]);

  lcd.setCursor(6, 1);
  lcd.write(maze[0][1]);

  lcd.setCursor(7, 1);
  lcd.write(maze[0][2]);
  
  lcd.setCursor(8, 1);
  lcd.write(maze[0][3]);
}
int gridCount = 1;


void loop() {

  
  closeLoopCtrlPart2(6, 5);
   

    //in a new grid
    if (((millis() - 500 * turns) / 3000 + 1) != gridCount){
      switch(dir){
        case('N') : 
          if (x + 1 < 4)
            modifyAndPrint(++x,y);
          break;
        case('S') : 
          if (x - 1 >= 0)
            modifyAndPrint(--x,y);
          break;
        case('W') : 
          if (y - 1 >= 0)
            modifyAndPrint(x,--y);
          break;
        case('E') : 
          if (y + 1 < 4)
            modifyAndPrint(x,++y);
          break;
      }
      gridCount = (millis() - 500 * turns) / 3000 + 1;
    }
    

}



void closeLoopCtrlPart2(float rt, float kp){
  //Control Loop for front sensor
  float ytf = sensorRead("front");

  //Check front distance and set left servo
  float etfl = rt - ytf;
  float utfl = kp * etfl;

  //Check front distance and set right servo
  float etfr = ytf - rt;
  float utfr = kp * etfr;

  float ytr = sensorRead("right");
  
  //Check right distance. Set left
  float etrl = rt - ytr;
  float utrl = kp * etrl;

  //Check right distance. Set Right
  float etrr = ytr - rt;
  float utrr = kp * etrr;

  int leftServoValue = saturationFunctionFront(utfl) + saturationFunctionRightLeft(utrl);
  int rightServoValue = saturationFunctionFront(utfr) - saturationFunctionRightLeft(utrr);

  if (leftServoValue <= 93 && leftServoValue >= 87){
    if (rightServoValue <= 93 && rightServoValue >= 87){

      //180 degree
      if (sensorRead("left") < 11.5){
        switch(dir){
          case('N') : dir = 'S';
            break;
          case('S') : dir = 'N';
            break;
          case('E') : dir = 'W';
            break;
          case('W') : dir = 'E';
            break;
        }

        rightServoValue = 80;
        leftServoValue = 80;
        RServo.write(rightServoValue);
        LServo.write(leftServoValue);
        delay(1100);
        ++turns;
        ++turns;
      }

      //Left turn
      else{
        switch(dir){
          case('N') : dir = 'W';
            break;
          case('S') : dir = 'E';
            break;
          case('E') : dir = 'N';
            break;
          case('W') : dir = 'S';
            break;
        }

        rightServoValue = 80;
        leftServoValue = 80;
        RServo.write(rightServoValue);
        LServo.write(leftServoValue);
        delay(550);
        ++turns;
      }
    }
  }
  else if (ytr >= 10.0){
     switch(dir){
      case('N') : dir = 'E';
        break;
      case('S') : dir = 'W';
        break;
      case('E') : dir = 'S';
        break;
      case('W') : dir = 'N';
        break;
    }

    //right turn
    LServo.write(100);
    RServo.write(80);
    delay(1250);
    RServo.write(100);
    LServo.write(100);
    delay(550);
    RServo.write(80);
    LServo.write(100);
    delay(1000);
    
  }
  else{
    RServo.write(rightServoValue);
    LServo.write(leftServoValue); 
  }

}






int saturationFunctionRightLeft(double val){
  int returnVal = 0;
  if (returnVal - val <= -5){
    return -5;
  }

  else if (returnVal - val >= 5){
    return 5;
  }

  else
    return returnVal - val;
}








int saturationFunctionFront(double val){
  int returnVal = 90;
  if (returnVal - val <= 80){
    return 80;
  }
  else if (returnVal - val >= 100){
    return 100;
  }
  else
    return returnVal - val;
}










float sensorRead(String sensorDirection){
  float sensorValue = 0;
  if (sensorDirection.equals("front")){
    sensorValue = 12.509 * pow(analogRead(SFSensor)*0.0048875855327468, -1.059) / 2.54;
    if (sensorValue > 11.80){
      sensorValue = 59.635 * pow(analogRead(LFSensor)*0.0048875855327468, -1.034) / 2.54;
      if (sensorValue > 59){
        sensorValue = 59;
      }
    }
  }

  else if (sensorDirection.equals("left")){
    sensorValue = 12.509 * pow(analogRead(SLSensor)*0.0048875855327468, -1.059) / 2.54;
    if (sensorValue > 11.80){
      sensorValue = 11.80;
    }
  }

   else if (sensorDirection.equals("right")){
    sensorValue = 12.509 * pow(analogRead(SRSensor)*0.0048875855327468, -1.059) / 2.54;
    if (sensorValue > 11.80){
      sensorValue = 11.80;
    }
  }
  else{
    sensorValue = -1;
  }
  return sensorValue;
}

void modifyAndPrint(int x, int y){
  maze[x][y] = 'O';
  
  if (x == 0 && y == 0){
    lcd.setCursor(5, 1);
  }
  if (x == 0 && y == 1){
    lcd.setCursor(6, 1);
  }
  if (x == 0 && y == 2){
    lcd.setCursor(7, 1);
  }
  if (x == 0 && y == 3){
    lcd.setCursor(8, 1);
  }


  if (x == 1 && y == 0){
    lcd.setCursor(5, 0);
  }
  if (x == 1 && y == 1){
    lcd.setCursor(6, 0);
  }
  if (x == 1 && y == 2){
    lcd.setCursor(7, 0);
  }
  if (x == 1 && y == 3){
    lcd.setCursor(8, 0);
  }




  if (x == 2 && y == 0){
    lcd.setCursor(0, 1);
  }
  if (x == 2 && y == 1){
    lcd.setCursor(1, 1);
  }
  if (x == 2 && y == 2){
    lcd.setCursor(2, 1);
  }
  if (x == 2 && y == 3){
    lcd.setCursor(3, 1);
  }


  if (x == 3 && y == 0){
    lcd.setCursor(0, 0);
  }
  if (x == 3 && y == 1){
    lcd.setCursor(1, 0);
  }
  if (x == 3 && y == 2){
    lcd.setCursor(2, 0);
  }
  if (x == 3 && y == 3){
    lcd.setCursor(3, 0);
  }


  lcd.print(maze[x][y]);
  return;
}


