#include <Pixy2UART7.h>
#include <Pixy2UART8.h>
#include <kociemba.h>
#include <ODriveArduino.h>
#include <SoftwareSerial.h>

//Pixy Mon Settings:
//saturation: 130
//brightness: 50
//Interface: UART
Pixy2UART7 pixyL;  //lower pixy cam (Serial 7)
Pixy2UART8 pixyH;  //higher pixy cam (Serial 8)

//Odrive TX - Teensy RX    Odrive RX - Teensy TX
ODriveArduino odriveU(Serial1), odriveR(Serial2), odriveF(Serial3);  //UP, RIGHT, and FRONT ODrives
ODriveArduino odriveD(Serial4), odriveL(Serial5), odriveB(Serial6);  //DOWN, LEFT, and BACK ODrives

//Camera Vision
//rgb values for UP, RIGHT, FRONT, DOWN, LEFT, and BACK faces of a solved cube, used to determine the color of a facet
int color[6][3] = { { 255, 49, 65 }, { 149, 255, 101 }, { 240, 242, 255 }, { 255, 116, 61 }, { 39, 193, 255 }, { 244, 255, 62 } };
//the location (x, y) of the color defining pixel on each facet, it is generally the center location of each facet.
int facetLocation[6][9][2] = { { { 115, 110 }, { 144, 100 }, { 0, 0 }, { 130, 120 }, { 0, 0 }, { 180, 100 }, { 162, 122 }, { 188, 118 }, { 205, 110 } },  //U1-U9
                               { { 0, 0 }, { 125, 95 }, { 145, 75 }, { 107, 131 }, { 0, 0 }, { 140, 100 }, { 101, 157 }, { 115, 145 }, { 140, 130 } },    //R1-R9
                               { { 175, 154 }, { 200, 140 }, { 220, 130 }, { 170, 190 }, { 0, 0 }, { 215, 155 }, { 178, 202 }, { 193, 193 }, { 0, 0 } },  //F1-F9
                               { { 0, 0 }, { 130, 185 }, { 110, 175 }, { 181, 182 }, { 0, 0 }, { 130, 165 }, { 205, 175 }, { 185, 165 }, { 160, 144 } },  //D1-D9
                               { { 104, 128 }, { 122, 140 }, { 140, 160 }, { 108, 155 }, { 0, 0 }, { 150, 190 }, { 0, 0 }, { 130, 190 }, { 145, 202 } },  //L1-L9
                               { { 162, 67 }, { 190, 90 }, { 0, 0 }, { 170, 100 }, { 0, 0 }, { 210, 130 }, { 170, 130 }, { 195, 140 }, { 215, 155 } } };  //B1-B9

char faceAssign[6] = { 'U', 'R', 'F', 'D', 'L', 'B' };  //the character representative of each face
//[i][j] = [primary facet color #][secondary facet color #], value is the char of the 3rd side
char cornerAssign[6][6] = { { '-', 'B', 'R', '-', 'F', 'L' },    //primary facet is U
                            { 'F', '-', 'D', 'B', '-', 'U' },    //primary facet is R
                            { 'L', 'U', '-', 'R', 'D', '-' },    //primary facet is F
                            { '-', 'F', 'L', '-', 'B', 'R' },    //primary facet is D
                            { 'B', '-', 'U', 'F', '-', 'D' },    //primary facet is L
                            { 'R', 'D', '-', 'L', 'U', '-' } };  //primary facet is B
int facetHardNum[6] = { 2, 9, 26, 27, 42, 47 };                  //facet numbers of the hard to read facets
int facetPNum[6] = { 11, 8, 29, 24, 53, 36 };                    //facet numbers of the primary facet for each hard to read facet
int facetSNum[6] = { 45, 20, 15, 44, 33, 0 };                    //facet numbers of the primary facet for each hard to read facet
int numP, numS;                                                  //the color number of the primary and secondary facet

//Kociemba
char cube[1][54];                //cube defined by the color of each facet denoted as a char: U, R, F, D, L, or B
int numMoves;                    //the number of moves in a solution
String scam[30];                 //array to store solution moves, size set to 30, should not exceed 24
elapsedMillis em;                //elapsed time
DMAMEM char buf479[479 * 1024];  // 479K in DMAMEM
char buf248[248 * 1024];         // 248K on in DTCM

//Motor Control
float pos[6];                                                                                                                    //rounded relative position of each motor
float error = .01;                                                                                                               //error for positioning .01 counts = 3.6 degrees
int randNum;                                                                                                                     //random number
String randscam[18] = { "U", "U'", "U2", "R", "R'", "R2", "F", "F'", "F2", "D", "D'", "D2", "L", "L'", "L2", "B", "B'", "B2" };  //possible moves
int timeI, timeS;                                                                                                                //initial times for inspection and solving

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A3));  //random seed, change often
  Serial.println("Begin");
  //Camera Vision
  pixyL.init();               //initialize object
  pixyH.init();               //initialize object
  pixyL.changeProg("video");  //switch program to video
  pixyH.changeProg("video");  //switch program to video

  //Kociemba
  em = 0;
  kociemba::set_memory(buf479, buf248);  // removing this line slows the computation by a factor of 4 (but saves a lot of RAM...)
  for (int i = 0; i < 6; i++) {          //define the color of the middle facets, these are constant
    cube[0][4 + i * 9] = faceAssign[i];
  }

  //Motor Control
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  Serial6.begin(115200);
  odriveU.setState(AXIS_STATE_IDLE);
  odriveR.setState(AXIS_STATE_IDLE);
  odriveF.setState(AXIS_STATE_IDLE);
  odriveD.setState(AXIS_STATE_IDLE);
  odriveL.setState(AXIS_STATE_IDLE);
  odriveB.setState(AXIS_STATE_IDLE);

  //initialize moves array
  for (int i = 0; i < 30; i++) {
    scam[i] = "-";
  }
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    //Calibration Sequence: "1"
    switch (c) {
      // idle: "i"
      case 'i':
        odriveU.setState(AXIS_STATE_IDLE);
        odriveR.setState(AXIS_STATE_IDLE);
        odriveF.setState(AXIS_STATE_IDLE);
        odriveD.setState(AXIS_STATE_IDLE);
        odriveL.setState(AXIS_STATE_IDLE);
        odriveB.setState(AXIS_STATE_IDLE);
        break;
      // closed loop control: "c"
      case 'c':
        //clear errors
        odriveU.clearErrors();
        odriveR.clearErrors();
        odriveF.clearErrors();
        odriveD.clearErrors();
        odriveL.clearErrors();
        odriveB.clearErrors();
        //reboot odrives and set relative positions to 0
        odriveU.reboot();
        odriveR.reboot();
        odriveF.reboot();
        odriveD.reboot();
        odriveL.reboot();
        odriveB.reboot();
        delay(2000);
        //set rounded positions to 0
        for (int i = 0; i < 6; i++) {
          pos[i] = 0;
        }
        //set state to closed loop control
        odriveU.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odriveR.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odriveF.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odriveD.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odriveL.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odriveB.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(10);
        //velocity gain (default: 20)
        odriveU.setParameter("axis0.controller.config.pos_gain", "100");
        odriveR.setParameter("axis0.controller.config.pos_gain", "100");
        odriveF.setParameter("axis0.controller.config.pos_gain", "100");
        odriveD.setParameter("axis0.controller.config.pos_gain", "140");
        odriveL.setParameter("axis0.controller.config.pos_gain", "100");
        odriveB.setParameter("axis0.controller.config.pos_gain", "100");
        //velocity gain (default: 0.167)
        odriveU.setParameter("axis0.controller.config.vel_gain", "0.25");
        odriveR.setParameter("axis0.controller.config.vel_gain", "0.25");
        odriveF.setParameter("axis0.controller.config.vel_gain", "0.25");
        odriveD.setParameter("axis0.controller.config.vel_gain", "0.35");
        odriveL.setParameter("axis0.controller.config.vel_gain", "0.25");
        odriveB.setParameter("axis0.controller.config.vel_gain", "0.25");
        //velocity integrator gain (default: 0.333)
        odriveU.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
        odriveR.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
        odriveF.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
        odriveD.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
        odriveL.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
        odriveB.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
        //velocity limit, set initially to 5 (low speed mode/minimum operation speed)
        setSpeed(8);
        //Electical Power - prevent spinout error (default: 20W)
        odriveU.setParameter("axis0.controller.config.spinout_electrical_power_threshold", 100);
        odriveR.setParameter("axis0.controller.config.spinout_electrical_power_threshold", 100);
        odriveF.setParameter("axis0.controller.config.spinout_electrical_power_threshold", 100);
        odriveD.setParameter("axis0.controller.config.spinout_electrical_power_threshold", 100);
        odriveL.setParameter("axis0.controller.config.spinout_electrical_power_threshold", 100);
        odriveB.setParameter("axis0.controller.config.spinout_electrical_power_threshold", 100);
        //Mechanical Power - prevent spinout error (default: 20W)
        odriveU.setParameter("axis0.controller.config.spinout_mechanical_power_threshold", -100);
        odriveR.setParameter("axis0.controller.config.spinout_mechanical_power_threshold", -100);
        odriveF.setParameter("axis0.controller.config.spinout_mechanical_power_threshold", -100);
        odriveD.setParameter("axis0.controller.config.spinout_mechanical_power_threshold", -100);
        odriveL.setParameter("axis0.controller.config.spinout_mechanical_power_threshold", -100);
        odriveB.setParameter("axis0.controller.config.spinout_mechanical_power_threshold", -100);
        break;
      //perform 20 random scrambles on the cube
      case 'p':
        setSpeed(8);  //set speed to 8
        for (int i = 0; i < 20; i++) {
          randNum = random(0, 18);  //random turn (a number from 0-17 inclusive)
          turn(randscam[randNum]);  //make a random move
          delay(150);
        }
        break;
      //get the cube layout
      case 'a':
        getCube();  //get the current cube configuration
        break;
      //solve the cube
      case 't':
        timeI = millis();  //initial inspection time
        //get the cube configuration from camera vision and solve the cube
        if (getCube()) {
          solveCube();  //solve the cube virtually
                        //solve the cube physically
          //setSpeed(100);             //set speed
          timeI = millis() - timeI;  //get total inspection time
          timeS = millis();          //get intial solve time
          for (int i = 0; i < numMoves; i++) {
            turn(scam[i]);
          }
          timeS = millis() - timeS;                                     //capture the total solve time
          Serial.println((String) "Inspection Time: " + timeI + "ms");  //time taken to look at the cube and solve it with the algorithm
          Serial.println((String) "Move Count: " + numMoves);
          Serial.println((String) "Solve Time: " + timeS + "ms");
        }
        break;
        //check pixels
      //get Pixel RGB
      case 'u':
        //getAvgRGB(facetLocation[0][0][0], facetLocation[0][0][1]);
        getAvgRGB(193, 193);
        break;
    }
  }
}

//scan the cube and assign the color of each facet, returns 1 if the configuration is correct, 0 otherwise
int getCube() {
  uint8_t r, g, b;                      //rgb values
  int error[6];                         //the error of a facet compared to each face/color
  int min;                              //the index with the smallest error
  int num;                              //facet number and generic variable
  int count[6] = { 0, 0, 0, 0, 0, 0 };  //counts the number of facets that are U, R, F, D, L, B
  //get the easy to read facets
  for (int i = 0; i < 6; i++) {    //cycle through faces
    for (int j = 0; j < 9; j++) {  //cycle through each facet on face i
      num = i * 9 + j;             //calculate facet number
      //easy to find facets, find them (except for j = 4 aka the constant facet in the middle of the face)
      if (j != 4 && !(num == facetHardNum[0] || num == facetHardNum[1] || num == facetHardNum[2] || num == facetHardNum[3] || num == facetHardNum[4] || num == facetHardNum[5])) {
        if (i % 2) {
          //grab pixel rgb values from higher camera (i = 1(R), 3(D), or 5(B))
          pixyL.video.getRGB(facetLocation[i][j][0], facetLocation[i][j][1], &r, &g, &b);
        } else {
          //grab pixel rgb values from higher camera (i = 0(U), 2(F), or 4(L))
          pixyH.video.getRGB(facetLocation[i][j][0], facetLocation[i][j][1], &r, &g, &b);
        }
        //calculate the error between a facet and each of the 6 colors and determine the lowest error
        for (int k = 0; k < 6; k++) {
          error[k] = (abs(color[k][0] - r)) + (abs(color[k][1] - g)) + (abs(color[k][2] - b));  //calculate error                                                                                      //find the average of the 5 measurements
          min = k > 0 ? (error[k] < error[min] ? k : min) : 0;                                  //determines the index with the smallest error after each cycle
        }

        //Hardcoded Filters to prevent color mixup
        //distinguish between red and orange
        if (min == 0 || min == 3) {  //if color is U or D (red or orange)
          if (b > g) {               //if blue is greater than green, the color is red
            min = 0;
          } else {
            min = 3;
          }
        }
        //distinguish between orange and yellow
        if (min == 3 || min == 5) {  //if color is D or B (orange or yellow)
          if (g < 170) {             //if green is low, the color is orange
            min = 3;
          } else {
            min = 5;
          }
        }
        //distinguish between green and blue
        if (min == 1 || min == 4) {  //if color is R or L (green or blue)
          if (g > b) {               //if green is greater than blue, the color is green
            min = 1;
          } else {
            min = 4;
          }
        }
        //distinguish between red and blue
        if (min == 0 || min == 4) {  //if color is U or L (red or blue)
          if (r > b) {               //if red is greater than blue, the color is red
            min = 0;
          } else {
            min = 4;
          }
        }
        //distinguish between blue and white
        if (min == 2 && r < 120) {  //if color is white and r is less than 120
          min = 4;
        }
        //distinguish between white and yellow
        if (min == 2 && b < 180) {  //if color is white and b is less than 200
          min = 5;
        }
        cube[0][num] = faceAssign[min];  //assign the color of the facet
      }
    }
  }
  //get the hard to read facets
  for (int i = 0; i < 6; i++) {  //find the color of each hard to read facet
    //get the color number of the primary facet for the hard to find facet i
    numP = (cube[0][facetPNum[i]] == 'R') + (cube[0][facetPNum[i]] == 'F') * 2 + (cube[0][facetPNum[i]] == 'D') * 3 + (cube[0][facetPNum[i]] == 'L') * 4 + (cube[0][facetPNum[i]] == 'B') * 5;
    //get the color number of the secondary facet for the hard to find facet i
    numS = (cube[0][facetSNum[i]] == 'R') + (cube[0][facetSNum[i]] == 'F') * 2 + (cube[0][facetSNum[i]] == 'D') * 3 + (cube[0][facetSNum[i]] == 'L') * 4 + (cube[0][facetSNum[i]] == 'B') * 5;
    cube[0][facetHardNum[i]] = cube[0][facetHardNum[i]] = cornerAssign[numP][numS];  //define hard facet
  }
  //print cube configuration
  for (int i = 0; i < 6; i++) {
    Serial.print((String)faceAssign[i] + ": ");
    for (int j = 0; j < 9; j++) {
      Serial.print(cube[0][i * 9 + j]);
    }
    Serial.println();
  }
  //check if cube configuration is correct
  for (int i = 0; i < 54; i++) {
    num = (cube[0][i] == 'R') + (cube[0][i] == 'F') * 2 + (cube[0][i] == 'D') * 3 + (cube[0][i] == 'L') * 4 + (cube[0][i] == 'B') * 5;
    count[num]++;
  }
  Serial.println();
  num = 1;
  for (int i = 0; i < 6; i++) {
    if (count[i] != 9) {
      num = 0;
      Serial.println((String) "Error: Face " + faceAssign[i] + " has " + count[i] + " facets");
      break;
    }
  }
  if (num == 1) {
    Serial.println("Cube Configuration is Correct");
  }
  return num;
}
//solve the cube with the kociemba algorithm
void solveCube() {
  for (auto s : cube) {
    em = 0;  //time taken to find a solution
    const char* solution = kociemba::solve(s);
    if (solution == nullptr)
      Serial.println("no solution found");
    else
      Serial.printf("Solution: %s \n", solution);
    //convert solution to an array of moves
    numMoves = 0;
    for (int i = 0; i < ((String)solution).length(); i++) {
      scam[numMoves] = solution[i++];                     //set move initially
      while (solution[i] != 32) {                         //32 is ascii for a single space
        scam[numMoves] = scam[numMoves] + solution[i++];  //add on the parts of the move
      }
      numMoves++;
    }
  }
}
//perform an input move on the cube
void turn(String move) {
  //3 types of moves: R, R', R2 (for face 'R')
  float angle = move.length() == 1 ? -0.25 : (move.charAt(1) == '2' ? 0.5 : 0.25);  //define the angle of the turn
  float relPos;                                                                     //relative position of the odrive
  //determine which settings to use
  if (angle == 0.5) {
    odriveU.setParameter("axis0.controller.config.pos_gain", "100");
    odriveR.setParameter("axis0.controller.config.pos_gain", "100");
    odriveF.setParameter("axis0.controller.config.pos_gain", "100");
    odriveD.setParameter("axis0.controller.config.pos_gain", "140");
    odriveL.setParameter("axis0.controller.config.pos_gain", "100");
    odriveB.setParameter("axis0.controller.config.pos_gain", "100");
    //velocity gain (default: 0.167)
    odriveU.setParameter("axis0.controller.config.vel_gain", "0.20");
    odriveR.setParameter("axis0.controller.config.vel_gain", "0.20");
    odriveF.setParameter("axis0.controller.config.vel_gain", "0.20");
    odriveD.setParameter("axis0.controller.config.vel_gain", "0.25");
    odriveL.setParameter("axis0.controller.config.vel_gain", "0.20");
    odriveB.setParameter("axis0.controller.config.vel_gain", "0.20");
    //velocity integrator gain (default: 0.333)
    odriveU.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    odriveR.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    odriveF.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    odriveD.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    odriveL.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    odriveB.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    //velocity limit, set initially to 5 (low speed mode/minimum operation speed)
    odriveU.setParameter("axis0.controller.config.vel_limit", 11);
    odriveR.setParameter("axis0.controller.config.vel_limit", 11);
    odriveF.setParameter("axis0.controller.config.vel_limit", 11);
    odriveD.setParameter("axis0.controller.config.vel_limit", 11);
    odriveL.setParameter("axis0.controller.config.vel_limit", 11);
    odriveB.setParameter("axis0.controller.config.vel_limit", 11);
  } else {
    odriveU.setParameter("axis0.controller.config.pos_gain", "100");
    odriveR.setParameter("axis0.controller.config.pos_gain", "100");
    odriveF.setParameter("axis0.controller.config.pos_gain", "100");
    odriveD.setParameter("axis0.controller.config.pos_gain", "140");
    odriveL.setParameter("axis0.controller.config.pos_gain", "100");
    odriveB.setParameter("axis0.controller.config.pos_gain", "100");
    //velocity gain (default: 0.167)
    odriveU.setParameter("axis0.controller.config.vel_gain", "0.25");
    odriveR.setParameter("axis0.controller.config.vel_gain", "0.25");
    odriveF.setParameter("axis0.controller.config.vel_gain", "0.25");
    odriveD.setParameter("axis0.controller.config.vel_gain", "0.35");
    odriveL.setParameter("axis0.controller.config.vel_gain", "0.25");
    odriveB.setParameter("axis0.controller.config.vel_gain", "0.25");
    //velocity integrator gain (default: 0.333)
    odriveU.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    odriveR.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    odriveF.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    odriveD.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    odriveL.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    odriveB.setParameter("axis0.controller.config.vel_integrator_gain", "0.333");
    //velocity limit, set initially to 5 (low speed mode/minimum operation speed)
    odriveU.setParameter("axis0.controller.config.vel_limit", 100);
    odriveR.setParameter("axis0.controller.config.vel_limit", 100);
    odriveF.setParameter("axis0.controller.config.vel_limit", 100);
    odriveD.setParameter("axis0.controller.config.vel_limit", 100);
    odriveL.setParameter("axis0.controller.config.vel_limit", 100);
    odriveB.setParameter("axis0.controller.config.vel_limit", 100);
  }
  //turn the right motor
  switch (move.charAt(0)) {
    case 'U':
      pos[0] += angle;              //add angle to rounded position
      odriveU.setPosition(pos[0]);  //execute movement
      do {
        relPos = odriveU.getParameterAsFloat("axis0.pos_vel_mapper.pos_rel");        //get relative position
      } while (!(angle < 0 ? relPos <= pos[0] + error : relPos >= pos[0] - error));  //0 if target position is reached, 1 if not (error included)
      break;
    case 'R':
      pos[1] += angle;              //add angle to rounded position
      odriveR.setPosition(pos[1]);  //execute movement
      do {
        relPos = odriveR.getParameterAsFloat("axis0.pos_vel_mapper.pos_rel");        //get relative position
      } while (!(angle < 0 ? relPos <= pos[1] + error : relPos >= pos[1] - error));  //0 if target position is reached, 1 if not (error included)
      break;
    case 'F':
      pos[2] += angle;              //add angle to rounded position
      odriveF.setPosition(pos[2]);  //execute movement
      do {
        relPos = odriveF.getParameterAsFloat("axis0.pos_vel_mapper.pos_rel");        //get relative position
      } while (!(angle < 0 ? relPos <= pos[2] + error : relPos >= pos[2] - error));  //0 if target position is reached, 1 if not (error included)
      break;
    case 'D':
      pos[3] += angle;              //add angle to rounded position
      odriveD.setPosition(pos[3]);  //execute movement
      do {
        relPos = odriveD.getParameterAsFloat("axis0.pos_vel_mapper.pos_rel");        //get relative position
      } while (!(angle < 0 ? relPos <= pos[3] + error : relPos >= pos[3] - error));  //0 if target position is reached, 1 if not (error included)
      break;
    case 'L':
      pos[4] += angle;              //add angle to rounded position
      odriveL.setPosition(pos[4]);  //execute movement
      do {
        relPos = odriveL.getParameterAsFloat("axis0.pos_vel_mapper.pos_rel");        //get relative position
      } while (!(angle < 0 ? relPos <= pos[4] + error : relPos >= pos[4] - error));  //0 if target position is reached, 1 if not (error included)
      break;
    case 'B':
      pos[5] += angle;              //add angle to rounded position
      odriveB.setPosition(pos[5]);  //execute movement
      do {
        relPos = odriveB.getParameterAsFloat("axis0.pos_vel_mapper.pos_rel");        //get relative position
      } while (!(angle < 0 ? relPos <= pos[5] + error : relPos >= pos[5] - error));  //0 if target position is reached, 1 if not (error included)
      break;
  }
}
void setSpeed(int speed) {
  odriveU.setParameter("axis0.controller.config.vel_limit", speed);
  odriveR.setParameter("axis0.controller.config.vel_limit", speed);
  odriveF.setParameter("axis0.controller.config.vel_limit", speed);
  odriveD.setParameter("axis0.controller.config.vel_limit", speed);
  odriveL.setParameter("axis0.controller.config.vel_limit", speed);
  odriveB.setParameter("axis0.controller.config.vel_limit", speed);
}
//get the average RGB values of a pixel at an X, Y location (average of 10 measurements)
void getAvgRGB(int X, int Y) {
  uint8_t r, g, b;  //rgb values
  float red = 0, green = 0, blue = 0;
  for (int i = 0; i < 10; i++) {
    uint8_t r, g, b;
    pixyH.video.getRGB(X, Y, &r, &g, &b);
    red += r;
    green += g;
    blue += b;
    delay(10);
  }
  Serial.print((String) "{" + int(red / 10));
  Serial.print((String) ", " + int(green / 10));
  Serial.print((String) ", " + int(blue / 10));
  Serial.println("}");
}