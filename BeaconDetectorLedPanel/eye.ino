
#include <Rainbowduino.h>
//#include <Wire.h>

uint32_t TRS = 0x123456;
uint32_t BLK = 0x000000;
uint32_t RED = 0xFF0000;
uint32_t REL = 0xFF1010;
uint32_t YEL = 0x808000;
uint32_t WHI = 0x808080;

// MASTER is right eye
int MASTER = 1;

uint32_t EYE_BROW[4][8] = {
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK},
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK},
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK},
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK}
};

uint32_t ARHS[8][8] = {
  {WHI, WHI, BLK, WHI, WHI, WHI, WHI, BLK},
  {WHI, WHI, WHI, WHI, WHI, WHI, WHI, WHI},
  {WHI, BLK, BLK, WHI, WHI, BLK, BLK, WHI},
  {WHI, WHI, BLK, WHI, WHI, BLK, WHI, WHI},
  {WHI, WHI, BLK, BLK, BLK, BLK, WHI, WHI},
  {WHI, WHI, BLK, BLK, BLK, BLK, WHI, WHI},
  {BLK, BLK, BLK, BLK, BLK, BLK, WHI, WHI},
  {BLK, BLK, BLK, BLK, BLK, BLK, WHI, WHI}
};

uint32_t MATRIX[8][8] = {
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK},
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK},
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK},
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK},
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK},
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK},
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK},
  {BLK, BLK, BLK, BLK, BLK, BLK, BLK, BLK}
};

uint32_t EYE[4][4] = {
  {TRS, RED, RED, TRS},
  {RED, REL, RED, RED},
  {RED, RED, RED, RED},
  {TRS, RED, RED, TRS}
};

uint32_t EYE_ANGRY[4][4] = {
  {TRS, TRS, TRS, TRS},
  {TRS, TRS, RED, TRS},
  {TRS, RED, REL, RED},
  {TRS, TRS, RED, TRS}
};

int angry = 0;
int eyeX = 2;
int eyeY = 2;
int baseEyeX = 2;
int baseEyeY = 2;
int targetEyeX = 2;
int targetEyeY = 2;


int TRANSIT_TIME = 15;
int remainingGlobalTime = (1000.0 / TRANSIT_TIME) * 100;//150.0;
bool useRemainingGlobalTime = false;

int timeSinceBlink;
float blinkStatus;
int TIME_BLINK;
int TIME_BLINK_RANDOM;

int timeSinceMove;
float moving;
int TIME_MOVE;
int TIME_MOVE_RANDOM;

void initEye() {
	angry = 0;
	eyeX = 2;
	eyeY = 2;
	baseEyeX = 2;
	baseEyeY = 2;
	targetEyeX = 2;
	targetEyeY = 2;


	//TRANSIT_TIME = 15;
	//remainingGlobalTime = (1000.0 / TRANSIT_TIME) * 100;//150.0;
	//useRemainingGlobalTime = false;

	timeSinceBlink = 20;
	blinkStatus = 0.0;
	TIME_BLINK = 20;
	TIME_BLINK_RANDOM = 10;

	timeSinceMove = 5;
	moving = 0;
	TIME_MOVE = 5;
	TIME_MOVE_RANDOM = 0;
}


int convert(float time, int base_value, int target_value){
  // Function: cos
  float cosRatio = cos((1.0 - time) * PI);
  return base_value - cosRatio * (target_value - base_value);
}

/*
void setup(){
  Rb.init();
  
//  Wire.begin(4);                // join i2c bus with address #4
//  Wire.onReceive(receiveEvent); // register event
}
*/

void eyeLoop(){
  // Clean next screen
  for (int x = 0; x<8; x++) {
    for (int y = 0; y<8; y++) {
      setPxl(x, y, BLK);
    }
  }
  if (remainingGlobalTime > 0 || ! useRemainingGlobalTime) {
    remainingGlobalTime = remainingGlobalTime - 1;
    // Master asks for blinks and moves
    if (MASTER == 1) {
      // Moving
      timeSinceMove = timeSinceMove - 1;
      if (timeSinceMove < 1) {
        timeSinceMove = TIME_MOVE + random(TIME_MOVE_RANDOM);
        int randomY = random(10);
        int randomX = 1;
        if (randomY > 2 && randomY < 8) {
          randomX = random(2);
        }
        int randomSevere = random(1);
        lookAt(randomX, randomY, randomSevere);
      }
      // Blinking
      timeSinceBlink = timeSinceBlink - 1;
      if (timeSinceBlink < 1) {
        timeSinceBlink = TIME_BLINK + random(TIME_BLINK_RANDOM);
        doBlink();
      }
    }
    int realOpenValue = 0;
    if (blinkStatus > 1) {
      realOpenValue = convert(2*(blinkStatus - 1), 0, 4);
      blinkStatus = blinkStatus - 0.05;
    } else if (blinkStatus > 0) {
      realOpenValue = convert(blinkStatus, 4, 0);
      blinkStatus = blinkStatus - 0.05;
    }
    // Compute position of eye
    if (moving > 0) {
      eyeX = convert(moving, baseEyeX, targetEyeX);
      eyeY = convert(moving, baseEyeY, targetEyeY);
      moving = moving - 0.01;
    } else {
      eyeX = targetEyeX;
      eyeY = targetEyeY;
    }
    // Draw eye
    for (int x = 0; x<4; x++) {
      for (int y = 0; y<4; y++) {
        if (angry == 1) {
          setPxl(x + eyeX, y + eyeY, EYE_ANGRY[x][y]);
        } else if (x >= realOpenValue) {
          setPxl(x + eyeX, y + eyeY, EYE[x][y]);
        }
      }
    }
    // Compute position of eyebrow
    
    if (angry == 0) {
      int eyeBrow[8] = {1,1,0,0,0,1,1,1};
      setEyeBrow(eyeBrow);
    } else {
      int eyeBrow[8] = {1,1,0,0,1,1,2,3};
      setEyeBrow(eyeBrow);
    }
    
    // Draw Eyebrow
    for (int x = 0; x<4; x++) {
      for (int y = 0; y<8; y++) {
        setPxl(x, y, EYE_BROW[x][y]);
      }
    }
    // END OF FRAME
    redraw();
    //delay(TRANSIT_TIME);
  } else {
    for (int x = 0; x<8; x++) {
      for (int y = 0; y<8; y++) {
        setPxl(x, y, ARHS[x][y]);
      }
    }
    redraw();
    //delay(5000);
  }
}

void setNewPosition(int newX, int newY, int newSevere) {
  if (newX < 5 && newX >= 0 && newY < 5 && newY >= 0) {
    baseEyeX = eyeX;
    baseEyeY = eyeY;
    targetEyeX = newX;
    targetEyeY = newY;
    moving = 1;
  }
}

void setEyeBrow(int xx[8]) {
  for (int x = 0; x<4; x++) {
    for (int y = 0; y<8; y++) {
      int yy = y;
      if (MASTER == 1) {
        yy = 7 - y;
      }
      if (x < xx[yy]) {
        EYE_BROW[x][y] = BLK;
      } else if (x == xx[yy]) {
        EYE_BROW[x][y] = YEL;
      } else {
        EYE_BROW[x][y] = TRS;
      }
    }
  }
}

void setPxl(int x, int y, uint32_t color) {
  if (color != TRS && x < 8 && x >= 0 && y < 8 && y >= 0) {
    MATRIX[x][y] = color;
  }
}

void redraw() {
  for (int x = 0; x<8; x++) {
    for (int y = 0; y<8; y++) {
      Rb.setPixelXY(x, y, MATRIX[x][y]);
    }
  }
}

void doBlink() {
  if (MASTER == 1) {
    //sendBlink();
  }
  blinkStatus = 1.5;
}

void lookAt(int x, int y, int newSevere) {
  if (x >=0 && x <= 2 && y >= 0 && y <= 10) {
    if (MASTER == 1) {
      //sendLookAt(x, y, newSevere);
      // MASTER = RIGHT --> invert
      y = 10 - y;
    }
    if (y == 0) {
      setNewPosition(2 + x, 0, newSevere);
    } else if (y == 1) {
      setNewPosition(2 + x, 1, newSevere);
    } else if (y == 2) {
      setNewPosition(2 + x, 2, newSevere);
    } else if (y == 3) {
      setNewPosition(2 + x, 2, newSevere);
    } else if (y == 4) {
      setNewPosition(2 + x, 3, newSevere);
    } else if (y == 5) {
      setNewPosition(2 + x, 4, newSevere);
    } else if (y == 6) {
      setNewPosition(2 + x, 3, newSevere);
    } else if (y == 7) {
      setNewPosition(2 + x, 3, newSevere);
    } else if (y == 8) {
      setNewPosition(2 + x, 2, newSevere);
    } else if (y == 9) {
      setNewPosition(2 + x, 3, newSevere);
    } else if (y == 10) {
      setNewPosition(2 + x, 4, newSevere);
    }
  }
}

/*
void sendLookAt(int x, int y, int newSevere) {
  char message[4];
  message[0] = 'L';
  message[1] = x;
  message[2] = y;
  message[3] = newSevere;
  sendEvent(message);
}

void sendBlink() {
  char message[1];
  message[0] = 'B';
  sendEvent(message);
}


void sendEvent(char* message) {
  Wire.beginTransmission(4);
  Wire.write(message);
  Wire.endTransmission();
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  char message[howMany];
  for (int i = 0; i < howMany; i++) {
    message[i] = Wire.read();
  }
  if (howMany > 0) {
    char type = message[0];
    if (type == 'B') {
      doBlink();
    } else if (type == 'L') {
      int xx = message[1];
      int yy = message[2];
      int newSevere = message[3];
      lookAt(xx, yy, newSevere);
    }
  }
}
*/
