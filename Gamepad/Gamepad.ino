//Controller program

#include "U8glib.h"
U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI



#define __DEBUG__
//#define __RF24__

// Baud rate must match the setting of UART modules(HC-05,HM-10 and etc),
// set module to master mode if requires
#define BAUD 57600

#define Console Serial
//#define BlueTooth Serial


    #include <SoftwareSerial.h>
//#undef BlueTooth
    #define BLE_TX       10 //接蓝牙RX
    #define BLE_RX       11 //接蓝牙TX
    #define BLEbaud  9600 
    SoftwareSerial BlueTooth(BLE_RX, BLE_TX);   //RX, TX
    //SoftwareSerial BlueTooth(11, 10); //11 接蓝牙 TX，  10接蓝牙RX

//Wireless Communication
#ifdef __RF24__
/*
  library  - https://github.com/tmrh20/RF24/
  tutorial - https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define CE  2   // ce pin
#define CSN 3   // csn pin
const char const address[]  = {0x01, 0xff, 0xff, 0xff, 0xff, 0xff}; // change if needs, address for communication
RF24 radio(CE, CSN); //create RF24 objects
#endif

// joystick pins
#define VRX_PIN   A0
#define VRY_PIN   A1
#define SW_PIN    A2

//
#define BEEPER_PIN A3

// pins for matrix switches
#define MATRIX_ROW_START 6 // pin 6 to 9
#define MATRIX_NROW 4
#define MATRIX_COL_START 2 // pin 2 to 5
#define MATRIX_NCOL 4

const char *walks1[] {
  "wkR", "wkL", "wk", "bk", //walk
};

const char *walks2[] {
  "trR", "trL", "tr", "bk", //trot小跑
};

const char *walks3[] {
  "crR", "crL", "cr", "bk", //crawl爬行
};



const char *walks4[] {
  "bkR", "bkL",  "tr", "bk", //back
};

const char **walkings[] {
  walks1, walks2, walks3, walks4,
};

char **walkptr; //pointer

const char *row1[] {
  "str", "sit", "buttUp", "pee",
};

const char *row2[] {
  "c","rest", "zero", "pu", //c = zero, rest = hold position   !Cannot pee after push up/show
};

// disabled
const char *row3[] {
  "d", "balance", "vt",  "show", "dropped", //balance = neutral position, Vt跺脚, show = continuous motion
};

char *cmdptr,prev_cmd[24] = "rest";
boolean is_gait = true;

char **instincts[] {
  walkings[0], row1, row2, row3
};



void setup() {
 //LCD Monitor
   // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  u8g.setFontPosTop();
  
//Serial setup
  Console.begin(BAUD);
  while (!Console); // wait for serial port to connect. Needed for native USB port only
#ifdef __DEBUG__
  Console.println("started!");
  BlueTooth.begin(BLEbaud);
 #endif //added
 
 //Joystick pin
  //pinMode(SW_PIN, INPUT);
  //digitalWrite(SW_PIN, HIGH);
  pinMode(SW_PIN, INPUT_PULLUP); //注意 Z 轴输入一定要上拉，不然电平不稳。

  //Wireless transmitter
#ifdef __RF24__
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
#endif
  tone(BEEPER_PIN, 300);
  delay(100);
  noTone(BEEPER_PIN);
}


void loop() {
  boolean sw1 = false;
  static long cmdPeriod = 0;
  int longClick = 0;
  int joy_x;
  int joy_y;
  int row = -1, col = -1;
  char token = 'k';

//Matrix pushbutton actions
  int matrix = scanmatrix(&longClick); //return curmatrix
  if (matrix >= 0) { //pushing button
    row = (matrix) / 4; 
    col = matrix % 4;
    
    if (row == 0) { //uppermost row == walking modes
      instincts[0] = walkings[col];
      col = 2; //default walk forward
    } else {
      cmdptr = instincts[row][col]; //action modes
      is_gait = false;
      goto __send;
    }
  }

//Detect joystick movement
  joy_x = analogRead(VRY_PIN);
  joy_y = analogRead(VRX_PIN);
  /*
  //碰到有问题的加
  joy_x = map(analogRead(VRY_PIN), 0, 1023, 0, 255);
  joy_y = map(analogRead(VRX_PIN), 0, 1023, 0, 255);
*/
  if (joy_x < 300) {
    col = 0;
    is_gait = true;
  } else if (joy_x > 900) {
    col = 1;
    is_gait = true;
  } else if (joy_y < 300) {
    col = 2;
    is_gait = true;
  } else if (joy_y > 900) {
    col = 3;
    is_gait = true;
  } else {
    if (is_gait)
      cmdptr = "balance";
  }
  sw1 = digitalRead(SW_PIN);
  if (!sw1) { //sw1 =low
    // disable servo
    token = 'd';
    cmdptr = "d";  //临时修改， 源代码：cmdptr = "";  是因为主板上蓝牙对接RX,TX， 
  }
  
//walk mode
  if (col >= 0) {
    walkptr = instincts[0];
    cmdptr = walkptr[col];
  }

//action mode
__send:
  if (millis() >= cmdPeriod && strcmp(cmdptr, prev_cmd) != 0) {
    //BlueTooth.write(token); //临时屏蔽，因为不是机器人主板上蓝牙对接主板的RX,TX，需要通过内部解释
#ifdef __RF24__
    radio.write(&token, 1);
#endif
    if (cmdptr != nullptr) {
      BlueTooth.print("{\"btn-dir\":\"");
      BlueTooth.print(cmdptr);
      BlueTooth.println("\"}");
      //BlueTooth.write(cmd, strlen(cmd));
#ifdef __RF24__
      radio.write(cmd, strlen(cmd));
#endif
      strcpy(prev_cmd, cmdptr);
#ifdef __DEBUG__
      //Console.write(token);
      Console.println(cmdptr);
      /*//查看输出字符串
      Console.print("{\"btn-dir\":\"");
      Console.print(cmdptr);
      Console.print("\"}");
      Console.println();
      */

    
     // picture loop
      u8g.firstPage();  
      do {
        //draw();
        //u8g.setFont(u8g_font_unifont);
        u8g.setFont(u8g_font_9x15);
          u8g.setPrintPos(0, 15); 
        //u8g.setFont(u8g_font_osb21);
            u8g.print("CMD: ");
            u8g.print(cmdptr);
            u8g.println("");
      } while( u8g.nextPage() );
  
#endif
    }
    cmdPeriod = millis() + 50; //50 = delay time
  }

}


//4x4 matrix detection
int scanmatrix(int *longClick) { 
  static int priorMatrix = -1;
  static long curMatrixStartTime = 0;

  *longClick = 0;
  // we will energize row lines then read column lines
  // first set all rows to high impedance mode
  for (int row = 0; row < MATRIX_NROW; row++) {
    pinMode(MATRIX_ROW_START + row, INPUT);
  }
  // set all columns to pullup inputs
  for (int col = 0; col < MATRIX_NCOL; col++) {
    pinMode(MATRIX_COL_START + col, INPUT_PULLUP); //enable internal pull-up resistor = inverted
  }

  // read each row/column combo until we find one that is active
  for (int row = 0; row < MATRIX_NROW; row++) {
    // set only the row we're looking at output low
    pinMode(MATRIX_ROW_START + row, OUTPUT);
    digitalWrite(MATRIX_ROW_START + row, LOW);

    for (int col = 0; col < MATRIX_NCOL; col++) {
      delayMicroseconds(100);
      if (digitalRead(MATRIX_COL_START + col) != LOW) { //did not push button
        continue;
      }
      // we found the first pushed button
      int curmatrix = row * MATRIX_NROW + col;
      //
      int clicktime = millis() - curMatrixStartTime;
      if (curmatrix != priorMatrix) {
        curMatrixStartTime = millis();
        priorMatrix = curmatrix;
        *longClick = 0;
      } else if (clicktime > 500) {
        // User has been holding down the same button continuously for a long time
        if (clicktime > 3000) {
          *longClick = 2;
        } else {
          *longClick = 1;
        }

      }
      return curmatrix;
    }
    pinMode(MATRIX_ROW_START + row, INPUT); // set back to high impedance
    //delay(1);
  }

  return -1;
}




void draw(void) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_osb21);
  u8g.drawStr( 0, 22, "Hello World!");
}
