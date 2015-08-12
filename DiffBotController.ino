#import <math.h>

#define remoteID 2 //polled control remote ID
#define robotID 2 //direct control robot ID

#define RIGHT_V   0
#define RIGHT_H   1
#define LEFT_V    2
#define LEFT_H    3

#define NUM_STICKS 4

#define BUT_L6    13
#define BUT_L5    12
#define BUT_L4    11
#define BUT_LT    9

#define BUT_R3    7
#define BUT_R2    6
#define BUT_R1    5
#define BUT_RT    8

#define BUT_RIGHT 3
#define BUT_LEFT  2

#define NUM_BUTTONS 8

#define USER      10          //user led

#define TIMEOUT   1000

#define FRAME_LEN 100         // 10hz

#define COUNT_L6 0
#define COUNT_L5 1
#define COUNT_L4 2
#define COUNT_LT 3
#define COUNT_R3 4
#define COUNT_R2 5
#define COUNT_R1 6
#define COUNT_RT 7
#define BUTTON_THRESHOLD 3
 
//Enums
enum {
  WAIT,
  RESPOND
};
 
enum {
  SYNC_0,
  SYNC_1,
  REMOTE_ID,
  CRC
};

enum {
  TASK_IDLE = 0,  //Robot stops (0 params)
  TASK_MANUAL = 1,  //Controls the motors directly (2 params, lmotor and rmotor (0 to 200, 100 is idle))
  TASK_SPIN = 2,  //Spins on the spot (4 params, direction (0 or 1), speed (0 to 100), time (ms, split into two parts))
  TASK_STOP = 4,
  TASK_DRIVE = 5,
  PING = 255  //A 'keep alive' type message, just to avoid timeout (0 params)
};
 
//Messages themselves
typedef struct _radio_msg {
  uint8_t hdr0;
  uint8_t hdr1;
  uint8_t id;
  uint8_t crc;
  //end data
  uint8_t next;
  int mode;
  long last;
} radio_msg_t;

typedef struct _respond_msg {
  uint8_t hdr0;
  uint8_t hdr1;
  uint8_t id;
  uint8_t lsth;
  uint8_t lstv;
  uint8_t rsth;
  uint8_t rstv;
  uint8_t but;
  uint8_t crc;
} respond_msg_t;
 
//Variables
uint32_t ltime;         // light timer
uint32_t rtime;         // direct control loop timer
uint8_t i;              // direct control loop counter

uint8_t sticks[4];      // direct control stick values
uint8_t mode = 0;       // unused
uint8_t type = 0;       // direct control task enum

uint8_t buttonArray[8] = {0}; //direct control debouncing
uint8_t countArray[8] = {0};  //direct control debouncing

void setup(){
  char buf[3] = {0};
  Serial.begin(57600);
  Serial.print("XBee Serial Commander V1.0 controller ");
  sprintf(buf, "%d", remoteID);
  Serial.println(buf);
  ltime = rtime = millis();
  pinMode(USER,OUTPUT);    // user LED
  
  // pullups for buttons
  digitalWrite(BUT_L6, HIGH);
  digitalWrite(BUT_L5, HIGH);
  digitalWrite(BUT_L4, HIGH);
  digitalWrite(BUT_LT, HIGH);
  digitalWrite(BUT_R3, HIGH);
  digitalWrite(BUT_R2, HIGH);
  digitalWrite(BUT_R1, HIGH);
  digitalWrite(BUT_RT, HIGH);
  
  digitalWrite(BUT_RIGHT, HIGH);
  digitalWrite(BUT_LEFT, HIGH);
 
  digitalWrite(USER, LOW);
}

void loop(){
  
    for (int i = 0; i < 4; i++) {
      if (i%2 == 0) {
        sticks[i] = (1023-analogRead(i))/5.12;
      }
      else {
        sticks[i] = analogRead(i)/5.12;
      }
      if (sticks[i] > 90 && sticks[i] < 110) {
        sticks[i] = 100;
      }
    }   
    
    // Detect and debounce buttons
    uint8_t buttons = 0;
    if(!digitalRead(BUT_R1) && !buttonArray[COUNT_R1]) countArray[COUNT_R1]++;
    else { countArray[COUNT_R1] = 0; buttonArray[COUNT_R1] = 0; }
    if(!digitalRead(BUT_R2) && !buttonArray[COUNT_R2]) countArray[COUNT_R2]++;
    else { countArray[COUNT_R2] = 0; buttonArray[COUNT_R2] = 0; }
    if(!digitalRead(BUT_R3) && !buttonArray[COUNT_R3]) countArray[COUNT_R3]++;
    else { countArray[COUNT_R3] = 0; buttonArray[COUNT_R3] = 0; }
    if(!digitalRead(BUT_L4) && !buttonArray[COUNT_L5]) countArray[COUNT_L4]++;
    else { countArray[COUNT_L4] = 0; buttonArray[COUNT_L4] = 0; }
    if(!digitalRead(BUT_L5) && !buttonArray[COUNT_L5]) countArray[COUNT_L5]++;
    else { countArray[COUNT_L5] = 0; buttonArray[COUNT_L5] = 0; }
    if(!digitalRead(BUT_L6) && !buttonArray[COUNT_L6]) countArray[COUNT_L6]++;
    else { countArray[COUNT_L6] = 0; buttonArray[COUNT_L6] = 0; }
    
    if(!digitalRead(BUT_RT)) buttons |= (1 << COUNT_RT);
    if(!digitalRead(BUT_LT)) buttons |= (1 << COUNT_LT);
    
    type = TASK_MANUAL;
    
    if(countArray[COUNT_R1] > BUTTON_THRESHOLD) {
      buttons |= (1 << COUNT_R1);
      type = TASK_SPIN;
      buttonArray[COUNT_R1] = 1;
    } else {
      buttons &= ~(1 << COUNT_R1);
    }
    if(countArray[COUNT_R2] > BUTTON_THRESHOLD) {
      buttons |= (1 << COUNT_R2);
      type = TASK_SPIN;
      buttonArray[COUNT_R2] = 1;
    } else {
      buttons &= ~(1 << COUNT_R2);
    }
    if(countArray[COUNT_R3] > BUTTON_THRESHOLD) {
      buttons |= (1 << COUNT_R3);
      type = TASK_SPIN;
      buttonArray[COUNT_R3] = 1;
    } else {
      buttons &= ~(1 << COUNT_R3);
    }
    /*if(countArray[COUNT_RT] > BUTTON_THRESHOLD) {
      buttons |= (1 << COUNT_RT);
      buttonArray[COUNT_RT] = 1;
    } else {
      buttons &= ~(1 << COUNT_RT);
    }*/
    if(countArray[COUNT_L4] > BUTTON_THRESHOLD) {
      buttons |= (1 << COUNT_L4);
      buttonArray[COUNT_L4] = 1;
    } else {
      buttons &= ~(1 << COUNT_L4);
    }
    if(countArray[COUNT_L5] > BUTTON_THRESHOLD) {
      buttons |= (1 << COUNT_L5);
      buttonArray[COUNT_L5] = 1;
    } else {
      buttons &= ~(1 << COUNT_L5);
    }
    if(countArray[COUNT_L6] > BUTTON_THRESHOLD) {
      buttons |= (1 << COUNT_L6);
      buttonArray[COUNT_L6] = 1;
    } else {
      buttons &= ~(1 << COUNT_L6);
    }
    /*if(countArray[COUNT_LT] > BUTTON_THRESHOLD) {
      buttons |= (1 << COUNT_LT);
      buttonArray[COUNT_LT] = 1;
    } else {
      buttons &= ~(1 << COUNT_LT);
    }*/
    
    uint8_t one;
    uint8_t two;
    uint8_t three = remoteID;
    uint8_t four = buttons;
    int d = (100-sticks[RIGHT_H]) * 0.3;
    int v = (100-sticks[LEFT_V]) * 0.5; 
    one = 100+max(-100, min(100, v+d));
    two = 100+max(-100, min(100, v-d));
    
    int8_t crc = 0xFF ^ robotID ^ type ^ one ^ two ^ three ^ four ^ i;

    //Remote Control
    // hdr0
    Serial.write(0xAA);
    // hdr1
    Serial.write(0x55);
    // rid
    Serial.write(robotID);
    // task type
    Serial.write(type);  //1 for mot cont
    // data field 1
    Serial.write(one);
    // data field 2
    Serial.write(two);
    // data field 3
    Serial.write(three);
    // data field 4
    Serial.write(four);
    // seqno
    Serial.write(i);
    // crc
    Serial.write(crc);

    // blink LED    
    if ((i/2)%(1+mode) == 0) {
      digitalWrite(USER,HIGH-digitalRead(USER));
    }
    i++;
    
    delay(100-(millis() - rtime));
    rtime = millis();
}

/* Revisions 
 *  V1.2 - Feb 11, 2012 - Updated for Arduino 1.0
 *  V1.1 - Feb 19, 2010 - Replaced Walk/Look with Right/Left 
 *         (since apparently, I used something called "southpaw")
 *  V1.0 - Feb 10, 2010 - Firmware Finalized
 */
 


