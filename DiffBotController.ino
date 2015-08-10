#import <math.h>

#define remoteID 3 //polled control remote ID
#define robotID 16 //direct control robot ID

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
  TASK_MANUAL,  //Controls the motors directly (2 params, lmotor and rmotor (0 to 200, 100 is idle))
  TASK_SPIN,  //Spins on the spot (4 params, direction (0 or 1), speed (0 to 100), time (ms, split into two parts))
  TASK_LED, //Sets the LED status of the top LEDs. (4 params, R, G, B, ledno (0, 1, 255))
  TASK_STOP,
  TASK_DRIVE,
  TASK_BRAKE = 250, //Stops. (0 params)
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

uint8_t buttarr[8] = {BUT_R1, BUT_R2, BUT_R3, BUT_L4, BUT_L5, BUT_L6, BUT_RT, BUT_LT}; //button to bit map for polled button state packing
radio_msg_t msg; //reusable message for reading polling requests into

uint8_t waitflag = 0; //flag for wait mode when responding to poll messages
uint32_t waitUntil; //time to wait until responding to a poll message
uint8_t directMode; //1 for direct control, 0 for polling
 
 
////////////////////////////////////////////////////////////////////////////////
//Finally, some actual code
 
/*
 * Packet structure:
 * HDR0, HDR1, RID, SEQNO, CRC
 */
int checkPacket() {
  while (Serial.available() > 0) {
    uint8_t crc = 0;
    msg.next = Serial.read();
    switch(msg.mode) {
      case SYNC_0:
        if (msg.next == 0xAA) msg.mode = SYNC_1;
        break;
      case SYNC_1:
        if (msg.next == 0x55) msg.mode = REMOTE_ID;
        else msg.mode = SYNC_0;
        break;
      case REMOTE_ID:
        msg.id = msg.next;
        if (msg.id == remoteID || msg.id == 250) msg.mode = CRC;
        else msg.mode = SYNC_0;
        break;
      case CRC:
        for (int i = 0; i < 3; i++) {
          crc ^= ((uint8_t *)&msg)[i];
        }
        msg.mode = SYNC_0;
        msg.crc = msg.next;
        msg.last = crc == msg.crc ? millis() : msg.last;
        return crc == msg.crc;
      default:
        break;
    }
  }
  return 0;
}
 
/*
 * Packet structure:
 * HDR0, HDR1, RID, RV, RH, LV, LH, BUTTONS, CRC
 * sticks in range 0 to 200 (really 1 to 199?), buttons as a combined structure in the order of
 * buttarr from lsv to msv
 */
void respond() {
   respond_msg_t outmsg;
   uint8_t *p = (uint8_t *) &outmsg;
   //Set up some response vars
 
   //Write the header bits
   outmsg.hdr0 = 0xAA;
   outmsg.hdr1 = 0x55;
   outmsg.id = remoteID;
 
   //Write the analog sticks (0 to 200)
   for (int i = 0; i < NUM_STICKS; i++) {
      uint8_t stick;
      if (i%2 == 0) {
        stick = (1023-analogRead(i)) >> 2;
        //stick = (1023-analogRead(i))/5.12;
      }
      else {
        stick = analogRead(i) >> 2;
        //stick = analogRead(i)/5.12;
      }
      if (stick > 90 && stick < 110) {
        stick = 100;
      }
      p[3+i] = stick;
    }
 
    //Write the buttons, in one big btye
    outmsg.but = 0;
    for (int i = 0; i < NUM_BUTTONS; i++) {
      outmsg.but |= !digitalRead(buttarr[i]) << i;
    }
  
    //Write the checksum, and we're done
    outmsg.crc = 0;
    for (uint8_t i = 0; i < sizeof(respond_msg_t)-1; i++) {
      outmsg.crc ^= p[i];
    }
 
    for (uint8_t i = 0; i < sizeof(respond_msg_t); i++) {
      Serial.write(p[i]);
    }
}
 
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
 
  //Set up the message
  msg.hdr0 = 0xAA;
  msg.hdr1 = 0x55;
  msg.last = 0;

  directMode = !digitalRead(BUT_RT) && !digitalRead(BUT_LT);
  if(directMode) {
    Serial.println("Directly controlling robot ");
    sprintf(buf, "%d", robotID);
    Serial.println(buf);
  } else {
    Serial.println("Polling");
  }
}

void pollingLoop(){
  if (waitflag) {
    if (micros() <= waitUntil || micros()-waitUntil > 0xFFFFFF)
      return;
      
    respond();
    waitflag = 0;
  }
    
  if(checkPacket()) {
    waitflag = 1;
    waitUntil = micros()+5000;
  }
  
  //Set the output pin to "are we connected?". Bright means yes.
  digitalWrite(USER, millis() - msg.last < TIMEOUT);
}

void directLoop(){
  
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
    if(!digitalRead(BUT_RT) && !buttonArray[COUNT_RT]) countArray[COUNT_RT]++;
    else { countArray[COUNT_RT] = 0; buttonArray[COUNT_RT] = 0; }
    if(!digitalRead(BUT_LT) && !buttonArray[COUNT_LT]) countArray[COUNT_LT]++;
    else { countArray[COUNT_LT] = 0; buttonArray[COUNT_LT] = 0; }
    
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
    uint8_t three = 0;
    uint8_t four = buttons;
    int d = (100-sticks[RIGHT_H]) * 0.5;
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

void loop() {
  if(directMode) directLoop(); else pollingLoop();
}

/* Revisions 
 *  V1.2 - Feb 11, 2012 - Updated for Arduino 1.0
 *  V1.1 - Feb 19, 2010 - Replaced Walk/Look with Right/Left 
 *         (since apparently, I used something called "southpaw")
 *  V1.0 - Feb 10, 2010 - Firmware Finalized
 */
 


