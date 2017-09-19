  /*
//ENGG1100 2016 omnicar auto 
// in AUTO mode, use switch() to check whether we need to go

* use the serial monitor to check what it can do. 
* Unlike in the readcom, message will only be displayed iff it is different to the previous one.

//programming the nano with switching the ABT-ON switch to power connecter side.
//control the robot via remote with switching the ABT-ON switch to ABT side

  // this is the keypad layout in remote control. 
  // IO PAD:
  // io4 | io3 | io2
  // ---------------
  // io0 | io9 | io8
  // ---------------
  // io7 | io6 | io5
  // Usage:
  //io0: tilting -5 degree
  //io2: start auto mode 
  //io3: disable
  //io4: disable
  //io5: rotate robot clockwise NORMAL and then slow
  //io6: rotate robot anti-clockwise NORMAL and then slow
  //io7: tilting +5 degree
  //io8: start launch 
  //io9: read sonar and display value 

 Joystick values will be used in the StateMachine(), hence to control the
 movement of robot in six linear directions.It will use moveforward(),moveforwardleft(),
 movebackwardleft() and ROT() to move robot. 

 *  You need a remote (ENGG1100 2014 or 2015) or smart phone with the code. 
 *  The PCB (ENGG1100 2015, with all components related to bluetooth and motors).
 *  Make sure bluetooth devices are paired (bluetooth LED will always ON if paired).
//View on top, AM1 is connected to the right hand side motor. 
Pin 1 of AM1 (rectangular  pad) to the lead of motor close to acrylic sheet.
//AM2 is in front of shooter, i.e. opposite to the shooting direction. 
Pin 1 of AM2 (rectangular  pad) to the lead of motor close to acrylic sheet.
//AM3 is connected to the left hand side motor. 
Pin 1 of AM3 (rectangular  pad) to the lead of motor close to acrylic sheet.
//adjust the voltage output of power module to 5.5V to 6.0V. 
This voltage cannot higher than 6.0V otherwise, RC Servo will burn.
//use jumper to short the pin 1 and 2 of MtrPwrsel.

//RC servo connector ASV3 to small RC servo ball loader. 
Pin 1 of ASV3 (square pad) to orange color cable (signal pin)
//ASV2 to tilting RC servo
//ASV1 to shooting RC servo

 * created 2015 
 * by Martin Leung (yyleung@mae.cuhk.edu.hk)
 * edited by wxdai (wxdai@ee.cuhk.edu.hk)
*/

#define ON   0
#define OFF  1
#define DEBUG_STRING ON    //change it to OFF, it will no display in serial monitor
#define resetpos 175
#define readylauncherpos 60
#define launchpos 175
#define loadball 175
#define TiltMAX 40
#define TiltMIN 10
#define DropBall 50 //for Tower Pro MC90S small metal gear RC servo
#define BarBall 120
#define TiltRST 10
#define M1MAXSPEED 255//use the maximum number 255
#define M2MAXSPEED 255
#define M3MAXSPEED 255
#define M1MINSPEED 40
#define M2MINSPEED 95
#define M3MINSPEED 40
/* conditions use in StateMachine() function   */
#define STATE0  ( (abs(X)<20 && abs(Y)<20) || Y<0 )    //statement will use more than once, i.e. better code management
#define STATE1 (Y>20 && ((X>0 && X<Y) || (X<0 && -X<Y) ) )
#define STATE2 (  Y>0 && X>20 && X>=Y)
#define STATE3  ( Y<0 && X>20 && X>-Y)
#define STATE4  ( Y<-20 && ((X>0 && X<-Y) || (X<0 && -X<=-Y) ) )
#define STATE5  ( Y<0 && X<-20 && -X>-Y)
#define STATE6  ( Y>0 && X<-20 && -X>=Y)
#define NO_OP 0
#define MANUAL 1
/* conditions use in StateMachine() function   */

/* cases in auto mode */
#define ZIGZAG_MOTION        10
#define MOVEFORWARD          11
#define ZIGZAG_MOTION_BACK   12
#define FIND_ZONE_A          13
#define FINISH_MOV_F         14
#define SLOW_ROT_CCW         15
#define FIND_NET_A           16
#define A_STOP_NET           17
#define A_SHOOT              18
#define FIND_GAP_NETA2B      19
#define FIND_OPEN_AREA       20
#define FIND_B               21
#define POINT2B              22
#define GO2B                 23
#define CW_FIND_GAP          24
#define FIND_NET_B_SUB_A     25
#define POINT2NET_B          26
#define B_SHOOT              27
/* cases in auto mode */

// time setting for auto mode
#define ZIGZAG_TIMELIMIT          10000   //zig zag time interval limit  //what is the values? 
#define SLOW_ROT_CCW_INTERVAL     3000 //slow rotate CCW in zone A time interval
#define CCW2NETA_DELAY            150   //time interval from slow rotate CCW to point to net //what is the values? 
#define NETA2B_CCW_INTERVAL       800 //after shooting at zone A, rotate robot CCW with time interval //what is the values? 
#define CCW2_B_INTERVAL           600 // time interval that robot will CCW to point to zone B //what is the values? 
#define CORNER_DELAY_TIME    500 //after that time interval the car is at the middle of the path //what is the values? 
#define MOV_F_DELAY         1500  //time interval for car to move to zone A from maze exit //what is the values? 
#define MANUAL_ROT_INTERVAL        1000 //after that interval rotation will be slow mode

// distance used for auto mode
#define ZONE_A2NET_DISTANCE     70  //what is the values? 
#define OPEN_AREA_DISTANCE      92  //what is the values?      
#define FIND_B_SONAR_DIS        89   //what is the values? 
#define DIST2B                  20  //what is the values? 
#define MAX_SONAR_DIS_MEASURE  7000  // ~ to 120cm 

// counter value for auto mode
#define FIND_B_IN_COUNT   4 //number of count for FIND_B_SONAR_DIS  //what is the values? 
#define OPEN_AREA_COUNT 1   //what is the values?   

//robot rotates routine setting
#define CW 0
#define CCW 1
#define SLOW 0
#define NORMAL 1
#define FASTREACH 2


#include "Motor_v3.h"
#include <Servo.h> 
/**********used to display information to serial port for troubleshooting the program **********/
#include <stdarg.h>
void p(char mode, char *fmt, ... ){ //if mode == 0 print
  if(mode == ON){
    char buf[100]; // resulting string limited to 100 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 100, fmt, args);
    va_end (args);
    Serial.print(buf);         
  }
}
/**********used to display information to serial port for troubleshooting the program **********/

static unsigned long time_counter_start=0;
static unsigned long shoot_time_counter=0;
static unsigned long loop_start_time=0;
int M1stepSPD = M1MAXSPEED;  // The maximum PWM speed for motor M1.
int M2stepSPD = M2MAXSPEED;
int M3stepSPD = M3MAXSPEED;  
int stepS = 30;  // The PWM  (+/-) speed step setting.
unsigned long sTickTimeout = 3000; // Bluetooth connection timeout (3000)(ms);
boolean sMotorStop = false;  // force MOTOR to STOP via soft IO
boolean sPCcontrol = false;  // pc control (enable = true = control through serial comm on PC)
boolean launchSTATE = false; //if it is true, ball will shoot when the state is checked.
unsigned long pLastTick;    // Timestamp of the last reveive message via bluetooth(ms). use for time out
int RobotRot = 0;          // rotate robot setting.
int MotorX = 128;          // value received by joystick.
int MotorY = 128;
byte RemoteCmd = NO_OP;  //if it is MANUAL, subroutine StateMachine will run. if it is AUTO, go to auto mode
boolean sDeviceFlag = false; //if it is true, bluetooth received data is ready, subroutine SetDevice will run when the state is checked
byte smstate = 0, oldstate=0;  
byte Past_RemoteCmd = NO_OP;

//PIN & IO for PCB, DONOT change

const int M1DIR = 2;
const int M1DIR_S = 4;
const int M1PWM = 5; 
const int M2DIR = 7;
const int M2DIR_S = 8;
const int M2PWM = 6; 
const int M3DIR = 12; 
const int M3DIR_S = 13;
const int M3PWM = 11; 

Servo sv1,sv2,sv3; 
const int trig = A0;
const int echo= A1;

volatile int tilting_step = 5;
int tilt_angle = TiltMIN;
  Motor M1(M1DIR,M1DIR_S, M1PWM);  // every Motor treat as an object.
  Motor M2(M2DIR,M2DIR_S, M2PWM);
  Motor M3(M3DIR,M3DIR_S, M3PWM);

/******* variable used for motor, RC servo, sonar  ******************/

/***** variable used for Bluetooth Communication *************************/
//String inputString = "";         // a string to hold incoming data
//String outputString = "";
boolean stringComplete = false;  // whether the string is complete
//String btString = "";        // String Buffer received from Bluetooth
String btDev = "";           // Device name
String btVal = "";           // Device value
char btChar = ' ';
int btStatus = 0;     // Parameter name (0), Parameter value (1)
int btSTATE = 0;      // Detemine the current STATE.  (Receive X (0), receive Y (1)
String btSendStr = "";
/***** variable used for Bluetooth Communication *************************/

/* variables for debug */
int M1sp=0,M2sp=0,M3sp=0;
/* variables for debug */

unsigned long Sdir,pSread,netDis; 
byte sonar_reach_cnt=0;
void setup() {  
  /** sonar setup **/
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);
  /** sonar setup **/
  /*** Motor setup   ***/
  pinMode(M1DIR, OUTPUT);
  pinMode(M1DIR_S, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M2DIR_S, OUTPUT);
  pinMode(M3DIR, OUTPUT);
  pinMode(M3DIR_S, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(M3PWM, OUTPUT); 
  M1.setProperties(M1stepSPD, (-1*M1stepSPD), stepS); //this function will set the maximum speed, minimum speed and step size of motor 
  M2.setProperties(M2stepSPD, (-1*M2stepSPD), stepS);
  M3.setProperties(M3stepSPD, (-1*M3stepSPD), 20); 
  smstate =0;
  /*** Motor setup   ***/
  /*** RCservo setup   ***/
  sv1.attach(9);
  sv2.attach(10);
  sv3.attach(3);    
  sv1.write(readylauncherpos);
  sv2.write(TiltRST);
  sv3.write(BarBall);
  /*** RCservo setup   ***/
  /*** Bluetooth setup ***/
  Serial.begin(115200);
  btDev.reserve(2);
  btVal.reserve(4);
  pLastTick = 0;   
  /*** Bluetooth setup ***/
  p(ON,"tV3\n");
}

/*****    functions that tell how to move your robot, 
the value for each motor is different for different robot,
you need find the value for your own. Otherwise the movement is poor.
*******/
//v1.1
void movebackwardright(){
  
  
}

void movebackward(){
  M1.set(200); //what is the value?
  M2.set(0); //what is the value?
  M3.set(-168); //what is the value?
}
void movestop(){
  M1.set(0); 
  M2.set(0); 
  M3.set(0);  
  while(M1sp!=0 || M2sp !=0 || M3sp !=0){                    
    updateMotor();
    p(DEBUG_STRING,"movestop %d, %d, %d\n",M1sp, M2sp, M3sp);        
    M1sp = M1.getSPEED(); 
    M2sp = M2.getSPEED();
    M3sp = M3.getSPEED();         
  }
}
void moveforward(){
int m1 = -168; //what is the value?
int m2 = 0; //what is the value?
int m3 = 178; //what is the value?
    M1.set(m1); 
    M2.set(m2); 
    M3.set(m3); 
    M1sp =0;
      while(M1sp!=m1 || M2sp !=m2 || M3sp !=m3){                    
        updateMotor();
        M1sp = M1.getSPEED(); 
        M2sp = M2.getSPEED();
        M3sp = M3.getSPEED();         
        p(DEBUG_STRING,"mfw %d, %d, %d\n",M1sp, M2sp, M3sp);        
      }                  
}
void moveforwardleft(){
    M1.set(0); //what is the value?
    M2.set(-190);//what is the value?
    M3.set(200); //what is the value?
}
void movebackwardleft(){
    M1.set(200);  //what is the value?
    M2.set(-190); //what is the value?
    M3.set(0);  //what is the value?
}
void SWALK_LEFT(){
    M1.set(162); 
    M2.set(-240); 
    M3.set(118);    
}
void SWALK_RIGHT(){
    M1.set(-162); 
    M2.set(240); 
    M3.set(-118);    
}

/****
robot rotation, if "a" is CW, robot rotates clockwise, else robot rotates counter clockwise
if "b" is NORMAL, robot rotates with normal speed, else robot rotates with slow speed
if "c" not equal to NORMAL, robot will change its speed to target values immediately.
you need to find your own M1sp, M2sp and M3sp for your robot. Otherwise the rotation may be poor.
***/
void ROT(byte a,byte b,byte c){
  if(a==CW){
    if(b==NORMAL){ M1sp=155; M2sp =155; M3sp =145; }     
      else{M1sp=135; M2sp =135; M3sp =122;} 
  }else{ //a == CCW
    if (b== NORMAL){M1sp=-155; M2sp =-155; M3sp =-145;}    
      else { M1sp=-135; M2sp =-135; M3sp =-122; }  //slow   
  }
    M1.set(M1sp); 
    M2.set(M2sp); 
    M3.set(M3sp);    
 int m1 = 0;
 int m2 = 0;
 int m3 = 0;   
  if(c != NORMAL){
      while(M1sp!=m1 || M2sp !=m2 || M3sp !=m3){                    
        updateMotor();
        p(DEBUG_STRING,"ROT FAST %d, %d, %d\n",M1sp, M2sp, M3sp);        
        m1 = M1.getSPEED(); 
        m2 = M2.getSPEED();
        m3 = M3.getSPEED();         
      }         
  }  
}



// this is arduino program loop.
// for details, please take a look of lecture notes.
// MAKE SURE you understand what you are doing before change the codes
void loop()
{ 
  int a=0,b=0;
  unsigned long duration; 
  loop_start_time = millis();//millis() Returns the number of milliseconds since the Arduino board began running the current program. 
  //p(DEBUG_STRING," timer %lu\n",loop_start_time);
  BluetoothCom();  //get data from bluetooth
  if (sDeviceFlag) {
    SetDevice();
    sDeviceFlag = false;
  }
  if ( pLastTick < (loop_start_time-sTickTimeout) || pLastTick == 0) {        
    // Bluetooth receive TIMEOUT
    M1.set(0);
    M2.set(0);
    M3.set(0);
  }
  else{    
    //Decide to do StateMachine Function 
    if (RemoteCmd == MANUAL) {
      StateMachine(MotorY, MotorX);
      RemoteCmd = NO_OP;
      if(launchSTATE){ //launcher start
        sv1.write(launchpos);
        //p(DEBUG_STRING,"launch\n");
        shoot_time_counter=loop_start_time;
        launchSTATE = false;
      }
      if(shoot_time_counter !=0){
        if(((loop_start_time-shoot_time_counter)>=1000) && 
          ((loop_start_time-shoot_time_counter)<3000)){   
          sv1.write(loadball);
          sv3.write(DropBall);
         // p(DEBUG_STRING,"loadball\n");
        }
        else if((loop_start_time-shoot_time_counter)>3000){
          sv1.write(readylauncherpos);
          sv3.write(BarBall);
          //p(DEBUG_STRING,"readylaunch\n");
          shoot_time_counter=0; 
        }
      }
    }else if(RemoteCmd > MANUAL){
      /*****
      here is the auto mode routine. we use switch cases to seperate the whole tasks into small tasks
      when you press the auto mode button in remote, the RemoteCmd will be set to ZIGZAG_MOTION.
      If a predefined condition happens, RemoteCmd will be set to ROT_CCW90DEG and so on.     
      ******/
        Sdir = read_sr04()/58; //sonar measured distance in cm
        Serial.print(loop_start_time);   
        p(DEBUG_STRING," Sdir=%d\n",Sdir);   

        switch(RemoteCmd){


          case ZIGZAG_MOTION:// move in zig zag to forward left and backward left
            //move zig zag depends on sonar readings
            if((Sdir <=5) && (pSread <=5)){ 
              //if the past and present sonar reading is less than 5cm
                movebackwardleft();
                p(DEBUG_STRING,"BL\n");
            }else if((Sdir >=18) && (pSread >=18)){ 
              //if the past and present sonar reading is greater than 18cm
              moveforwardleft();
              p(DEBUG_STRING,"FL\n");
            }
            
            //check when to stop or go to next case
            if((Sdir >40) && (pSread >40)){
              //if the past and present sonar reading is greater than 40cm
              //suppose we reach the end of the wall and find an open area at crossing
             //stop motors immediately, set time interval CORNER_DELAY_TIME, go to next case
                 movestop();   
                 if (Sdir >= 50){moveforwardleft();}
                 else if (Sdir<50){movebackwardleft();}
                 time_counter_start = loop_start_time + CORNER_DELAY_TIME;
                 RemoteCmd = MOVEFORWARD;
              
            }else if(loop_start_time > time_counter_start){
               // time interval ZIGZAG_TIMELIMIT is over, time out auto mode , restore to MANUAL mode
              movestop();
              RemoteCmd=MANUAL;
              p(DEBUG_STRING,"time out ZIGZAG_MOTION\n");
            }
            break;
            


          case MOVEFORWARD:
            if(loop_start_time > time_counter_start){
              moveforward();
              if ((Sdir <= 6) && (pSread <= 6)){
                // moveforward on next way, until the sonar reading is less than 6 cm.
                // stop immediately and go to next case.
                movestop();
                RemoteCmd = ZIGZAG_MOTION_BACK;
                movebackwardright();
                p(DEBUG_STRING,"MF\n");
              }
            }
            break;


          case ZIGZAG_MOTION_BACK:
            // move in zigzag to forward right and backward right.
 
       //v1.1     
            break;
                        
        case FIND_ZONE_A:
              // moveforward to find zone A.

       //1.1     
            break;
            
        case FINISH_MOV_F:
           if(loop_start_time > time_counter_start){     
            //after time interval MOV_F_DELAY, stop robot, go to next case
            movestop();
            RemoteCmd=SLOW_ROT_CCW;
            launchSTATE = true;
          } 
         break;

        case SLOW_ROT_CCW: 
            //rotate ccw slowly until find the net.
            ROT(CCW,SLOW,FASTREACH); 
            RemoteCmd=FIND_NET_A;
            time_counter_start = loop_start_time + SLOW_ROT_CCW_INTERVAL; 
            p(DEBUG_STRING,"SLOW_ROT_CCW,interval %d\n",SLOW_ROT_CCW_INTERVAL);
         break;
            
        case FIND_NET_A:
          if((Sdir < ZONE_A2NET_DISTANCE && pSread < ZONE_A2NET_DISTANCE) ){
           //present and past sonar readings are less than ZONE_A2NET_DISTANCE cm, robot is nearly point to net
           //set time interval CCW2NETA_DELAY hence robot will point to net A better 
            RemoteCmd=A_STOP_NET;
            time_counter_start = loop_start_time + CCW2NETA_DELAY;
            p(DEBUG_STRING,"SRCCW90-2-A_STOP_NET,CCW2NETA_DELAY=%d\n",CCW2NETA_DELAY);
          }else if(loop_start_time > time_counter_start){
               // time interval SLOW_ROT_CCW_INTERVAL is over, time out auto mode , restore to MANUAL mode
              movestop();
              RemoteCmd=MANUAL;
              p(DEBUG_STRING,"time out SLOW_ROT_CCW_INTERVAL\n");
            }
          break;  
          
          case A_STOP_NET: //rotate robot to shoot more accurately
          if(loop_start_time > time_counter_start){ 
            RemoteCmd=A_SHOOT;
            movestop(); 
          }
          break;
           
       case A_SHOOT:
          //shooting a ball 
            if(launchSTATE){ //launcher start
              sv1.write(launchpos);
              //p(DEBUG_STRING,"A_SHOOT launch\n");
              shoot_time_counter=loop_start_time;
              launchSTATE = false;
            }
            if(shoot_time_counter !=0){
              if(((loop_start_time-shoot_time_counter)>=1000) && 
                ((loop_start_time-shoot_time_counter)<3000)){   
                sv1.write(loadball);
                sv3.write(DropBall);
                //p(DEBUG_STRING,"A_SHOOT loadball\n");
              }
              else if((loop_start_time-shoot_time_counter)>3000){
                //after shooting and load ball, rotate robot CCW with time interval NETA2B_CCW_INTERVAL
                sv1.write(readylauncherpos);
                sv3.write(BarBall);
                p(DEBUG_STRING,"in A_SHOOT_2_FIND_GAP_NETA2B\n");
                RemoteCmd = FIND_GAP_NETA2B;  
                time_counter_start = loop_start_time + NETA2B_CCW_INTERVAL; 
                moveforward();
                //ROT(CW,NORMAL,NORMAL); 
              }
            }
            break;
       
         case FIND_GAP_NETA2B:
            if(loop_start_time >time_counter_start){
                //after time interval NETA2B_CCW_INTERVAL, rotate robot CCW slowly, go to next case
                sonar_reach_cnt = 0;
                ROT(CCW,SLOW,FASTREACH); 
                RemoteCmd = FIND_OPEN_AREA;
                p(DEBUG_STRING,"FGNA2B_2_FIND_OPEN_AREA\n"); 
            }            
            break;
            
         case FIND_OPEN_AREA: 
           if(Sdir > OPEN_AREA_DISTANCE){
             //if sonar reading is greater than the OPEN_AREA_DISTANCE, increase the counter sonar_reach_cnt by one, 
             //else set it to zero
               sonar_reach_cnt++;
           }else if(pSread < OPEN_AREA_DISTANCE){
              sonar_reach_cnt=0;}
           if(sonar_reach_cnt > OPEN_AREA_COUNT){
             //if the counter greater than OPEN_AREA_COUNT, go to next case
              RemoteCmd=FIND_B;
              sonar_reach_cnt = 0;
              time_counter_start = loop_start_time + 2000; //time limit for B
              p(DEBUG_STRING,"FGAPB2FIND_B slow2000\n");               
            }else {p(DEBUG_STRING,"FGAPB cnt=%d\n",sonar_reach_cnt); }
          break;
          
          case FIND_B:
              if(Sdir < FIND_B_SONAR_DIS){
                //if sonar reading less than FIND_B_SONAR_DIS, increase counter sonar_reach_cnt by one,
               //else set it to zero 
                sonar_reach_cnt++;
              }else sonar_reach_cnt =0;
              if(sonar_reach_cnt == FIND_B_IN_COUNT){                
                //if counter equal to FIND_B_IN_COUNT, set time interval CCW2_B_INTERVAL, 
                //set net distance netDis to current sonar reading, go to next case
                    time_counter_start = loop_start_time +CCW2_B_INTERVAL;
                    RemoteCmd = POINT2B;            
                    netDis = Sdir;
                    p(DEBUG_STRING,"in FIND_B,go2POINT2B,CCW2_B_INTERVAL=%d\n",CCW2_B_INTERVAL);
              }else if(loop_start_time > time_counter_start){
                //time out, go to manual mode
                    movestop();
                    p(DEBUG_STRING,"timeout_in FIND_B 2000\n");
                    RemoteCmd = MANUAL;
               }else{
                    p(DEBUG_STRING,"FIND_BnoD;cnt=%d\n",sonar_reach_cnt);
                }
                  break;
                  
          case POINT2B: 
           if(netDis > Sdir) netDis = Sdir;
           if((loop_start_time >time_counter_start ) || (Sdir > netDis +30) ) {
               // if sonar reading is larger than the minimum net distance by 30cm OR
               // time interval CCW2_B_INTERVAL is over, stop robot, move robot forward, go to next case
              movestop();
              RemoteCmd = GO2B; 
              moveforward();  
              time_counter_start = loop_start_time;
              p(DEBUG_STRING,"P2B-2-GO2B,min Sonar d=%d\n",netDis);  
            }else{
              p(DEBUG_STRING,"POINT2B,son min=%d\n",netDis);              
            } 
            break;  
                 
          case GO2B:
              if(Sdir < DIST2B ){
                //if sonar reading is less than DIST2B, stop robot, rotate robot CW, go to next case
              }else{
                  p(DEBUG_STRING,"GO2B,stime=%lu,now=%lu\n",loop_start_time,time_counter_start);              
              }
              break;     
            
//  remaining tasks need build by yourself.
// don't forget to set RemoteCmd = MANUAL;  at the end of case .
                              
        }
      
      }
    }      
                            
   
    
  updateMotor(); // update motor value 
   if(!DEBUG_STRING){ //display message for debug
     a =M1.getSPEED(); if(M1sp !=a) {M1sp=a; b=1;}
     a =M2.getSPEED(); if(M2sp !=a) {M2sp=a; b=1;}
     a =M3.getSPEED(); if(M3sp !=a) {M3sp=a; b=1;}     
     if(b){
      //loop_start_time = millis();
      p(DEBUG_STRING,"%d,%d,%d,CMD=%d\n",M1sp, M2sp, M3sp,RemoteCmd);        
     } 
  }
  pSread = Sdir;
  slowDown(); 
}

/** simple sonar control, *****/
/**** read sonar if it is less than 30cm, stop robot and lanuch,    ****/
/***      else move forward     ***/
void sonar_control(){
 unsigned long duration;
 duration = read_sr04();
 p(DEBUG_STRING," %lu uS, %lu cm\n",duration, (duration/58));    
 duration = duration/58;
 if(duration <=30){
     launchSTATE =true;
     p(DEBUG_STRING,"sonar launch\n");    
     RemoteCmd= NO_OP;
 }else{ //move forward
   p(DEBUG_STRING,"move forward\n"); 
    M1.set(-220); 
    M2.set(0); 
    M3.set(154);         
 }   
}
/**** Bluetooth Communication *****/
/***** to receive and read every byte as a commad.     ****/
/*** Student DO NOT modify this function, otherwise it will not work. ****/
void BluetoothCom() {
  while(Serial.available()) {
    btChar = (char)Serial.read();
    switch (btChar) {
    case '=':
    case ':':
      btStatus =1;
      break;
    case ';':
      btStatus =0;
      sDeviceFlag = true; 
      return;
      break;
    default:
      switch (btStatus) {
      case 0:
        btDev +=btChar;
        break;
      case 1:
        btVal +=btChar;
        break;
      }
    }
  }
}

/** read sonar routine return duration in uS***/
unsigned long read_sr04(void){
  unsigned long data=0;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  data = pulseIn(echo, HIGH,MAX_SONAR_DIS_MEASURE);
  if(data ==0) return (MAX_SONAR_DIS_MEASURE);
    else return data;
}

/*** SetDevice()  ******
When user presses a key or move the joystick, robot needs to do something.
Student can modify part of this function.
when a key pressed (e.g. io0), bluetooth will receive a command  "io0=0;".
when a key released (e.g. io3), bluetooth will receive a command "io3=1;".
For example, if you want to do something when io9 is pressed,
simply find the else if statment of else if (btDev == "io9"){} then modify the codes inside the {}.  
***********/
void SetDevice() {
  unsigned long duration;
  //p(DEBUG_STRING,"SetDevice\n"); 
  pLastTick = millis(); // this is to record the last command received. to prevent timeout or lost communication.
  if( btDev == "x" && btSTATE ==0) {    
    btSTATE = 1;
    MotorX = btVal.toInt(); //get joystick x value
  }
  else if (btDev == "y" && btSTATE == 1) {
    btSTATE = 0;
    MotorY = btVal.toInt(); //get joystick y value
    if (sMotorStop == false) {
      if (RemoteCmd == NO_OP) {
        RemoteCmd= MANUAL;        
      }  
    }    
  }
  // this is the keypad part in remote control. 
  // IO PAD:
  // io4 | io3 | io2
  // ---------------
  // io0 | io9 | io8
  // ---------------
  // io7 | io6 | io5
  // Usage:
  //io0: tilting -5 degree
  //io2: toggle sonar control
  //io3: disable
  //io4: disable
  //io5: rotate robot clockwise NORMAL
  //io6: rotate robot anti-clockwise NORMAL
  //io7: tilting +5 degree
  //io8: start launch 
  //io9: read sonar and display value 
  else if (btDev == "pc") {
    sPCcontrol = !sPCcontrol;    
  }   
  else if (btDev == "io0") {
    if(btVal == "0"){
      tilt_angle += tilting_step;
      if(tilt_angle > TiltMAX) tilt_angle = TiltMAX;
      sv2.write(tilt_angle);
      p(DEBUG_STRING,"tilt_angle %d\n",tilt_angle);    
    }
  }   
  else if (btDev == "io2") {
     if(btVal == "0" ) {
      if(RemoteCmd != ZIGZAG_MOTION){
        Sdir = read_sr04()/58; //sonar measured distance in cm
        pSread = Sdir;
        RemoteCmd=ZIGZAG_MOTION;
        moveforwardleft();
        time_counter_start = loop_start_time + ZIGZAG_TIMELIMIT; //set time limit 
        p(DEBUG_STRING,"enter AUTO mode\n");
      }else{
        RemoteCmd= MANUAL;
        p(DEBUG_STRING,"change from Auto to MANUAL\n");
      }
    }
 }
  else if (btDev == "io3") {
   if(btVal == "0"){
   /*   M1stepSPD =M1stepSPD + 10;
      if(M1stepSPD >M1MAXSPEED) M1stepSPD = M1MAXSPEED; 
      M1.setProperties(M1stepSPD, (M1stepSPD*-1), stepS);
      M2stepSPD =M2stepSPD + 10;
      if(M2stepSPD >M2MAXSPEED) M2stepSPD = M2MAXSPEED; 
      M2.setProperties(M2stepSPD, (M2stepSPD*-1), stepS);
      M3stepSPD =M3stepSPD + 10;
      if(M3stepSPD >M3MAXSPEED) M3stepSPD = M3MAXSPEED; 
      M3.setProperties(M3stepSPD, (M3stepSPD*-1), stepS); */
    }     
    // p(DEBUG_STRING,"incease stepSPD to %d %d %d\n",M1stepSPD,M2stepSPD,M3stepSPD); 
     p(DEBUG_STRING,"disable io3\n");
  } 
   else if (btDev == "io4") {
    if(btVal == "0"){
   /*   M1stepSPD = M1stepSPD - 10;
      if(M1stepSPD < M1MINSPEED) M1stepSPD = M1MINSPEED;
      M1.setProperties(M1stepSPD, (M1stepSPD*-1), stepS);
      M2stepSPD = M2stepSPD - 10;
      if(M2stepSPD < M2MINSPEED) M2stepSPD = M2MINSPEED;
      M2.setProperties(M2stepSPD, (M2stepSPD*-1), stepS);
      M3stepSPD = M3stepSPD - 10;
      if(M3stepSPD < M3MINSPEED) M3stepSPD = M3MINSPEED;
      M3.setProperties(M3stepSPD, (M3stepSPD*-1), stepS); */
    }
    //p(DEBUG_STRING,"decease stepSPD to %d %d %d\n",M1stepSPD,M2stepSPD,M3stepSPD);  
      p(DEBUG_STRING,"disable io4\n");
  }   
  else if (btDev == "io6" && RobotRot !=2 ) {
    if (btVal == "1") {    
      RobotRot = 0;
    }
    if (btVal == "0") {
      RobotRot = 1;
      time_counter_start=loop_start_time+MANUAL_ROT_INTERVAL;
    }
  }
  else if (btDev == "io5" && RobotRot !=1 ) {
    if (btVal == "1") {
      RobotRot = 0;
    }
    if (btVal == "0") {
      RobotRot = 2;
      time_counter_start=loop_start_time+MANUAL_ROT_INTERVAL;
    }
  }  
  else if (btDev == "io7") {
     if(btVal=="0"){
      tilt_angle -= tilting_step;
      if(tilt_angle < TiltMIN) tilt_angle = TiltMIN;
      sv2.write(tilt_angle);
      p(DEBUG_STRING,"tilt_angle %d\n",tilt_angle);    
    }
 }
  else if (btDev == "io8") {
    if(btVal == "0" ) {
      launchSTATE = true; 
    }
  }
  else if (btDev == "io9") {
    if(btVal=="0"){ 
      p(DEBUG_STRING,"sonar start\t");
      duration = read_sr04();      
      p(DEBUG_STRING," %lu uS, %lu cm\n",duration, (duration/58));    
    }
  }
  btDev = "";
  btVal = "";
}

/**** StateMachine **************
With it, robot can move in 6 directions and two rotational motions via the remote control
for details please take a look of lecture notes.
usually speaking, student no need to modify the code.   
****/
void StateMachine(int X, int Y)
{
  X = X-128;  
  if(X==0)X=1;  // X cannot be 0.
  Y = Y-128;  
  if(Y==0)Y=1;
  switch (smstate) {
  case 0:  // stop 
    M1.set(0); 
    M2.set(0); 
    M3.set(0);
    if( M1.getSPEED()==0 && M2.getSPEED()==0 && M3.getSPEED()==0 ) {
      if(RobotRot == 1)smstate = 8;
      else if(RobotRot == 2) smstate = 7;        
      else if STATE1 smstate = 1;  //we use #define STATE1 in the top, i.e. better code management
      else if STATE2 smstate = 2;
      else if STATE3 smstate = 3;
      else if STATE4 smstate = 4;
      else if STATE5 smstate = 5;
      else if STATE6 smstate = 6;
    }
    break;
  case 1:  //forward
    //M1.set(-220); 
    //M2.set(0); 
    //M3.set(154);  
    moveforward();
    p(DEBUG_STRING,"moveforward()\n");    
    if STATE1 smstate = 1;
    else if STATE2 smstate = 2;
    else if STATE6 smstate = 6;
    else if STATE0 smstate = 0;
    break;
  case 2:  //forward right
    M1.set(-230); 
    M2.set(210); 
    M3.set(0);
    if STATE1 smstate = 1;
    else if STATE2 smstate = 2;
    else if STATE3 smstate = 3;
    else smstate = 0;
    break;
  case 3:  //backward right
    M1.set(0); 
    M2.set(210); 
    M3.set(-154);
    if STATE2 smstate = 2;
    else if STATE3 smstate = 3;
    else if STATE4 smstate = 4;
    else smstate = 0;
    break;
  case 4:  //backward
    M1.set(230); 
    M2.set(0); 
    M3.set(-154);
    if STATE3 smstate = 3;
    else if STATE4 smstate = 4;
    else if STATE5 smstate = 5;
    else smstate = 0;
    break;
  case 5:  //backward left
    //M1.set(230); 
    //M2.set(-210); 
    //M3.set(0);
    movebackwardleft();
    p(DEBUG_STRING,"movebackwardleft()\n");
    if STATE4 smstate = 4;
    else if STATE5 smstate = 5;
    else if STATE6 smstate = 6;
    else smstate = 0;
    break;
  case 6: //forward left
    //M1.set(0); 
    //M2.set(-210); 
    //M3.set(154);
    moveforwardleft();
    p(DEBUG_STRING,"moveforwardleft()\n");
    if STATE5 smstate = 5;
    else if STATE6 smstate = 6;
    else if STATE1 smstate = 1;
    else smstate = 0;
    break;
  case 7:    //rotate clockwise  
    //M1.set(-180); 
    //M2.set(-180); 
    //M3.set(-105);
    if(loop_start_time < time_counter_start){
      ROT(CW, NORMAL,NORMAL);      
      p(DEBUG_STRING,"R_CW,N,N\n");
    }else{
      ROT(CW,SLOW,FASTREACH);      
      p(DEBUG_STRING,"R_CW_N_F\n");
   }   
    if (RobotRot == 0) smstate = 0;
    break;
  case 8: //rotate counter clockwise
    //M1.set(180); 
    //M2.set(180); 
    //M3.set(105); 
    if(loop_start_time < time_counter_start){
      ROT(CCW, NORMAL,NORMAL);  
      p(DEBUG_STRING,"R_CCW,N,N\n");
    }else{
      ROT(CCW,SLOW,FASTREACH);      
      p(DEBUG_STRING,"R_CCW_N_F\n");
   }   
    if (RobotRot == 0) smstate = 0;
    break;
  default:
    smstate=0;
    break;
  }
  if(oldstate != smstate){
    p(DEBUG_STRING,"SM,X=%d,Y=%d,old=%d,go2STATE,%d\n",X,Y,oldstate,smstate);
    oldstate = smstate;
  }   

}

/*** updateMotor()
this function is to update Motor speed that are changed in program from remote, or sonar or others.
Student should not modify this function
***/
void updateMotor() {  
  if(sMotorStop) {  // the Motor Stop function got the highest priority, 
    M1.set(0);
    M2.set(0);
    M3.set(0);
  }
  M1.update();
  M2.update();
  M3.update(); 
}

// this function is to slowdown the main loop by delay.
// Student should not modify this function
void slowDown() {
  while((millis() - loop_start_time)< 20){
    // do notthing
  } 
  
}



