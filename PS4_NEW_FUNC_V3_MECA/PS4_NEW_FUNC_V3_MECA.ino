/****************************************************************************/
/*MADE FOR  : ABU ROBOCON 2020 PRASING ROBOT CONTROL VIR PS4 DUALSHOCKS CONTROLLER   
/*DESIGN BY : PAEM_25(Pongsakorn <GOT> Klaypichai) 
/*DESIGN AT : Suratthani Technical College, Electronics Department , ET315   
/*VESRSION  : 1.0 BETA ALPHA AND OMEGA 
/****************************************************************************/
#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>
#include <math.h> 
#define _pwmLF  4 
#define _pwmRF  5 
#define _pwmLB  6 
#define _pwmRB  7 
#define _shtMT  8 
#define _armMT  9
//FRONT_MOTOR_RIGTH
#define _int_LF_A  26
#define _int_LF_B  27
//FRONT_MOTOR_LEFT
#define _int_RF_A  28
#define _int_RF_B  29
//BACK_MOTOR_RIGTH
#define _int_LB_A  30
#define _int_LB_B  31
//BACK_MOTOR_LEFT
#define _int_RB_A  32
#define _int_RB_B  33
//SHOOT_MOTOR
#define _int_SHT_A 34
#define _int_SHT_B 35
//ARM_MOTOR 
#define _int_ARM_A 36
#define _int_ARM_B 37

#define lim1 38
#define lim2 40
#define kick1 42

                /*   FNT_R     FNT_L     BCK_R      BCK_L*/

byte _dvPin_PWM[]={_pwmLF    ,   _pwmRF ,   _pwmLB ,   _pwmRB ,    _armMT ,    _shtMT };
byte _dvPin_INA[]={_int_LF_A ,_int_RF_A ,_int_LB_A ,_int_RB_A ,_int_ARM_A ,_int_SHT_A };
byte _dvPin_INB[]={_int_LF_B ,_int_RF_B ,_int_LB_B ,_int_RB_B ,_int_ARM_B ,_int_SHT_B };
USB Usb;
BTD Btd(&Usb); 
//PS4BT PS4(&Btd, PAIR);
PS4BT PS4(&Btd);
long double xL,yL,xR,yR,_x,_y,_angleM,_mag,_W1,_W2,_W3,_W4;
bool _report=true;
bool _break_type=true;
bool _break_status=true;
bool _feed_on = false;
bool _kick_on = false;
void setup() {
  for(int i=0;i<6;i++){
    pinMode(_dvPin_PWM[i],OUTPUT);
    pinMode(_dvPin_INA[i],OUTPUT);
    pinMode(_dvPin_INB[i],OUTPUT);
    digitalWrite(_dvPin_PWM[i],LOW);
    digitalWrite(_dvPin_INA[i],LOW);
    digitalWrite(_dvPin_INB[i],LOW);
  }
  pinMode(lim1,INPUT_PULLUP);
  pinMode(lim2,INPUT_PULLUP);
  pinMode(kick1,OUTPUT);
  digitalWrite(kick1,HIGH);
  Serial.begin(115200);
  while(Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}
bool computeAngle (void){
      Usb.Task();
      xL = PS4.getAnalogHat(LeftHatX);
      yL = PS4.getAnalogHat(LeftHatY);
      xR = PS4.getAnalogHat(RightHatX);
      yR = PS4.getAnalogHat(RightHatY);
       if(((xR>137)||( xR<117))||((yR > 137) ||(yR < 117))){
            if(PS4.getButtonPress(L1)){
              _x = map(xR,0,255,150,-150);
              _y = map(yR,0,255,150,-150);
            }else{
              _x = map(xR,0,255,100,-100);
              _y = map(yR,0,255,100,-100);
            }
            _mag = sqrt(sq(_x)+sq(_y));
            _angleM = atan2(_y,_x); 
            _W2= _W3 = sin(_angleM -(PI/2))*_mag;
            _W1= _W4 = _W2*-1;
            _break_status =false;
            _W1=constrain(_W1, -249, 249);
            _W2=constrain(_W2, -249, 249);
            _W3=constrain(_W3, -249, 249);
            _W4=constrain(_W4, -249, 249);
          
       }else if(((xL>137)||( xL<117))||((yL > 137) ||(yL < 117))){  
            
            if(PS4.getButtonPress(L1)){
              _x = map(xL,255,0,250,-250);
              _y = map(yL,0,255,250,-250);
            }else{
              _x = map(xL,255,0,230,-230);
              _y = map(yL,0,255,230,-230);
            }
            
            _mag = sqrt(sq(_x)+sq(_y));
            _angleM = atan2(_y,_x); 
            
            _W1 = sin(_angleM +(PI/4))*_mag*-1*1.414;
            _W4 = sin(_angleM +(PI/4))*_mag*1.414;  
            _W3 = sin(_angleM -(PI/4))*_mag*1.414;  
            _W2 = sin(_angleM -(PI/4))*_mag*-1*1.414;
           
            _break_status =false;
       }else {
          _W1=_W2=_W3=_W4=_mag=0;
       }
       _W1=constrain(_W1, -249, 249);
       _W2=constrain(_W2, -249, 249);
       _W3=constrain(_W3, -249, 249);
       _W4=constrain(_W4, -249, 249);
          
       Usb.Task();
       
}
void loop() {
    Usb.Task();
    if(PS4.connected()){
      computeAngle();
      if((_W1<0.75&&_W1>-0.75)&&(_W2<0.75&&_W2>-0.75)){
        break_free();
      }else{
        Motor(_W1,_W2,_W3,_W4);
      }report();
      feed_drv();
      arm_drv();
      kick_drv();
       if(PS4.getButtonPress(R1)){
        _break_type=false;
      }else{
        _break_type=true;
      }
      
      if(PS4.getButtonPress(UP)){
        analogWrite(_dvPin_PWM[0],150);
        digitalWrite(_dvPin_INA[0],HIGH);
        digitalWrite(_dvPin_INB[0],LOW);
      }
      if(PS4.getButtonPress(LEFT)){
        analogWrite(_dvPin_PWM[1],150);
        digitalWrite(_dvPin_INA[1],HIGH);
        digitalWrite(_dvPin_INB[1],LOW);
      }
      if(PS4.getButtonPress(DOWN)){
        analogWrite(_dvPin_PWM[2],150);
        digitalWrite(_dvPin_INA[2],HIGH);
        digitalWrite(_dvPin_INB[2],LOW);
      }
      if(PS4.getButtonPress(RIGHT)){
        analogWrite(_dvPin_PWM[3],150);
        digitalWrite(_dvPin_INA[3],HIGH);
        digitalWrite(_dvPin_INB[3],LOW);
      }


      
      if(!_feed_on&&!_kick_on){
        if(PS4.getButtonPress(L1)){    
            if(PS4.getAngle(Pitch)>180){
      
                PS4.setLed(Red);
            }else {
                PS4.setLed(White);
            }
        }else{
            if(PS4.getAngle(Pitch)>180){
      
                PS4.setLed(0x12FF24);
            }else {
                PS4.setLed(0xFFC107);
            }
        }
      }
      
      if(PS4.getButtonClick(PS)){
        PS4.disconnect();
      }
      if (PS4.getButtonClick(L3)) {
        Serial.print("BATT LEVEL : ");
        Serial.println(PS4.getBatteryLevel()); 
      }
      if(PS4.getButtonClick(OPTIONS)){
        
        _break_type =!_break_type;
      }
      Usb.Task();
    }else{
      Usb.Task();
      break_free();
//      Free();
    }    
}
void arm_drv(){
  
    if(PS4.getButtonClick(CROSS)){
      Serial.println("X");
    }else  if(PS4.getButtonClick(CIRCLE)){
      Serial.println("O");
    }
    if(PS4.getButtonPress(CIRCLE)&&(digitalRead(lim2)!=0)){
        analogWrite(_dvPin_PWM[4],80);
        digitalWrite(_dvPin_INA[4],HIGH);
        digitalWrite(_dvPin_INB[4],LOW);
        //PS4.setRumbleOn(127,0);
        
    }else if(PS4.getButtonPress(CROSS)&&(digitalRead(lim1)!=0)){
        analogWrite(_dvPin_PWM[4],80);
        digitalWrite(_dvPin_INA[4],LOW);
        digitalWrite(_dvPin_INB[4],HIGH);
        
        // PS4.setRumbleOn(0,127);
    }else{
        analogWrite(_dvPin_PWM[4],0);
        digitalWrite(_dvPin_INA[4],LOW);
        digitalWrite(_dvPin_INB[4],LOW);
        // PS4.setRumbleOff();
    }
  
}

void feed_drv(){
  if(PS4.getButtonClick(TRIANGLE)){
      _feed_on = !_feed_on;
      PS4.setLed(0xFF0000);
      Serial.print("FEED");
      Serial.println(_feed_on?" : ON":" : OFF");
  }
  
  if( _feed_on){
    analogWrite(_dvPin_PWM[5],120);
    digitalWrite(_dvPin_INA[5],HIGH);
    digitalWrite(_dvPin_INB[5],LOW);
    
  }else{
    analogWrite(_dvPin_PWM[5],0);
    digitalWrite(_dvPin_INA[5],0);
    digitalWrite(_dvPin_INB[5],0);
   
  }
 
  
}

void kick_drv(){
   if(PS4.getButtonClick(SQUARE)){
      _kick_on = !_kick_on;
      Serial.print("KICK");
      Serial.println(_kick_on?" : ON":" : OFF");
      PS4.setLed(0xF600F);
  }
  
  if( _kick_on){
    digitalWrite(kick1,LOW);
    
  }else{
    digitalWrite(kick1,HIGH);;
   
  }
}
bool report(void){
    
   
    
    if(_report){  
      Serial.print(225);
      Serial.print("\t   ");
      Serial.print(0);
      Serial.print("\t ");
      Serial.print(-225);
      Serial.print("\t  W1: ");
      Serial.print(double( _W1));
      Serial.print("  \t  W2: ");
      Serial.print(double( _W2));
      Serial.print("  \t  W3: ");
      Serial.print(double( _W3));
      Serial.print("  \t  W4: ");
      Serial.print(double( _W4));
      if(_W1>0&&_W2>0){
        Serial.print("  \t trun RIGHT");
        
      }else if(_W1<0&&_W2<0){
         Serial.print("  \t trun LEFT"); 
      }

      
      if(PS4.getButtonPress(L1)){    
        Serial.print("  \t  BOOST+++");
      }
   
      
      Serial.println("");
    }
     
    if((_W1==0)&&(_W2==0)){
        _report=false;
    }else{
        _report=true; 
    }
}
void break_free(void){
  
    _W1=_W2=_W3=_W4=_mag=0;
    if(_break_type){
      for(int i= 0;i<4;i++){
        analogWrite(_dvPin_PWM[i],0);
        digitalWrite(_dvPin_INA[i],HIGH);
        digitalWrite(_dvPin_INB[i],HIGH);
      }
      //Serial.println("Free braek");
    }else{
      for(int i= 0;i<4;i++){
        analogWrite(_dvPin_PWM[i],0);
        digitalWrite(_dvPin_INA[i],LOW);
        digitalWrite(_dvPin_INB[i],LOW);
      }
     // Serial.println("Fast braek");
    }
    _break_status =true;
  
}
void Motor (int _A,int _B,int _C,int _D){
  
  if(_A>0){
     digitalWrite(_dvPin_INA[0],HIGH);
     digitalWrite(_dvPin_INB[0],LOW);
     _A =  _A;
  }else{
     digitalWrite(_dvPin_INA[0],LOW);
     digitalWrite(_dvPin_INB[0],HIGH);
     _A =  _A*-1;
  }
  if(_B>0){
     digitalWrite(_dvPin_INA[1],HIGH);
     digitalWrite(_dvPin_INB[1],LOW);
      _B =  _B;
  }else{
     digitalWrite(_dvPin_INA[1],LOW);
     digitalWrite(_dvPin_INB[1],HIGH);
      _B =  _B*-1;
  }
  if(_C>0){
     digitalWrite(_dvPin_INA[2],HIGH);
     digitalWrite(_dvPin_INB[2],LOW);
      _C =  _C;
  }else{
     digitalWrite(_dvPin_INA[2],LOW);
     digitalWrite(_dvPin_INB[2],HIGH);
      _C =  _C*-1;
  }
  if(_D>0){
     digitalWrite(_dvPin_INA[3],HIGH);
     digitalWrite(_dvPin_INB[3],LOW);
      _D =  _D;
  }else{
     digitalWrite(_dvPin_INA[3],LOW);
     digitalWrite(_dvPin_INB[3],HIGH);
      _D =  _D*-1;
  }
  analogWrite(_pwmLF,_A);
  analogWrite(_pwmRF,_B);
  analogWrite(_pwmLB,_C);
  analogWrite(_pwmRB,_D);
  //_A=_B=_C=_D=0;
}
