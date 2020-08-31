/****************************************************************************/
/*MADE FOR  : ABU ROBOCON FIJI TRY ROBOT CONTROL VIR PS4 DUALSHOCKS CONTROLLER     
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
#define _tryMM  8

//FRONT_MOTOR_RIGTH
#define _int_LF_A  22
#define _int_LF_B  23
//FRONT_MOTOR_LEFT
#define _int_RF_A  24
#define _int_RF_B  25
//BACK_MOTOR_RIGTH
#define _int_LB_A  26
#define _int_LB_B  27
//BACK_MOTOR_LEFT
#define _int_RB_A  28
#define _int_RB_B  29
//SHOOT_MOTOR
#define _try_MM_A 30
#define _try_MM_B 31


 
                /*   FNT_R     FNT_L     BCK_R      BCK_L*/

byte _dvPin_PWM[]={_pwmLF    ,   _pwmRF ,   _pwmLB ,   _pwmRB ,  _tryMM    };
byte _dvPin_INA[]={_int_LF_A ,_int_RF_A ,_int_LB_A ,_int_RB_A ,_try_MM_A   };
byte _dvPin_INB[]={_int_LF_B ,_int_RF_B ,_int_LB_B ,_int_RB_B ,_try_MM_B   };
USB Usb;
BTD Btd(&Usb); 
//PS4BT PS4(&Btd, PAIR);
PS4BT PS4(&Btd);
long double xL,yL,xR,yR,_x,_y,_angleM,_mag,_W1,_W2,_W3,_W4;
bool _report=true;
bool _break_type=true;
bool _break_status=true;
void setup() {
  
  for(int i=0;i<5;i++){
    pinMode(_dvPin_PWM[i],OUTPUT);
    pinMode(_dvPin_INA[i],OUTPUT);
    pinMode(_dvPin_INB[i],OUTPUT);
    digitalWrite(_dvPin_PWM[i],LOW);
    digitalWrite(_dvPin_INA[i],LOW);
    digitalWrite(_dvPin_INB[i],LOW);
  }

  
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
            if(PS4.getButtonPress(L1)||PS4.getButtonPress(R1)){
              _x = map(xR,0,255,100,-100);
              _y = map(yR,0,255,100,-100);
            }else{
              _x = map(xR,0,255,80,-80);
              _y = map(yR,0,255,80,-80);
            }
              _mag = sqrt(sq(_x)+sq(_y));
              _angleM = atan2(_y,_x); 
              _W1=_W2=_W3=_W4 = sin(_angleM -(PI/2))*_mag*-1;
              _W1=constrain(_W1, -249, 249);
              _W2=constrain(_W2, -249, 249);
              _W3=constrain(_W3, -249, 249);
              _W4=constrain(_W4, -249, 249);
              _break_status =false;
              
       }else if(((xL>137)||( xL<117))||((yL > 137) ||(yL < 117))){  
            
            if(PS4.getButtonPress(L1)||PS4.getButtonPress(R1)){
              _x = map(xL,255,0,250,-250);
              _y = map(yL,0,255,250,-250);
            }else{
              _x = map(xL,255,0,170,-170);
              _y = map(yL,0,255,170,-170);
            }
            
            _mag = sqrt(sq(_x)+sq(_y));
            _angleM = atan2(_y,_x); 
              
            _W1 = sin(_angleM +(PI/4))*_mag*-1;
            _W4 = sin(_angleM +(PI/4))*_mag;   
            _W3 = sin(_angleM -(PI/4))*_mag*-1;    
            _W2 = sin(_angleM -(PI/4))*_mag;
           
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
      if((_W1<10&&_W1>-10)&&(_W2<10&&_W2>-10)){
        break_free();
      }else{
        Motor(_W1,_W2,_W3,_W4);
      }report();
      ///feed_drive();

      if(PS4.getButtonPress(L2)||PS4.getButtonPress(R2)){
        _break_type=false;
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

      if(PS4.getButtonPress(SQUARE)){
        analogWrite(_dvPin_PWM[4],120);
        digitalWrite(_dvPin_INA[4],HIGH);
        digitalWrite(_dvPin_INB[4],LOW);
        Serial.println("  SQ"); 
      }else if(PS4.getButtonPress(CIRCLE)){
        analogWrite(_dvPin_PWM[4],120);
        digitalWrite(_dvPin_INA[4],LOW);
        digitalWrite(_dvPin_INB[4],HIGH);
         Serial.println("  CC"); 
      }else {
        analogWrite(_dvPin_PWM[4],0);
        digitalWrite(_dvPin_INA[4],HIGH);
        digitalWrite(_dvPin_INB[4],HIGH);
      }
      



      




      
      if(PS4.getButtonPress(L1)||PS4.getButtonPress(R1)){    
          if(PS4.getAngle(Pitch)>180){
    
              PS4.setLed(Red);
          }else {
              PS4.setLed(White);
          }
      }else{
        
         /*if(PS4.getAngle(Pitch)>180){
    
              PS4.setLed(0xEA005E);
          }else {
              PS4.setLed(0x00B7C3);
          }*/
          
          if(PS4.getAngle(Pitch)>180){
    
              PS4.setLed(0xCA00FF);
          }else {
              PS4.setLed(0xA4FF00);
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

bool report(void){\
    
   
    
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
      if(PS4.getButtonPress(L1)||PS4.getButtonPress(R1)){    
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
      Serial.println("Free braek");
    }else{
      for(int i= 0;i<4;i++){
        analogWrite(_dvPin_PWM[i],0);
        digitalWrite(_dvPin_INA[i],LOW);
        digitalWrite(_dvPin_INB[i],LOW);
      }
      Serial.println("Fast braek");
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
