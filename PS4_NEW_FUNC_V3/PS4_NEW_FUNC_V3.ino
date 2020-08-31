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
#define _int_SHT_A 34
#define _int_SHT_B 35
//ARM_MOTOR 
#define _int_ARM_A A6
#define _int_ARM_B A7

#define _lim_SW  A0
#define _inf_SS  A1
#define _opc_SS  A2 
                /*   FNT_R     FNT_L     BCK_R      BCK_L*/

byte _dvPin_PWM[]={_pwmLF    ,   _pwmRF ,   _pwmLB ,   _pwmRB ,     _armMT,    _shtMT };
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
void setup() {
  for(int i=0;i<6;i++){
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
              _x = map(xR,0,255,150,-150);
              _y = map(yR,0,255,150,-150);
            }else{
              _x = map(xR,0,255,120,-120);
              _y = map(yR,0,255,120,-120);
            }
            _mag = sqrt(sq(_x)+sq(_y));
            _angleM = atan2(_y,_x); 
            _W1=_W2=_W3=_W4 = sin(_angleM -(PI/2))*_mag*-1;
          
       }else if(((xL>137)||( xL<117))||((yL > 137) ||(yL < 117))){  
            if(PS4.getButtonPress(L1)||PS4.getButtonPress(R1)){
              _x = map(xL,0,255,230,-230);
              _y = map(yL,0,255,230,-230);
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
         
           // _W3 = sin(_angleM +(PI/4))*_mag*1*1.245;
            //_W4 = cos(_angleM +(PI/4))*_mag*1*1.245;
            _break_status =false;
       }else {
          _W1=_W2=_W3=_W4=_mag=0;
       }
       constrain(_W1, -250, 250);
       constrain(_W2, -250, 250);
       constrain(_W3, -250, 250);
       constrain(_W4, -250, 250);
       Usb.Task();
       
}
void loop() {
    Usb.Task();
    if(PS4.connected()){
      computeAngle();
      Motor(_W1,_W2,_W3,_W4);
      report();
      if((_W1==0)&&(_W2==0)){
        break_free();
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
      
      if(PS4.getButtonPress(L1)||PS4.getButtonPress(R1)){    
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
  if(!_break_status){ 
    if(_break_type){
      for(int i= 0;i<4;i++){
        analogWrite(_dvPin_PWM[i],0);
        digitalWrite(_dvPin_INA[i],LOW);
        digitalWrite(_dvPin_INB[i],LOW);
      }
      Serial.println("Fast braek");
    }else{
      for(int i= 0;i<4;i++){
        analogWrite(_dvPin_PWM[i],0);
        digitalWrite(_dvPin_INA[i],HIGH);
        digitalWrite(_dvPin_INB[i],HIGH);
      }
      Serial.println("Free braek");
    }
    _break_status =true;
  }
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
