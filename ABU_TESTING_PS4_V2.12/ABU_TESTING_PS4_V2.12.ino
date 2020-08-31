#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>

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
#define _int_SHT_A 30
#define _int_SHT_B 31
//ARM_MOTOR 
#define _int_ARM_A A6
#define _int_ARM_B A7

#define _lim1_SW  A0
#define _lim2_SW  A1
#define _rly  A2 
                /*   FNT_R     FNT_L     BCK_R      BCK_L*/
byte _dvPin_PWM[]={_pwmLF    ,   _pwmRF ,   _pwmLB ,   _pwmRB ,     _armMT,    _shtMT };
byte _dvPin_INA[]={_int_LF_A ,_int_RF_A ,_int_LB_A ,_int_RB_A ,_int_ARM_A ,_int_SHT_A };
byte _dvPin_INB[]={_int_LF_B ,_int_RF_B ,_int_LB_B ,_int_RB_B ,_int_ARM_B ,_int_SHT_B };
/*
_dvPin_PWM[]
_dvPin_INA[]
_dvPin_INA[]
*/
volatile bool _CW_st_A  = HIGH;
volatile bool _CW_st_B  = LOW;
volatile bool _CCW_st_A = LOW;
volatile bool _CCW_st_B = HIGH;
byte _setSPD =  0;
byte _levelSPD[] = {150,170,180,190};
byte _dftSPD =  165; 

byte _shootSPD = 230;
byte _minSPD=140; 
byte _maxSPD;
bool _feedEnable  =false;
bool _feedOn      =false;
bool _shootEnable =false;
bool _armEnable   =false;
bool _dirEnable   =false;
bool _stop=false;
uint16_t oldP;

long pre=0;
byte getAdc=0;
//BASIC FUNC//                                                
void _dr_F_R_CW (void);   /*drive front motor rigth cw*/ void _dr_F_R_CCW (void);   /*drive front motor rigth cw*/
void _dr_F_L_CW (void);   /*drive front motor left*/     void _dr_F_L_CCW (void);   /*drive front motor left*/
void _dr_B_R_CW (void);   /*drive back motor rigth*/     void _dr_B_R_CCW (void);   /*drive back motor rigth*/
void _dr_B_L_CW (void);   /*drive back motor left*/      void _dr_B_L_CCW (void);   /*drive back motor left*/
//ADV FUNC// 
void _stp(void);          /*FAST BREAK*/                 void _free(void);          /*SOFT BREAK*/   
void _fwd(void);          /*MOVE FORWRAD*/               void _bwd(void);           /*MOVE BACKWRAD*/
void _sldR(void);         /*SLIDE TO RIGHT*/             void _sldL(void);          /*SLIDE TO LEFT*/
void _rttR(void);         /*ROTATE CLOCKWISE(RIGTH)*/    void _rttL(void);          /*ROTATE REVESE-CLOCKWISE(LEFT)*/

float datax;
float datay;
float x;
float y;
  float AnalogDataX, AnalogDataY;
USB Usb;
BTD Btd(&Usb); 
//PS4BT PS4(&Btd, PAIR);
PS4BT PS4(&Btd);
void setup() {
  // put your setup code here, to run once:
  for(int i=0;i<6;i++){
    pinMode(_dvPin_PWM[i],OUTPUT);
    pinMode(_dvPin_INA[i],OUTPUT);
    pinMode(_dvPin_INB[i],OUTPUT);
    digitalWrite(_dvPin_PWM[i],LOW);
    digitalWrite(_dvPin_INA[i],LOW);
    digitalWrite(_dvPin_INB[i],LOW);
  }
  pinMode(A0,INPUT_PULLUP);
  pinMode(A1,INPUT_PULLUP);
  
  pinMode(_rly,OUTPUT);
  digitalWrite(_rly,HIGH);
  pinMode(40,OUTPUT);
  pinMode(41,OUTPUT);
  //delay(5000);
  
  //pinMode(3,OUTPUT); 
Serial.begin(115200);
  //#if !defined(__MIPSEL__)
    //while (!Serial); 
  //#endif
 while(Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    digitalWrite(40,HIGH);
   
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
  digitalWrite(40,LOW);
  digitalWrite(41,HIGH);
  _setSPD = _dftSPD;
  Serial.print("FEED ENABLE 1 : ");
  Serial.println(_feedEnable);
}
void Srl(int _d0,int _d1,int _d2,int _d3,byte _d4){
    Serial.print("A:  ");
    Serial.print(_d0);
    Serial.print("  B:  ");
    Serial.print(_d1);
    Serial.print("  C:  ");
    Serial.print(_d2);
    Serial.print("  D:  ");
    Serial.print(_d3);
    Serial.print("  ####  datax:  "+String(datax));
    Serial.print("  datay:  "+String(datay));
    Serial.println(" _(:3 」∠)_ "+String(_d4));
    Serial.println("");
}

void loop() {
  
  Usb.Task();//<--- to push report to ps4 controler
     
  if (PS4.connected()) { //<--- to check ps4 controler connection
          byte _rawSPD = PS4.getAnalogButton(L2);
          _maxSPD = map(_rawSPD,0,255,_minSPD,220); 
          /*if (PS4.getButtonPress(L1) || PS4.getButtonPress(R1)) {  
               revolveCtrl();
          }else{  
              // _dirEnable = directionCtrl();  
               //_setSPD = 0;
          }*/
     Usb.Task();
     if(!isMiddleR()){
         byte AnalogX =  PS4.getAnalogHat(RightHatX);
         byte AnalogY =  PS4.getAnalogHat(RightHatY);
         int anax,anay ;
         if(PS4.getButtonPress(L1) || PS4.getButtonPress(R1)){
          anax = map(AnalogX,0,255,200,-200);
          anay = map(AnalogY,255,0,200,-200);
          //Serial.println("BOOST");
          }else{
          anax = map(AnalogX,0,255,110,-110);
          anay = map(AnalogY,255,0,110,-110);
          }  
          double q = sqrt(sq(anax/10)+sq(anay/10));
             // p = constrain(p, 0, 110);
              Serial.print("  datax:  "+String(anax));
              Serial.print("  datay:  "+String(anay));
              Serial.print("  ###### ");
              Serial.println(q*10);
        /*  if(anax>0){
            anax=anax+30;
            Motor(-anax,-anax,-anax,-anax);
          }else{
            anax=anax-30;
            Motor(-anax,-anax,-anax,-anax);
          }*/
     }else if(!isMiddle()){    
        AnalogDataY =  PS4.getAnalogHat(LeftHatY);
        AnalogDataX =  PS4.getAnalogHat(LeftHatX);
        Usb.Task();
        if(PS4.getButtonPress(L1) || PS4.getButtonPress(R1)){
          datax = map(AnalogDataX,0,255,200,-200);
          datay = map(AnalogDataY,255,0,200,-200);
          //Serial.println("BOOST");
        }else{
          datax = map(AnalogDataX,0,255,110,-110);
          datay = map(AnalogDataY,255,0,110,-110);
        }  
        Usb.Task();
            double p = sqrt(sq(abs(datax)/10)+sq(abs(datay)/10));
            _stop =true;
             // p = constrain(p, 0, 110);
            /*  Serial.print("  datax:  "+String(datax));
              Serial.print("  datay:  "+String(datay));
              Serial.print("  ###### ");
              Serial.println(p*10);*/
           if(datay > datax && datay > 0 && datax > 0){
                Srl(-p*10,p*10,-p*10,p*10,1);
                Motor(-datay,datay-datax,datax-datay,datay);   
              }else if(datax > datay && datay > 0 && datax > 0){
                Srl(-p*10,-p*10,p*10,p*10,2);
                Motor(-datax,datay-datax,datax-datay,datax);
                
              }else if(datax > 0 && 0 > datay && datax > -datay){
                Srl(-p*10,-p*10,p*10,p*10,3);
                Motor(-datay-datax,-datax,datax,datay+datax);
                
              }else if(datax > 0 && 0 > datay && -datay > datax){
                 Srl(p*10,-p*10,p*10,-p*10,4);
                Motor(-datay-datax,datay,-datay,datay+datax);
              }else if( 0 > datax && datax > datay){    
                 Srl(p*10,-p*10,p*10,-p*10,5);    
                Motor(-datay,datay-datax,datax-datay,datay);  
              }else if(0 > datay && 0 > datax){   
                Srl(p*10,p*10,-p*10,-p*10,6);    
                Motor(-datax,datay-datax,datax-datay,datax);
              }else if(datay>0&&0 > datax && -datax>datay){
                Srl(p*10,p*10,-p*10,-p*10,7);  
                Motor(-datay-datax,-datax,datax,datay+datax);
               
              }else if(datay>0&& 0 > datax && datay>-datax){    
                 Srl(-p*10,p*10,-p*10,p*10,8);
                Motor(-datay-datax,datay,-datay,datay+datax);
              }
       }else if(_stop){
                Serial.print("  A: 0");
                Serial.print("  B: 0");
                Serial.print("  C: 0");
                Serial.println("  D: 0  ＼(≧▽≦)／");
                //Serial.print("   datax :"+String(datax));
                //Serial.println("  datay :"+String(datay));
                //Motor_Stop();
                _abs();
                _stop=false;
              }



  
          if(PS4.getButtonClick(TRIANGLE)){
              if(!_feedEnable){
                  _feedEnable = true;
                  
                  Serial.print("FEED ENABLE : ");
                  Serial.println(_feedEnable ? "ENABLE":"UNENABLE");
                  Serial.println("FEED STARTING...");
                  
              }else{   
                  _shootEnable  = true;
                  Serial.print("SHOOT ENABLE : ");
                  Serial.println(_shootEnable ? "ENABLE":"UNENABLE");
                  Serial.println("SHOOTING...");
                  
              }
          }

          bool stp1 = digitalRead(_lim1_SW);
          bool stp2 = digitalRead(_lim2_SW);
          /*if(stp1== 0 ||stp2==0){
            Serial.println("STOP");
          }*/
          if(PS4.getButtonPress(CIRCLE)||PS4.getButtonPress(SQUARE)){
              if(PS4.getButtonPress(CIRCLE)&&stp2){
                  Serial.println("UP ╰（‵□′）╯");
                  _armEnable = true;
                  _ARM_UP ();
              }else if(PS4.getButtonPress(SQUARE)&&stp1){
                  Serial.println("DOWN ( ⓛ ω ⓛ *)");
                  _armEnable = true;
                  _ARM_DOWN ();
              }else if(_armEnable){
                 Serial.println("OFF (¬_¬ )");
                 _armEnable = false;
                 _ARM_OFF ();
              }
          }else if(_armEnable){
             Serial.println("OFF  (¬_¬ )");
             _armEnable = false;
             _ARM_OFF ();
          }
          
          if (PS4.getButtonClick(CROSS)){
            if(_feedEnable){
                _feedEnable   = false;
                _shootEnable  = false;
                _feedOn       = false;
                _FEED_OFF();
                Serial.print("FEED ENABLE : ");
                Serial.println(_feedEnable ? "ENABLE":"UNENABLE");
                Serial.println("FEED OFF...");                
                Serial.println("EVERTHING WAS CANCLE...");                
            }
          }
          Usb.Task();//<--- to push report to ps4 controler
          if(_feedEnable&&_feedOn!=true){
              _FEED_ON();
              dly(2500);
              Serial.println("FEED READY...");
              PS4.setLed(Green); 
              _feedOn = true;
          }
          
          
          if(_feedEnable&&_shootEnable&&_feedOn){
             Serial.println("FEED BANG!!!!!");
             digitalWrite(_rly,LOW);
              dly(3000);
             digitalWrite(_rly,HIGH); 
              _feedEnable   = false;
              _shootEnable  = false;
              _feedOn       = false;
              Serial.println("FEED OFF...");
              _FEED_OFF();
          }


          digitalWrite(_rly,HIGH); 
          STATUS();      
  }else{
      _abs();
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
  _A=_B=_C=_D=0;
}
void dly(long _i){
    long _t = millis(); 
    while(millis()-_t<_i){
              ;         
    }
}
void STATUS (void){
    if (PS4.getButtonClick(SHARE)) {
        Serial.println("CHANG DIRECTION");
    }
    if (PS4.getButtonClick(PS)) {
        Serial.println("DISCONECTED FROM PS4 CONTROLLER ");
        Serial.println("(╮°-°)╮┳━━┳   (╯°□°)╯︵ ┻━┻");
        PS4.disconnect();  
    }
    if (PS4.getButtonClick(L3)) {
        Serial.print("BATT LEVEL : ");
        Serial.println(PS4.getBatteryLevel()); 
    }
    if(!_feedEnable){
      if(PS4.getAngle(Pitch)>180){

          PS4.setLed(0x12FF24);
      }else {
          PS4.setLed(0xFFC107);
      }
      
    }
}
bool revolveCtrl(void){
    if(!_dirEnable){
       _setSPD = 80;
    }
    if (PS4.getButtonPress(R1)){    
        Serial.println("ROTATE CLOCLWISE :"+String(_setSPD));  
        _rttR();
        return true;
      }
    if (PS4.getButtonPress(L1)){ 
        Serial.println("ROTATE COUNTERCLOCLWISE :"+String(_setSPD)); 
        _rttL();
        return true;
    }
     //_setSPD = 0;
}

bool directionCtrl(void){

  
      if(PS4.getAnalogHat(LeftHatY) <95&&(PS4.getAnalogHat(LeftHatX)>105&&PS4.getAnalogHat(LeftHatX)<150)){
          _setSPD = map(PS4.getAnalogHat(LeftHatY),127,0,0,_maxSPD);
          _fwd();  
          Serial.println("MOVE FORWARD : "+String(_setSPD));
          return true;
      }else  if(PS4.getAnalogHat(LeftHatY)>160&&(PS4.getAnalogHat(LeftHatX)>105&&PS4.getAnalogHat(LeftHatX)<150)){
          _setSPD= map(PS4.getAnalogHat(LeftHatY),127,255,0,_maxSPD);           
          _bwd();
          Serial.println("MOVE BACKWARD : "+String(_setSPD));  
          return true;
      }else if(PS4.getAnalogHat(LeftHatX) <95&&(PS4.getAnalogHat(LeftHatY)>105&&PS4.getAnalogHat(LeftHatY)<150)){
          _setSPD = map(PS4.getAnalogHat(LeftHatX),127,0,0,_maxSPD);           
          _sldL();
          Serial.println("SLIDE TO LEFT :"+String(_setSPD));
          return true;
      }else if(PS4.getAnalogHat(LeftHatX)>160&&(PS4.getAnalogHat(LeftHatY)>105&&PS4.getAnalogHat(LeftHatY)<150)){
          _setSPD = map(PS4.getAnalogHat(LeftHatX),127,255,0,_maxSPD);         
          _sldR();
          Serial.println("SLIDE TO RIGTH :"+String(_setSPD));
          return true;
      }else 
      
      
      
      
      
      
      if(PS4.getAnalogHat(LeftHatY) <95&&PS4.getAnalogHat(LeftHatY) >15&&PS4.getAnalogHat(LeftHatX)<95&&PS4.getAnalogHat(LeftHatX) >15){
           _setSPD = map(PS4.getAnalogHat(LeftHatY),127,0,0,_maxSPD);
          Serial.println("FRONT TO LEFT : "+String(_setSPD));
          return true;
      }else if(PS4.getAnalogHat(LeftHatY) <95&&PS4.getAnalogHat(LeftHatY) >15&&PS4.getAnalogHat(LeftHatX)>160&&PS4.getAnalogHat(LeftHatX)<240){
          _setSPD  = map(PS4.getAnalogHat(LeftHatY),127,0,0,_maxSPD);
          Serial.println("FRONT TO RIGHT : "+String(_setSPD));
          return true;
      }else if(PS4.getAnalogHat(LeftHatY) >160&&PS4.getAnalogHat(LeftHatY) <240&&PS4.getAnalogHat(LeftHatX)>160&&PS4.getAnalogHat(LeftHatX)<240){
          _setSPD  = map(PS4.getAnalogHat(LeftHatY),127,255,0,_maxSPD);
          Serial.println("BACK TO RIGHT : "+String(_setSPD));
          return true;
      }else if(PS4.getAnalogHat(LeftHatY) >160&&PS4.getAnalogHat(LeftHatY) <240&&PS4.getAnalogHat(LeftHatX)<95&&PS4.getAnalogHat(LeftHatX)>15){
          _setSPD = map(PS4.getAnalogHat(LeftHatY),127,255,0,_maxSPD);
          Serial.println("BACK TO LEFT : "+String(_setSPD));
          return true;
      }else {
          _free();
          return false;
      }
      
}

bool ADCGET(void){
   if (PS4.getAnalogButton(L2)<15 && PS4.getAnalogButton(R2)<15) {
      return true;
   }else{
      return false;
   }
}
bool isMiddle(void){
  if((PS4.getAnalogHat(LeftHatX) < 137 && PS4.getAnalogHat(LeftHatX) > 117)&&(PS4.getAnalogHat(LeftHatY) < 137 && PS4.getAnalogHat(LeftHatY) > 117)){  
    return true;
  }else {
    return false;
  }
}
bool isMiddleR(void){
  if((PS4.getAnalogHat(RightHatX) < 137 && PS4.getAnalogHat(RightHatX) > 117)&&(PS4.getAnalogHat(RightHatY) < 137 && PS4.getAnalogHat(RightHatY) > 117)){  
    return true;
  }else {
    return false;
  }
}

void _fwd(void){
  _dr_F_L_CW();   _dr_F_R_CCW();
  _dr_B_L_CW();   _dr_B_R_CCW(); 
} 
void _bwd(void){
  _dr_F_L_CCW();  _dr_F_R_CW();
  _dr_B_L_CCW();  _dr_B_R_CW();
  
}
void _sldR(void){ 
  _dr_F_L_CW();     _dr_F_R_CCW(); 
  _dr_B_L_CCW();    _dr_B_R_CW();  
}
void _sldL(void){ 
  _dr_F_L_CCW();    _dr_F_R_CW();
  _dr_B_L_CW();     _dr_B_R_CCW();
}
void _rttR(void){
  _dr_F_L_CW();    _dr_F_R_CCW();
  _dr_B_L_CW();    _dr_B_R_CCW();
}
void _rttL(void){
  _dr_F_L_CCW();   _dr_F_R_CW();
  _dr_B_L_CCW();   _dr_B_R_CW();
}

//BASIC FUNC///*  FR 0 FL 1 BR 2 BL 3 */
void _dr_F_R_CW (void){   
	_cwDrive(0,_setSPD); 
}

void _dr_F_L_CW (void){   
	_cwDrive(1,_setSPD);
}

void _dr_B_R_CW (void){   
	_cwDrive(2,_setSPD);
}

void _dr_B_L_CW (void){   
	_cwDrive(3,_setSPD);
}

//BASIC FUNC///*  FR 0 FL 1 BR 2 BL 3 */
void _dr_F_R_CCW (void){  
	_ccwDrive(0,_setSPD);
}

void _dr_F_L_CCW (void){   
	_ccwDrive(1,_setSPD);
}

void _dr_B_R_CCW (void){   
	_ccwDrive(2,_setSPD);
}

void _dr_B_L_CCW (void){   
	_ccwDrive(3,_setSPD);
}
void _cwDrive(byte _id,byte _spd) {
  analogWrite(_dvPin_PWM[_id],_spd);
  digitalWrite(_dvPin_INA[_id],_CW_st_A);
  digitalWrite(_dvPin_INB[_id],_CW_st_B);
}
void _ccwDrive(byte _id,byte _spd) {
  analogWrite(_dvPin_PWM[_id],_spd);
  digitalWrite(_dvPin_INA[_id],_CCW_st_A);
  digitalWrite(_dvPin_INB[_id],_CCW_st_B);
}


void _ARM_UP (void){
  analogWrite(_dvPin_PWM[4],80);
  digitalWrite(_dvPin_INA[4],_CW_st_A);
  digitalWrite(_dvPin_INB[4],_CW_st_B);
}

void _ARM_SHOOT (void){
  analogWrite(_dvPin_PWM[4],140);
  digitalWrite(_dvPin_INA[4],_CW_st_A);
  digitalWrite(_dvPin_INB[4],_CW_st_B);
  
}
void _ARM_DOWN (void){
   analogWrite(_dvPin_PWM[4],20);
  digitalWrite(_dvPin_INA[4],_CCW_st_A);
  digitalWrite(_dvPin_INB[4],_CCW_st_B);
}
void _ARM_OFF (void){
   analogWrite(_dvPin_PWM[4],0);
  digitalWrite(_dvPin_INA[4],LOW);
  digitalWrite(_dvPin_INB[4],LOW);
}


void _FEED_ON (void){
  analogWrite(_dvPin_PWM[5],_shootSPD );
  digitalWrite(_dvPin_INA[5],_CW_st_A);
  digitalWrite(_dvPin_INB[5],_CW_st_B);
}
void _FEED_CNT (void){
   analogWrite(_dvPin_PWM[5],_shootSPD );
  digitalWrite(_dvPin_INA[5],_CCW_st_A);
  digitalWrite(_dvPin_INB[5],_CCW_st_B);
}

void _FEED_OFF (void){
  analogWrite(_dvPin_PWM[5],0);
  digitalWrite(_dvPin_INA[5],HIGH);
  digitalWrite(_dvPin_INB[5],HIGH);
}


//BASIC FUNC//
void _free(void){
  for(int i=0;i<4;i++){
    analogWrite(_dvPin_PWM[i],0);
    digitalWrite(_dvPin_INA[i],HIGH);
    digitalWrite(_dvPin_INB[i],HIGH);
  }
  
}
void _abs(void){
 for(int i=0;i<4;i++){
    analogWrite(_dvPin_PWM[i],0);
    digitalWrite(_dvPin_INA[i],LOW);
    digitalWrite(_dvPin_INB[i],LOW);
 }
}
