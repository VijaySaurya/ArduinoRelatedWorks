#include<LiquidCrystal.h>
#include<EEPROM.h>
#include<SimpleModbusSlave.h>
LiquidCrystal lcd(3,2,4,5,6,7);
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 rtc;
#define RWP_MPV_PIN 21
#define HPP_MPV_PIN 18
#define RWP_CURRENT_PIN 25
#define HWP_CURRENT_PIN 26
#define TWT_FLOATY_HL 28
#define TWT_FLOATY_LL 15
#define RWT_FLOATY_PIN 29
#define LPS_SWITCH_PIN 30
#define HPS_SWITCH_PIN 31
#define RWP_RELAY_PIN  22
#define HPP_RELAY_PIN  23
#define FLUSH_RELAY_PIN 12
#define REJECT_FLOW_PIN 20
#define PERMEATE_FLOW_PIN 19
#define SWITCH_ADC_PIN 27
#define BACKWASH_TRIGGER_PIN 13
//#define POWER_PIN 24
#define ENABLE_PIN 14
#define TDS_PERMEATE_PIN 24
#define m "hello"

enum
{  
  OPM_U,  //0.operational minutes upper 2 byte
  OPM_L,  //1.operational minutes lower 2 byte
  TDS1,   //2.permeate tds
  TDS2,   //3.inlet tds
  RWPC,   //4.raw water pump current
  HPPC,   //5.high pressure pump current
  PFR,    //6.permate flow rate
  RFR,    //7.reject flow rate
  TWV_U,  //8.treated water volume upper 2 byte
  TWV_L,  //9.treated water volume lower 2 byte
  RWV_U,  //10.reject water volume upper 2 byte
  RWV_L,  //11.reject water volume lower 2 byte  
  BCC,    //12.backwash cycle count
  TLB,    //13.tank level
  UVST,   //14.UV state
  TEMP,   //15.Temperature of water
  TSC,    //15.trips state counter
  //configuration parameters
  SFLG,   //16.set flag
  RFLG,   //17.reset flag
  SRN,    //18.machine serial no
  CC1,    //18.current constant1
  CC2,    //19.current constnat2
  RWPLC,  //20.raw water pump lower current
  RWPUC,  //21.raw water pump upper current
  HPPLC,  //22.high pressure pump lower current
  HPPUC,  //23.high pressure pump upper current
  BTT,    //24.backwash triger time
  FC1,    //25.flow constant1
  FC2,    //26.flow constant2
  RWPWT,  //27.raw water pump wait timeout
  LPSWT,  //28.lower pressure switch wait timeout
  FINT,   //29.flushing time
  CST,    //30.current sampling time
  FST,    //31.flow sampling time
  MS,     //32.machine state 
  TBCK,   //33.trigger backwash
  TIME_STAMP_U,//34.timestamp upper
  TIME_STAMP_L,//35.timestamp lower  
  HOLDING_REG_SIZE
  };
unsigned int holdingRegs[HOLDING_REG_SIZE]; 

bool mainsState = LOW;
byte machineStatus;
bool twtFloaty_HL = 0,twtFloaty_LL = 0,rwtFloaty = 0,lpsSwitch = 0,hpsSwitch = 0;
byte machinetripState,mpvtripState,lastmpvtripState,lastmachinetripState,lastlpsSwitch,lastState_fValve;
unsigned long currentMillis, lastRunMillis, machineCheckMillis,flushdisplayMillis,lpswaitdisplayMillis,displayMeasureMillis,dataMeasureMillis,savedataMillis,settingCheckMillis,powerCheckMillis;
unsigned long updateModbusDataMillis;
bool CLOSED,DONE,YES,ON,OF,NOTDONE,OPEN;
bool IsMPV;
byte flushtime_Count=0;
byte displayCount =0;

char daysOfTheWeek[7][12] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
bool rtc_status = HIGH;

float  Voltage_P = 0;
bool   INPUT_POWER = LOW;
bool   LAST_INPUT_POWER = LOW;
/***************************** RO configuration parameters ******************************************/

bool             MACHINE_STATE;  
bool             RESET_REQUEST;
bool             remoteBackwashTrigger;

byte             CURRENT_CONSTANT_1;
byte             CURRENT_CONSTANT_2;
byte             RWP_UPPER_CURRENT;
byte             RWP_LOWER_CURRENT;
byte             HPP_LOWER_CURRENT;
byte             HPP_UPPER_CURRENT;
byte             TRIP_STATE_COUNTER;

unsigned int     MACHINE_SR_NO;
unsigned int     FLOW_CONSTANT_1;
unsigned int     FLOW_CONSTANT_2;
unsigned int     RWP_WAIT_TIME;
unsigned int     LPS_WAIT_TIME;
unsigned int     FLUSH_TIME_INTERVAL;
unsigned int     CURRENT_SAMPLING_TIME;
unsigned int     FLOW_SAMPLING_TIME;
unsigned int     TIMEOUT_CONSTANT;
unsigned int     BACKWASH_TRIGGER_TIME;



/**************************** RO measured parameters ***************************************************/

bool             par_changed=LOW;
byte             SLAVE_ID;
byte             TANK_LVL;
byte             TANK_WATER_TEMP;
byte             TEMP_OFFSET;

float            RWP_CURRENT;               //RO runtime value
float            HPP_CURRENT;               //RO runtime value
String           Date_time="";

double           Temperature_thermocouple;
double           Temperature_junction;

unsigned int     TDS_PERMEATE;              //dynamic value appear only when the purification is under running condition
unsigned int     TDS_INLET;                 //dynamic value 
unsigned int     BACKWASH_CYCLE_COUNT;      //to be saved into eeprom
unsigned int     PERMEATE_FLOW_RATE;        //RO runtime value
unsigned int     REJECT_FLOW_RATE;          //RO runtime value
unsigned int     BAUD_RATE;

unsigned long    OPERATION_MINUTES;         //Operation minute store into eeprom
unsigned long    LAST_BACKWASH_TIME;        //in seconds in unix time stamp to be saved into eeprom
unsigned long    TOTAL_TREATED_VOLUME;      //total treated volume saved into eeprom
unsigned long    TOTAL_REJECT_VOLUME;       //total reject volume saved into eeprom
unsigned long    CURRENT_TIME_STAMP;
/**************************** RO Calibration Constant *********************************************/

float            SWITCH_CONFIGURATION[5]={2.48,3.43,4.12,2.02};
float            SWITCH_OFFSET=0.05;
unsigned int     RO_LOOP_RATE;
unsigned int     RO_MEASUREMENT_UPDATE;
unsigned int     MACHINE_CHECK_RATE;
unsigned int     RO_UPDATE_DISPLAY = 2000;



/**************************************************************************************************/
class settingParameter 
   {  
    //-------------------------------------configuration parameter------------------------------------------------------------------//

    byte tempOffset;
    byte  currentConstant1,currentConstant2,rwpLowerCurrent,rwpUpperCurrent,hppLowerCurrent,hppUpperCurrent;
    unsigned int flowConstant1,flowConstant2;  
    unsigned int rwpwaitTimeout,lpswaitTimeout,flushingInterval,currentSamplingTime,flowSamplingTime,backwashTriggerTime;    
    
    //-------------------------------------Remote Control Parameter-----------------------------------------------------------------//
    bool machineState;
    unsigned int serialNo;    
    //-------------------------------------Log Parameter----------------------------------------------------------------------------//
   
    unsigned int backWashCycleCount,lastBackWashTime,backWashTriggerTime;
    unsigned long treatedWaterVolume,rejectWaterVolume;
    unsigned long operational_minute;
     
    //-------------------------------------modbus config parameter------------------------------------------------------------------//

    byte slaveId;
    unsigned int baudRate;
       
    
     public:
        settingParameter()
        {                            
             //-------Initializie Ro parameters-----------//
              currentConstant1     =  100;
              currentConstant2     =  100;                      
              rwpLowerCurrent      =  0;
              rwpUpperCurrent      =  10;
              hppLowerCurrent      =  0;
              hppUpperCurrent      =  10;        
              backwashTriggerTime  =  1440;                      
              flowConstant1        =  59;
              flowConstant2        =  59;         
              rwpwaitTimeout       =  10000;
              lpswaitTimeout       =  10000;
              flushingInterval     =  10000;
              currentSamplingTime  =  1000;
              flowSamplingTime     =  1000;                     
             //---Initialize Remote Control Parameters------//                      
              machineState         =  false; 
              serialNo             =  12345;
             //--------Initialize logdata parameters---------//         
              treatedWaterVolume   =  0;
              rejectWaterVolume    =  0;                     
              operational_minute   =  0;
              backWashCycleCount   =  0;
              lastBackWashTime     =  0; //this time is in minute since last  
              baudRate             =  4800;
              slaveId              =  1;
              tempOffset           =  0;
         } 
         void getData();
         void setData();
};
void settingParameter::getData()
{
        MACHINE_STATE             =   machineState;          //machine  lock or unlocked
        MACHINE_SR_NO             =   serialNo;
        
        CURRENT_CONSTANT_1        =   currentConstant1;
        CURRENT_CONSTANT_2        =   currentConstant2;                  
        RWP_LOWER_CURRENT         =   rwpLowerCurrent;
        RWP_UPPER_CURRENT         =   rwpUpperCurrent;  
        HPP_LOWER_CURRENT         =   hppLowerCurrent;
        HPP_UPPER_CURRENT         =   hppUpperCurrent;               
        BACKWASH_TRIGGER_TIME     =   backwashTriggerTime;
        FLOW_CONSTANT_1           =   flowConstant1;
        FLOW_CONSTANT_2           =   flowConstant2;                 
        RWP_WAIT_TIME             =   rwpwaitTimeout;
        LPS_WAIT_TIME             =   lpswaitTimeout;
        FLUSH_TIME_INTERVAL       =   flushingInterval;
        CURRENT_SAMPLING_TIME     =   currentSamplingTime;
        FLOW_SAMPLING_TIME        =   flowSamplingTime;               
     
        TOTAL_TREATED_VOLUME      =   treatedWaterVolume;
        TOTAL_REJECT_VOLUME       =   rejectWaterVolume;               
        OPERATION_MINUTES         =   operational_minute;
        BACKWASH_CYCLE_COUNT      =   backWashCycleCount;  
        LAST_BACKWASH_TIME        =   lastBackWashTime ; 
        BAUD_RATE                 =   baudRate;
        SLAVE_ID                  =   slaveId;  
        TEMP_OFFSET               =   tempOffset;          
} 
                 
void settingParameter::setData()
{                 
        machineState              =   MACHINE_STATE;
        serialNo                  =   MACHINE_SR_NO;             
        currentConstant1          =   CURRENT_CONSTANT_1;
        currentConstant2          =   CURRENT_CONSTANT_2;         
        rwpLowerCurrent           =   RWP_LOWER_CURRENT;
        rwpUpperCurrent           =   RWP_UPPER_CURRENT;
        hppLowerCurrent           =   HPP_LOWER_CURRENT; 
        hppUpperCurrent           =   HPP_UPPER_CURRENT;      
        backwashTriggerTime       =   BACKWASH_TRIGGER_TIME; 
        flowConstant1             =   FLOW_CONSTANT_1;
        flowConstant2             =   FLOW_CONSTANT_2;        
        rwpwaitTimeout            =   RWP_WAIT_TIME;
        lpswaitTimeout            =   LPS_WAIT_TIME;
        flushingInterval          =   FLUSH_TIME_INTERVAL;
        currentSamplingTime       =   CURRENT_SAMPLING_TIME;
        flowSamplingTime          =   FLOW_SAMPLING_TIME;         
        treatedWaterVolume        =   TOTAL_TREATED_VOLUME;
        rejectWaterVolume         =   TOTAL_REJECT_VOLUME;         
        operational_minute        =   OPERATION_MINUTES;
        backWashCycleCount        =   BACKWASH_CYCLE_COUNT;
        lastBackWashTime          =   LAST_BACKWASH_TIME;
        baudRate                  =   BAUD_RATE;
        slaveId                   =   SLAVE_ID;
        tempOffset                =   TEMP_OFFSET;
}  

/************************************************* << SWITCH BUTTON CLASS >> *************************************************************/

class switchButton
{
    byte pin;
    int analogReadValue;
    byte lastButtonState;
    unsigned long lastDebounceTime,debounceDelay;
    public:
      bool s1,s2,s3;
      float voltage;
      switchButton(byte analogPin)
      {
          pin = analogPin;
          pinMode(pin,INPUT);
          s1=s2=s3=LOW;
          voltage=0;
          analogReadValue=0;
          lastButtonState=0;
          debounceDelay=0;
          lastDebounceTime=0;
      }
      void setState(bool a,bool b,bool c);
      void switchDetect(float *,float);
};

void switchButton::setState(bool a,bool b,bool c)
{
    s1=a;
    s2=b;
    s3=c;
}
void switchButton::switchDetect(float *a,float offset)
{
    voltage=0;
    analogReadValue=analogRead(pin);
    voltage=(analogReadValue*5.0)/1023.0;
  //  Serial.print("voltage -> ");
  //  Serial.println(voltage);
    byte buttonState=4;
    for(int i=0;i<4;i++)
    {
        if(voltage<(a[i]+offset) && voltage>(a[i]-offset))
        {
            buttonState=i;
            break;
        }
        else 
        {
            buttonState=4;
        }
    }
    if(buttonState!=lastButtonState)
    {
        lastDebounceTime=currentMillis;
       /* Serial.print("last debounce time -> ");
        Serial.println(lastDebounceTime);
        Serial.print("current millis -> ");
        Serial.println(currentMillis);*/
    }
    if(currentMillis-lastDebounceTime>=debounceDelay)
    {
        //Serial.print("button state -> ");
        //Serial.println();
        switch(buttonState)
        {
            case 0:
                setState(1,0,0);
                break;

            case 1:
                setState(0,1,0);
                break;

            case 2:
                setState(0,0,1);
                break;

            case 3:
                setState(1,1,0);
                break;

            default:
                setState(0,0,0);
                break;
        }
    }
    lastButtonState=buttonState;
}

/*********************************************************** << END OF SWITCH BUTTON CLASS >> *************************************************************/

/********************************************************* << LCD READ CLASS >> ***************************************************************************/

class LcdRead:public switchButton
{
    byte init_curPos,curPos,final_curPos;
    char arr1[16];
    public:
        bool shiftButton,selectButton,enterButton;
        bool status_of_setting; 
        LcdRead(byte analogPin):switchButton(analogPin)
        {
            pinMode(analogPin,INPUT);
            status_of_setting = LOW;
            init_curPos=0;
            curPos=0;
            final_curPos=0;
            shiftButton=selectButton=enterButton=LOW;
            for(int i=0;i<16;i++)
            {
                arr1[i]='0';
            }
        }
        unsigned int readValue(char variable[16],unsigned int num);
        byte readValue(char variable[16],byte num);
        byte calculateDigits(unsigned int num,char *arr);
        void setDigit(byte digit,byte len,char *arr);
        void setDisplay(String parameter, byte a, byte b, char *arr);
        byte shiftCursor(byte curPos,byte init_curPos,byte final_curPos);
        void incrCursor(byte curPos,char *arr);
        unsigned int charToDec(char arr[16],byte a,byte b);
        void button();
        
};
unsigned int LcdRead::readValue(char parameter[16],unsigned int num)
{
    bool lastShiftButton=HIGH,lastSelectButton=HIGH,lastEnterButton=HIGH;
    unsigned long timeoutMillis=0;
    init_curPos=5;
    final_curPos=10;  
    unsigned int number = num;    
    byte digits = calculateDigits(number, arr1);  
    setDigit(5, digits, arr1);  
    setDisplay(parameter, init_curPos, final_curPos, arr1);  
   
    timeoutMillis=currentMillis;  
   while (1)
    {    
        button(); 
        if (shiftButton == HIGH && lastShiftButton==LOW) //This block of code will shift the cursor position
        {     
 
              curPos = shiftCursor(curPos,init_curPos, final_curPos);
              timeoutMillis=currentMillis;      
        }
        if (selectButton == HIGH && lastSelectButton==LOW) //This block of code will increment the value at cursor position
        {           
              //Serial.println("Increment Cursor"); 
              incrCursor(curPos, arr1)  ;  
              timeoutMillis=currentMillis;      
        }      
        if(enterButton==HIGH && lastEnterButton==LOW)
        {
              num = charToDec(arr1,5,10);
              status_of_setting=HIGH; 
              lcd.noCursor();
              return num; 
              break;
        }
        else if (currentMillis-timeoutMillis>=20000)
        {
               lcd.clear();
               lcd.setCursor(0, 0);
               lcd.print(F("TimeOut Error!"));                 
               status_of_setting = LOW;
               return num;
        }      
        lastShiftButton=shiftButton;
        lastSelectButton=selectButton; 
        lastEnterButton=enterButton;  
    }
}

byte LcdRead::readValue(char parameter[16],byte num)
{
  bool lastShiftButton=HIGH,lastSelectButton=HIGH,lastEnterButton=HIGH;
  unsigned long timeoutMillis=0;
  init_curPos=6;
  final_curPos=9;
  byte number = num;
  byte digits= calculateDigits(number,arr1);
  setDigit(3,digits,arr1);
  setDisplay(parameter, init_curPos, final_curPos, arr1);
  switchDetect(SWITCH_CONFIGURATION,SWITCH_OFFSET);
  timeoutMillis=currentMillis;  
  while(1)
  {
      button();
      if(shiftButton==HIGH && lastShiftButton==LOW)
      {
          curPos = shiftCursor(curPos,init_curPos,final_curPos);
          timeoutMillis=currentMillis;
      }
      if(selectButton==HIGH && lastSelectButton==LOW)
      {
          incrCursor(curPos,arr1);
          timeoutMillis=currentMillis;
      }
      if(enterButton==HIGH && lastEnterButton==LOW)
      {
          num = charToDec(arr1,6,9);
          status_of_setting=HIGH;
          lcd.noCursor();
          return num;
          break;
      }
      else if (currentMillis-timeoutMillis>=20000)
      {
               lcd.clear();
               lcd.setCursor(0, 0);
               lcd.print(F("TimeOut Error!"));                 
               status_of_setting = LOW;
               return num;
      }  
      lastShiftButton=shiftButton;
      lastSelectButton=selectButton; 
      lastEnterButton=enterButton;
  }
}

byte LcdRead::calculateDigits(unsigned int number, char *arr)
{
    byte l = 0;
    while (number != 0)
    {
        arr[l] = number % 10 + '0';
        number = number / 10;
        l++;
    }
    return l;
}

void LcdRead::setDigit(byte digit,byte len,char *arr)
{
    byte l=len;
    char a[16];
    for (byte i = 0; i < 16; i++)
    a[i] = arr[i];
    for (byte i = 0; i < digit; i++)
    {
        if (i < (digit - len))
        {
            arr[i] = '0';
        }
        else
        {
            arr[i] = a[l - 1];
            l--;
        }
    }
}

void LcdRead::setDisplay(String parameter,byte x, byte y, char *arr)
{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(parameter);
    init_curPos=x;
    final_curPos=y;
    char buff[16];
    for(byte i=0;i<16;i++)
    {
        buff[i]=arr[i];
        arr[i]='0';
    }
    byte  i=0;
    for(byte j=init_curPos;j<final_curPos;j++)
    {
        arr[j]=buff[i];
        i++;
    }
    for(byte j=init_curPos;j<final_curPos;j++)
    {
        lcd.setCursor(j,1);
        lcd.print(arr[j]);
    }
    lcd.setCursor(init_curPos,1);
    curPos=init_curPos;
    lcd.cursor();
}

byte LcdRead::shiftCursor(byte curPos,byte init_curPos,byte final_curPos)
{
    curPos=curPos+1;
    if(curPos==final_curPos)
    {
        curPos=init_curPos;
    }
    lcd.setCursor(curPos,1);
    return curPos;
}

void LcdRead::incrCursor(byte curPos,char *arr)
{
    arr[curPos]=arr[curPos]+1;
    if(arr[curPos]>'9')
    {
        arr[curPos]='0';
    }
    lcd.setCursor(curPos,1);
    lcd.print(arr[curPos]);
    lcd.setCursor(curPos,1);
}

unsigned int LcdRead::charToDec(char *arr,byte init_curPos,byte final_curPos)
{
    byte b;
    unsigned long c =0;
    for(byte i=init_curPos;i<final_curPos;i++)
    {
        b=arr[i]-48;
        c=c*10+b;
    }
    return c;
}

void LcdRead::button()
{
    switchDetect(SWITCH_CONFIGURATION,SWITCH_OFFSET);
    
    shiftButton   =  s1;
   // Serial.print("s1 = ");
   // Serial.println(s1);
    selectButton  =  s2;
    //Serial.print("s2 = ");
    //Serial.println(s2);
    enterButton   =  s3;
  //  Serial.print("s3 = ");
   // Serial.println(s3);
}


/***************************************************** << END OF LCD READ CLASS >> ********************************************************************/

/*********************************************************** FLOW RATE MEASUREMENT CLASS ***************************************************/
class Flow
{
    bool flowPulse,lastflowPulse,fallingEdge;
    unsigned long lastflowMillis,samplingInterval,countPulse;
    byte flowPin;
    unsigned int flowsamplingTime;
    float flowConstant;
   public:
     double flowRate,flowVolume;
     Flow(byte digitalPin)
     {
          flowPin = digitalPin;
          pinMode(flowPin,INPUT_PULLUP);
     }
     void initalizeValue(unsigned int a, unsigned int b);
     void measureFlow();
  
};
void Flow::initalizeValue(unsigned int a, unsigned int b)
{
    flowsamplingTime = a;
    flowConstant = b;
    flowPulse=lastflowPulse=fallingEdge=0;
    flowRate = 0;
}
void Flow::measureFlow()
{
    if(currentMillis - lastflowMillis >= FLOW_SAMPLING_TIME)
    {
        samplingInterval = currentMillis - lastflowMillis;
        flowRate = (((1000/samplingInterval)*countPulse)/flowConstant);
        countPulse = 0;
        lastflowMillis = currentMillis;
    }
    else
    {
      flowPulse = digitalRead(flowPin);
      //Serial.print(flowPulse);
      if(flowPulse==LOW && lastflowPulse == HIGH)
      {
        fallingEdge = 1;
      }
      else if(fallingEdge == 1)
      {
        if(flowPulse == HIGH && lastflowPulse == LOW)
        {
          countPulse+=1;
          fallingEdge = 0;
        }
      }
      lastflowPulse = flowPulse;
    }
}

/******************************************************** END OF FLOW RATE CLASS **************************************************************************/

/**************************************************************** CURRENT MEASURE CLASS ******************************************************************/

class current 
{
    byte currentPin;
    float vpp,vrms;
    int   readValue,minValue,maxValue;
    unsigned int  currentConstant;
    unsigned int  samplingInterval;
    unsigned long lastCurrentMillis;
    public:
      float irms;
      current(byte pin)
      {
          currentPin = pin;
          pinMode(currentPin,INPUT);
      }
      void initalizeValue(unsigned int a,unsigned int b);
      void currentMeasure();
};
void current::initalizeValue(unsigned int a, unsigned int b)
{
    samplingInterval  = a;
    currentConstant   = b;
    readValue         = 0;
    minValue          = 1023;
    maxValue          = 0;
    lastCurrentMillis = 0;
    vpp               = 0;
    vrms              = 0;
    irms              = 0;
}
void current::currentMeasure()
{
    readValue = analogRead(currentPin);
    if(readValue > maxValue)
    {
        maxValue = readValue;
    }
    if(readValue < minValue)
    {
        minValue = readValue;
    }
    if(currentMillis - lastCurrentMillis > samplingInterval)
    {
        lastCurrentMillis = currentMillis;
        vpp  = ((maxValue - minValue) * 5.0) / 1023.0;
        vrms = (vpp / 2.0) * 0.707;
        irms = (vrms * 1000) / currentConstant;
        maxValue =0;
        minValue =1023;
    }
}

/********************************************************************** END OF CURRENT MEASURE CLASS *****************************************************/

/******************************************************************** BACKWASH PROCESS CLASS *************************************************************/

class backWash
{
    bool trigger;    // Flag for check backwash start or not 
    byte triggerPin;
    unsigned long backwashMillis,triggerTime;
    public:
        bool triggerBackwash;  // Flag for start backwash
        byte backwashSecondCount,backwashMinuteCount,backwashHourCount;
        unsigned int backwashDayCount;
        unsigned long totalRunTime,lastBackwashCall;
        unsigned int backwashCycleCount;
        unsigned int backwashTriggerTime;
        backWash(byte pin)
        {
          triggerPin = pin;
          pinMode(triggerPin,OUTPUT);
          digitalWrite(triggerPin,LOW);
          triggerBackwash = trigger = false;
          backwashMillis = triggerTime = 0;
          backwashSecondCount = backwashMinuteCount = backwashHourCount = 0;
          backwashDayCount = 0;
          totalRunTime = lastBackwashCall = 0;
          backwashCycleCount = 0;
          backwashTriggerTime = 0;
        }
        void checkBackwash(unsigned int triggerTime);                 // Backwash check function
};
void backWash::checkBackwash(unsigned int backwashTriggerTime)
{
    if((currentMillis - backwashMillis) >= 5000)
    {
        backwashMillis = currentMillis;
        backwashSecondCount+=5;              //backwash time count from mahine run(in seconds)
        if(backwashSecondCount/60>=1)
        {
            backwashSecondCount = 0;
            backwashMinuteCount+=1;          //backwash time count from mahine run(in minutes)
            if(backwashMinuteCount/60>=1)
            {
                backwashMinuteCount = 0;
                backwashHourCount+=1;        //backwash time count from mahine run(in hours)
                if(backwashHourCount/24>=1)
                {
                    backwashDayCount+=1;      //backwash day count from mahine run
                }
            }
        }
        totalRunTime = backwashDayCount*1440+backwashHourCount*60+backwashMinuteCount;   // Total machine run time in minute since machine start
        //Serial.print("<<<<< TOTAL MACHINE RUN TIME : ");
        //Serial.println(totalRunTime);
        if((totalRunTime - lastBackwashCall) >= backwashTriggerTime)
        {
          backwashMillis = currentMillis;
          lastBackwashCall = totalRunTime;
          triggerBackwash = true;
        }
    }
    if(triggerBackwash == true)
    {
        if(trigger!=true)
        {
            digitalWrite(triggerPin,HIGH);
            trigger = true;
            triggerTime = currentMillis;
        }
        else if(currentMillis-triggerTime >= 2000)
        {
            digitalWrite(triggerPin,LOW);
            triggerBackwash = false;
            trigger = false;
            backwashCycleCount+=1;
            par_changed = HIGH;
            //Serial.print("Machine Run Time : ");
            //Serial.print(totalRunTime);
            //Serial.println(" Minute");
            //Serial.print("NUMBER OF BACKWASH : ");
            //Serial.println(backwashCycleCount);
          
            
        }
    }
}

/************************************************************** END OF BACKWASH PROCESS CLASS *************************************************************/


/*********************************************************************** MOTOR RUN CONTROL CLASS **********************************************************/

class motorRun
{
      bool on, off, closed, opens, Isfirstcall, rwp, hpp, done, undone, lpsState,yes,no,flushComplete;
      byte rwpPin, hppPin,fValvePin;    
      unsigned long tryMillis,flushMillis;
    public:
      bool motorRun_status, machineRunning;
      bool rwpState, hppState,fValveState,lpswaitTimeout;   
      motorRun(byte a, byte b,byte c)
      {
          rwpPin    = a;
          hppPin    = b;
          fValvePin = c;
          pinMode(rwpPin, OUTPUT);
          digitalWrite(rwpPin,LOW);
          pinMode(hppPin, OUTPUT);
          digitalWrite(hppPin,LOW);
          pinMode(fValvePin,OUTPUT);
          digitalWrite(fValvePin,LOW);
          on = opens = done =yes= HIGH;
          off = closed = undone =no= LOW;
          rwpState = hppState = rwp = hpp = lpsState = fValveState= off;
          Isfirstcall = 0;
          lpswaitTimeout = 0;
          //LPS_WAIT_TIMEOUT = 20000;//this time can be chnaged from controller setting
          //FLUSH_TIME_INTERVAL = 10000;
          machineRunning = on;
          tryMillis = 0;
      }
      void startRWP() 
      {
          digitalWrite(rwpPin, on);
          //Serial.println("START RWP");
          rwpState = on;
      }
      void stopRWP() 
      {
          digitalWrite(rwpPin, off);
          //Serial.println("STOP RWP");
          rwpState = off;
      }
      void startHPP() 
      {
          digitalWrite(hppPin, on);
          //Serial.println("START HPP");
          hppState = on;
      }
      void stopHPP() 
      {
          digitalWrite(hppPin, off);
          //Serial.println("STOP HPP");
          hppState = off;
      }
      void startfValve() 
      {
          digitalWrite(fValvePin,on);
          //Serial.println("START FLUSH VALVE");
          fValveState=on;
      }
      void stopfValve() 
      {
          digitalWrite(fValvePin,off);
          //Serial.println("STOP FLUSH VALVE");
          fValveState=off;
      }
      void Run(bool a, bool b, bool c);
      bool shutDown()
      {
          stopRWP();
          stopHPP();
          stopfValve();
      }
};

void motorRun::Run(bool a,bool b,bool c)
{
    rwp      = a;
    hpp      = b;
    lpsState = c;
    if(rwp==HIGH && hpp==LOW && fValveState==HIGH)
    {
        stopfValve();
    }
    if(rwpState == off && rwp == on)
    {
        startRWP();
    }
    if(rwpState == on && rwp == off)
    {
        stopRWP();
        stopfValve();
        lpswaitTimeout=LOW;
    }
    if(hppState==off && hpp == on)
    {
        if(flushComplete != yes)
        {
            if(fValveState == off)
            {
                startfValve();
                flushMillis = currentMillis;
            }
            else if(currentMillis - flushMillis >= FLUSH_TIME_INTERVAL)
            {
                stopfValve();
                flushComplete = yes;
            }
            motorRun_status = undone;
        }
        else if(lpsState == closed)
        {
            startHPP();
        }
        else if (Isfirstcall == 0)
        {
            tryMillis = currentMillis;
            Isfirstcall = 1;
            motorRun_status = undone;
        }
        else if(currentMillis - tryMillis > RWP_WAIT_TIME)
        {
            stopRWP();
            Isfirstcall = 0;
            lpswaitTimeout = 1;
            motorRun_status = done;
        }
    }

    if(hppState == on && hpp == off)
    {
        if(lpsState == opens)
        {
            stopHPP();
            stopfValve();
            lpswaitTimeout =0;
            Isfirstcall =0;
        }
        else if(Isfirstcall == 0)
        {
            tryMillis = currentMillis;
            Isfirstcall = 1;
            motorRun_status = undone;
        }
        else if(currentMillis - tryMillis > LPS_WAIT_TIME)
        {
            stopHPP();
            stopfValve();
            Isfirstcall = 0;
            motorRun_status = done;
        }
    }
    
    //If the call for state change is completed motorRun_status will set
    if (rwpState == rwp && hppState == hpp)
    {
        motorRun_status = done;
        flushComplete=no;        
    }      
      
    //if the rwp and hpp is running then the machineRunning variable will set indicating the machine is running
    if (rwpState == on && hppState == on)
      machineRunning = yes;
    else
      machineRunning = no;
 
}

/********************************************************************  END OF MOTOR RUN CONTROL CLASS **********************************************************/

//............. OBJECT DECLERATION.......................................//
settingParameter roParameters;
current rwpCurrent(RWP_CURRENT_PIN);
current hppCurrent(HWP_CURRENT_PIN);
Flow permeateFlow(PERMEATE_FLOW_PIN);
Flow rejectFlow(REJECT_FLOW_PIN);
LcdRead setting(SWITCH_ADC_PIN);
backWash mpv(BACKWASH_TRIGGER_PIN);
motorRun rocontrolPump(RWP_RELAY_PIN, HPP_RELAY_PIN, FLUSH_RELAY_PIN);
//......................................................................//

void setup()
{
  OCR0A = 0xAF;   //Compare match register value
  Serial.begin(9600);
  lcd.begin(16,2);
  machineInformation();  // Display Machine Information 
  delay(2000);
 
  /*................ EEPROM ASSIGN FOR RO PARAMETER......................*/
  if(0)  // For EEROM ASSIGN if(1) 
  {
    byte checkByte=0;
    EEPROM.write(0,checkByte);
  }
  checkEEPROM();
  /*.....................................................................*/

    if (! rtc.begin()) 
  {
   lcd.print("Couldn't find RTC");
    while (1);
  }

  if (! rtc.isrunning()) 
  {
   lcd.print("RTC is NOT running!");
  }
  lcd.clear();
  rtcTime();
  delay(2000);
  RO_LOOP_RATE = 1000;
  CLOSED=OF=NOTDONE=LOW;
  DONE=YES=ON=OPEN=HIGH;
  IsMPV = YES;
  pinMode(RWP_MPV_PIN, INPUT);
  pinMode(HPP_MPV_PIN, INPUT);
  pinMode(TWT_FLOATY_HL, INPUT);
  pinMode(TWT_FLOATY_LL, INPUT);
  pinMode(RWT_FLOATY_PIN, INPUT);
  pinMode(LPS_SWITCH_PIN, INPUT);
  pinMode(HPS_SWITCH_PIN,INPUT);
  pinMode(TDS_PERMEATE,INPUT);
  rwpCurrent.initalizeValue(CURRENT_SAMPLING_TIME , CURRENT_CONSTANT_1);
  hppCurrent.initalizeValue(CURRENT_SAMPLING_TIME , CURRENT_CONSTANT_2);
  permeateFlow.initalizeValue(FLOW_SAMPLING_TIME,FLOW_CONSTANT_1);
  rejectFlow.initalizeValue(FLOW_SAMPLING_TIME,FLOW_CONSTANT_2);

  /*...........................modbus configuration..................................................................*/  
  modbus_configure(&Serial, BAUD_RATE, SERIAL_8N2, SLAVE_ID,ENABLE_PIN, HOLDING_REG_SIZE, holdingRegs);
  modbus_update_comms(BAUD_RATE, SERIAL_8N2, SLAVE_ID);
  updateRODataOnModbus();   
  updateConfigDataOnModbus(); 
  /*.................................................................................................................*/
  
 // TIMSK0 |= (1<<OCIE0A);
  TIMSK0 |= _BV(OCIE0A);
}
/****************************************************************** MAIN RO LOOP HERE ********************************************************************************/

SIGNAL(TIMER0_COMPA_vect)
{
  currentMillis = millis();
/*  powerCheckMillis = currentMillis;
  if(currentMillis-powerCheckMillis>=10)
  {
    Voltage_P = analogRead(POWER_PIN)*5.0/1024;
    powerCheckMillis = currentMillis;
  }
  if(Voltage_P<=4.6)
  {
    INPUT_POWER = LOW;
  }
  else
  {
    INPUT_POWER = HIGH;
  }
  if(INPUT_POWER == LOW && LAST_INPUT_POWER == HIGH)
  {
    writeParameter();
    //Serial.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  }
  LAST_INPUT_POWER = INPUT_POWER; */
}
void loop()
{
  
    if(currentMillis - settingCheckMillis >= 1000)
    {
          settingCheckMillis = currentMillis;
          controllerSetting();

          if(currentMillis-updateModbusDataMillis>=10000)
          {
                updateModbusDataMillis=currentMillis;
                rtcTime();
                updateRODataOnModbus();      
                if(holdingRegs[RFLG]==HIGH)
                {      
                    //if the reset flag of holding register is set then update the configuration data in the device with modbus with data available on modbus registers
                    updateConfigDataFromModbus();      
                }
          }         
          modbus_update();  
    }
    if(rocontrolPump.machineRunning == HIGH)
    {
      rwpCurrent.currentMeasure();
      hppCurrent.currentMeasure();
      permeateFlow.measureFlow();
      rejectFlow.measureFlow();
      mpv.checkBackwash(BACKWASH_TRIGGER_TIME);
      //Serial.println("FLOW MEASURE");
      if(currentMillis-dataMeasureMillis>=2000)
      {
        
            dataMeasureMillis=currentMillis;
            OPERATION_MINUTES= mpv.totalRunTime;
            PERMEATE_FLOW_RATE=permeateFlow.flowRate;
            REJECT_FLOW_RATE=rejectFlow.flowRate;
            TOTAL_TREATED_VOLUME+=(PERMEATE_FLOW_RATE*1000.0);     //unit of total treated volume  in ml
            TOTAL_REJECT_VOLUME+=(REJECT_FLOW_RATE*1000.0);        //unit of total reject volume in ml
            RWP_CURRENT=rwpCurrent.irms;
            HPP_CURRENT=hppCurrent.irms;
            TDS_PERMEATE= measureTDS(TDS_PERMEATE_PIN);
            //Serial.println("FLOW MEASURE DATA UPDATE");
            
      } 
    }
    else 
    {   
        RWP_CURRENT=0;
        HPP_CURRENT=0;     
        PERMEATE_FLOW_RATE=0;
        REJECT_FLOW_RATE=0;
        TDS_PERMEATE=0;         
    }   
    if(currentMillis - lastRunMillis >= RO_LOOP_RATE) 
    {
        lastRunMillis = currentMillis;  
        machinetripState = machineTrip(CLOSED,CLOSED,CLOSED,CLOSED);    
        mpvtripState = mpvState(); 
       //Serial.print("Machine Trip State: ");
       //Serial.println(machinetripState);
       // Serial.print("MPV Trip State: ");
       // Serial.println( mpvtripState);
        if (machinetripState != lastmachinetripState || mpvtripState != lastmpvtripState)    //If there is change in trip state or mpv state
        {        
                   
            rocontrolPump.motorRun_status = NOTDONE;
            if(mpvtripState!=1)
            {
                if(machinetripState!= 2)
                {
                      switch(mpvtripState)
                      {
                          case 0:
                            statusDisplay(10);                             //mpv is in backwash mode
                            break;
                            
                          case 2:
                            statusDisplay(12);                             //Mot On delay timer is ON
                            break;
                            
                          case 3:
                            statusDisplay(13);                             //MPV connection not OK
                            break;
                            
                          default:
                            break;
                      } 
                } 
                else
                {
                      statusDisplay(machinetripState);                     //mpv is in backwash mode
                }        
            }
            else
            {
                  statusDisplay(machinetripState);
                  //Serial.println(F("state change"));
                  flushtime_Count=0;       
            }

        }
        if(rocontrolPump.motorRun_status != DONE)
        {        
            if(IsMPV == YES) //check the presence of mpv
            { 
                if(mpvtripState!=1)
                {
                    if(machinetripState!= 2)
                    {
                        switch (mpvtripState)
                        {
                            case 0:                      
                              rocontrolPump.Run(ON, OF, lpsSwitch);              //  0:BACKWASH MODE                      
                             // Serial.println("BACKWASH");
                              break;
                              
                            case 1:                      
                              rocontrolPump.Run(ON, ON, lpsSwitch);              //  1:FILTER MODE
                             // Serial.println("FILTER");
                              break;
                            
                            case 2:                      
                              rocontrolPump.Run(OF, OF, lpsSwitch);              //  2:MOT ON DELAY MODE
                             // Serial.println("MOTOR ON DELAY");
                              break;
                            
                            case 3:                      
                              rocontrolPump.Run(OF, OF, lpsSwitch);              //  3:NOT AVAILABLE
                             // Serial.println("NOT AVAILABLE");
                              break;
                        }
                    }
                    else
                    {
                        
                       // Serial.println("BACKWASH..................................................");
                        rocontrolPump.Run(OF, OF, lpsSwitch);     // 0:BACKWASH
                    }
                }
                else if(machinetripState==0)//if there is no trip state
                {                    
                        rocontrolPump.Run(ON, ON, lpsSwitch);
                       // Serial.println("FILTER");
                }
                else
                {                       
                        rocontrolPump.Run(OF, OF, lpsSwitch);
                       // Serial.println("DEPEND ON LPS");
                }
            }
            else
            {
                  if(machinetripState==0)
                        rocontrolPump.Run(ON,ON,lpsSwitch);
                  else
                        rocontrolPump.Run(OF,OF,lpsSwitch);
            }
            machineCheckMillis=currentMillis;      //reset the health check timer to monitor the state after the set state is done
      }

      else if(machinetripState ==0 && mpvtripState == 1)
      {
          MACHINE_CHECK_RATE = 5000;
          if(currentMillis-machineCheckMillis>= MACHINE_CHECK_RATE)
          {
              machineCheckMillis = currentMillis;
              if(rocontrolPump.rwpState==ON)
              {
                  if(rwpCurrent.irms<RWP_LOWER_CURRENT)
                  {
                      machineStatus = 5;
                  }
                  else if(rwpCurrent.irms>=RWP_UPPER_CURRENT)
                  {
                      machineStatus =  6;
                  }
              }
              if(rocontrolPump.hppState == ON)
              {
                  if(HPP_CURRENT<HPP_LOWER_CURRENT)
                  {
                        machineStatus =  7;
                  }
                  else if(HPP_CURRENT>=HPP_UPPER_CURRENT)
                  {
                        machineStatus =  8;
                  }
              }
              if(rocontrolPump.rwpState==ON && lpsSwitch == OPEN)
              {
                    rocontrolPump.Run(ON,ON,lpsSwitch);
                    rocontrolPump.motorRun_status = NOTDONE;
              }
          }
        if(rocontrolPump.machineRunning == ON)
        {
            // when high pressure pump and low pressure pump both in on condition and lps changed their state closed to open then off high pressure pump and wait for lps closed 
            if(lpsSwitch==OPEN && lastlpsSwitch == CLOSED)
            {
                rocontrolPump.Run(ON,OF,lpsSwitch);
                rocontrolPump.motorRun_status = NOTDONE;
            }
        }
     }
     lastmachinetripState    =  machinetripState;
     lastmpvtripState        =  mpvtripState;
     lastlpsSwitch           =  lpsSwitch;
  }


    if(rocontrolPump.machineRunning == ON && mpvtripState == 1)
    {
        if(currentMillis - displayMeasureMillis >= RO_UPDATE_DISPLAY)
        {
            displayMeasureMillis = currentMillis;
            float OPH;
            switch(displayCount)
            {
                case 0:
                    OPH = float(OPERATION_MINUTES)/60;
                    //Serial.print("<<<<<<<<<<<<<<<<<<<<<<<< OPERATIONAL MINUTE>>>>>>>>>>>>>>>>>>>>>>>>>");
                    //Serial.println(OPH);
                    displayMeasurement("OPH",OPH,"Hr");
                    break;
              
                case 1:
                    displayMeasurement("RWPC",RWP_CURRENT,"A");
                    break;

                case 2:
                    displayMeasurement("HPPC",HPP_CURRENT,"A");
                    break;

                case 3:
                    displayMeasurement("PFR",PERMEATE_FLOW_RATE,"LPH");
                    break;

                case 4:
                    displayMeasurement("RFR",REJECT_FLOW_RATE,"LPH");
                    break;

                case 5:
                    displayMeasurement("TTV",TOTAL_TREATED_VOLUME/1000,"L");
                    break;

                case 6:
                    displayMeasurement("TRV",TOTAL_REJECT_VOLUME/1000,"L");
                    break;

                case 7:
                    displayMeasurement("TDS",TDS_PERMEATE,"PPM");
                    break;
        }
        displayCount+=1;
        if(displayCount == 8)
        {
            displayCount = 0;
        }
     }
   }
   else
   {
      if(rocontrolPump.rwpState==ON && machinetripState == 0 && mpvtripState==1)
      {
          if(rocontrolPump.fValveState==ON)
          {
              if(currentMillis-flushdisplayMillis>=1000)
              {
                     flushdisplayMillis=currentMillis;          
                     lcd.clear();
                     lcd.setCursor(0,0);
                     lcd.print(F("Flushing-ON.."));
                     lcd.setCursor(7,1);
                     lcd.print(flushtime_Count);
                     lcd.print(F("s"));  
                     flushtime_Count+=1;
              }              
          }
        else if(rocontrolPump.fValveState==OF && lastState_fValve==ON)
        {
              if(flushtime_Count!=0)
              {
                  flushtime_Count=0;
              }
              if(currentMillis-lpswaitdisplayMillis>=1000)
              {
                      lpswaitdisplayMillis=currentMillis;          
                      if(lpsSwitch!=CLOSED)
                      {
                         statusDisplay(11);
                      }
              }
        }
       lastState_fValve=rocontrolPump.fValveState; 
      }
    }
}

/************************************************************** END OF MAIN LOOP ********************************************************************************/

/************************************************************ FLOATY,LPS,HPS MEASURE FUNCTION *******************************************************************/
void tripswitchDetect()
{   
    if(digitalRead(TWT_FLOATY_HL)==HIGH)
    {
        twtFloaty_HL = HIGH;
    }
    else
    { 
        twtFloaty_HL = LOW; 
    }
    if(digitalRead(TWT_FLOATY_LL)==HIGH)
    {
        twtFloaty_LL = HIGH;
    }
    else
    {
        twtFloaty_LL = LOW;  
    }
    if(digitalRead(RWT_FLOATY_PIN)==HIGH)
    {
        rwtFloaty = HIGH;
    }
    else
    {
        rwtFloaty = LOW;
    }
    if(digitalRead(LPS_SWITCH_PIN)==HIGH)
    {
        lpsSwitch = HIGH;
       // Serial.print("LPS SWITCH : ");
       // Serial.println(lpsSwitch);
  
    }
    else
    {
        lpsSwitch = LOW;
       // Serial.print("LPS SWITCH : ");
       // Serial.println(lpsSwitch);
    }
    if(digitalRead(HPS_SWITCH_PIN)==HIGH)
    {
        hpsSwitch = HIGH;
    }
    else
    {
        hpsSwitch = LOW;  
    }
}

bool floatValve(bool level1,bool level2)
{
    if(level1==HIGH && level2==HIGH)
    {
        //Serial.println("tank Full");
        TANK_LVL=0;
        return  LOW;
    }
    else if(level1==LOW && level2==LOW)
    {
        //Serial.println("tank is empty");
        TANK_LVL=2;
        return HIGH;
    }
    else if(level1==HIGH && level2==LOW)
    {
        if(rocontrolPump.machineRunning==HIGH)
        {
            //Serial.println("tank is about to full");
            TANK_LVL=1;
            return HIGH;
        }
        else
        {
            //Serial.println("tank is about to empty");
            TANK_LVL=1;
            return LOW;
        }
    }
    else if(level1==LOW && level2==HIGH)
    {
          //Serial.println("FLOATY connection not correct");
          //return;
    }     
} 

/************************************************************ END OF FLOATY,LPS,HPS MEASURE FUNCTION **********************************************************/
 
/***********************************************************FUNCTION TO CHECK MACHINE TRIP STATE **************************************************************/
byte machineTrip(bool twtf,bool rwtf,bool hps,bool rst)
{   
      byte trip_state=0;  
      tripswitchDetect();  
      bool twtstate=floatValve(twtFloaty_LL,twtFloaty_HL);
      if (twtstate==twtf)
      {
          rocontrolPump.lpswaitTimeout=LOW;
          machineStatus=0;
          trip_state = 1;     
      }
      else if (rwtFloaty ==rwtf)
      {
          rocontrolPump.lpswaitTimeout=LOW;
          machineStatus=0;
          trip_state = 2;             
      }
      else if (hpsSwitch ==hps)
      {
          rocontrolPump.lpswaitTimeout=LOW;
          machineStatus=0;
          trip_state = 3;                   
      }
      else if (rocontrolPump.lpswaitTimeout==HIGH)
      {
          machineStatus=0;
          trip_state = 9;  
          if(lpsSwitch == LOW)
          {
              trip_state = 11;
              rocontrolPump.lpswaitTimeout=LOW;
          }                           
      }
      else
      {    
          if(machineStatus!=0)
            trip_state=machineStatus;
          else
            trip_state = 0;                                  
      }
      return trip_state;
      //the trip condition based on current limits can also be added here
}

/***********************************************************END OF FUNCTION TO CHECK MACHINE TRIP STATE **********************************************************/


/****************************************************** FUNCTION TO CHECK MPV STATE *****************************************************************************/

//This function will continuosly check the state of MPV
byte mpvState()
{
    byte mpv_counter;
    //check for backwash
    bool rwps = digitalRead(RWP_MPV_PIN);
    bool hpps = digitalRead(HPP_MPV_PIN);
 
    if (rwps == OF && hpps == OF)
      mpv_counter = 0;                      //both raw water pump and high pressure pump should be off
      
    if (rwps == OF && hpps == ON)
      mpv_counter = 1;                      //raw water pump should be off and high pressure pump should be ON
      
    if (rwps == ON && hpps == OF)
      mpv_counter = 2;                      //raw water pump should be on and high pressure pump should be off
      
    if (rwps == ON && hpps == ON)
      mpv_counter = 3;                      //both raw water pump and high pressure pump should be on
      
    return mpv_counter;
}

/****************************************************** END OF FUNCTION TO CHECK MPV STATE *****************************************************************************/

/********************************************************** FUNCTION FOR SHOW DISPLAY **********************************************************************************/

void statusDisplay(byte statusValue)
{
  switch(statusValue)
  {
    case 0:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("**<< SWAJAl >>**"));      
      break;
      
    case 1:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("TW TANK FULL!..."));
      break;
      
    case 2:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("RW TANK EMPTY!.."));
      break;
      
    case 3:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("HIGH PRESSURE!.."));
      break;
      
    case 4:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("RESTARTING..."));
      break;
        
    case 5:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("RWP Dry Run!.."));
      break;
      
    case 6:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("RWP Overload!.."));
      break;
      
    case 7:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("HPP Dry Run!.."));
      break;
          
    case 8:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("HPP Overload!.."));
      break;
      
    case 9:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("LOW PRESSURE!..!"));
      break;
       
    case 10:
      lcd.clear();
      lcd.setCursor(0,1);
      lcd.print(F("  BACKWASH-ON  "));
      break;
      
    case 11:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Waiting For LPS"));
      lcd.setCursor(5,1);
      lcd.print("CLOSE");
      break;
      
    case 12:
      lcd.clear();
      lcd.setCursor(0,1);
      lcd.print(F("MOT ON DELAY"));
      break;
    
    case 13:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F(" MPV CONNECTION "));
      lcd.setCursor(0,1);
      lcd.print(F("NOT CORRECT....!"));
      break;
       
    default:
      break;    
  }
}

/********************************************************** FUNCTION FOR SHOW DISPLAY **********************************************************************************/

/********************************************************** FUNCTION FOR SHOW DISPLAY MEASUREMENT **********************************************************************/

void displayMeasurement(String parameter, float value ,String unit)
{
  lcd.clear();
  lcd.print(" $ MACHINE ON $");
  lcd.setCursor(0,1);
  lcd.print(parameter);
  lcd.print(": ");
  lcd.print(value);
  lcd.print(" ");
  lcd.print(unit);
}

void displayMeasurement(String parameter, unsigned long value ,String unit)
{
  lcd.clear();
  lcd.print(" $ MACHINE ON $ ");
  lcd.setCursor(0,1);
  lcd.print(parameter);
  lcd.print(": ");
  lcd.print(value);
  lcd.print(" ");
  lcd.print(unit);
}

void displayMeasurement(String parameter, unsigned int value ,String unit)
{
  lcd.clear();
  lcd.print(" $ MACHINE ON $");
  lcd.setCursor(0,1);
  lcd.print(parameter);
  lcd.print(": ");
  lcd.print(value);
  lcd.print(" ");
  lcd.print(unit);
}

/********************************************************** END OF FUNCTION FOR SHOW DISPLAY MEASUREMENT ******************************************************************/

/******************************************************** MACHINE INFORMATION DISPLAY *************************************************************************************/
void machineInformation()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("   * Swajal *  ");
  lcd.setCursor(0,1);
  lcd.print("Water Pvt. Ltd.");
}

/******************************************************** END OF MACHINE INFORMATION DISPLAY ******************************************************************************/

/**************************************************** EEPROM CHECK FUNCTION(Set RO Parameter read and write ) *************************************************************/
void checkEEPROM()
{
    int checkByte_addr=0;
    byte checkByte;
    int eepromAddress=0;
    checkByte=EEPROM.read(checkByte_addr);
    if(checkByte!=1)
    {
        checkByte=1;
        EEPROM.write(checkByte_addr,checkByte);    
        eepromAddress=checkByte_addr+sizeof(checkByte);
        EEPROM.put(eepromAddress,roParameters);
        roParameters.getData();
    }
    else
    {
        readParameter();
    }    
}  
void readParameter()
{
      int eepromAddress=sizeof(byte);
      EEPROM.get(eepromAddress, roParameters);
      roParameters.getData();     
      getBackwashInformation();
}
void writeParameter()
{
      int eeAddress=sizeof(byte);
      roParameters.setData();
      setBackwashInformation();
      EEPROM.put(eeAddress, roParameters);
      //Serial.println("************************************************** DATA WRITE IN EEPROM ********************************************************");
}   
/************************************************************END OF EEPROM FUNCTION ***********************************************************************/

/*****************************************************Function to perform the control setting*************************************************************/

            /*.............................. function for setting up the various constant for the program run.............................*/

void controllerSetting()
{
    bool yes = HIGH, no = LOW;
    unsigned int Timeup = 0;
    unsigned long timeoutMillis=0;
    unsigned long pass1=0;
    byte SW1=0,SW2=0;
    setting.button();
    if (setting.shiftButton == HIGH && setting.selectButton == HIGH)
    {
       // Serial.println(F("Inside Setting Control"));
        //IsSettingsChanged = 1;
        int wait = 2;
        for (int i = 0; i < 50; i++)//The wait for two button press to enter in setting is set to 2s
        {
            setting.button();
            SW1 = setting.shiftButton;
            SW2 = setting.selectButton;
            wait = wait + SW1 * SW2;
            delay(1);
           // Serial.print(F("wait count="));
           // Serial.println(wait);      
        }
        //check for wait count
        if (wait > 50)
        {      
          //  Serial.println(F("setting start"));
            wait = 0;    
            unsigned int password=0;
            password = setting.readValue("Enter Password", password);
            switch (password)
            { 
     
                case 111:
                    statusDisplay(0);
                    break;             

                case 112:
                    FLOW_CONSTANT_1       = setting.readValue("FLOW CONSTANT_1",FLOW_CONSTANT_1);
                    FLOW_CONSTANT_2       = setting.readValue("FLOW CONSTANT_2",FLOW_CONSTANT_2);
                    CURRENT_CONSTANT_1    = setting.readValue("CURRENT CONST_1",CURRENT_CONSTANT_1);
                    CURRENT_CONSTANT_2    = setting.readValue("CURRENT CONST_2",CURRENT_CONSTANT_2);
                    FLOW_SAMPLING_TIME    = setting.readValue("FLOW SAMPLING_T",FLOW_SAMPLING_TIME);
                    CURRENT_SAMPLING_TIME = setting.readValue("CURRENT SAMPLING",CURRENT_SAMPLING_TIME);
                    break;
                    
                case 116:
                    BACKWASH_TRIGGER_TIME = setting.readValue("BACKWASH TRIGG_T",BACKWASH_TRIGGER_TIME);
                    break;

                case 201:
                    TEMP_OFFSET           = setting.readValue("TEMP OFFSET",TEMP_OFFSET);
                    break;
                    
                case 215:
                    LPS_WAIT_TIME         = setting.readValue("LPS WAIT TIMEOUT",LPS_WAIT_TIME); 
                    FLUSH_TIME_INTERVAL   = setting.readValue("FLUSH INTERVAL",FLUSH_TIME_INTERVAL);
                    break;

                case 216:
                    RWP_LOWER_CURRENT     = setting.readValue("RWPC LOWER LIMIT",RWP_LOWER_CURRENT);
                    RWP_UPPER_CURRENT     = setting.readValue("RWPC UPPER LIMIT",RWP_UPPER_CURRENT);
                    HPP_LOWER_CURRENT     = setting.readValue("HPPC LOWER LIMIT",HPP_LOWER_CURRENT);
                    HPP_UPPER_CURRENT     = setting.readValue("HPPC UPPER LIMIT",HPP_UPPER_CURRENT);
                    RWP_WAIT_TIME         = setting.readValue("RWP WAIT TIME",RWP_WAIT_TIME);
                    break;

                case 205:
                    BAUD_RATE              = setting.readValue("BAUDRATE",BAUD_RATE);
                    SLAVE_ID              = setting.readValue("SLAVE ID",SLAVE_ID);
                    break;

                case 255:
                    password = 0;
                    password= setting.readValue("RESET T&R VOLUME",password);
                    if(password == 100)
                    {
                      TOTAL_TREATED_VOLUME = 0;
                    }
                    if(password == 101)
                    {
                      TOTAL_REJECT_VOLUME = 0;
                    }
                  /*  if(password == 102)
                    {
                      OPERATION_MINUTE = 0;
                    }*/
                    break;
                   
                case 919:
                    unsigned int year;
                    byte month;
                    byte date;
                    byte hr;
                    byte mint;
                    byte sec;
                    lcd.clear();
                    year=setting.readValue("Enter Year",year);
                    month=setting.readValue("Enter Month",month);
                    date=setting.readValue("Enter Date",date);
                    hr=setting.readValue("Enter Hour",hr);
                    mint=setting.readValue("Enter Minute",mint);
                    sec=setting.readValue("Enter Sec",sec);
                    rtc.adjust(DateTime(year,month,date,hr,mint,sec));              // set date time manually using push button
                    break;   
      
                default:
                    lcd.clear();
                    lcd.print(F("Incorrect Key"));
                    lcd.noCursor();
                    delay(1000);
                    break;
          } 
          statusDisplay(0);   
          if(setting.status_of_setting==LOW)
          {
              statusDisplay(0);
              return;
          }
       }
    }
    if(currentMillis-savedataMillis>=60000)           //this the time will decide the write operation over eeprom
    {
          savedataMillis=currentMillis;
          par_changed=HIGH;
    }
    if(par_changed==HIGH||setting.status_of_setting==HIGH)
    {
        par_changed=LOW;
        setting.status_of_setting=LOW; 
        writeParameter();                             //the the parameter which have been changed  
   
    }    
}

/************************************************ SET & GET BACKWASH DATA(READ or WRITE) FUNCTION **************************************************/
void setBackwashInformation()   // call in EEPROM Write Function
{
    BACKWASH_CYCLE_COUNT = mpv.backwashCycleCount;
    LAST_BACKWASH_TIME   = mpv.lastBackwashCall;
}

void getBackwashInformation()  // call in EEPROM Read Function
{
    unsigned long value = OPERATION_MINUTES;
    
    mpv.backwashDayCount = value/1440;          //divide the total minutes by 1440( Total minute in a day ) to get day count
    value = value%1440;                         //store the remainder in the variable value
  
    mpv.backwashHourCount = value/60;           //divid the remainder by 60 to calculate the hour count
    
    mpv.backwashMinuteCount = value%60;         //store the remainder in the variable for the minute count  
 
    mpv.backwashCycleCount = BACKWASH_CYCLE_COUNT;   // Number of backwash since machine start
    mpv.lastBackwashCall   = LAST_BACKWASH_TIME;      // last backwash time
}

/************************************************ END OF SET & GET BACKWASH DATA(READ or WRITE) FUNCTION ************************************/

/******************************************************* SET RTC TIME FUNCTION **************************************************************/

void rtcTime()
{
   DateTime now = rtc.now();
   if(rtc_status==HIGH)
   {
        rtc_status=LOW;
        lcd.setCursor(0,1);
        lcd.print("TIME: ");
        lcd.setCursor(7, 1);
        
        lcd.print(now.hour());
        lcd.print(':');
        lcd.print(now.minute());
        lcd.print(':');
        lcd.print(now.second());
        lcd.print("   ");
        
        lcd.setCursor(0, 0);
        lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);
        lcd.print(" ,");
        lcd.print(now.day());
        lcd.print('/');
        lcd.print(now.month());
        lcd.print('/');
        lcd.print(now.year());
   }
   CURRENT_TIME_STAMP=now.unixtime();
  /*  delay(5000);
    lcd.clear();
    lcd.print(now.unixtime());
    delay(3000);
    lcd.clear();*/
}
/*********************************************************** END OF RTC TIME FUNCTION ****************************************************/

/*********************************************************** TDS MEASURE FUNCTION ********************************************************/
unsigned int measureTDS(byte pin)
{
   int tds;   
   float voltage=analogRead(pin)*5.0/1023.0; 
   Serial.print("Voltage : ");
   Serial.println(voltage);  
   tds =  543.8301*voltage + 6.932361;
   if(tds<0)
   {
      return 0;
   }
   else
   {       
      return tds;    
   }
}

/********************************************************* END OF TDS MEASURE FUNCTION *************************************************/
/**************************************************** END OF RTC FUNCTION ******************************************************************/

void updateRODataOnModbus()
{  
      holdingRegs[OPM_U]  = ((unsigned long)OPERATION_MINUTES>>16)& 0xffff;      //1.
      holdingRegs[OPM_L]  = (unsigned long)OPERATION_MINUTES & 0xffff;
      holdingRegs[TDS1]   = TDS_PERMEATE;                         //2.
      holdingRegs[TDS2]   = TDS_INLET;                            //3.
      holdingRegs[RWPC]   = int(RWP_CURRENT*10);                  //4.
      holdingRegs[HPPC]   = int(HPP_CURRENT*10);                  //5.
      holdingRegs[PFR]    = PERMEATE_FLOW_RATE;                   //6.
      holdingRegs[RFR]    = REJECT_FLOW_RATE;                     //7.
      holdingRegs[TWV_U]  = (TOTAL_TREATED_VOLUME>>16) & 0xffff;  //8.
      holdingRegs[TWV_L]  = (TOTAL_TREATED_VOLUME) & 0xffff;      
      holdingRegs[RWV_U]  = (TOTAL_REJECT_VOLUME>>16)& 0xffff;    //9.
      holdingRegs[RWV_L]  = TOTAL_REJECT_VOLUME& 0xffff;          
      holdingRegs[BCC]    = BACKWASH_CYCLE_COUNT;                 //10.
      holdingRegs[TLB]    = TANK_LVL;                             //11.
      holdingRegs[UVST]   = mainsState;                           //12.
      holdingRegs[TEMP]   = TANK_WATER_TEMP;                      //13.
      holdingRegs[TSC]    = TRIP_STATE_COUNTER;                   //14.
      holdingRegs[TIME_STAMP_U]=((unsigned long)CURRENT_TIME_STAMP>>16)&0xffff;
      holdingRegs[TIME_STAMP_L]=(unsigned long)CURRENT_TIME_STAMP&0xffff;                            
}

void updateConfigDataOnModbus()
{
      holdingRegs[CC1]    = CURRENT_CONSTANT_1;          //1.
      holdingRegs[CC2]    = CURRENT_CONSTANT_2;          //2.
      holdingRegs[RWPLC]  = RWP_LOWER_CURRENT;          //3.
      holdingRegs[RWPUC]  = RWP_UPPER_CURRENT;          //4.
      holdingRegs[HPPLC]  = HPP_LOWER_CURRENT;          //5.
      holdingRegs[HPPUC]  = HPP_UPPER_CURRENT;          //6.
      holdingRegs[BTT]    = BACKWASH_TRIGGER_TIME;      //7.
      holdingRegs[FC1]    = FLOW_CONSTANT_1;             //8.
      holdingRegs[FC2]    = FLOW_CONSTANT_2;             //9.
      holdingRegs[RWPWT]  = RWP_WAIT_TIME;              //10.
      holdingRegs[LPSWT]  = LPS_WAIT_TIME;              //11.
      holdingRegs[FINT]   = FLUSH_TIME_INTERVAL;        //12.
      holdingRegs[CST]    = CURRENT_SAMPLING_TIME;      //13. 
      holdingRegs[FST]    = FLOW_SAMPLING_TIME;         //14.
      holdingRegs[MS]     = MACHINE_STATE;              //15.
      holdingRegs[TEMP]   = TANK_WATER_TEMP;            //16.
      holdingRegs[SRN]    = MACHINE_SR_NO;              //17.
      holdingRegs[SFLG]   = HIGH;//set the flag high when update the data on modbus
  }
void updateConfigDataFromModbus()
{ 
      holdingRegs[RFLG]     = LOW;
      CURRENT_CONSTANT_1     = holdingRegs[CC1];         //1.
      CURRENT_CONSTANT_2     = holdingRegs[CC2];         //2.
      RWP_LOWER_CURRENT     = holdingRegs[RWPLC];       //3.
      RWP_UPPER_CURRENT     = holdingRegs[RWPUC];       //4.
      HPP_LOWER_CURRENT     = holdingRegs[HPPLC];       //5.
      HPP_UPPER_CURRENT     = holdingRegs[HPPUC];       //6.
      BACKWASH_TRIGGER_TIME = holdingRegs[BTT];         //7.
      FLOW_CONSTANT_1        = holdingRegs[FC1];         //8.
      FLOW_CONSTANT_2        = holdingRegs[FC2];         //9.
      RWP_WAIT_TIME      = holdingRegs[RWPWT];       //10.
      LPS_WAIT_TIME      = holdingRegs[LPSWT];       //11.
      FLUSH_TIME_INTERVAL   = holdingRegs[FINT];        //12.
      CURRENT_SAMPLING_TIME = holdingRegs[CST];         //13.
      FLOW_SAMPLING_TIME    = holdingRegs[FST];         //14.
      MACHINE_STATE         = holdingRegs[MS];          //15.
      MACHINE_SR_NO         = holdingRegs[SRN];         //16.
      remoteBackwashTrigger = holdingRegs[TBCK];        //17.
      par_changed=HIGH;//set the flag high to save the parameters after updating 
  } 
