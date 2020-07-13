#include <Elegoo_GFX.h>    // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>
#include <EEPROM.h>

#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

/******************* UI details */
#define BUTTON_X 40
#define BUTTON_Y 30
#define BUTTON_W 60
#define BUTTON_H 30
#define BUTTON_SPACING_X 20
#define BUTTON_SPACING_Y 20
#define BUTTON_TEXTSIZE 1

#define MEASUREMENT_X BUTTON_X
#define ALARM_X BUTTON_X + BUTTON_W + BUTTON_SPACING_X
#define BATTERY_X BUTTON_X + 2*BUTTON_W + 2*BUTTON_SPACING_X

// text box where numbers go
#define TEXT_X 10
#define TEXT_Y 10
#define TEXT_W 220
#define TEXT_H 50
#define TEXT_TSIZE 3
#define TEXT_TCOLOR ILI9341_MAGENTA
// the data (phone #) we store in the textfield
#define TEXT_LEN 12
char textfield[TEXT_LEN+1] = "";
uint8_t textfield_i=0;

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

#define TS_MINX 120
#define TS_MAXX 900

#define TS_MINY 70
#define TS_MAXY 920
// We have a status line for like, is FONA working
#define STATUS_X 10
#define STATUS_Y 65

#define MINPRESSURE 5
#define MAXPRESSURE 1000

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);


volatile double hvCurrent;
volatile double hvVoltage;
double currentMax = 0;
double currentMin = 0;
double voltageMax = -1;
double voltageMin = -1;
int socMax = -1;
int socMin = -1;
unsigned long hvIsoRes = 0;
String hvIsoMode;
int soc = 0;
int temperature = 0;
String hvilAlarm = "NA";
String overcurrentAlarm = "NA";
String hvilInterruptAlarm = "NA";
String hvOutOfRangeAlarm = "NA";
bool batOn = false;
bool contactorIsOn = false;
bool notAcknowledged;
bool currentFlag, voltageFlag, socFlag, resetFlag;

#define HVIL_PIN 21
#define CONTACTOR_PIN 25

Elegoo_GFX_Button measurement;
Elegoo_GFX_Button alarm;
Elegoo_GFX_Button battery;
Elegoo_GFX_Button batteryOn;
Elegoo_GFX_Button batteryOff;
Elegoo_GFX_Button acknowledge;  //Casey
  
  
// TCB Struct
struct MyStruct
{
  void (*myTask)(void*);
  void* taskDataPtr;
  struct MyStruct* next;
  struct MyStruct* prev;
}; 
typedef struct MyStruct TCB;
//tft data struct

typedef struct {
  TCB* tftTask;
  TCB* measureTask;
  TCB* socTask;
  TCB* contactorTask;
  TCB* alarmTask;

  Elegoo_GFX_Button* measurement;
  Elegoo_GFX_Button* alarm;
  Elegoo_GFX_Button* battery;
  Elegoo_GFX_Button* batteryOn;
  Elegoo_GFX_Button* batteryOff;
  Elegoo_GFX_Button* acknowledge; // Casey

  Elegoo_TFTLCD* tft;
  
} StartUpData;

typedef struct
{
  int* tftModePtr;
  int* temperature;
  volatile double* hvCurrent;
  volatile double* hvVoltage;
  unsigned long* hvIsoRes;
  String* hvIsoMode;
  int* soc; // up to user to define data type
  bool* batOn;
  String* hvilAlarm;
  String* overcurrentAlarm;
  String* hvOutOfRangeAlarm;
  String* hvilInterruptAlarm;
  bool* notAcknowledged;

  Elegoo_GFX_Button* measurement;
  Elegoo_GFX_Button* alarm;
  Elegoo_GFX_Button* battery;
  Elegoo_GFX_Button* batteryOn;
  Elegoo_GFX_Button* batteryOff;
  Elegoo_GFX_Button* acknowledge; // Casey

  Elegoo_TFTLCD* tft;
  //TouchScreen* ts;
} TFTData;

//measurement data struct
typedef struct
{
  int* temperature;
  volatile double* hvCurrent;
  volatile double* hvVoltage;
  unsigned long* hvIsoRes;
  String* hvIsoMode;
  int* tempIsoResIndex;
  int* hvCurrentIndex;
  int* hvVoltageIndex;
  int* hvIsoModeIndex;
  bool* currentFlag;
  bool* voltageFlag;
  bool* resetFlag;
  double* currentMax;
  double* currentMin;
  double* voltageMax;
  double* voltageMin;
  int* socMax;
  int* socMin;
} MeasurementData;

//soc data struct
typedef struct
{
  int* temperature;
  int* soc;
  int* socIndex;
  volatile double* hvCurrent;
  volatile double* hvVoltage;
  int* socMax;
  int* socMin;
  bool* socFlag;
} SoCData;

//contactor control data struct
typedef struct
{
  bool* batOn;
  bool* contactorIsOn;
  String* hvilInterruptAlarm;
  String* hvilAlarm;
  String* overcurrentAlarm;
  String* hvOutOfRangeAlarm;
} ContactorData;

// alarm control data struct
typedef struct
{
  String* hvilAlarm;
  String* hvilInterruptAlarm;
  String* overcurrentAlarm;
  String* hvOutOfRangeAlarm;
  int* hvilAlarmIndex;
  int* overcurrentAlarmIndex;
  int* hvOutOfRangeAlarmIndex;
  bool* notAcknowledged;
  unsigned long* oneSecond;
  unsigned long* twoSeconds; // up to user to define data type
  unsigned long* threeSeconds;
  volatile double* hvCurrent;
  volatile double* hvVoltage;
} AlarmData;

typedef struct
{
  volatile double* hvVoltagePtr;
  volatile double* hvCurrentPtr;
} CommunicationData;

typedef struct {
  bool* currentFlag;
  bool* voltageFlag;
  bool* socFlag;
  bool* resetFlag;
  volatile double* hvCurrent;
  volatile double* hvVoltage;
  int* socMax;
  int* socMin;
  double* currentMax;
  double* currentMin;
  double* voltageMax;
  double* voltageMin;
} DataLogTaskData;

TCB startupTask;
TCB programScheduleTask;

TCB tftTask;
int tftMode = 0; // 0 = Measurement, 1 = Alarm, 2 = Battery on/off
TFTData tftData;

TCB measureTask;
MeasurementData measureTaskData;
int tempIsoResIndex = 0;
int hvCurrentIndex = 0;
int hvVoltageIndex = 0;
int hvIsoModeIndex = 0;
unsigned long oneSecondMeasure = millis(); //time keepers for cycling data
unsigned long twoSecondsMeasure = millis();
unsigned long threeSecondsMeasure = millis();

TCB socTask;
SoCData socData;

TCB contactorTask;
ContactorData contactorData;

TCB alarmTask;
AlarmData alarmData;
unsigned long oneSecondAlarm = millis(); // time keepers for cycling data
unsigned long twoSecondsAlarm = millis();
unsigned long threeSecondsAlarm = millis();
int hvilAlarmIndex = 0;
int overcurrentAlarmIndex = 0;
int hvOutOfRangeAlarmIndex = 0;

TCB communicateTask;
CommunicationData communicationData;

TCB dataLogTask;
DataLogTaskData dataLogTaskData;

TCB remoteTerminalTask;

TCB* currentTask = NULL;

void startup(void* arg) {
    // tft set up code copied from phonecal
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial.println(F("TFT LCD test"));


  #ifdef USE_Elegoo_SHIELD_PINOUT
    Serial.println(F("Using Elegoo 2.4\" TFT Arduino Shield Pinout"));
  #else
    Serial.println(F("Using Elegoo 2.4\" TFT Breakout Board Pinout"));
  #endif

  Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());

  tft.reset();

   uint16_t identifier = tft.readID();
   if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x4535) {
    Serial.println(F("Found LGDP4535 LCD driver"));
  }else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else if(identifier==0x0101)
  {     
      identifier=0x9341;
       Serial.println(F("Found 0x9341 LCD driver"));
  }
  else if(identifier==0x1111)
  {     
      identifier=0x9328;
       Serial.println(F("Found 0x9328 LCD driver"));
  }
  else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Elegoo 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_Elegoo_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Elegoo_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    identifier=0x9328;
  
  }

  Serial.println("[1] Reset EEPROM");
  Serial.println("[2] Operating HV Current Range [Hi, Lo]");
  Serial.println("[3] Operating HV Voltage Range [Hi, Lo]");
  Serial.println("\nEnter your menu choice [1-3] <char>");
  
  StartUpData* startUpData = arg;
  tft.begin(identifier);

  tft.setRotation(2);
  tft.fillScreen(BLACK);

  startUpData->measurement = &measurement;
  startUpData->alarm = &alarm;
  startUpData->battery = &battery;
  startUpData->batteryOn = &batteryOn;
  startUpData->batteryOff = &batteryOff;
  startUpData->acknowledge = &acknowledge;  // Casey

  // initiate buttons for different modes
  startUpData->measurement->initButton(&tft, BUTTON_X, BUTTON_Y, BUTTON_W, BUTTON_H, ILI9341_WHITE, ILI9341_DARKCYAN, ILI9341_WHITE, "MEASURE", BUTTON_TEXTSIZE);
  startUpData->measurement->drawButton();
  startUpData->alarm->initButton(&tft, ALARM_X, BUTTON_Y, BUTTON_W, BUTTON_H, ILI9341_WHITE, ILI9341_DARKCYAN, ILI9341_WHITE, "ALARM", BUTTON_TEXTSIZE);
  startUpData->alarm->drawButton();
  startUpData->battery->initButton(&tft, BATTERY_X, BUTTON_Y, BUTTON_W, BUTTON_H, ILI9341_WHITE, ILI9341_DARKCYAN, ILI9341_WHITE, "BATT", BUTTON_TEXTSIZE);
  startUpData->battery->drawButton();

  // initiate contactor on/off buttons
  startUpData->batteryOn->initButton(&tft, 118, 130, 75, 75, ILI9341_WHITE, ILI9341_DARKCYAN, ILI9341_WHITE, "ON", BUTTON_TEXTSIZE);
  startUpData->batteryOff->initButton(&tft, 118, 220, 75, 75, ILI9341_WHITE, ILI9341_DARKCYAN, ILI9341_WHITE, "OFF", BUTTON_TEXTSIZE);

  // initiate acknowledge button for alarms
  startUpData->acknowledge->initButton(&tft, 118, 220, 150, 75, ILI9341_WHITE, ILI9341_DARKCYAN, ILI9341_WHITE, "IGHT DEN", 2); // doesn't fit the whole word? Casey

  startUpData->tft = &tft;

  Elegoo_TFTLCD tft =  *startUpData->tft;
  
  // initial screen is measurement
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(10, 60);
  tft.print("State of Charge: ");
  tft.setCursor(10, 80);
  tft.print("Temperature: ");
  tft.setCursor(10, 100);
  tft.print("HV Current: ");
  tft.setCursor(10, 120);
  tft.print("HV Voltage: ");
  tft.setCursor(10, 140);
  tft.print("HV Isolation Resistance: ");
  tft.setCursor(10, 160);
  tft.print("HV Isolation Ckt Operating Mode: ");
  tft.setCursor(10, 180);
  tft.print("HVIL (HV Interlock Loop) Status: ");
  pinMode(HVIL_PIN, INPUT_PULLUP);
  pinMode(CONTACTOR_PIN, OUTPUT);
  
  // prepare schedule task
  programScheduleTask.myTask = &programSchedule;


  startUpData->measureTask->myTask = &measure; // point to measurement function
  startUpData->measureTask->taskDataPtr = &measureTaskData; // point to measurement data struct
  measureTaskData.tempIsoResIndex = &tempIsoResIndex; // assigning data pointers
  measureTaskData.hvCurrentIndex = &hvCurrentIndex;
  measureTaskData.hvVoltageIndex = &hvVoltageIndex;
  measureTaskData.hvIsoModeIndex = &hvIsoModeIndex;
  measureTaskData.temperature = &temperature;
  measureTaskData.hvCurrent = &hvCurrent;
  measureTaskData.hvVoltage = &hvVoltage;
  measureTaskData.hvIsoRes = &hvIsoRes;
  measureTaskData.hvIsoMode = &hvIsoMode;
  measureTaskData.currentFlag = &currentFlag;
  measureTaskData.voltageFlag = &voltageFlag;
  measureTaskData.voltageMax = &voltageMax;
  measureTaskData.voltageMin = &voltageMin;
  measureTaskData.currentMax = &currentMax;
  measureTaskData.currentMin = &currentMin;
  measureTaskData.socMax = &socMax;
  measureTaskData.socMin = &socMin;
  measureTaskData.resetFlag = &resetFlag;

  
  // prepare tft task
  startUpData->tftTask->myTask = &tftUpdate; // point to tft function
  startUpData->tftTask->taskDataPtr = &tftData; // point to tft task data struct
  tftData.temperature = &temperature;           // assigning data pointers
  tftData.hvCurrent = &hvCurrent;
  tftData.hvVoltage = &hvVoltage;
  tftData.hvIsoRes = &hvIsoRes;
  tftData.hvIsoMode = &hvIsoMode;
  tftData.batOn = &batOn;
  tftData.tftModePtr = &tftMode;
  tftData.soc = &soc;
  tftData.hvilAlarm = &hvilAlarm;
  tftData.overcurrentAlarm = &overcurrentAlarm;
  tftData.hvOutOfRangeAlarm = &hvOutOfRangeAlarm;
  tftData.alarm = &alarm;
  tftData.measurement = &measurement;
  tftData.battery = &battery;
  tftData.batteryOn = &batteryOn;
  tftData.batteryOff = &batteryOff;
  tftData.acknowledge = &acknowledge;
  tftData.tft = &tft;
  tftData.hvilInterruptAlarm = &hvilInterruptAlarm;
  tftData.notAcknowledged = &notAcknowledged;
  //tftData.ts = &ts;
  
  

  startUpData->socTask->myTask = &socCalc; // point to soc function
  startUpData->socTask->taskDataPtr = &socData; // point to doc task data
  socData.soc = &soc; // assign data pointers
  //socData.socIndex = &socIndex;
  socData.hvCurrent = &hvCurrent;
  socData.hvVoltage = &hvVoltage;
  //socData.socMax = &socMax;
  //socData.socMin = &socMin;
  //socData.socFlag = &socFlag;
  //socData.temperature = &temperature;

  startUpData->contactorTask->myTask = &contactorControl; // point to contactor task function
  startUpData->contactorTask->taskDataPtr = &contactorData; // point to contactor task data
  contactorData.batOn = &batOn; // assigning data pointers
  contactorData.contactorIsOn = &contactorIsOn;
  contactorData.hvilInterruptAlarm = &hvilInterruptAlarm;
  contactorData.hvilAlarm = &hvilAlarm;
  contactorData.overcurrentAlarm = &overcurrentAlarm;
  contactorData.hvOutOfRangeAlarm = &hvOutOfRangeAlarm;

  startUpData->alarmTask->myTask = &alarmControl;// point to alarm task function
  startUpData->alarmTask->taskDataPtr = &alarmData;// point to alarm task data
  alarmData.oneSecond = &oneSecondAlarm; // assigning data pointers
  alarmData.twoSeconds = &twoSecondsAlarm;
  alarmData.threeSeconds = &threeSecondsAlarm;
  alarmData.hvilAlarm = &hvilAlarm;
  alarmData.overcurrentAlarm = &overcurrentAlarm;
  alarmData.hvilInterruptAlarm = &hvilInterruptAlarm;
  alarmData.hvOutOfRangeAlarm = &hvOutOfRangeAlarm;
  alarmData.hvilAlarmIndex = &hvilAlarmIndex;
  alarmData.overcurrentAlarmIndex = &overcurrentAlarmIndex;
  alarmData.hvOutOfRangeAlarmIndex = &hvOutOfRangeAlarmIndex;
  alarmData.notAcknowledged = &notAcknowledged;
  alarmData.hvCurrent = &hvCurrent;
  alarmData.hvVoltage = &hvVoltage;

  communicateTask.myTask = &communicate;
  communicateTask.taskDataPtr = &communicationData;
  communicationData.hvVoltagePtr = &hvVoltage;
  communicationData.hvCurrentPtr = &hvCurrent;

  dataLogTask.myTask = &dataLog;
  dataLogTask.taskDataPtr = &dataLogTaskData;
  dataLogTaskData.currentFlag = &currentFlag;
  dataLogTaskData.voltageFlag = &voltageFlag;
  dataLogTaskData.socFlag = &socFlag;
  dataLogTaskData.hvCurrent = &hvCurrent;
  dataLogTaskData.hvVoltage = &hvVoltage;
  dataLogTaskData.socMax = &socMax;
  dataLogTaskData.socMin = &socMin;
  dataLogTaskData.voltageMax = &voltageMax;
  dataLogTaskData.voltageMin = &voltageMin;
  dataLogTaskData.currentMax = &currentMax;
  dataLogTaskData.currentMin = &currentMin;
  dataLogTaskData.resetFlag = &resetFlag;

  remoteTerminalTask.myTask = &remoteTerminal;
  remoteTerminalTask.taskDataPtr = &dataLogTaskData;

  insertTask(&currentTask, startUpData->measureTask);
  insertTask(&currentTask, startUpData->tftTask);
  insertTask(&currentTask, startUpData->socTask);
  insertTask(&currentTask, startUpData->contactorTask);
  insertTask(&currentTask, startUpData->alarmTask);
  insertTask(&currentTask, &dataLogTask);
  insertTask(&currentTask, &remoteTerminalTask);

  attachInterrupt(digitalPinToInterrupt(HVIL_PIN), hvilInterruptAlarmFunction, RISING);
  
  programScheduleTask.myTask(arg); // start the program schedule
}


void programSchedule(void* arg) {
  StartUpData* startUpDataPtr = arg;
  unsigned long timeExecuted;
  int i = 0;

  /*TCB* currentTask = NULL;
  insertTask(&currentTask, startUpDataPtr->measureTask);
  insertTask(&currentTask, startUpDataPtr->tftTask);
  insertTask(&currentTask, startUpDataPtr->socTask);
  insertTask(&currentTask, startUpDataPtr->contactorTask);
  insertTask(&currentTask, startUpDataPtr->alarmTask);*/

  while(1) { // run program
    
    while (currentTask->prev != NULL) {
      currentTask = currentTask->prev;
    }
    
    if (i%10 == 1) {
      removeTask(&currentTask, startUpDataPtr->tftTask);
      removeTask(&currentTask, &remoteTerminalTask);
    }
    if (i%10 == 0 && i != 0) {
      insertTask(&currentTask, startUpDataPtr->tftTask);
      insertTask(&currentTask, &remoteTerminalTask);
    }
    if(i%50 == 1) {
      removeTask(&currentTask, &dataLogTask);
    }
    if (i%50 == 0 && i != 0) {
      insertTask(&currentTask, &dataLogTask);
    }
    timeExecuted = millis();

    while(currentTask != NULL) {
      currentTask->myTask(currentTask->taskDataPtr);

      currentTask = currentTask->next;
    }

    while (millis() - timeExecuted < 100);
    //Serial.println(millis() - timeExecuted);
    i++;
  }
  
}

void insertTask(TCB** head, TCB* task) {
  TCB* temporary = *head;

  if (temporary == NULL) {
    *head = task;
    task->next = NULL;
    task->prev = NULL;
  }
  else {
    while(temporary->next != NULL) {
      temporary = temporary->next;
    }
    temporary->next = task;
    temporary->next->prev = temporary;
    temporary->next->next = NULL;
  }
}

void removeTask(TCB** head, TCB* task) {
  TCB* temporary = *head;

  if (temporary == task) {
    *head = temporary->next;
    (*head)->prev = NULL;
  }
  else {
    while(temporary->next != task) {
      temporary = temporary->next;
    }
  
    temporary->next = temporary->next->next;
    if(temporary->next != NULL)
      temporary->next->prev = temporary;
  }
}

void tftUpdate(void* arg) {
  
  // put your main code here, to run repeatedly:
  TFTData* tftDataPtr = arg; //dereference the data
  TFTData tftData = *tftDataPtr;
  
  Elegoo_TFTLCD tft = *tftData.tft;
    
  digitalWrite(13, HIGH);
  TSPoint p = ts.getPoint(); //detect touch point
  digitalWrite(13, LOW);
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { // if within pressure bounds, register touch
    p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
    p.y = (tft.height()-map(p.y, TS_MINY, TS_MAXY, tft.height(), 0));
  }
  
  // check if measurement button pressed
  if (measurement.contains(p.x, p.y) && !*tftDataPtr->notAcknowledged) { // added by Casey
    tftDataPtr->measurement->press(true);
  }
  else {
    tftDataPtr->measurement->press(false);
  }
  if (tftDataPtr->measurement->justReleased()) {
      // Serial.print("Released: "); Serial.println(b);
      tftDataPtr->measurement->drawButton();  // draw normal
  }
    
  if (tftDataPtr->measurement->justPressed()) {    // prepare measurement screen
      tftDataPtr->measurement->drawButton(true);  // draw invert!
      tft.fillRect(0, 60, 300, 500, ILI9341_BLACK);
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(10, 60);
      tft.print("State of Charge: ");
      tft.setCursor(10, 80);
      tft.print("Temperature: ");
      tft.setCursor(10, 100);
      tft.print("HV Current: ");
      tft.setCursor(10, 120);
      tft.print("HV Voltage: ");
      tft.setCursor(10, 140);
      tft.print("HV Isolation Resistance: ");
      tft.setCursor(10, 160);
      tft.print("HV Isolation Ckt Operating Mode: ");
      tft.setCursor(10, 180);
      tft.print("HVIL (HV Interlock Loop) Status: ");
      *tftData.tftModePtr = 0;
  }

  // This doesn't work because the screen is drawn only when buttons are pressed.
  // Also we will want to combine all the alarms into one boolean but I've just been using the hvil alarm for testing
  /*if (*tftData.hvilAlarm != "ANA") { // Casey
    *tftData.tftModePtr = 1;
  }*/
  
  // check if alarm button pressed
  if (tftDataPtr->alarm->contains(p.x, p.y)) {
    tftDataPtr->alarm->press(true);
  } else {
    tftDataPtr->alarm->press(false);
  }
  if (tftDataPtr->alarm->justReleased()) {
      // Serial.print("Released: "); Serial.println(b);
      tftDataPtr->alarm->drawButton();  // draw normal
  }
  // if pressed, set up alarm screen
  if (tftDataPtr->alarm->justPressed() || *tftDataPtr->notAcknowledged && *tftData.tftModePtr != 1) {
      if (tftDataPtr->alarm->justPressed())
        tftDataPtr->alarm->drawButton(true);  // draw invert!
      tft.fillRect(0, 60, 300, 500, ILI9341_BLACK);
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(10, 60);
      tft.print("High Voltage Interlock Alarm: ");
      tft.setCursor(10, 80);
      tft.print("HVIL Interrupt: ");
      tft.setCursor(10, 100);
      tft.print("Overcurrrent: ");
      tft.setCursor(10, 120);
      tft.print("High Voltage Out of Range: ");
      tftDataPtr->acknowledge->drawButton();  // Casey
      *tftData.tftModePtr = 1;   
  }

  
  
  
  //check if battery button pressed
  if (tftDataPtr->battery->contains(p.x, p.y) && !*tftDataPtr->notAcknowledged) {  // added by Casey
    tftDataPtr->battery->press(true);
  }
  else {
    tftDataPtr->battery->press(false);
  }
  if (tftDataPtr->battery->justReleased()) {
      // Serial.print("Released: "); Serial.println(b);
      tftDataPtr->battery->drawButton();  // draw normal
    }
  // if pressed, set up battery screen
  if (tftDataPtr->battery->justPressed()) {
      tftDataPtr->battery->drawButton(true);  // draw invert!
      tft.fillRect(0, 60, 300, 500, ILI9341_BLACK);
      tftDataPtr->batteryOn->drawButton();
      tftDataPtr->batteryOff->drawButton();
      *tftData.tftModePtr = 2;
  }
  
  
   // mode 0 is measurement mode, updates the measurement values
   if (*tftData.tftModePtr == 0) {
      tft.fillRect(10, 70, 100, 7, ILI9341_BLACK);
      tft.setCursor(10, 70);
      tft.print(*tftData.soc);
      
      tft.fillRect(10, 90, 100, 7, ILI9341_BLACK);
      tft.setCursor(10, 90);
      tft.print(*tftData.temperature);

      tft.fillRect(10, 110, 100, 7, ILI9341_BLACK);
      tft.setCursor(10, 110);
      tft.print(*tftData.hvCurrent);

      tft.fillRect(10, 130, 100, 7, ILI9341_BLACK);
      tft.setCursor(10, 130);
      tft.print(*tftData.hvVoltage);

      tft.fillRect(10, 150, 100, 7, ILI9341_BLACK);
      tft.setCursor(10, 150);
      tft.print(*tftData.hvIsoRes);

      tft.fillRect(10, 170, 100, 7, ILI9341_BLACK);
      tft.setCursor(10, 170);
      tft.print(*tftData.hvIsoMode);
      
      tft.fillRect(10, 190, 100, 7, ILI9341_BLACK);
      tft.setCursor(10, 190);
      
      if (!digitalRead(HVIL_PIN)) {
        tft.print("CLOSED");
      }
      else {
        tft.print("OPEN");
      }
      
   }
   
   // mode 1 is alarm mode, updates the alarm values
   if (*tftData.tftModePtr == 1) {
    tft.fillRect(10, 70, 30, 7, ILI9341_BLACK);
    tft.setCursor(10, 70);
    tft.print(*tftData.hvilAlarm);
  
    tft.fillRect(10, 90, 30, 7, ILI9341_BLACK);
    tft.setCursor(10, 90);
    tft.print(*tftData.hvilInterruptAlarm);

    tft.fillRect(10, 110, 30, 7, ILI9341_BLACK);
    tft.setCursor(10, 110);
    tft.print(*tftData.overcurrentAlarm);

    tft.fillRect(10, 130, 30, 7, ILI9341_BLACK);
    tft.setCursor(10, 130);
    tft.print(*tftData.hvOutOfRangeAlarm);
    
      //check if acknowledge button is pressed
    if (tftDataPtr->acknowledge->contains(p.x, p.y)) {  // added by Casey
      tftDataPtr->acknowledge->press(true);
    }
    else {
      tftDataPtr->acknowledge->press(false);
    }
    if (tftDataPtr->acknowledge->justReleased()) {
        // Serial.print("Released: "); Serial.println(b);
        tftDataPtr->acknowledge->drawButton();  // draw normal
      }
  
    if (tftDataPtr->acknowledge->justPressed()) {
        tftDataPtr->acknowledge->drawButton(true);  // draw invert!
        *tftDataPtr->notAcknowledged = false;
    }
    
   }
   
  
   if (*tftData.tftModePtr == 2) {
    if (tftDataPtr->batteryOn->contains(p.x, p.y)) {
      tftDataPtr->batteryOn->press(true);
    }
    else {
      tftDataPtr->batteryOn->press(false);
    }
    if (tftDataPtr->batteryOn->justReleased()) {
        // Serial.print("Released: "); Serial.println(b);
        tftDataPtr->batteryOn->drawButton();  // draw normal
      }
      
    if (tftDataPtr->batteryOn->justPressed() && (*tftData.hvilInterruptAlarm == "NA")) {
        tftDataPtr->batteryOn->drawButton(true);  // draw invert!
        *tftData.batOn = true;
    }

   // mode 2 is battery mode, draws battery on/off buttons
    if (tftDataPtr->batteryOff->contains(p.x, p.y)) {
      tftDataPtr->batteryOff->press(true);
    }
    else {
      tftDataPtr->batteryOff->press(false);
    }
    if (tftDataPtr->batteryOff->justReleased()) {
        // Serial.print("Released: "); Serial.println(b);
        tftDataPtr->batteryOff->drawButton();  // draw normal
      }
      
    if (tftDataPtr->batteryOff->justPressed()) {
        tftDataPtr->batteryOff->drawButton(true);  // draw invert!
        *tftData.batOn = false;
    }
   }

   
}


void measure(void* arg){
  MeasurementData* dataPtr = arg; // dereference data
  MeasurementData data = *dataPtr;
  
  communicate(arg);
  if (*data.resetFlag) {
    *data.currentMax = 0;
    *data.currentMin = 0;
    *data.voltageMax = -1;
    *data.voltageMin = -1;
    *data.socMax = -1;
    *data.socMin = -1;
  }
  else {
    if (*dataPtr->hvVoltage > voltageMax || voltageMax == -1) {
      voltageFlag = true;
      voltageMax = *dataPtr->hvVoltage;
    }
    if (*dataPtr->hvVoltage < voltageMin || voltageMin == -1) {
      voltageMin = *dataPtr->hvVoltage;
      voltageFlag = true;
    }
  
    if (*dataPtr->hvCurrent > currentMax || currentMax == 0) {
      currentMax = *dataPtr->hvCurrent;
      currentFlag = true;
    }
    if (*dataPtr->hvCurrent < currentMin || currentMin == 0) {
      currentMin = *dataPtr->hvCurrent;
      currentFlag = true;
    }
  }
}

void socCalc(void* arg){
  SoCData* dataPtr = arg;
  SoCData data = *dataPtr;

  double vOC = *data.hvVoltage + 0.5 * (*data.hvCurrent);

  if (vOC <= 200) {
    *dataPtr->soc = 0;
    *data.soc=0;
  }
  else if (vOC <= 250) {
    if (temperature <= -10)
      *dataPtr->soc = 0.2 * vOC - 40;
    else
      *dataPtr->soc = 0;
  }
  else if (vOC <= 300) {
    if (temperature <= -10)
      *dataPtr->soc = 0.5 * vOC - 115;
    else if (temperature <= 0)
      *dataPtr->soc = 0.4 * vOC - 100;
    else if (temperature <= 25)
      *dataPtr->soc = 0.2*vOC - 50;
    else
      *dataPtr->soc = 0;
  }
  else if (vOC <= 350) {
    if (temperature <= -10)
      *dataPtr->soc = 1.3*vOC - 355;
    else if (temperature <= 0)
      *dataPtr->soc = 1.2 * vOC - 340;
    else if (temperature <= 25)
      *dataPtr->soc = vOC - 290;
    else if (temperature <= 45)
      *dataPtr->soc = vOC - 300;
  }
  else if (vOC < 400) {
    if (temperature <= 0)
      *dataPtr->soc = 0.4*vOC - 60;
    else if (temperature <= 25)
      *dataPtr->soc = 0.8*vOC - 220;
    else if (temperature <= 45)
      *dataPtr->soc = vOC - 300;
    else
      *dataPtr->soc = 100;
  }
  else if (vOC >= 400) {
    *dataPtr->soc = 100;
  }

  if (*dataPtr->soc > socMax || socMax == -1) {
    socFlag = true;
    socMax = *dataPtr->soc;
  }
  if (*dataPtr->soc < socMin || socMin == -1) {
    socMin = *dataPtr->soc;
    socFlag = true;
  }
  
}
  
void contactorControl(void* arg){
  ContactorData* dataPtr = arg; // dereference data
  ContactorData data = *dataPtr;

  if (*data.hvilInterruptAlarm != "NA" || *data.overcurrentAlarm != "NA" || *data.hvOutOfRangeAlarm != "NA" || *data.hvilAlarm != "NA") {
    *data.batOn = false;
  }

  if (*data.batOn == false){
    digitalWrite(CONTACTOR_PIN, LOW);
    *data.contactorIsOn = false; // acknowledge
  }
  
  else { // manage on/off state of contactor pin
    digitalWrite(CONTACTOR_PIN, HIGH);
    *data.contactorIsOn = true; // acknowledge
  }
  
}

void alarmControl(void* arg){
  AlarmData* dataPtr = arg; // dereference data
  AlarmData data = *dataPtr;
  unsigned long currentTime = millis();
  double scaledCurrent = *data.hvCurrent;//(double)(*tftData.hvCurrent)*50/1024 - 25;
  double scaledVoltage = *data.hvVoltage;//(double)(*tftData.hvVoltage)*450/1024;

  String stateArray[3] = {"NA", "ANA", "AA"};


  //hvil alarm
  if (!digitalRead(HVIL_PIN)) {
    *data.hvilInterruptAlarm = stateArray[0];
    *dataPtr->hvilAlarm = stateArray[0];
  }
  else if (*dataPtr->hvilAlarm == stateArray[0]) {
      *dataPtr->hvilAlarm = stateArray[1];
      *dataPtr->notAcknowledged = true;
  }
  //overcurrent alarm
  if ((scaledCurrent < -5 || scaledCurrent > 20) && *dataPtr->overcurrentAlarm == stateArray[0]) {
    *dataPtr->overcurrentAlarm = stateArray[1];
    *dataPtr->notAcknowledged = true;
  }
  else if(!(scaledCurrent < -5 || scaledCurrent > 20))
    *dataPtr->overcurrentAlarm = stateArray[0];

  //voltage alarm
  if ((scaledVoltage < 280 || scaledVoltage > 405) && *dataPtr->hvOutOfRangeAlarm == stateArray[0]) {
    *dataPtr->hvOutOfRangeAlarm = stateArray[1];
    *dataPtr->notAcknowledged = true;
  }
  else if (!(scaledVoltage < 280 || scaledVoltage > 405))
    *dataPtr->hvOutOfRangeAlarm = stateArray[0];
  
  if ((*dataPtr->hvilAlarm != stateArray[1]) && (*dataPtr->hvilInterruptAlarm != stateArray[1]) && 
		(*dataPtr->overcurrentAlarm != stateArray[1]) && (*dataPtr->hvOutOfRangeAlarm != stateArray[1])) {
    *dataPtr->notAcknowledged = false;
  }

  
  if(!*dataPtr->notAcknowledged) {
    if (*dataPtr->hvilAlarm == "ANA") {
      *data.hvilAlarm = stateArray[2];
    }
    if (*dataPtr->hvilInterruptAlarm == "ANA") {
      *data.hvilInterruptAlarm = stateArray[2];
    }
    if (*dataPtr->overcurrentAlarm == "ANA") {
      *data.overcurrentAlarm = stateArray[2];
    }
    if (*dataPtr->hvOutOfRangeAlarm == "ANA") {
      *data.hvOutOfRangeAlarm = stateArray[2];
    }
  }
  
}

void communicate(void* arg){
  
  MeasurementData* dataPtr = arg;
  byte buffer[4];
  Serial1.write(1);
  if (Serial1.available()) {
    Serial1.readBytes(buffer, 4);
  
    *dataPtr->hvVoltage = (buffer[0] << 8) | buffer[1];
    *dataPtr->hvCurrent = (buffer[2] << 8) | buffer[3];

    *dataPtr->hvVoltage = (double)(*dataPtr->hvVoltage)*450/1024;
    *dataPtr->hvCurrent = (double)(*dataPtr->hvCurrent)*50/1024 - 25;
  }
}

void dataLog(void* arg) {
  DataLogTaskData* dataPtr = arg;
  DataLogTaskData data = *(DataLogTaskData*)arg;

  

  if (*data.currentFlag || *data.resetFlag) {
    byte* byteCurrentMin = (byte*)data.currentMin;
    byte* byteCurrentMax = (byte*)data.currentMax;
    
    EEPROM.write(0, *byteCurrentMin);
    EEPROM.write(1, *(byteCurrentMin + 1));
    EEPROM.write(2, *(byteCurrentMin + 2));
    EEPROM.write(3, *(byteCurrentMin + 3));
    EEPROM.write(4, *byteCurrentMax);
    EEPROM.write(5, *(byteCurrentMax + 1));
    EEPROM.write(6, *(byteCurrentMax + 2));
    EEPROM.write(7, *(byteCurrentMax + 3));
    *dataPtr->currentFlag = false;
  }

  if (*data.voltageFlag || *data.resetFlag) {
    byte* byteVoltageMin = (byte*)data.voltageMin;
    byte* byteVoltageMax = (byte*)data.voltageMax;
    
    EEPROM.write(8, *byteVoltageMin);
    EEPROM.write(9, *(byteVoltageMin + 1));
    EEPROM.write(10, *(byteVoltageMin + 2));
    EEPROM.write(11, *(byteVoltageMin + 3));
    EEPROM.write(12, *byteVoltageMax);
    EEPROM.write(13, *(byteVoltageMax + 1));
    EEPROM.write(14, *(byteVoltageMax + 2));
    EEPROM.write(15, *(byteVoltageMax + 3));
    *dataPtr->voltageFlag = false;
  }

  if (*data.socFlag || *data.resetFlag) {
    byte* byteSoCMin = (byte*)data.socMin;
    byte* byteSocMax = (byte*)data.socMax;
    
    EEPROM.write(16, *byteSoCMin);
    EEPROM.write(17, *(byteSoCMin + 1));
    EEPROM.write(18, *(byteSocMax));
    EEPROM.write(19, *(byteSocMax + 1));
    *dataPtr->socFlag = false;
  }
  
  if (*data.resetFlag) {
    *dataPtr->resetFlag = false;
  }
}

void remoteTerminal(void* arg) {
  DataLogTaskData data = *(DataLogTaskData*)arg;
  
  if (Serial.available()) {
    char input = Serial.read();
    if (input == '1') {
      *data.resetFlag = true;
    }
    else if (input == '2') {
      byte byteMin[4];
      byte byteMax[4];
      double minimum;
      double maximum;
      byteMin[0] = EEPROM.read(0);
      byteMin[1] = EEPROM.read(1);
      byteMin[2] = EEPROM.read(2);
      byteMin[3] = EEPROM.read(3);
      byteMax[0] = EEPROM.read(4);
      byteMax[1] = EEPROM.read(5);
      byteMax[2] = EEPROM.read(6);
      byteMax[3] = EEPROM.read(7);

      minimum = *(double*)byteMin;
      maximum = *(double*)byteMax;
      Serial.print("[");
      Serial.print(maximum);
      Serial.print(", ");
      Serial.print(minimum);
      Serial.println("]");
    }
    else if (input == '3') {
      byte byteMin[4];
      byte byteMax[4];
      double minimum;
      double maximum;
      byteMin[0] = EEPROM.read(8);
      byteMin[1] = EEPROM.read(9);
      byteMin[2] = EEPROM.read(10);
      byteMin[3] = EEPROM.read(11);
      byteMax[0] = EEPROM.read(12);
      byteMax[1] = EEPROM.read(13);
      byteMax[2] = EEPROM.read(14);
      byteMax[3] = EEPROM.read(15);

      minimum = *(double*)byteMin;
      maximum = *(double*)byteMax;
      Serial.print("[");
      Serial.print(maximum);
      Serial.print(", ");
      Serial.print(minimum);
      Serial.println("]");
    }
  }
}

void setup() {
  StartUpData startUpData;
  startUpData.tftTask = &tftTask;
  startUpData.contactorTask = &contactorTask;
  startUpData.alarmTask = &alarmTask;
  startUpData.socTask = &socTask;
  startUpData.measureTask = &measureTask;
  startupTask.taskDataPtr = &startUpData;
  startupTask.myTask = &startup; // point to startup function and start the startup task
  startupTask.myTask(startupTask.taskDataPtr);
}

void hvilInterruptAlarmFunction() {
  hvilInterruptAlarm = "ANA";
  notAcknowledged = true;
}

void loop() {
  
}
