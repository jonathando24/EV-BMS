//#include <TimerOne.h>


#define voltagePin 14 //A0
#define currentPin 15 //A1

volatile int16_t voltage = 69;
volatile int16_t current = 420;

struct MyStruct {
  void (*myTask)(void*);
  void* taskDataPtr;
  struct MyStruct* next;
  struct MyStruct* prev;
};
typedef struct MyStruct TCB;


typedef struct MeasurementTaskData {
  int* voltagePtr;
  int* currentPtr;
};

TCB measurementTask;
TCB communicationTask;

TCB* currentTaskPtr;

MeasurementTaskData measurementTaskData;


void measurement(void* arg){
  MeasurementTaskData* dataPtr = arg;
  *dataPtr->voltagePtr = analogRead(voltagePin);
  *dataPtr->currentPtr = analogRead(currentPin);
}

void communicate(void* arg){
  MeasurementTaskData* dataPtr = arg;
  byte buffer[4];
  buffer[0] = *dataPtr->voltagePtr >> 8;
  buffer[1] = *dataPtr->voltagePtr & 511;
  buffer[2] = *dataPtr->currentPtr >> 8;
  buffer[3] = *dataPtr->currentPtr & 511;
  Serial.write(buffer, 4);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1000);
  pinMode(voltagePin, INPUT_PULLUP);
  pinMode(currentPin, INPUT_PULLUP);
  measurementTaskData.voltagePtr = &voltage;
  measurementTaskData.currentPtr = &current;
  
  measurementTask.myTask = &measurement;
  measurementTask.taskDataPtr = &measurementTaskData;
  measurementTask.prev = NULL;
  measurementTask.next = NULL;
  
  communicationTask.myTask = &communicate;
  communicationTask.taskDataPtr = &measurementTaskData;
  communicationTask.prev = NULL;
  communicationTask.next = NULL;

}

void loop() {
  // put your main code here, to run repeatedly:
  MeasurementTaskData data;
  data.voltagePtr = &voltage;
  data.currentPtr = &current;

  unsigned long startTime = millis();
  //code
  currentTaskPtr = &measurementTask;

  while (currentTaskPtr != NULL){
    currentTaskPtr->myTask(currentTaskPtr->taskDataPtr);
    currentTaskPtr = currentTaskPtr->next;
  }

  while(millis() - startTime < 100);
}

void serialEvent() {
  if (Serial.available()){ 
    // get the new byte:
    int input = Serial.read();
    if (input == 1) {
      communicationTask.myTask(&measurementTaskData);
    }
  }
}
