#include <Servo.h>

#define SERVOS 8

struct ServoState{
  short Angle[8];
};

Servo servos[SERVOS];
int target[SERVOS];
int current[SERVOS];

void setup(){
  Serial.begin(9600);
  for(int i = 0; i < SERVOS; i++){
    servos[i].attach(2 + i);
  }

  for(int i = 0; i < SERVOS; i++){
    servos[i].write(current[i] = target[i] = 20);
  }

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

int ReadRX(struct ServoState* st){
  int retVal = 0;
  short* a = st->Angle;

  for(int i = 0; i < 8; i++){
    if((a[i] = Serial.parseInt()) < 0){
      return 0;
    }
  }

  return 1;
}

int WriteTX(struct ServoState* st){
  short* a = st->Angle;
  char buf[128];
  sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d",
  a[0], a[1], a[2], a[3],
  a[4], a[5], a[6], a[7]
    );
  return Serial.println(buf);
}

void select(int i){
  unsigned int I = (unsigned int)i;

  for(int j = 0; j < 3; j++){
    if((I >> j) & 0x0001){
      digitalWrite(j + 11, HIGH); 
    }
    else{
      digitalWrite(j + 11, LOW);        
    }
  }
}

unsigned char serialTrans = 0xFF;
int running = 0;
int selectedServo = 0;
ServoState incoming;

void loop(){
  if(Serial.available() > 24){
    running = 1;
    ReadRX(&incoming);
  }
  if(!running) return;


  for(int i = SERVOS; i--;){
    target[i] = incoming.Angle[i];

    if(current[i] - target[i]){
      current[i] += (current[i] - target[i]) > 0 ? -1 : 1; //((target[i] - current[i]) >> 3);//
    }
    servos[i].write(current[i]);
  }

  //if(!(serialTrans--)){
  ServoState curr;
  for(int i = SERVOS; i--;){
    select(i);
    curr.Angle[i] = analogRead(0);
  }

  WriteTX(&curr);
  //}
  //delay(100);

}


