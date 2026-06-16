#include <SPI.h>
#include <RF24.h>
#include <avr/wdt.h>

#define CE_PIN 9
#define CSN_PIN 10
#define PIPE 0xE8E8F0F0E1ULL

#define LED_R 2
#define LED_G 3
#define LED_B 4

#define DIP1 A0
#define DIP2 A1
#define DIP3 A2

RF24 radio(CE_PIN,CSN_PIN);

struct Packet{
  uint8_t magic;
  uint8_t seq;
  uint8_t pgm;
  uint8_t pvw;
  uint16_t crc;
};

Packet pkt;

uint8_t camID=1;
uint8_t lastSeq=0xFF;
uint8_t lastState=255;

unsigned long lastRx=0;
unsigned long lastBlink=0;
bool blinkState=false;

uint16_t crc16(const uint8_t*d,size_t l){
  uint16_t c=0xFFFF;
  while(l--){
    c^=((uint16_t)*d++)<<8;
    for(uint8_t i=0;i<8;i++)
      c=(c&0x8000)?(c<<1)^0x1021:(c<<1);
  }
  return c;
}

uint8_t readID(){
  uint8_t id=0;
  if(digitalRead(DIP1)==LOW)id|=1;
  if(digitalRead(DIP2)==LOW)id|=2;
  if(digitalRead(DIP3)==LOW)id|=4;
  if(id==0)id=1;
  if(id>4)id=1;
  return id;
}

void radioInit(){
  radio.begin();
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1,PIPE);
  radio.startListening();
}

void setLed(uint8_t s){
  if(s==lastState)return;
  lastState=s;
  digitalWrite(LED_R,s==1);
  digitalWrite(LED_G,s==2);
  digitalWrite(LED_B,s==3);
}

void setup(){

  wdt_enable(WDTO_8S);

  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);

  pinMode(DIP1,INPUT_PULLUP);
  pinMode(DIP2,INPUT_PULLUP);
  pinMode(DIP3,INPUT_PULLUP);

  camID=readID();

  radioInit();

  setLed(3);
}

void loop(){

  wdt_reset();

  while(radio.available()){

    radio.read(&pkt,sizeof(pkt));

    if(pkt.magic!=0xA5)continue;

    uint8_t b[4]={pkt.magic,pkt.seq,pkt.pgm,pkt.pvw};

    if(crc16(b,4)!=pkt.crc)continue;

    if(lastSeq!=0xFF){

      uint8_t diff=pkt.seq-lastSeq;

      if(diff==0)continue;

      if(diff>128 && (millis()-lastRx)<2000)continue;

    }

    lastSeq=pkt.seq;
    lastRx=millis();

    bool pgm=bitRead(pkt.pgm,camID-1);
    bool pvw=bitRead(pkt.pvw,camID-1);

    if(pgm)
      setLed(1);
    else if(pvw)
      setLed(2);
    else
      setLed(3);

  }

  if(millis()-lastRx>700){

    if(millis()-lastBlink>250){

      lastBlink=millis();

      blinkState=!blinkState;

      digitalWrite(LED_R,LOW);
      digitalWrite(LED_G,LOW);
      digitalWrite(LED_B,blinkState);

    }

    if(millis()-lastRx>5000){

      radioInit();

    }

  }

}
