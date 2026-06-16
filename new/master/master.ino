#include <SPI.h>
#include <RF24.h>
#include <avr/wdt.h>

#define DEBUG 0

#define CE_PIN 9
#define CSN_PIN 10
#define PIPE 0xE8E8F0F0E1ULL

const uint8_t PGM_PINS[4]={A3,A2,A1,A0};
const uint8_t PVW_PINS[4]={1,5,6,7};

#define LED_MODE 13
#define SEND_INTERVAL 50UL
#define HEARTBEAT 500UL
#define DEBOUNCE 5UL

RF24 radio(CE_PIN,CSN_PIN);

struct Packet{
  uint8_t magic;
  uint8_t seq;
  uint8_t pgm;
  uint8_t pvw;
  uint16_t crc;
};

Packet pkt;

char rxBuf[32];
uint8_t rxPos=0;

uint8_t vmixPgm=0,vmixPvw=0;
uint8_t phyPgm=0,phyPvw=0;
uint8_t lastSendPgm=0,lastSendPvw=0;
uint8_t seqNum=0;

unsigned long lastSerial=0;
unsigned long lastScan=0;
unsigned long lastTx=0;

uint16_t crc16(const uint8_t*d,size_t l){
  uint16_t c=0xFFFF;
  while(l--){
    c^=((uint16_t)*d++)<<8;
    for(uint8_t i=0;i<8;i++)
      c=(c&0x8000)?(c<<1)^0x1021:(c<<1);
  }
  return c;
}

uint8_t readMask(const uint8_t p[4]){
  uint8_t m=0;
  for(uint8_t i=0;i<4;i++)
    if(digitalRead(p[i])==LOW) bitSet(m,i);
  return m;
}

void parseLine(char *s){
  if(!strncmp(s,"PGM:",4)){
    vmixPgm=(uint8_t)strtol(s+4,NULL,2);
    lastSerial=millis();
  }else if(!strncmp(s,"PVW:",4)){
    vmixPvw=(uint8_t)strtol(s+4,NULL,2);
    lastSerial=millis();
  }else if(!strncmp(s,"MASK:",5)){
    char *c=strchr(s,',');
    if(c){
      *c=0;
      vmixPgm=(uint8_t)strtol(s+5,NULL,2);
      vmixPvw=(uint8_t)strtol(c+1,NULL,2);
      lastSerial=millis();
    }
  }
}

void pollSerial(){
  while(Serial.available()){
    char ch=Serial.read();
    if(ch=='\r') continue;
    if(ch=='\n'){
      rxBuf[rxPos]=0;
      if(rxPos) parseLine(rxBuf);
      rxPos=0;
    }else if(rxPos<31){
      rxBuf[rxPos++]=ch;
    }else{
      rxPos=0;
    }
  }
}

void radioInit(){
  radio.begin();
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(15,15);
  radio.openWritingPipe(PIPE);
  radio.stopListening();
}

void sendState(uint8_t pgm,uint8_t pvw){
  pkt.magic=0xA5;
  pkt.seq=seqNum++;
  pkt.pgm=pgm;
  pkt.pvw=pvw;
  uint8_t b[4]={pkt.magic,pkt.seq,pkt.pgm,pkt.pvw};
  pkt.crc=crc16(b,4);

  if(!radio.write(&pkt,sizeof(pkt))){
    radioInit();
  }

#if DEBUG
  Serial.print("TX ");
  Serial.print(pkt.seq);
  Serial.print(" ");
  Serial.print(pkt.pgm,BIN);
  Serial.print(" ");
  Serial.println(pkt.pvw,BIN);
#endif
}

void setup(){
  wdt_enable(WDTO_8S);
  Serial.begin(115200);

  for(uint8_t i=0;i<4;i++){
    pinMode(PGM_PINS[i],INPUT_PULLUP);
    pinMode(PVW_PINS[i],INPUT_PULLUP);
  }

  pinMode(LED_MODE,OUTPUT);
  radioInit();
}

void loop(){
  wdt_reset();

  pollSerial();

  unsigned long now=millis();

  if(now-lastScan>=DEBOUNCE){
    lastScan=now;
    phyPgm=readMask(PGM_PINS);
    phyPvw=readMask(PVW_PINS);
  }

  bool vmix=(now-lastSerial)<1000UL;
  digitalWrite(LED_MODE,vmix);

  uint8_t outPgm=vmix?vmixPgm:phyPgm;
  uint8_t outPvw=vmix?vmixPvw:phyPvw;

  bool changed=(outPgm!=lastSendPgm)||(outPvw!=lastSendPvw);

  if(changed || (now-lastTx)>=HEARTBEAT){
    if((now-lastTx)>=SEND_INTERVAL || changed){
      lastTx=now;
      lastSendPgm=outPgm;
      lastSendPvw=outPvw;
      sendState(outPgm,outPvw);
    }
  }
}
