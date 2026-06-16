#include <SPI.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10
#define PIPE 0xE8E8F0F0E1ULL

#define LED_R 2
#define LED_G 3
#define LED_B 4

#define DIP1 A0
#define DIP2 A1
#define DIP3 A2

RF24 radio(CE_PIN, CSN_PIN);

struct Packet {
  uint8_t magic;
  uint8_t seq;
  uint8_t pgm;
  uint8_t pvw;
  uint16_t crc;
};

Packet pkt;
unsigned long lastRx = 0;
uint8_t camID = 1;
uint8_t lastSeq = 0xFF;  // accept first seq

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

uint8_t readID() {
  uint8_t id = 0;
  if (digitalRead(DIP1) == LOW) id |= 1;
  if (digitalRead(DIP2) == LOW) id |= 2;
  if (digitalRead(DIP3) == LOW) id |= 4;
  return (id == 0) ? 1 : id;
}

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

void setup() {
  Serial.begin(9600);
  delay(200);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(DIP1, INPUT_PULLUP);
  pinMode(DIP2, INPUT_PULLUP);
  pinMode(DIP3, INPUT_PULLUP);

  camID = readID();

  radio.begin();
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1, PIPE);
  radio.startListening();

  Serial.print("RX SECURE READY | CAM ID = ");
  Serial.println(camID);
}

void loop() {
  if (radio.available()) {
    radio.read(&pkt, sizeof(pkt));
    if (pkt.magic != 0xA5) {
      Serial.println("BAD MAGIC");
      return;
    }
    uint8_t buf[4] = { pkt.magic, pkt.seq, pkt.pgm, pkt.pvw };
    uint16_t c = crc16_ccitt(buf, 4);
    if (c != pkt.crc) {
      Serial.println("CRC FAIL");
      return;
    }
    // replay protection (seq difference handling with wrap)
    if (lastSeq != 0xFF) {
      uint8_t diff = pkt.seq - lastSeq;
      if (diff == 0) {
        Serial.println("REPLAY/DUPE");
        return;
      }
      if (diff > 128) {  // too old
        Serial.println("SEQ OLD");
        return;
      }
    }
    lastSeq = pkt.seq;
    lastRx = millis();

    bool pgm = bitRead(pkt.pgm, camID - 1);
    bool pvw = bitRead(pkt.pvw, camID - 1);

    Serial.print("SEQ=");
    Serial.print(pkt.seq);
    Serial.print(" PGM=");
    Serial.print(pkt.pgm, BIN);
    Serial.print(" PVW=");
    Serial.println(pkt.pvw, BIN);

    if (pgm) setLED(1, 0, 0);
    else if (pvw) setLED(0, 1, 0);
    else setLED(0, 0, 1);
  }

  // timeout blink if no packet recently
  if (millis() - lastRx > 1500) {
    bool blink = (millis() / 300) % 2;
    setLED(0, 0, blink);
  }
}
