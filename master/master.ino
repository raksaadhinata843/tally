#include <SPI.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10
#define PIPE 0xE8E8F0F0E1ULL

RF24 radio(CE_PIN, CSN_PIN);

// PIN ASLI sesuai permintaan
const uint8_t PGM_PINS[6] = {A3, A2, A1, A0, 4, 3};
const uint8_t PVW_PINS[6] = {1, 5, 6, 7, 8, 2};

#define LED_MODE 13 // indikator mode: HIGH=VMIX aktif, LOW=PHYS aktif
#define SEND_INTERVAL 100UL
#define SERIAL_TIMEOUT 1000UL
#define DEBOUNCE_MS 20UL
#define MAX_TX_RETRIES 3

struct Packet {
  uint8_t magic;
  uint8_t seq;
  uint8_t pgm;
  uint8_t pvw;
  uint16_t crc;
};

Packet pkt;
unsigned long lastSend = 0;
unsigned long lastSerialRx = 0;
uint8_t lastPgm = 0, lastPvw = 0;
uint8_t vmixPgm = 0, vmixPvw = 0;
uint8_t seqNum = 0;

// CRC16-CCITT (0x1021)
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

uint8_t readPhysicalMask(const uint8_t pins[6]) {
  uint8_t m = 0;
  for (int i = 0; i < 6; i++) if (digitalRead(pins[i]) == LOW) bitSet(m, i);
  return m;
}

void handleSerialLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  if (s.startsWith("PGM:")) {
    vmixPgm = (uint8_t) strtol(s.substring(4).c_str(), NULL, 2);
  } else if (s.startsWith("PVW:")) {
    vmixPvw = (uint8_t) strtol(s.substring(4).c_str(), NULL, 2);
  } else if (s.startsWith("MASK:")) {
    int comma = s.indexOf(',');
    if (comma > 0) {
      vmixPgm = (uint8_t) strtol(s.substring(5, comma).c_str(), NULL, 2);
      vmixPvw = (uint8_t) strtol(s.substring(comma + 1).c_str(), NULL, 2);
    }
  }
  lastSerialRx = millis();
}

void setup() {
  Serial.begin(9600);
  delay(200);
  for (int i = 0; i < 6; i++) {
    pinMode(PGM_PINS[i], INPUT_PULLUP);
    pinMode(PVW_PINS[i], INPUT_PULLUP);
  }
  pinMode(LED_MODE, OUTPUT);

  radio.begin();
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(5, 3); // hardware retry config
  radio.openWritingPipe(PIPE);
  radio.stopListening();

  pkt.magic = 0xA5;
  pkt.seq = 0;
  Serial.println("TX AUTO-DETECT SECURE READY");
}

void loop() {
  // serial non-blocking
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleSerialLine(line);
  }

  // physical read with simple debounce
  static unsigned long lastPhysicalRead = 0;
  if (millis() - lastPhysicalRead >= DEBOUNCE_MS) {
    lastPhysicalRead = millis();
    uint8_t curPgm = readPhysicalMask(PGM_PINS);
    uint8_t curPvw = readPhysicalMask(PVW_PINS);
    if (curPgm != lastPgm || curPvw != lastPvw) {
      lastPgm = curPgm;
      lastPvw = curPvw;
    }
  }

  // decide mode (auto-detect): VMIX if recent serial
  bool vmixActive = (millis() - lastSerialRx) <= SERIAL_TIMEOUT;
  digitalWrite(LED_MODE, vmixActive ? HIGH : LOW);

  uint8_t usePgm = vmixActive ? vmixPgm : lastPgm;
  uint8_t usePvw = vmixActive ? vmixPvw : lastPvw;

  if (millis() - lastSend >= SEND_INTERVAL) {
    lastSend = millis();
    pkt.seq = seqNum++;
    pkt.pgm = usePgm;
    pkt.pvw = usePvw;
    uint8_t buf[4] = { pkt.magic, pkt.seq, pkt.pgm, pkt.pvw };
    pkt.crc = crc16_ccitt(buf, 4);

    bool ok = false;
    for (int attempt = 0; attempt < MAX_TX_RETRIES; attempt++) {
      ok = radio.write(&pkt, sizeof(pkt));
      if (ok) break;
      delay(5 + attempt * 5);
    }

    Serial.print(vmixActive ? "VMIX " : "PHYS ");
    Serial.print("SEQ="); Serial.print(pkt.seq);
    Serial.print(" PGM="); Serial.print(pkt.pgm, BIN);
    Serial.print(" PVW="); Serial.print(pkt.pvw, BIN);
    Serial.print(" TX_OK="); Serial.println(ok ? "1" : "0");
  }
}
