#include <SPI.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10
#define PIPE 0xE8E8F0F0E1ULL

RF24 radio(CE_PIN, CSN_PIN);

// PIN ASLI sesuai permintaan
const uint8_t PGM_PINS[6] = {A3, A2, A1, A0, 4, 3};
const uint8_t PVW_PINS[6] = {1, 5, 6, 7, 8, 2};

struct Packet {
  uint8_t pgm;
  uint8_t pvw;
};

Packet pkt;
uint8_t lastPgm = 0, lastPvw = 0;
uint8_t vmixPgm = 0, vmixPvw = 0;

uint8_t readPhysicalMask(const uint8_t pins[6]) {
  uint8_t m = 0;
  for (int i = 0; i < 6; i++) if (digitalRead(pins[i]) == LOW) bitSet(m, i);
  return m;
}

void setup() {
  for (int i = 0; i < 6; i++) {
    pinMode(PGM_PINS[i], INPUT_PULLUP);
    pinMode(PVW_PINS[i], INPUT_PULLUP);
  }
  pinMode(LED_MODE, OUTPUT);
}

void loop() {
  // physical read with simple debounce
  static unsigned long lastPhysicalRead = 0;
  if (millis() - lastPhysicalRead >= DEBOUNCE_MS) {
    lastPhysicalRead = millis();
    uint8_t curPgm = readPhysicalMask(PGM_PINS);
    uint8_t curPvw = readPhysicalMask(PVW_PINS);
  }

  // decide mode (auto-detect): VMIX if recent serial
  bool vmixActive = (millis() - lastSerialRx) <= SERIAL_TIMEOUT;
  digitalWrite(LED_MODE, vmixActive ? HIGH : LOW);

  uint8_t usePgm = vmixActive ? vmixPgm : lastPgm;
  uint8_t usePvw = vmixActive ? vmixPvw : lastPvw;

  if (millis() - lastSend >= SEND_INTERVAL) {
    pkt.pgm = usePgm;
    pkt.pvw = usePvw;
    uint8_t buf[2] = { pkt.pgm, pkt.pvw };
  }
}
