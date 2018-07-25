#include <Arduino.h>
#include "wiring_private.h"
#include <SPI.h>
#include "FlashStorage.h"
#include "ZeroTimer.h"
#include "MemoryFree.h"

#if defined(SERIAL_BUFFER_SIZE) && SERIAL_BUFFER_SIZE == 256
//OK
#else
#error Wrong buffer size, change SERIAL_BUFFER_SIZE in RingBuffer.h
#endif

#define IWV          (0x0  << 3) // 24
#define V1WV         (0x1  << 3) // 24
#define V2WV         (0x2  << 3) // 24
#define ADC_CRC      (0x4  << 3) // 16
#define CTRL_CRC     (0x5  << 3) // 16
#define CNT_SNAPSHOT (0x7  << 3) // 16
#define CONFIG       (0x8  << 3) // 8
#define STATUS0      (0x9  << 3) // 8
#define LOCK         (0xA  << 3) // 8
#define SYNC_SNAP    (0xB  << 3) // 8
#define COUNTER0     (0xC  << 3) // 8
#define COUNTER1     (0xD  << 3) // 8
#define EMI_CTRL     (0xE  << 3) // 8
#define STATUS1      (0xF  << 3) // 8
#define TEMPOS       (0x18 << 3) // 8

#define DUMMY_MSG   0x00
#define LOCK_BYTE   0xCA
#define UNLOCK_BYTE 0x9C

#define POLY 0xEDB88320

#define B64_INVALID 66
const uint8_t b64_alphabet[] PROGMEM = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
const uint8_t b64_reverse_alphabet[] PROGMEM = {
  66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66,
  66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 62, 66, 66, 66, 63, 52, 53,
  54, 55, 56, 57, 58, 59, 60, 61, 66, 66, 66, 66, 66, 66, 66,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 66, 66, 66, 66, 66, 66, 26, 27, 28,
  29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 66, 66,
  66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66,
  66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66,
  66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66,
  66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66,
  66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66, 66,
  66, 66, 66, 66, 66, 66
};

const uint8_t PIN_INT = 8, PIN_CS = A2, PIN_BUTTON = 9, PIN_GREEN_LED = 13, PIN_RED_LED = 11, PIN_RELAY = 12, PIN_GPIO0_ESP = 6, PIN_RESET_ESP = 5, PIN_RX_EN_ESP = 7, PIN_CAN_ENABLE = 2, PIN_CAN = A0, PIN_WIFI = A1;
const uint32_t MIN_MEASURE_SAMPLES = 10, MAX_MEASURE_SAMPLES = 800, OVELOAD_PERIOD = 4200, RX_BUFFER_SIZE = 200, RAW_BUFFER_SIZE = 9 * 200;
const uint32_t DMA_CH_TX = 0, DMA_CH_RX = 1;
const uint8_t BUTTON_DEBOUNCE = 12;
const float RAW_TO_AMP = 1.0 / 170240.0, RAW_TO_VOLT = 1.0 / 13565.65, RAW_TO_HZ = 8000.0, FLOAT_INVALID = 0x7FC00000;
const int32_t MIN_ADC_RANGE = -5320000, MAX_ADC_RANGE = 5320000;

union floatValue {
  float value;
  uint8_t bytes[4];
};
union uint32Value {
  uint32_t value;
  uint8_t bytes[4];
};
union int32Value {
  int32_t value;
  uint8_t bytes[4];
};
union int64Value {
  int64_t value;
  uint8_t bytes[8];
};
struct samplesBuffer {
  boolean isReady, overload, overflow;
  uint32Value sampleCount;
  int64_t rmsCurrentSum, rmsVoltage1Sum, rmsVoltage2Sum, activePowerSum;
  floatValue iRms, v1Rms, v2Rms, hz, w, va, var, phi;
  uint8_t waveform[MAX_MEASURE_SAMPLES][9];
};
struct integralMeasureBuffer {
  boolean isReady;
  uint32_t sampleCount;
  int64_t rmsCurrentSum, rmsVoltage1Sum, activePowerSum;
};
struct rawDataBuffer {
  boolean isReady;
  uint32_t sampleCount;
  uint8_t rawData[RAW_BUFFER_SIZE];
};
struct b64_crc32_coder {
  uint32Value crc;
  uint8_t i;
  uint8_t a3[3];
  uint8_t a4[4];
};
struct serial {
  uint8_t myID;
  uint32_t bufferIndex;
  int8_t pinTxEn;
  b64_crc32_coder encoder, decoder;
  void (*preTransmitAction)(), (*postTransmitAction)();
  Stream* stream;
  uint8_t buffer[RX_BUFFER_SIZE];
};
struct settings {
  boolean valid;
  boolean automaticPowerOn;
  uint8_t canID;
  uint8_t wifiIP[4];
  uint8_t wifiGateway[4];
  uint8_t wifiSubnet[4];
  uint8_t wifiName[33];
  uint8_t wifiPassword[33];
  uint8_t wifiLoginPassword[33];
};
struct dmacdescriptor {
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
};

volatile samplesBuffer bufferLow = { .isReady = false, .overload = false, .overflow = false, .sampleCount = { .value = 0 }, .rmsCurrentSum = 0, .rmsVoltage1Sum = 0, .rmsVoltage2Sum = 0, .activePowerSum = 0 };
volatile samplesBuffer bufferHigh = { .isReady = false, .overload = false, .overflow = false, .sampleCount = { .value = 0 }, .rmsCurrentSum = 0, .rmsVoltage1Sum = 0, .rmsVoltage2Sum = 0, .activePowerSum = 0 };

volatile integralMeasureBuffer integralBufferLow = { .isReady = false, .sampleCount = 0, .rmsCurrentSum = 0, .rmsVoltage1Sum = 0, .activePowerSum = 0 };
volatile integralMeasureBuffer integralBufferHigh = { .isReady = false, .sampleCount = 0, .rmsCurrentSum = 0, .rmsVoltage1Sum = 0, .activePowerSum = 0 };

volatile rawDataBuffer rawBufferLow = { .isReady = false, .sampleCount = 0 };
volatile rawDataBuffer rawBufferHigh = { .isReady = false, .sampleCount = 0 };

serial serialUsb = { .myID = 0, .bufferIndex = 0, .pinTxEn = -1 };
serial serialCan = { .myID = 1, .bufferIndex = 0, .pinTxEn = -1 };
serial serialWifi = { .myID = 0, .bufferIndex = 0, .pinTxEn = PIN_RX_EN_ESP };

settings flashSettings;
FlashStorage(flashMemory, settings);

volatile boolean relay = false, interruptEnable = true, clearRawBuffer = false;
boolean hasCan = false, hasWifi = false, serialBridgeWifi = false, serialUsbStreaming = false, ledStatusOld = false, buttonStatus = false, buttonStatusOld = false;
uint32_t lastBufferSentTime = 0, lastUsbStreaming = 0;
uint8_t buttonTimer = 0;
floatValue kwh = { .value = 0 }, kvah = { .value = 0 }, kvarh = { .value = 0 };
float kwhCorrection = 0, kvahCorrection = 0, kvarhCorrection = 0;

volatile uint32_t overloadTimer = 0, measureCounter = 0;
int64_t iOld = 0, v1Old = 0, v2Old = 0;
int64_t iFiltOld = 0, v1FiltOld = 0, v2FiltOld = 0;
boolean signOld = false, dataSelection = false, dataSelectionIntegral = false, dataSelectionRaw = false;

uint8_t dmaTxBuffer[10] = { IWV | 0b100, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, dmaRxBuffer[10];
volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));

// TX on D4 (SERCOM2.0) and RX on D3 (SERCOM2.1)
Uart Serial2 (&sercom2, 3, 4, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM2_Handler() {
  Serial2.IrqHandler();
}

void disableCanReceiver() {
  pinMode(3, INPUT_PULLUP);
}

void enableCanReceiver() {
  pinPeripheral(3, PIO_SERCOM_ALT);
}

void setup() {
  flashSettings = flashMemory.read();
  checkSettings();

  pinMode(PIN_CAN, INPUT_PULLUP);
  pinMode(PIN_WIFI, INPUT_PULLUP);
  hasCan = !digitalRead(PIN_CAN);
  hasWifi = !digitalRead(PIN_WIFI);

  pinMode(PIN_INT, INPUT_PULLUP);
  pinMode(PIN_CS, OUTPUT); nativeDigitalWrite(PIN_CS, HIGH);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(PIN_GREEN_LED, OUTPUT);
  nativePinStrength(PIN_GREEN_LED, 1);
  nativeDigitalWrite(PIN_GREEN_LED, HIGH);

  pinMode(PIN_RED_LED, OUTPUT);
  nativePinStrength(PIN_RED_LED, 1);
  nativeDigitalWrite(PIN_RED_LED, HIGH);

  if (hasCan) {
    pinMode(PIN_CAN_ENABLE, OUTPUT);
    nativeDigitalWrite(PIN_CAN_ENABLE, LOW);
  }

  if (hasWifi) {
    pinMode(PIN_GPIO0_ESP, INPUT_PULLUP);
    pinMode(PIN_RESET_ESP, OUTPUT); nativeDigitalWrite(PIN_RESET_ESP, LOW);
    delay(25);
    pinMode(PIN_RESET_ESP, INPUT_PULLUP);

    pinMode(PIN_RX_EN_ESP, INPUT_PULLUP);
  }

  SPI.begin();
  SPI.beginTransaction(SPISettings(5600000, MSBFIRST, SPI_MODE3));
  while (readRegister(STATUS0, 1) & 0x01);
  do {
    writeRegister(LOCK, UNLOCK_BYTE);
    writeRegister(CONFIG, 0x00);
    writeRegister(EMI_CTRL, 0x55);
    writeRegister(LOCK, LOCK_BYTE);
  } while (readRegister(CONFIG, 1) != 0x00 || readRegister(EMI_CTRL, 1) != 0x55 || (readRegister(STATUS0, 1) & 0x04) == 0x00);

  if (flashSettings.valid && flashSettings.automaticPowerOn) relay = true;
  pinMode(PIN_RELAY, OUTPUT); nativeDigitalWrite(PIN_RELAY, relay);

  SerialUSB.begin(0);
  serialUsb.stream = &SerialUSB;
  initCoder(&serialUsb.decoder);
  serialUsb.preTransmitAction = NULL;
  serialUsb.postTransmitAction = NULL;

  Serial1.begin(230400);
  serialWifi.stream = &Serial1;
  initCoder(&serialWifi.decoder);
  serialWifi.preTransmitAction = NULL;
  serialWifi.postTransmitAction = NULL;

  Serial2.begin(115200);
  serialCan.stream = &Serial2;
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(4, PIO_SERCOM_ALT);
  if (flashSettings.valid) serialCan.myID = flashSettings.canID;
  initCoder(&serialCan.decoder);
  serialCan.preTransmitAction = disableCanReceiver;
  serialCan.postTransmitAction = enableCanReceiver;

  NVIC_EnableIRQ(DMAC_IRQn);
  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
  DMAC->WRBADDR.reg = (uint32_t)wrb;
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);

  DMAC->CHID.reg = DMAC_CHID_ID(DMA_CH_TX);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << DMA_CH_TX));
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGSRC(SERCOM4_DMAC_ID_TX) | DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK;
  descriptor_section[DMA_CH_TX].descaddr = 0;
  descriptor_section[DMA_CH_TX].dstaddr = (uint32_t) &SERCOM4->SPI.DATA.reg;
  descriptor_section[DMA_CH_TX].btcnt = sizeof(dmaTxBuffer);
  descriptor_section[DMA_CH_TX].srcaddr = (uint32_t)dmaTxBuffer + sizeof(dmaTxBuffer);
  descriptor_section[DMA_CH_TX].btctrl = DMAC_BTCTRL_VALID | DMAC_BTCTRL_SRCINC;

  DMAC->CHID.reg = DMAC_CHID_ID(DMA_CH_RX);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << DMA_CH_RX));
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGSRC(SERCOM4_DMAC_ID_RX) | DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK;
  descriptor_section[DMA_CH_RX].descaddr = 0;
  descriptor_section[DMA_CH_RX].srcaddr = (uint32_t) &SERCOM4->SPI.DATA.reg;
  descriptor_section[DMA_CH_RX].btcnt = sizeof(dmaRxBuffer);
  descriptor_section[DMA_CH_RX].dstaddr = (uint32_t)dmaRxBuffer + sizeof(dmaRxBuffer);
  descriptor_section[DMA_CH_RX].btctrl = DMAC_BTCTRL_VALID | DMAC_BTCTRL_DSTINC;

  attachInterrupt(digitalPinToInterrupt(PIN_INT), adcInterrupt, FALLING);
  TC.startTimer(5000, buttonTask);
  TCC.startTimer(50000, ledTask);
  lastBufferSentTime = millis();
}

void loop() {
  if (serialBridgeWifi) {
    detachInterrupt(digitalPinToInterrupt(PIN_INT));
    interruptEnable = false;
    relay = false;
    nativeDigitalWrite(PIN_RELAY, relay);

    pinMode(PIN_RESET_ESP, OUTPUT); nativeDigitalWrite(PIN_RESET_ESP, LOW);
    pinMode(PIN_GPIO0_ESP, OUTPUT); nativeDigitalWrite(PIN_GPIO0_ESP, LOW);
    delay(25);
    pinMode(PIN_RESET_ESP, INPUT_PULLUP);

    while (true) {
      if (SerialUSB.dtr()) processSerial(&serialUsb);
      else {
        if (SerialUSB.available()) Serial1.write(SerialUSB.read());
        if (Serial1.available()) SerialUSB.write(Serial1.read());
      }
    }
  }

  yeldTask();

  if (serialUsbStreaming) {
    if (rawBufferLow.isReady) {
      SerialUSB.write((uint8_t*)rawBufferLow.rawData, (uint32_t)rawBufferLow.sampleCount);
      rawBufferLow.isReady = false;
    }
    else if (rawBufferHigh.isReady) {
      SerialUSB.write((uint8_t*)rawBufferHigh.rawData, (uint32_t)rawBufferHigh.sampleCount);
      rawBufferHigh.isReady = false;
    }

    if (SerialUSB.available()) {
      switch (SerialUSB.read()) {
        case 0:
          lastUsbStreaming = millis();
          break;
        case '\n':
          serialUsbStreaming = false;
          break;
        case 20:
          relay = false;
          nativeDigitalWrite(PIN_RELAY, relay);
          break;
        case 21:
          relay = true;
          nativeDigitalWrite(PIN_RELAY, relay);
          break;
      }
    }

    if (millis() - lastUsbStreaming > 500) serialUsbStreaming = false;
  }
  else processSerial(&serialUsb);

  if (hasCan && processSerial(&serialCan)) {
    yeldTask();
    clearRawBuffer = true;
  }

  if (hasWifi && processSerial(&serialWifi)) {
    yeldTask();
    clearRawBuffer = true;
  }

  if (millis() - lastBufferSentTime > 2000UL) {
    lastBufferSentTime = millis();
    bufferLow.isReady = false;
    bufferHigh.isReady = false;
  }
}

// [0x01][0x00] [from] [to] [len3][len2][len1][len0] [... data ...] [crc3][crc2][crc1][crc0]
boolean processSerial(serial* s) {
  boolean returnValue = false;
  if (!s->stream->available()) return returnValue;

  uint8_t data = s->stream->read();
  if (data == '\n' || data == '\r') {
    if (s->decoder.i) {
      a4_to_a3(s->decoder.a3, s->decoder.a4);
      for (uint8_t k = 0; k < s->decoder.i - 1; k++) {
        s->buffer[s->bufferIndex++] = s->decoder.a3[k];
        if (s->bufferIndex >= RX_BUFFER_SIZE) s->bufferIndex = RX_BUFFER_SIZE - 1;
      }
    }

    if (s->bufferIndex >= 12) {
      for (uint32_t i = 0; i < s->bufferIndex - 4; i++) {
        s->decoder.crc.value ^= s->buffer[i];
        for (uint8_t k = 0; k < 8; k++) s->decoder.crc.value = s->decoder.crc.value & 1 ? (s->decoder.crc.value >> 1) ^ POLY : s->decoder.crc.value >> 1;
      }
      s->decoder.crc.value = ~s->decoder.crc.value;

      if (s->buffer[0] == 0x01 && s->buffer[1] == 0x00 &&
          (s->buffer[3] == s->myID || s->buffer[3] == 0x00) &&
          s->buffer[s->bufferIndex - 4] == s->decoder.crc.bytes[3] &&
          s->buffer[s->bufferIndex - 3] == s->decoder.crc.bytes[2] &&
          s->buffer[s->bufferIndex - 2] == s->decoder.crc.bytes[1] &&
          s->buffer[s->bufferIndex - 1] == s->decoder.crc.bytes[0]) {
        uint32Value len;
        len.bytes[3] = s->buffer[4];
        len.bytes[2] = s->buffer[5];
        len.bytes[1] = s->buffer[6];
        len.bytes[0] = s->buffer[7];

        if (len.value == s->bufferIndex - 12) {
          if (len.value == 0) writeVoid(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2]);
          else {
            returnValue = true;
            switch (s->buffer[8]) {
              case 0: //send board type
                if (len.value != 1) break;
                writeBoardType(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2]);
                break;

              case 10: // send status
                if (len.value != 1) break;
                writeStatus(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2]);
                break;

              case 11: // send status and measures
                if (len.value != 1) break;
                if (bufferLow.isReady) {
                  prepareBuffer(&bufferLow);
                  writeStatusMeasures(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2], &bufferLow);
                  bufferLow.isReady = false;
                  lastBufferSentTime = millis();
                }
                else if (bufferHigh.isReady) {
                  prepareBuffer(&bufferHigh);
                  writeStatusMeasures(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2], &bufferHigh);
                  bufferHigh.isReady = false;
                  lastBufferSentTime = millis();
                }
                else writeStatus(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2]);
                break;

              case 12: // send status, measures and waveform
                if (len.value != 1) break;
                if (bufferLow.isReady) {
                  prepareBuffer(&bufferLow);
                  writeStatusMeasuresWaveform(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2], &bufferLow);
                  bufferLow.isReady = false;
                  lastBufferSentTime = millis();
                }
                else if (bufferHigh.isReady) {
                  prepareBuffer(&bufferHigh);
                  writeStatusMeasuresWaveform(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2], &bufferHigh);
                  bufferHigh.isReady = false;
                  lastBufferSentTime = millis();
                }
                else writeStatus(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2]);
                break;

              case 20: // relay
                if (len.value != 2) break;
                relay = s->buffer[9] != 0;
                nativeDigitalWrite(PIN_RELAY, relay);
                writeBoolean(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2], relay);
                break;

              case 250: // send settings
                if (len.value != 1) break;
                writeSettings(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2], &flashSettings);
                break;

              case 251: // save settings
                if (len.value != 115) break;
                flashSettings.valid = true;
                flashSettings.automaticPowerOn = s->buffer[10];
                serialCan.myID = flashSettings.canID = s->buffer[11];
                memcpy(flashSettings.wifiIP, s->buffer + 12, 4);
                memcpy(flashSettings.wifiGateway, s->buffer + 16, 4);
                memcpy(flashSettings.wifiSubnet, s->buffer + 20, 4);
                memcpy(flashSettings.wifiName, s->buffer + 24, 33);
                memcpy(flashSettings.wifiPassword, s->buffer + 57, 33);
                memcpy(flashSettings.wifiLoginPassword, s->buffer + 90, 33);
                checkSettings();
                flashMemory.write(flashSettings);
                if (hasWifi) {
                  pinMode(PIN_RESET_ESP, OUTPUT); nativeDigitalWrite(PIN_RESET_ESP, LOW);
                  delay(25);
                  pinMode(PIN_RESET_ESP, INPUT_PULLUP);
                }
                writeBoolean(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2], true);
                break;

              case 253: // serial streaming
                if (len.value != 1) break;
                writeBoolean(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2], true);
                lastUsbStreaming = millis();
                {
                  floatValue f = { .value = RAW_TO_AMP };
                  SerialUSB.write(f.bytes[3]);
                  SerialUSB.write(f.bytes[2]);
                  SerialUSB.write(f.bytes[1]);
                  SerialUSB.write(f.bytes[0]);
                  f.value = RAW_TO_VOLT;
                  SerialUSB.write(f.bytes[3]);
                  SerialUSB.write(f.bytes[2]);
                  SerialUSB.write(f.bytes[1]);
                  SerialUSB.write(f.bytes[0]);
                  f.value = RAW_TO_VOLT;
                  SerialUSB.write(f.bytes[3]);
                  SerialUSB.write(f.bytes[2]);
                  SerialUSB.write(f.bytes[1]);
                  SerialUSB.write(f.bytes[0]);
                }
                serialUsbStreaming = true;
                break;

              case 254: // serial bridge wifi
                if (len.value != 1) break;
                if (hasWifi) serialBridgeWifi = true;
                writeBoolean(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2], true);
                break;

              case 255: // reset
                if (len.value != 1) break;
                writeBoolean(s->stream, &(s->encoder), s->preTransmitAction, s->postTransmitAction, s->pinTxEn, s->buffer[3], s->buffer[2], true);
                NVIC_SystemReset();
                break;
            }
          }
        }
      }
    }

    s->bufferIndex = 0;
    initCoder(&s->decoder);
  }
  else {
    uint8_t decoded = b64_reverse_alphabet[data];
    if (decoded != B64_INVALID) {
      s->decoder.a4[s->decoder.i++] = decoded;
      if (s->decoder.i >= 4) {
        a4_to_a3(s->decoder.a3, s->decoder.a4);
        for (uint8_t k = 0; k < 3; k++) {
          s->buffer[s->bufferIndex++] = s->decoder.a3[k];
          if (s->bufferIndex >= RX_BUFFER_SIZE) s->bufferIndex = RX_BUFFER_SIZE - 1;
        }
        s->decoder.i = 0;
      }
    }
  }

  return returnValue;
}

void prepareBuffer(volatile samplesBuffer* data) {
  if (data->overflow) {
    data->iRms.value = FLOAT_INVALID;
    data->v1Rms.value = FLOAT_INVALID;
    data->v2Rms.value = FLOAT_INVALID;
    data->hz.value = FLOAT_INVALID;
    data->w.value = FLOAT_INVALID;
    data->va.value = FLOAT_INVALID;
    data->var.value = FLOAT_INVALID;
    data->phi.value = FLOAT_INVALID;
  }
  else {
    data->iRms.value = sqrt((float)data->rmsCurrentSum / (float)data->sampleCount.value) * RAW_TO_AMP;
    data->v1Rms.value = sqrt((float)data->rmsVoltage1Sum / (float)data->sampleCount.value) * RAW_TO_VOLT;
    data->v2Rms.value = sqrt((float)data->rmsVoltage2Sum / (float)data->sampleCount.value) * RAW_TO_VOLT;
    data->hz.value = RAW_TO_HZ / (float)data->sampleCount.value;
    data->w.value = (float)data->activePowerSum / (float)data->sampleCount.value * RAW_TO_AMP * RAW_TO_VOLT;
    data->va.value = data->iRms.value * data->v1Rms.value;
    data->var.value = sqrt(data->va.value * data->va.value - data->w.value * data->w.value);
    data->phi.value = data->w.value / data->va.value;
  }
}

void yeldTask() {
  if (integralBufferLow.isReady) {
    float iRms = sqrt((float)integralBufferLow.rmsCurrentSum / (float)integralBufferLow.sampleCount) * RAW_TO_AMP;
    float v1Rms = sqrt((float)integralBufferLow.rmsVoltage1Sum / (float)integralBufferLow.sampleCount) * RAW_TO_VOLT;
    float w = (float)integralBufferLow.activePowerSum / (float)integralBufferLow.sampleCount * RAW_TO_AMP * RAW_TO_VOLT;
    float va = iRms * v1Rms;
    float var = sqrt(va * va - w * w);
    float dt = (float)integralBufferLow.sampleCount / RAW_TO_HZ;

    float add = w * dt / 3600.0 / 1000.0;
    float y = add - kwhCorrection;
    float t = kwh.value + y;
    kwhCorrection = (t - kwh.value) - y;
    kwh.value = t;

    add = va * dt / 3600.0 / 1000.0;
    y = add - kvahCorrection;
    t = kvah.value + y;
    kvahCorrection = (t - kvah.value) - y;
    kvah.value = t;

    add = var * dt / 3600.0 / 1000.0;
    y = add - kvarhCorrection;
    t = kvarh.value + y;
    kvarhCorrection = (t - kvarh.value) - y;
    kvarh.value = t;

    integralBufferLow.isReady = false;
  }
  else if (integralBufferHigh.isReady) {
    float iRms = sqrt((float)integralBufferHigh.rmsCurrentSum / (float)integralBufferHigh.sampleCount) * RAW_TO_AMP;
    float v1Rms = sqrt((float)integralBufferHigh.rmsVoltage1Sum / (float)integralBufferHigh.sampleCount) * RAW_TO_VOLT;
    float w = (float)integralBufferHigh.activePowerSum / (float)integralBufferHigh.sampleCount * RAW_TO_AMP * RAW_TO_VOLT;
    float va = iRms * v1Rms;
    float var = sqrt(va * va - w * w);
    float dt = (float)integralBufferHigh.sampleCount / RAW_TO_HZ;

    float add = w * dt / 3600.0 / 1000.0;
    float y = add - kwhCorrection;
    float t = kwh.value + y;
    kwhCorrection = (t - kwh.value) - y;
    kwh.value = t;

    add = va * dt / 3600.0 / 1000.0;
    y = add - kvahCorrection;
    t = kvah.value + y;
    kvahCorrection = (t - kvah.value) - y;
    kvah.value = t;

    add = var * dt / 3600.0 / 1000.0;
    y = add - kvarhCorrection;
    t = kvarh.value + y;
    kvarhCorrection = (t - kvarh.value) - y;
    kvarh.value = t;

    integralBufferHigh.isReady = false;
  }
}

void ledTask() {
  if (interruptEnable) nativeDigitalWrite(PIN_RED_LED, overloadTimer == 0);
  else nativeDigitalWrite(PIN_RED_LED, HIGH);

  if (interruptEnable) {
    nativeDigitalWrite(PIN_GREEN_LED, measureCounter < 399 || measureCounter > 401);
    measureCounter = 0;
    ledStatusOld = true;
  }
  else {
    ledStatusOld = !ledStatusOld;
    nativeDigitalWrite(PIN_GREEN_LED, ledStatusOld);
  }
}

void buttonTask() {
  boolean button = digitalRead(PIN_BUTTON);
  if (button && buttonTimer) buttonTimer--;
  else if (!button && buttonTimer < BUTTON_DEBOUNCE) buttonTimer++;

  if (buttonTimer == 0) buttonStatus = false;
  if (buttonTimer == BUTTON_DEBOUNCE) buttonStatus = true;

  if (interruptEnable && buttonStatus && !buttonStatusOld) {
    relay = !relay;
    nativeDigitalWrite(PIN_RELAY, relay);
  }
  buttonStatusOld = buttonStatus;
}

void checkSettings() {
  if (flashSettings.valid) {
    if (flashSettings.automaticPowerOn) flashSettings.automaticPowerOn = 1;
    if (flashSettings.canID < 1 || flashSettings.canID > 64) flashSettings.canID = 1;
    flashSettings.wifiName[sizeof(flashSettings.wifiName) - 1] = 0;
    flashSettings.wifiPassword[sizeof(flashSettings.wifiPassword) - 1] = 0;
    flashSettings.wifiLoginPassword[sizeof(flashSettings.wifiLoginPassword) - 1] = 0;
  }
  else {
    flashSettings.valid = true;
    flashSettings.automaticPowerOn = false;
    flashSettings.canID = 1;
    memset(flashSettings.wifiIP, 0, sizeof(flashSettings.wifiIP));
    memset(flashSettings.wifiGateway, 0, sizeof(flashSettings.wifiGateway));
    memset(flashSettings.wifiSubnet, 0, sizeof(flashSettings.wifiSubnet));
    memset(flashSettings.wifiName, 0, sizeof(flashSettings.wifiName));
    memset(flashSettings.wifiPassword, 0, sizeof(flashSettings.wifiPassword));
    memset(flashSettings.wifiLoginPassword, 0, sizeof(flashSettings.wifiLoginPassword));
  }
}

uint32_t readRegister(uint8_t reg, uint8_t size) {
  uint32_t out = 0;
  nativeDigitalWrite(PIN_CS, LOW);

  SPI.transfer(reg | 0b100);
  for (uint8_t i = 0; i < size; i++) {
    out <<= 8;
    out |= SPI.transfer(DUMMY_MSG);
  }

  nativeDigitalWrite(PIN_CS, HIGH);
  return out;
}

void writeRegister(uint8_t reg, uint8_t value) {
  nativeDigitalWrite(PIN_CS, LOW);

  SPI.transfer(reg);
  SPI.transfer(value);

  nativeDigitalWrite(PIN_CS, HIGH);
}

int64_t mulA(int64_t n) { // n * 0.99951171875
  return (n * 2048 - n) / 2048;
}

int64_t mulB(int64_t n) { // n * 0.9990234375
  return (n * 1024 - n) / 1024;
}

void DMAC_Handler() {
  uint8_t active_channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk;
  if (active_channel == DMA_CH_TX) {
    DMAC->CHID.reg = DMAC_CHID_ID(DMA_CH_TX);
    DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL | DMAC_CHINTENCLR_TERR | DMAC_CHINTENCLR_SUSP;
  }
  else if (active_channel == DMA_CH_RX) {
    DMAC->CHID.reg = DMAC_CHID_ID(DMA_CH_TX);
    DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL | DMAC_CHINTENCLR_TERR | DMAC_CHINTENCLR_SUSP;

    DMAC->CHID.reg = DMAC_CHID_ID(DMA_CH_RX);
    DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL | DMAC_CHINTENCLR_TERR | DMAC_CHINTENCLR_SUSP;

    nativeDigitalWrite(PIN_CS, HIGH);

    int64Value i = { .value = 0 };
    i.bytes[4] = dmaRxBuffer[1];
    i.bytes[3] = dmaRxBuffer[2];
    i.bytes[2] = dmaRxBuffer[3];
    if (i.bytes[4] & 0x80) i.value |= 0xFFFFFF0000000000;

    int64Value v1 = { .value = 0 };
    v1.bytes[4] = dmaRxBuffer[4];
    v1.bytes[3] = dmaRxBuffer[5];
    v1.bytes[2] = dmaRxBuffer[6];
    if (v1.bytes[4] & 0x80) v1.value |= 0xFFFFFF0000000000;

    int64Value v2 = { .value = 0 };
    v2.bytes[4] = dmaRxBuffer[7];
    v2.bytes[3] = dmaRxBuffer[8];
    v2.bytes[2] = dmaRxBuffer[9];
    if (v2.bytes[4] & 0x80) v2.value |= 0xFFFFFF0000000000;

    v2.value = -v2.value;

    if (!serialUsbStreaming || clearRawBuffer) {
      clearRawBuffer = false;
      rawBufferLow.isReady = false;
      rawBufferLow.sampleCount = 0;
      rawBufferHigh.isReady = false;
      rawBufferHigh.sampleCount = 0;
    }
    else if (serialUsbStreaming) {
      if (dataSelectionRaw) {
        rawBufferHigh.rawData[rawBufferHigh.sampleCount++] = i.bytes[4];
        rawBufferHigh.rawData[rawBufferHigh.sampleCount++] = i.bytes[3];
        rawBufferHigh.rawData[rawBufferHigh.sampleCount++] = i.bytes[2];
        rawBufferHigh.rawData[rawBufferHigh.sampleCount++] = v1.bytes[4];
        rawBufferHigh.rawData[rawBufferHigh.sampleCount++] = v1.bytes[3];
        rawBufferHigh.rawData[rawBufferHigh.sampleCount++] = v1.bytes[2];
        rawBufferHigh.rawData[rawBufferHigh.sampleCount++] = v2.bytes[4];
        rawBufferHigh.rawData[rawBufferHigh.sampleCount++] = v2.bytes[3];
        rawBufferHigh.rawData[rawBufferHigh.sampleCount++] = v2.bytes[2];
        if (rawBufferHigh.sampleCount >= RAW_BUFFER_SIZE) {
          if (rawBufferLow.isReady) {
            rawBufferHigh.isReady = false;
            rawBufferHigh.sampleCount = 0;
          }
          else {
            rawBufferHigh.isReady = true;
            dataSelectionRaw = false;
            rawBufferLow.sampleCount = 0;
          }
        }
      }
      else {
        rawBufferLow.rawData[rawBufferLow.sampleCount++] = i.bytes[4];
        rawBufferLow.rawData[rawBufferLow.sampleCount++] = i.bytes[3];
        rawBufferLow.rawData[rawBufferLow.sampleCount++] = i.bytes[2];
        rawBufferLow.rawData[rawBufferLow.sampleCount++] = v1.bytes[4];
        rawBufferLow.rawData[rawBufferLow.sampleCount++] = v1.bytes[3];
        rawBufferLow.rawData[rawBufferLow.sampleCount++] = v1.bytes[2];
        rawBufferLow.rawData[rawBufferLow.sampleCount++] = v2.bytes[4];
        rawBufferLow.rawData[rawBufferLow.sampleCount++] = v2.bytes[3];
        rawBufferLow.rawData[rawBufferLow.sampleCount++] = v2.bytes[2];
        if (rawBufferLow.sampleCount >= RAW_BUFFER_SIZE) {
          if (rawBufferHigh.isReady) {
            rawBufferLow.isReady = false;
            rawBufferLow.sampleCount = 0;
          }
          else {
            rawBufferLow.isReady = true;
            dataSelectionRaw = true;
            rawBufferHigh.sampleCount = 0;
          }
        }
      }
    }

    int64Value iFilt;
    iFilt.value = mulA(i.value) - mulA(iOld) + mulB(iFiltOld);
    iOld = i.value;
    iFiltOld = iFilt.value;

    int64Value v1Filt;
    v1Filt.value = mulA(v1.value) - mulA(v1Old) + mulB(v1FiltOld);
    v1Old = v1.value;
    v1FiltOld = v1Filt.value;

    int64Value v2Filt;
    v2Filt.value = mulA(v2.value) - mulA(v2Old) + mulB(v2FiltOld);
    v2Old = v2.value;
    v2FiltOld = v2Filt.value;

    iFilt.value /= 65536;
    v1Filt.value /= 65536;
    v2Filt.value /= 65536;

    boolean overload = i.value > (int64_t)MAX_ADC_RANGE * (int64_t)65536 || i.value < (int64_t)MIN_ADC_RANGE * (int64_t)65536 ||
                       v1.value > (int64_t)MAX_ADC_RANGE * (int64_t)65536 || v1.value < (int64_t)MIN_ADC_RANGE * (int64_t)65536 ||
                       v2.value > (int64_t)MAX_ADC_RANGE * (int64_t)65536 || v2.value < (int64_t)MIN_ADC_RANGE * (int64_t)65536;
    if (overload) overloadTimer = OVELOAD_PERIOD;
    else if (overloadTimer) overloadTimer--;

    if (dataSelection && bufferHigh.sampleCount.value < MAX_MEASURE_SAMPLES) {
      bufferHigh.waveform[bufferHigh.sampleCount.value][0] = iFilt.bytes[0];
      bufferHigh.waveform[bufferHigh.sampleCount.value][1] = iFilt.bytes[1];
      bufferHigh.waveform[bufferHigh.sampleCount.value][2] = iFilt.bytes[2];
      bufferHigh.waveform[bufferHigh.sampleCount.value][3] = v1Filt.bytes[0];
      bufferHigh.waveform[bufferHigh.sampleCount.value][4] = v1Filt.bytes[1];
      bufferHigh.waveform[bufferHigh.sampleCount.value][5] = v1Filt.bytes[2];
      bufferHigh.waveform[bufferHigh.sampleCount.value][6] = v2Filt.bytes[0];
      bufferHigh.waveform[bufferHigh.sampleCount.value][7] = v2Filt.bytes[1];
      bufferHigh.waveform[bufferHigh.sampleCount.value][8] = v2Filt.bytes[2];

      bufferHigh.rmsCurrentSum += iFilt.value * iFilt.value;
      bufferHigh.rmsVoltage1Sum += v1Filt.value * v1Filt.value;
      bufferHigh.rmsVoltage2Sum += v2Filt.value * v2Filt.value;

      bufferHigh.activePowerSum += v1Filt.value * iFilt.value;

      if (overload) bufferHigh.overload = true;

      bufferHigh.sampleCount.value++;
    }
    if (!dataSelection && bufferLow.sampleCount.value < MAX_MEASURE_SAMPLES) {
      bufferLow.waveform[bufferLow.sampleCount.value][0] = iFilt.bytes[0];
      bufferLow.waveform[bufferLow.sampleCount.value][1] = iFilt.bytes[1];
      bufferLow.waveform[bufferLow.sampleCount.value][2] = iFilt.bytes[2];
      bufferLow.waveform[bufferLow.sampleCount.value][3] = v1Filt.bytes[0];
      bufferLow.waveform[bufferLow.sampleCount.value][4] = v1Filt.bytes[1];
      bufferLow.waveform[bufferLow.sampleCount.value][5] = v1Filt.bytes[2];
      bufferLow.waveform[bufferLow.sampleCount.value][6] = v2Filt.bytes[0];
      bufferLow.waveform[bufferLow.sampleCount.value][7] = v2Filt.bytes[1];
      bufferLow.waveform[bufferLow.sampleCount.value][8] = v2Filt.bytes[2];

      bufferLow.rmsCurrentSum += iFilt.value * iFilt.value;
      bufferLow.rmsVoltage1Sum += v1Filt.value * v1Filt.value;
      bufferLow.rmsVoltage2Sum += v2Filt.value * v2Filt.value;

      bufferLow.activePowerSum += v1Filt.value * iFilt.value;

      if (overload) bufferLow.overload = true;

      bufferLow.sampleCount.value++;
    }

    boolean sign = v1Filt.value > 0;
    if (sign && !signOld) {
      if (dataSelection && bufferHigh.sampleCount.value > MIN_MEASURE_SAMPLES) {
        if (bufferHigh.sampleCount.value < MAX_MEASURE_SAMPLES) {
          if (dataSelectionIntegral) {
            integralBufferHigh.sampleCount += bufferHigh.sampleCount.value;
            integralBufferHigh.rmsCurrentSum += bufferHigh.rmsCurrentSum;
            integralBufferHigh.rmsVoltage1Sum += bufferHigh.rmsVoltage1Sum;
            integralBufferHigh.activePowerSum += bufferHigh.activePowerSum;

            if (!integralBufferLow.isReady) {
              integralBufferHigh.isReady = true;

              integralBufferLow.sampleCount = 0;
              integralBufferLow.rmsCurrentSum = 0;
              integralBufferLow.rmsVoltage1Sum = 0;
              integralBufferLow.activePowerSum = 0;

              dataSelectionIntegral = false;
            }
          }
          else {
            integralBufferLow.sampleCount += bufferHigh.sampleCount.value;
            integralBufferLow.rmsCurrentSum += bufferHigh.rmsCurrentSum;
            integralBufferLow.rmsVoltage1Sum += bufferHigh.rmsVoltage1Sum;
            integralBufferLow.activePowerSum += bufferHigh.activePowerSum;

            if (!integralBufferHigh.isReady) {
              integralBufferLow.isReady = true;

              integralBufferHigh.sampleCount = 0;
              integralBufferHigh.rmsCurrentSum = 0;
              integralBufferHigh.rmsVoltage1Sum = 0;
              integralBufferHigh.activePowerSum = 0;

              dataSelectionIntegral = true;
            }
          }
        }

        if (bufferLow.isReady) {
          bufferHigh.overload = false;
          bufferHigh.overflow = false;
          bufferHigh.sampleCount.value = 0;
          bufferHigh.rmsCurrentSum = 0;
          bufferHigh.rmsVoltage1Sum = 0;
          bufferHigh.rmsVoltage2Sum = 0;
          bufferHigh.activePowerSum = 0;
        }
        else {
          bufferHigh.overflow = bufferHigh.sampleCount.value >= MAX_MEASURE_SAMPLES;
          bufferHigh.isReady = true;

          bufferLow.overload = false;
          bufferLow.overflow = false;
          bufferLow.sampleCount.value = 0;
          bufferLow.rmsCurrentSum = 0;
          bufferLow.rmsVoltage1Sum = 0;
          bufferLow.rmsVoltage2Sum = 0;
          bufferLow.activePowerSum = 0;

          dataSelection = false;
        }
      }
      else if (!dataSelection && bufferLow.sampleCount.value > MIN_MEASURE_SAMPLES) {
        if (bufferLow.sampleCount.value < MAX_MEASURE_SAMPLES) {
          if (dataSelectionIntegral) {
            integralBufferHigh.sampleCount += bufferLow.sampleCount.value;
            integralBufferHigh.rmsCurrentSum += bufferLow.rmsCurrentSum;
            integralBufferHigh.rmsVoltage1Sum += bufferLow.rmsVoltage1Sum;
            integralBufferHigh.activePowerSum += bufferLow.activePowerSum;

            if (!integralBufferLow.isReady) {
              integralBufferHigh.isReady = true;

              integralBufferLow.sampleCount = 0;
              integralBufferLow.rmsCurrentSum = 0;
              integralBufferLow.rmsVoltage1Sum = 0;
              integralBufferLow.activePowerSum = 0;

              dataSelectionIntegral = false;
            }
          }
          else {
            integralBufferLow.sampleCount += bufferLow.sampleCount.value;
            integralBufferLow.rmsCurrentSum += bufferLow.rmsCurrentSum;
            integralBufferLow.rmsVoltage1Sum += bufferLow.rmsVoltage1Sum;
            integralBufferLow.activePowerSum += bufferLow.activePowerSum;

            if (!integralBufferHigh.isReady) {
              integralBufferLow.isReady = true;

              integralBufferHigh.sampleCount = 0;
              integralBufferHigh.rmsCurrentSum = 0;
              integralBufferHigh.rmsVoltage1Sum = 0;
              integralBufferHigh.activePowerSum = 0;

              dataSelectionIntegral = true;
            }
          }
        }

        if (bufferHigh.isReady) {
          bufferLow.overload = false;
          bufferLow.overflow = false;
          bufferLow.sampleCount.value = 0;
          bufferLow.rmsCurrentSum = 0;
          bufferLow.rmsVoltage1Sum = 0;
          bufferLow.rmsVoltage2Sum = 0;
          bufferLow.activePowerSum = 0;
        }
        else {
          bufferLow.overflow = bufferLow.sampleCount.value >= MAX_MEASURE_SAMPLES;
          bufferLow.isReady = true;

          bufferHigh.overload = false;
          bufferHigh.overflow = false;
          bufferHigh.sampleCount.value = 0;
          bufferHigh.rmsCurrentSum = 0;
          bufferHigh.rmsVoltage1Sum = 0;
          bufferHigh.rmsVoltage2Sum = 0;
          bufferHigh.activePowerSum = 0;

          dataSelection = true;
        }
      }
    }
    signOld = sign;

    measureCounter++;
  }
}

void adcInterrupt() {
  nativeDigitalWrite(PIN_CS, LOW);

  DMAC->CHID.reg = DMAC_CHID_ID(DMA_CH_TX);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  DMAC->CHID.reg = DMAC_CHID_ID(DMA_CH_RX);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}