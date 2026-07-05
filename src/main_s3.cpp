#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>

// ESP32-S3 DevKitC-1
// Pinout retenu pour le lidar Xiaomi LDS02RR / C102:
// - GPIO14 : PWM / MOT_EN
// - GPIO18 : RX ESP32-S3 <- TX lidar
// - GPIO17 : TX ESP32-S3 -> RX lidar (optionnel)
//
// Pourquoi ces GPIO:
// - GPIO17/18 correspondent à U1TXD/U1RXD sur le DevKitC-1
// - on évite GPIO19/20 car ils sont liés au USB D-/D+ sur cette carte
// - on évite aussi les pins de boot / flash / USB réservées

static const uint8_t PWM_PIN = 14;
static const uint8_t LIDAR_RX_PIN = 18;
static const uint8_t LIDAR_TX_PIN = 17;
static const uint8_t MOTOR_PWM_CHANNEL = 0;
static const uint8_t MOTOR_PWM_RES_BITS = 10;
static const uint16_t MOTOR_PWM_MAX_DUTY = (1u << MOTOR_PWM_RES_BITS) - 1;

static const uint16_t PWM_RANGE = 1023;
static const uint16_t PWM_START_DUTY = 512;
// Le moteur tient mieux à 20 kHz sur ce LDS: on démarre directement sur une
// fréquence haute et inaudible, plus stable pour le pilotage du rotor.
static const uint32_t PWM_START_FREQ = 20000;
static const uint32_t MOTOR_STABILIZE_MS = 3000;
static const uint32_t LIDAR_BAUD = 115200;
static const uint32_t SCAN_BAUDS[] = {9600, 19200, 38400, 57600, 115200, 230400};
static const uint8_t SCAN_BAUD_COUNT = sizeof(SCAN_BAUDS) / sizeof(SCAN_BAUDS[0]);
static const uint32_t SCAN_WINDOW_MS = 3500;
static const uint32_t RAW_FRAME_TIMEOUT_MS = 20;
static const uint32_t SNIFF_FLUSH_BYTES = 48;
static const uint32_t SNIFF_FLUSH_TIMEOUT_MS = 10;
static const uint16_t PACKET_SIZE = 47;
static const uint8_t LDS02_POINTS = 12;
static const uint8_t LDS55AA_POINTS = 8;
static const uint8_t LDS55AA_PACKET_SIZE = 36;
static const uint16_t TUYA_MAX_FRAME = 128;
static const uint16_t SCAN_BINS = 360;
// Seuil léger pour écarter les retours les plus faibles.
// On veut supprimer les parasites sans tuer les points utiles.
static const uint8_t L55AA_QUALITY_MIN = 60;
static const uint16_t L55AA_ISOLATED_JUMP_MM = 2500;
static const uint16_t L55AA_NEIGHBOR_AGREE_MM = 900;
static const uint8_t L55AA_NEIGHBOR_MIN = 2;
static const uint16_t MAP_HALF_CELLS = 20;
static const uint16_t MAP_CELL_MM = 300;
static const uint16_t MAP_SIZE = MAP_HALF_CELLS * 2 + 1;
static const uint32_t UART_MAP_PERIOD_MS = 1000;
static const uint32_t MOTOR_SWEEP_STEP_MS = 1500;
static const uint32_t WIFI_AP_BAUD = 115200;
static const char *AP_SSID = "LDS-C102-S3";
static const char *AP_PASSWORD = "lds12345";
static const char *STA_SSID = "Freebox-5B979A";
static const char *STA_PASSWORD = "9v237q6rbfqtb6fzq3vt7t";
static const IPAddress STA_IP(192, 168, 1, 220);
static const IPAddress STA_GW(192, 168, 1, 254);
static const IPAddress STA_MASK(255, 255, 255, 0);
static const uint32_t WIFI_RETRY_MS = 10000;
static const uint32_t WIFI_STATUS_EVERY_MS = 5000;
static const uint32_t WIFI_START_DELAY_MS = 1200;
static const int RX_BUFFER_CAPACITY = 1024;

HardwareSerial lidarSerial(1);
WebServer webServer(80);

static String usbLine;
static uint8_t packetBuf[TUYA_MAX_FRAME];
static uint8_t packetLen = 0;
static uint32_t lastRxByteMs = 0;
static uint32_t packetStartMs = 0;
static uint32_t lastMapMs = 0;
static uint32_t motorChangedMs = 0;
static uint32_t decodedPackets = 0;
static uint32_t rejectedPackets = 0;
static uint32_t badChecksumStreak = 0;
static uint32_t scanValidPackets = 0;
static uint32_t scanInvalidPackets = 0;
static uint16_t scanBestChecksumDelta = 0xFFFF;
static uint32_t scanWindowsDone = 0;
static uint32_t scanWindowStartedMs = 0;
static uint8_t scanBaudIndex = 4;
static bool scanInvert = false;
static bool scanMode = false;
static bool sniffMode = false;

static bool motorEnabled = true;
static bool motorInverted = false;
static bool lidarTxEnabled = false;
static bool lidarInverted = false;
static bool uartSummaryEnabled = true;
static bool rawLogEnabled = false;
static bool uartMapStreaming = false;
static bool motorSweepActive = false;
static bool motorSweepFineActive = false;
static uint8_t motorSweepIndex = 0;
static uint32_t motorSweepLastMs = 0;
static bool tapMode = false;
// Mode d'exploration brute du flux lidar:
// - on découpe les octets par début de trame 0x55 0xAA
// - on imprime la trame entière en hex/ASCII
// - on ajoute quelques hypothèses de checksum pour guider le décodage
static bool baseExploreMode = true;
static uint8_t baseBuf[128];
static uint8_t baseLen = 0;
static uint32_t baseLastMs = 0;
static uint32_t tapLastMs = 0;
static uint8_t tapBuf[96];
static uint8_t tapLen = 0;
static const uint16_t MOTOR_SWEEP_VALUES[] = {0, 100, 250, 512, 768, 1023};
static const uint8_t MOTOR_SWEEP_COUNT = sizeof(MOTOR_SWEEP_VALUES) / sizeof(MOTOR_SWEEP_VALUES[0]);
static const uint16_t MOTOR_SWEEP_FINE_VALUES[] = {0, 32, 64, 96, 128, 160, 192, 224, 250, 288, 320, 384, 448, 512, 640, 768, 896, 1023};
static const uint8_t MOTOR_SWEEP_FINE_COUNT = sizeof(MOTOR_SWEEP_FINE_VALUES) / sizeof(MOTOR_SWEEP_FINE_VALUES[0]);

static uint16_t pwmDuty = PWM_START_DUTY;
static uint32_t pwmFreq = PWM_START_FREQ;
static uint32_t lidarBaud = LIDAR_BAUD;
static bool lidarStarted = false;
static uint16_t scanDistanceMm[SCAN_BINS];
static uint8_t scanQuality[SCAN_BINS];
static uint32_t scanUpdatedMs[SCAN_BINS];
static uint8_t sniffBuf[64];
static uint8_t sniffLen = 0;
static uint32_t sniffStartMs = 0;
static uint16_t tuyaExpectedLen = 0;
static uint32_t wifiLastRetryMs = 0;
static uint32_t wifiLastStatusMs = 0;
static uint32_t wifiStaStartAtMs = 0;
static bool wifiStaStarted = false;
static bool mdnsStarted = false;
static wl_status_t wifiLastStatus = WL_IDLE_STATUS;
static String apSsid;
static void handleCommand(const String &lineIn);

enum FrameKind : uint8_t {
  FRAME_NONE = 0,
  FRAME_XV = 1,
  FRAME_TUYA = 2,
};

static FrameKind packetKind = FRAME_NONE;

static void printAsciiByte(uint8_t b) {
  Serial.write((b >= 32 && b <= 126) ? (char)b : '.');
}

static void printRawFrame(const uint8_t *p, uint8_t len, uint32_t startMs) {
  Serial.printf("[RX %lu] len=%u HEX=", (unsigned long)startMs, (unsigned)len);
  for (uint8_t i = 0; i < len; ++i) {
    if (i) Serial.write(' ');
    if (p[i] < 16) Serial.write('0');
    Serial.printf("%02X", p[i]);
  }
  Serial.print(F(" ASCII="));
  for (uint8_t i = 0; i < len; ++i) {
    printAsciiByte(p[i]);
  }
  Serial.println();
}

static uint16_t checksumGot(const uint8_t *p) {
  uint32_t chk32 = 0;
  for (uint8_t i = 0; i < 20; i += 2) {
    const uint16_t word = (uint16_t)p[i] | ((uint16_t)p[i + 1] << 8);
    chk32 = (chk32 << 1) + word;
  }
  return (uint16_t)((chk32 & 0x7FFF) + (chk32 >> 15));
}

static uint16_t checksumExpected(const uint8_t *p) {
  return (uint16_t)p[20] | ((uint16_t)p[21] << 8);
}

static bool packetChecksumOk(const uint8_t *p) {
  const uint16_t expected = checksumExpected(p);
  const uint16_t got = checksumGot(p);
  return (got & 0x7FFF) == expected;
}

static uint16_t checksumDelta(const uint8_t *p) {
  const uint16_t expected = checksumExpected(p);
  const uint16_t got = checksumGot(p);
  return (got >= expected) ? (got - expected) : (expected - got);
}

// Variante utile pour diagnostiquer un flux proche du format XV/Neato mais avec
// des mots 16 bits interprétés dans l'autre ordre d'octets.
static uint16_t checksumGotWordsBE(const uint8_t *p) {
  uint32_t chk32 = 0;
  for (uint8_t i = 0; i < 20; i += 2) {
    const uint16_t word = ((uint16_t)p[i] << 8) | (uint16_t)p[i + 1];
    chk32 = (chk32 << 1) + word;
  }
  return (uint16_t)((chk32 & 0x7FFF) + (chk32 >> 15));
}

static uint16_t checksumDeltaWordsBE(const uint8_t *p) {
  const uint16_t expected = checksumExpected(p);
  const uint16_t got = checksumGotWordsBE(p);
  return (got >= expected) ? (got - expected) : (expected - got);
}

// CRC8 table used by the ROBOTIS LDS-02 / LD19 family.
// The sensor sends 47-byte frames:
// 0x54 0x2C | speed(2) | start angle(2) | 12x( distance(2) + confidence(1) )
// | end angle(2) | timestamp(2) | crc(1)
static const uint8_t LDS02_CRC_TABLE[256] = {
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
  0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
  0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
  0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
  0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
  0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
  0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
  0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
  0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
  0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
  0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
  0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
  0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
  0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
  0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
  0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

static uint8_t lds02Crc8(const uint8_t *p, uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; ++i) {
    crc = LDS02_CRC_TABLE[(crc ^ p[i]) & 0xFF];
  }
  return crc;
}

static bool lds02PacketOk(const uint8_t *p, uint16_t len) {
  if (len != PACKET_SIZE) return false;
  if (p[0] != 0x54) return false;
  if (p[1] != 0x2C && p[1] != (uint8_t)(0x2C | (0x07 << 5))) return false;
  return lds02Crc8(p, (uint16_t)(len - 1)) == p[len - 1];
}

static uint16_t lds02ReadU16LE(const uint8_t *p, uint16_t index);

static bool lds02PacketPlausible(const uint8_t *p) {
  const uint16_t speed = lds02ReadU16LE(p, 2);
  const uint16_t startAngle100 = lds02ReadU16LE(p, 4);
  const uint16_t endAngle100 = lds02ReadU16LE(p, 42);
  const uint16_t angleDelta100 = (uint16_t)((endAngle100 + 36000 - startAngle100) % 36000);
  if (speed < 500 || speed > 8000) return false;
  if (startAngle100 >= 36000 || endAngle100 >= 36000) return false;
  if (angleDelta100 == 0 || angleDelta100 > 3000) return false;
  return true;
}

static uint16_t lds02ReadU16LE(const uint8_t *p, uint16_t index) {
  return (uint16_t)p[index] | ((uint16_t)p[index + 1] << 8);
}

// Plusieurs lidar "55 AA" encodent les angles en 1/64 de degré avec un biais
// fixe de 640 unités. On normalise ici avant toute interpolation.
static float lds55aaRawAngleToDeg(uint16_t rawAngle) {
  return ((float)rawAngle - 640.0f) / 64.0f;
}

static float lds55aaAngleSpanDeg(uint16_t startRaw, uint16_t endRaw) {
  int32_t delta64 = (int32_t)endRaw - (int32_t)startRaw;
  if (delta64 < 0) delta64 += 360 * 64;
  return (float)delta64 / 64.0f;
}

static float lds02AngleHundredthsToDeg(uint16_t angle100) {
  return (float)angle100 / 100.0f;
}

static float lds02AngleStepHundredths(uint16_t startAngle, uint16_t endAngle) {
  if (startAngle <= endAngle) {
    return (float)(endAngle - startAngle) / (float)(LDS02_POINTS - 1);
  }
  return (float)(36000 + endAngle - startAngle) / (float)(LDS02_POINTS - 1);
}

static void printLds02Packet(const uint8_t *p) {
  const uint16_t speed = lds02ReadU16LE(p, 2);
  const uint16_t startAngle100 = lds02ReadU16LE(p, 4);
  const uint16_t endAngle100 = lds02ReadU16LE(p, 42);
  const uint16_t timestamp = lds02ReadU16LE(p, 44);
  const float startDeg = lds02AngleHundredthsToDeg(startAngle100);
  const float endDeg = lds02AngleHundredthsToDeg(endAngle100);
  const float step100 = lds02AngleStepHundredths(startAngle100, endAngle100);
  Serial.printf("[LDS02] speed=%u start=%0.2f end=%0.2f ts=%u ",
                (unsigned)speed,
                startDeg,
                endDeg,
                (unsigned)timestamp);
  for (uint8_t i = 0; i < LDS02_POINTS; ++i) {
    const uint16_t off = 6 + (uint16_t)i * 3;
    const uint16_t distance = lds02ReadU16LE(p, off);
    const uint8_t confidence = p[off + 2];
    Serial.printf("| p%u d=%u c=%u ", (unsigned)i, (unsigned)distance, (unsigned)confidence);
  }
  Serial.printf("| step=%0.2f\n", step100 / 100.0f);
}

static void storePoint(float angleDeg, uint16_t distanceMm, uint8_t quality);

static void storeLds02Points(const uint8_t *p) {
  const uint16_t startAngle100 = lds02ReadU16LE(p, 4);
  const uint16_t endAngle100 = lds02ReadU16LE(p, 42);
  const float startDeg = lds02AngleHundredthsToDeg(startAngle100);
  const float stepDeg = lds02AngleStepHundredths(startAngle100, endAngle100) / 100.0f;

  for (uint8_t i = 0; i < LDS02_POINTS; ++i) {
    const uint16_t off = 6 + (uint16_t)i * 3;
    const uint16_t distance = lds02ReadU16LE(p, off);
    const uint8_t confidence = p[off + 2];
    storePoint(startDeg + stepDeg * (float)i, distance, confidence);
  }
}

static uint8_t tuyaChecksum(const uint8_t *p, uint16_t len) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < len; ++i) {
    sum += p[i];
  }
  return (uint8_t)(sum & 0xFF);
}

static bool tuyaPacketOk(const uint8_t *p, uint16_t len) {
  if (len < 7) return false;
  return tuyaChecksum(p, (uint16_t)(len - 1)) == p[len - 1];
}

static bool tuyaVersionLooksPlausible(uint8_t ver) {
  return ver == 0x00 || ver == 0x03;
}

static void printTuyaFrame(const uint8_t *p, uint16_t len, uint32_t startMs) {
  const uint8_t ver = p[2];
  const uint8_t cmd = p[3];
  const uint16_t dataLen = ((uint16_t)p[4] << 8) | p[5];
  const uint8_t expected = p[len - 1];
  const uint8_t got = tuyaChecksum(p, (uint16_t)(len - 1));
  Serial.printf("[TUYA %lu] len=%u ver=0x%02X cmd=0x%02X dlen=%u crc=%02X/%02X %s DATA=",
                (unsigned long)startMs,
                (unsigned)len,
                ver,
                cmd,
                (unsigned)dataLen,
                got,
                expected,
                (got == expected) ? "ok" : "bad");
  const uint16_t payloadStart = 6;
  const uint16_t payloadEnd = (len > 0) ? (uint16_t)(len - 1) : 0;
  for (uint16_t i = payloadStart; i < payloadEnd; ++i) {
    if (i > payloadStart) Serial.write(' ');
    if (p[i] < 16) Serial.write('0');
    Serial.printf("%02X", p[i]);
  }
  Serial.println();
}

static void resetPacket() {
  packetLen = 0;
  packetKind = FRAME_NONE;
  tuyaExpectedLen = 0;
}

static bool motorStable(uint32_t nowMs) {
  return motorEnabled && (uint32_t)(nowMs - motorChangedMs) >= MOTOR_STABILIZE_MS;
}

static void markMotorChange() {
  motorChangedMs = millis();
}

static uint16_t motorLogicalDuty() {
  const uint16_t out = motorEnabled ? pwmDuty : 0;
  return motorInverted ? (PWM_RANGE - out) : out;
}

static void configureMotorPwm() {
  const uint32_t freq = (pwmFreq == 0) ? 1 : pwmFreq;
  ledcSetup(MOTOR_PWM_CHANNEL, freq, MOTOR_PWM_RES_BITS);
  ledcAttachPin(PWM_PIN, MOTOR_PWM_CHANNEL);
  ledcWrite(MOTOR_PWM_CHANNEL, motorLogicalDuty());
}

static void refreshMotorPwm(uint32_t nowUs) {
  (void)nowUs;
  ledcWrite(MOTOR_PWM_CHANNEL, motorLogicalDuty());
}

static void setMotorEnabled(bool enabled) {
  motorEnabled = enabled;
  markMotorChange();
  configureMotorPwm();
  Serial.printf("[MOTOR] %s freq=%u duty=%u invertpwm=%u output=%u\n",
                motorEnabled ? "on" : "off",
                (unsigned)pwmFreq,
                pwmDuty,
                motorInverted ? 1 : 0,
                motorLogicalDuty());
}

static void setPwmDuty(uint16_t duty) {
  pwmDuty = constrain(duty, (uint16_t)0, PWM_RANGE);
  motorSweepActive = false;
  markMotorChange();
  configureMotorPwm();
  Serial.printf("[MOTOR] pwm=%u output=%u\n",
                pwmDuty,
                motorLogicalDuty());
}

static bool freqAllowed(uint32_t freq) {
  return freq == 1000 || freq == 5000 || freq == 10000 || freq == 20000;
}

static void setPwmFreq(uint32_t freq) {
  if (!freqAllowed(freq)) {
    Serial.println(F("[MOTOR] freq autorisee: 1000 5000 10000 20000"));
    return;
  }
  pwmFreq = freq;
  motorSweepActive = false;
  configureMotorPwm();
  Serial.printf("[MOTOR] freq=%u duty=%u output=%u\n",
                (unsigned)pwmFreq,
                pwmDuty,
                motorLogicalDuty());
}

static void setPwmInverted(bool inverted) {
  motorInverted = inverted;
  motorSweepActive = false;
  markMotorChange();
  configureMotorPwm();
  Serial.printf("[MOTOR] invertpwm=%u output=%u\n",
                motorInverted ? 1 : 0,
                motorLogicalDuty());
}

static void startMotorSweep() {
  motorSweepActive = true;
  motorSweepFineActive = false;
  motorSweepIndex = 0;
  motorSweepLastMs = 0;
  Serial.println(F("[MOTOR] sweep=on"));
}

static void startMotorSweepFine() {
  motorSweepActive = false;
  motorSweepFineActive = true;
  motorSweepIndex = 0;
  motorSweepLastMs = 0;
  Serial.println(F("[MOTOR] sweep=fine on"));
}

static void stopMotorSweep() {
  if (!motorSweepActive && !motorSweepFineActive) return;
  motorSweepActive = false;
  motorSweepFineActive = false;
  Serial.println(F("[MOTOR] sweep=off"));
}

static void pollMotorSweep(uint32_t nowMs) {
  if (!motorSweepActive) return;
  if ((uint32_t)(nowMs - motorSweepLastMs) < MOTOR_SWEEP_STEP_MS) return;
  motorSweepLastMs = nowMs;
  pwmDuty = MOTOR_SWEEP_VALUES[motorSweepIndex];
  markMotorChange();
  configureMotorPwm();
  Serial.printf("[MOTOR] sweep step=%u pwm=%u output=%u\n",
                (unsigned)motorSweepIndex,
                pwmDuty,
                motorLogicalDuty());
  motorSweepIndex = (uint8_t)((motorSweepIndex + 1) % MOTOR_SWEEP_COUNT);
}

static void pollMotorSweepFine(uint32_t nowMs) {
  if (!motorSweepFineActive) return;
  if ((uint32_t)(nowMs - motorSweepLastMs) < MOTOR_SWEEP_STEP_MS) return;
  motorSweepLastMs = nowMs;
  pwmDuty = MOTOR_SWEEP_FINE_VALUES[motorSweepIndex];
  markMotorChange();
  configureMotorPwm();
  Serial.printf("[MOTOR] sweepfine step=%u pwm=%u output=%u\n",
                (unsigned)motorSweepIndex,
                pwmDuty,
                motorLogicalDuty());
  motorSweepIndex = (uint8_t)((motorSweepIndex + 1) % MOTOR_SWEEP_FINE_COUNT);
}

static void pollMotorPwm(uint32_t nowMs) {
  (void)nowMs;
  refreshMotorPwm(micros());
}

static void startLidarSerial() {
  if (lidarStarted) {
    lidarSerial.end();
    lidarStarted = false;
  }
  lidarSerial.setRxBufferSize(RX_BUFFER_CAPACITY);
  lidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN,
                    lidarTxEnabled ? (int8_t)LIDAR_TX_PIN : (int8_t)-1,
                    lidarInverted);
  lidarStarted = true;
  Serial.printf("[LIDAR] begin baud=%u tx=%u inverted=%u rx=GPIO%d tx=GPIO%d\n",
                (unsigned)lidarBaud,
                lidarTxEnabled ? 1 : 0,
                lidarInverted ? 1 : 0,
                LIDAR_RX_PIN,
                lidarTxEnabled ? LIDAR_TX_PIN : -1);
}

static void startScanCombo() {
  scanMode = true;
  scanWindowStartedMs = millis();
  scanValidPackets = 0;
  scanInvalidPackets = 0;
  scanBestChecksumDelta = 0xFFFF;
  decodedPackets = 0;
  rejectedPackets = 0;
  badChecksumStreak = 0;
  lidarBaud = SCAN_BAUDS[scanBaudIndex];
  lidarInverted = scanInvert;
  uartSummaryEnabled = false;
  rawLogEnabled = false;
  uartMapStreaming = false;
  lidarSerial.end();
  lidarStarted = false;
  Serial.printf("[SCAN] start baud=%u invertuart=%u window=%lu\n",
                (unsigned)lidarBaud,
                lidarInverted ? 1 : 0,
                (unsigned long)SCAN_WINDOW_MS);
}

static void finishScanCombo(uint32_t nowMs) {
  Serial.printf("[SCAN] done baud=%u invertuart=%u valid=%u invalid=%u best_delta=%u elapsed=%lums\n",
                (unsigned)lidarBaud,
                lidarInverted ? 1 : 0,
                (unsigned)scanValidPackets,
                (unsigned)scanInvalidPackets,
                (unsigned)scanBestChecksumDelta,
                (unsigned long)(nowMs - scanWindowStartedMs));
  ++scanWindowsDone;
  ++scanBaudIndex;
  if (scanBaudIndex >= SCAN_BAUD_COUNT) {
    scanBaudIndex = 0;
    scanInvert = !scanInvert;
  }
  startScanCombo();
}

// Le mode sniff ne cherche pas à reconstruire un paquet complet.
// Il affiche simplement les octets bruts par rafales courtes pour vérifier
// si le capteur parle vraiment du XV/Neato, d'un flux escape/ASCII, ou d'un
// autre protocole qui ne commence pas par 0xFA.
static void flushSniff() {
  if (sniffLen == 0) {
    return;
  }
  Serial.printf("[SNIFF %lu] len=%u HEX=",
                (unsigned long)sniffStartMs,
                (unsigned)sniffLen);
  for (uint8_t i = 0; i < sniffLen; ++i) {
    if (i) Serial.write(' ');
    if (sniffBuf[i] < 16) Serial.write('0');
    Serial.printf("%02X", sniffBuf[i]);
  }
  Serial.print(F(" ASCII="));
  for (uint8_t i = 0; i < sniffLen; ++i) {
    printAsciiByte(sniffBuf[i]);
  }
  Serial.println();
  sniffLen = 0;
}

static void flushTap() {
  if (tapLen == 0) return;
  Serial.printf("[TAP %lu] len=%u BYTES=[",
                (unsigned long)tapLastMs,
                (unsigned)tapLen);
  for (uint8_t i = 0; i < tapLen; ++i) {
    if (i) Serial.write(',');
    if (tapBuf[i] < 16) Serial.write('0');
    Serial.printf("%02X", tapBuf[i]);
  }
  Serial.println(F("]"));
  tapLen = 0;
}

static uint8_t xor8(const uint8_t *p, uint8_t len) {
  uint8_t x = 0;
  for (uint8_t i = 0; i < len; ++i) x ^= p[i];
  return x;
}

static void flushBaseExplore() {
  if (baseLen == 0) return;

  uint8_t sum8 = 0;
  for (uint8_t i = 0; i < baseLen; ++i) {
    sum8 = (uint8_t)(sum8 + baseBuf[i]);
  }

  Serial.printf("[BASE %lu] len=%u HEX=",
                (unsigned long)baseLastMs,
                (unsigned)baseLen);
  for (uint8_t i = 0; i < baseLen; ++i) {
    if (i) Serial.write(' ');
    if (baseBuf[i] < 16) Serial.write('0');
    Serial.printf("%02X", baseBuf[i]);
  }
  Serial.print(F(" ASCII="));
  for (uint8_t i = 0; i < baseLen; ++i) {
    printAsciiByte(baseBuf[i]);
  }
  if (baseLen >= 4 && baseBuf[0] == 0x55 && baseBuf[1] == 0xAA) {
    Serial.printf(" hdr=%02X %02X ver=%02X cmd=%02X sum8=%02X xor8=%02X",
                  baseBuf[0],
                  baseBuf[1],
                  baseBuf[2],
                  baseBuf[3],
                  sum8,
                  xor8(baseBuf, baseLen));
    if (baseLen == LDS55AA_PACKET_SIZE && baseBuf[2] == 0x10 && baseBuf[3] == 0x08) {
      const uint16_t speed = lds02ReadU16LE(baseBuf, 4);
      const uint16_t startRaw16 = lds02ReadU16LE(baseBuf, 6);
      const uint16_t endRaw16 = lds02ReadU16LE(baseBuf, 32);
      const uint8_t crcByte = baseBuf[35];
      const uint8_t timestampByte = baseBuf[34];
      const float startDeg = lds55aaRawAngleToDeg(startRaw16);
      const float endDeg = lds55aaRawAngleToDeg(endRaw16);
      const float spanDeg = lds55aaAngleSpanDeg(startRaw16, endRaw16);
      const float stepDeg = spanDeg / (float)(LDS55AA_POINTS - 1);
      Serial.printf(" speed=%u start16=%u end16=%u start=%0.2f end=%0.2f span=%0.2f ts=%u crc=%02X step=%0.2f",
                    (unsigned)speed,
                    (unsigned)startRaw16,
                    (unsigned)endRaw16,
                    startDeg,
                    endDeg,
                    spanDeg,
                    (unsigned)timestampByte,
                    (unsigned)crcByte,
                    stepDeg);
      for (uint8_t i = 0; i < LDS55AA_POINTS; ++i) {
        const uint16_t off = 8 + (uint16_t)i * 3;
        const uint16_t distance = lds02ReadU16LE(baseBuf, off);
        const uint8_t quality = baseBuf[off + 2];
        Serial.printf(" | p%u d=%u q=%u", (unsigned)i, (unsigned)distance, (unsigned)quality);
      }
      for (uint8_t i = 0; i < LDS55AA_POINTS; ++i) {
        const uint16_t off = 8 + (uint16_t)i * 3;
        const uint16_t distance = lds02ReadU16LE(baseBuf, off);
        const uint8_t quality = baseBuf[off + 2];
        const float angleDeg = startDeg + stepDeg * (float)i;
        storePoint(angleDeg, distance, quality);
      }
      ++decodedPackets;
    } else {
      ++rejectedPackets;
    }
  } else {
    Serial.printf(" sum8=%02X xor8=%02X", sum8, xor8(baseBuf, baseLen));
    ++rejectedPackets;
  }
  Serial.println();
  baseLen = 0;
}

static void feedBaseExplore(uint8_t b, uint32_t nowMs) {
  if (!baseExploreMode) return;

  if (baseLen == 0) {
    if (b != 0x55) return;
    baseBuf[baseLen++] = b;
    baseLastMs = nowMs;
    return;
  }

  if (baseLen == 1) {
    if (b == 0xAA) {
      baseBuf[baseLen++] = b;
      baseLastMs = nowMs;
      return;
    }
    if (b == 0x55) {
      baseBuf[0] = 0x55;
      baseLen = 1;
      baseLastMs = nowMs;
      return;
    }
    baseLen = 0;
    return;
  }

  if (baseLen < LDS55AA_PACKET_SIZE && baseLen < sizeof(baseBuf)) {
    baseBuf[baseLen++] = b;
  } else {
    flushBaseExplore();
    baseLen = 0;
    if (b == 0x55) {
      baseBuf[0] = 0x55;
      baseLen = 1;
    }
    baseLastMs = nowMs;
    return;
  }

  if (baseLen == LDS55AA_PACKET_SIZE) {
    flushBaseExplore();
  }

  baseLastMs = nowMs;
}

static void storePoint(float angleDeg, uint16_t distanceMm, uint8_t quality) {
  if (distanceMm < 80 || distanceMm > 12000) return;
  if (quality < L55AA_QUALITY_MIN) return;
  while (angleDeg < 0.0f) angleDeg += 360.0f;
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  const uint16_t bin = (uint16_t)(angleDeg + 0.5f) % SCAN_BINS;
  const uint16_t offsets[] = {
    (uint16_t)((bin + SCAN_BINS - 2) % SCAN_BINS),
    (uint16_t)((bin + SCAN_BINS - 1) % SCAN_BINS),
    (uint16_t)((bin + 1) % SCAN_BINS),
    (uint16_t)((bin + 2) % SCAN_BINS),
  };
  uint8_t neighborAgree = 0;
  for (uint8_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); ++i) {
    const uint16_t neighborDistance = scanDistanceMm[offsets[i]];
    if (neighborDistance == 0) continue;
    const uint16_t delta = (neighborDistance > distanceMm) ? (neighborDistance - distanceMm) : (distanceMm - neighborDistance);
    if (delta <= L55AA_NEIGHBOR_AGREE_MM) {
      ++neighborAgree;
    }
  }
  if (neighborAgree < L55AA_NEIGHBOR_MIN && quality < 180) {
    return;
  }
  scanDistanceMm[bin] = distanceMm;
  scanQuality[bin] = quality;
  scanUpdatedMs[bin] = millis();
}

static void printDecodedPacket(const uint8_t *p) {
  const int angleIndex = (int)p[1] - 0xA0;
  const float baseAngle = (angleIndex >= 0) ? (float)(angleIndex * 4) : -1.0f;
  const uint16_t speedRaw = ((uint16_t)p[3] << 8) | p[2];
  Serial.printf("[LIDAR] pkt idx=%d angle=%0.1f raw_speed=%u ",
                angleIndex, baseAngle, (unsigned)speedRaw);
  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t off = 4 + i * 4;
    const uint16_t distance = ((uint16_t)(p[off + 1] & 0x3F) << 8) | p[off];
    const uint8_t quality = p[off + 2];
    Serial.printf("| p%u d=%u q=%u ", (unsigned)i, (unsigned)distance, (unsigned)quality);
  }
  Serial.println();
}

static void decodePacket(const uint8_t *p) {
  if (!lds02PacketOk(p, PACKET_SIZE)) {
    ++rejectedPackets;
    if (scanMode) ++scanInvalidPackets;
    ++badChecksumStreak;
    if (sniffMode) {
      printRawFrame(p, PACKET_SIZE, packetStartMs);
    }
    if (sniffMode) {
      Serial.printf("[LDS02] pkt invalid hdr=%02X %02X crc=%02X calc=%02X\n",
                    p[0],
                    p[1],
                    p[PACKET_SIZE - 1],
                    lds02Crc8(p, (uint16_t)(PACKET_SIZE - 1)));
    }
    return;
  }

  if (!lds02PacketPlausible(p)) {
    ++rejectedPackets;
    if (scanMode) ++scanInvalidPackets;
    ++badChecksumStreak;
    if (sniffMode || rawLogEnabled) {
      printRawFrame(p, PACKET_SIZE, packetStartMs);
    }
    Serial.printf("[LDS02] pkt implausible speed=%u start=%u end=%u\n",
                  (unsigned)lds02ReadU16LE(p, 2),
                  (unsigned)lds02ReadU16LE(p, 4),
                  (unsigned)lds02ReadU16LE(p, 42));
    return;
  }

  if (rawLogEnabled || sniffMode) {
    printRawFrame(p, PACKET_SIZE, packetStartMs);
  }

  printLds02Packet(p);
  storeLds02Points(p);

  ++decodedPackets;
  if (scanMode) {
    ++scanValidPackets;
  }
  badChecksumStreak = 0;
}

static void feedByte(uint8_t b) {
  if (packetLen == 0) {
    if (b == 0x54) {
      packetBuf[packetLen++] = b;
      packetStartMs = millis();
    }
    return;
  }

  if (packetLen == 1) {
    if (b != 0x2C) {
      if (b == 0x54) {
        packetBuf[0] = b;
        packetLen = 1;
        packetStartMs = millis();
      } else {
        resetPacket();
      }
      return;
    }
  }

  if (packetLen < PACKET_SIZE) {
    packetBuf[packetLen++] = b;
  } else {
    resetPacket();
    return;
  }

  if (packetLen == PACKET_SIZE) {
    decodePacket(packetBuf);
    resetPacket();
    return;
  }

  if (packetLen >= 2 && b == 0x54) {
    packetBuf[0] = 0x54;
    packetLen = 1;
    packetStartMs = millis();
  }
}

static void printUartMap() {
  char grid[MAP_SIZE][MAP_SIZE];
  for (uint16_t y = 0; y < MAP_SIZE; ++y) {
    for (uint16_t x = 0; x < MAP_SIZE; ++x) grid[y][x] = ' ';
  }

  const int16_t center = MAP_HALF_CELLS;
  for (uint16_t i = 0; i < MAP_SIZE; ++i) {
    grid[center][i] = '-';
    grid[i][center] = '|';
  }
  grid[center][center] = '+';

  uint16_t plotted = 0;
  for (uint16_t deg = 0; deg < SCAN_BINS; ++deg) {
    const uint16_t distance = scanDistanceMm[deg];
    if (distance == 0) continue;
    const float theta = ((float)deg - 90.0f) * DEG_TO_RAD;
    const int16_t x = (int16_t)lroundf(cosf(theta) * (float)distance / (float)MAP_CELL_MM);
    const int16_t y = (int16_t)lroundf(sinf(theta) * (float)distance / (float)MAP_CELL_MM);
    const int16_t gx = center + x;
    const int16_t gy = center - y;
    if (gx < 0 || gy < 0 || gx >= (int16_t)MAP_SIZE || gy >= (int16_t)MAP_SIZE) continue;
    char &cell = grid[gy][gx];
    if (gx == center && gy == center) {
      cell = '+';
    } else if (cell == ' ' || cell == '-' || cell == '|') {
      cell = '*';
    } else if (cell == '*') {
      cell = '#';
    }
    ++plotted;
  }

  Serial.printf("[UARTMAP] points=%u plotted=%u cell=%umm radius=%umm decoded=%u rejected=%u\n",
                (unsigned)plotted,
                (unsigned)plotted,
                (unsigned)MAP_CELL_MM,
                (unsigned)(MAP_HALF_CELLS * MAP_CELL_MM),
                (unsigned)decodedPackets,
                (unsigned)rejectedPackets);
  for (uint16_t y = 0; y < MAP_SIZE; ++y) {
    Serial.write('[');
    for (uint16_t x = 0; x < MAP_SIZE; ++x) Serial.write(grid[y][x]);
    Serial.println(']');
  }
}

static const char *wifiStatusText(wl_status_t status) {
  switch (status) {
    case WL_NO_SHIELD: return "no_shield";
    case WL_IDLE_STATUS: return "idle";
    case WL_NO_SSID_AVAIL: return "no_ssid";
    case WL_SCAN_COMPLETED: return "scan_done";
    case WL_CONNECTED: return "connected";
    case WL_CONNECT_FAILED: return "connect_failed";
    case WL_CONNECTION_LOST: return "lost";
    case WL_DISCONNECTED: return "disconnected";
    default: return "unknown";
  }
}

static void beginStaConnection(const char *reason) {
  Serial.printf("[WIFI] begin ssid=%s reason=%s\n", STA_SSID, reason);
  WiFi.begin(STA_SSID, STA_PASSWORD);
}

static void handleRoot() {
  webServer.send(200, F("text/html"), F(
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>LDS C102</title>"
      "<style>"
      "body{margin:0;background:#0b0f14;color:#e8eef2;font-family:system-ui,Arial,sans-serif}"
      "main{max-width:1100px;margin:auto;padding:14px}"
      "canvas{width:100%;aspect-ratio:1;background:#05070a;border:1px solid #28303a;display:block}"
      "h2{margin:0 0 8px 0;font-size:22px}"
      ".meta{display:flex;gap:14px;flex-wrap:wrap;font:12px ui-monospace,monospace;color:#aeb8c2;margin:8px 0 12px}"
      ".bar{display:flex;gap:8px;flex-wrap:wrap;margin:10px 0}"
      "button,input{font:inherit;padding:8px 10px}"
      ".stat{font:12px ui-monospace,monospace;white-space:pre-wrap;color:#aeb8c2}"
      "</style></head><body><main>"
      "<h2>LDS C102 - carte 2D</h2>"
      "<div id='meta' class='meta'>chargement...</div>"
      "<canvas id='c' width='820' height='820'></canvas>"
      "<div class='bar'>"
      "<button onclick=\"cmd('motor on')\">motor on</button>"
      "<button onclick=\"cmd('motor off')\">motor off</button>"
      "<button onclick=\"cmd('pwm 250')\">pwm 250</button>"
      "<button onclick=\"cmd('freq 20000')\">freq 20k</button>"
      "<button onclick=\"cmd('invertpwm 1')\">invertpwm 1</button>"
      "<button onclick=\"toggleHalf()\">half on/off</button>"
      "<button onclick=\"cmd('status')\">status</button>"
      "<input id='q' placeholder='commande'><button onclick=\"cmd(q.value)\">send</button>"
      "</div><div id='s' class='stat'></div>"
      "<script>"
      "const c=document.getElementById('c'),x=c.getContext('2d'),s=document.getElementById('s'),q=document.getElementById('q'),m=document.getElementById('meta');"
      "let halfOnly=true;"
      "async function cmd(v){if(!v)return;await fetch('/cmd?c='+encodeURIComponent(v));}"
      "function toggleHalf(){halfOnly=!halfOnly;}"
      "function draw(d){const w=c.width,h=c.height,cx=w/2,cy=h/2,max=6000,scale=w/(max*2.05);"
      "x.fillStyle='#05070a';x.fillRect(0,0,w,h);x.strokeStyle='#25303a';x.lineWidth=1;"
      "for(let r=1000;r<=6000;r+=1000){x.beginPath();x.arc(cx,cy,r*scale,0,7);x.stroke();}"
      "for(let a=0;a<360;a+=30){const r=(a-90)*Math.PI/180;x.beginPath();x.moveTo(cx,cy);x.lineTo(cx+Math.cos(r)*cx,cy+Math.sin(r)*cy);x.stroke();}"
      "x.beginPath();x.moveTo(cx,0);x.lineTo(cx,h);x.moveTo(0,cy);x.lineTo(w,cy);x.stroke();"
      "let n=0;for(const p of d.points){if(halfOnly&&p.a>=180)continue;const a=(p.a-90)*Math.PI/180,rr=p.d*scale;"
      "const q=Math.max(0,Math.min(255,p.q*4));x.fillStyle='rgb('+(70+q)+','+(210-q/2)+','+(125-q/4)+')';"
      "x.fillRect(cx+Math.cos(a)*rr-2,cy+Math.sin(a)*rr-2,4,4);"
      "x.strokeStyle='rgba(64,209,125,0.15)';x.beginPath();x.moveTo(cx,cy);x.lineTo(cx+Math.cos(a)*rr,cy+Math.sin(a)*rr);x.stroke();n++;}"
      "x.fillStyle='#f2c14e';x.beginPath();x.arc(cx,cy,5,0,7);x.fill();"
      "m.textContent=`points=${n} decoded=${d.decoded} rejected=${d.rejected} motor=${d.motor?'on':'off'} stable=${d.stable?'yes':'no'} `+"
      "`pwm=${d.pwm}/1023 freq=${d.freq} baud=${d.baud}`;"
      "s.textContent=`points=${n} motor=${d.motor} stable=${d.stable} pwm=${d.pwm}/1023 freq=${d.freq}\\n`+"
      "`baud=${d.baud} decoded=${d.decoded} rejected=${d.rejected} ageMs=${d.age}`;}"
      "async function tick(){try{const r=await fetch('/scan');draw(await r.json());}catch(e){s.textContent=e;}setTimeout(tick,250);}tick();"
      "</script></main></body></html>"));
}

static void handleScan() {
  const uint32_t nowMs = millis();
  String json;
  json.reserve(4096);
  json += F("{\"motor\":");
  json += motorEnabled ? 1 : 0;
  json += F(",\"stable\":");
  json += motorStable(nowMs) ? 1 : 0;
  json += F(",\"pwm\":");
  json += pwmDuty;
  json += F(",\"freq\":");
  json += pwmFreq;
  json += F(",\"baud\":");
  json += lidarBaud;
  json += F(",\"decoded\":");
  json += decodedPackets;
  json += F(",\"rejected\":");
  json += rejectedPackets;
  json += F(",\"age\":");
  json += nowMs;
  json += F(",\"points\":[");

  uint16_t sent = 0;
  for (uint16_t i = 0; i < SCAN_BINS; ++i) {
    if (scanDistanceMm[i] == 0) continue;
    if ((uint32_t)(nowMs - scanUpdatedMs[i]) > 15000) continue;
    if (sent > 0) json += ',';
    json += F("{\"a\":");
    json += i;
    json += F(",\"d\":");
    json += scanDistanceMm[i];
    json += F(",\"q\":");
    json += scanQuality[i];
    json += '}';
    ++sent;
  }
  json += F("]}");
  webServer.send(200, F("application/json"), json);
}

static void handleCmd() {
  if (!webServer.hasArg("c")) {
    webServer.send(400, F("text/plain"), F("missing c"));
    return;
  }
  handleCommand(webServer.arg("c"));
  webServer.send(200, F("text/plain"), F("ok"));
}

static void setupWifi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setAutoConnect(true);
  WiFi.setSleep(false);
  WiFi.hostname("lds-c102");
  apSsid = String(AP_SSID);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  if (WiFi.config(STA_IP, STA_GW, STA_MASK)) {
    Serial.printf("[WIFI] static_ip=%s gw=%s mask=%s\n",
                  STA_IP.toString().c_str(),
                  STA_GW.toString().c_str(),
                  STA_MASK.toString().c_str());
  } else {
    Serial.println(F("[WIFI] static_ip config failed"));
  }
  WiFi.softAP(apSsid.c_str(), AP_PASSWORD);
  wifiStaStartAtMs = millis() + WIFI_START_DELAY_MS;
  wifiStaStarted = false;
  wifiLastRetryMs = millis();
  wifiLastStatus = WiFi.status();

  webServer.on("/", handleRoot);
  webServer.on("/map", handleRoot);
  webServer.on("/scan", handleScan);
  webServer.on("/cmd", handleCmd);
  webServer.begin();

  Serial.printf("[WIFI] AP=%s pass=%s ap_ip=%s\n",
                apSsid.c_str(),
                AP_PASSWORD,
                WiFi.softAPIP().toString().c_str());
  Serial.printf("[WIFI] STA=%s\n", STA_SSID);
  Serial.println(F("[BASE] default=on"));
}

static void ensureMdns() {
  if (mdnsStarted || WiFi.status() != WL_CONNECTED) return;
  if (MDNS.begin("lds-c102")) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
    Serial.println(F("[WIFI] mDNS=http://lds-c102.local"));
  } else {
    Serial.println(F("[WIFI] mDNS failed"));
  }
}

static void pollWifi(uint32_t nowMs) {
  const wl_status_t status = WiFi.status();
  if (status != wifiLastStatus) {
    wifiLastStatus = status;
    Serial.printf("[WIFI] status=%s\n", wifiStatusText(status));
    if (status == WL_CONNECTED) {
      Serial.printf("[WIFI] sta_ip=%s rssi=%d ap_ip=%s\n",
                    WiFi.localIP().toString().c_str(),
                    WiFi.RSSI(),
                    WiFi.softAPIP().toString().c_str());
    }
  }

  if (status == WL_CONNECTED) {
    ensureMdns();
    if ((uint32_t)(nowMs - wifiLastStatusMs) >= WIFI_STATUS_EVERY_MS) {
      wifiLastStatusMs = nowMs;
      Serial.printf("[WIFI] sta_ip=%s rssi=%d ap_ip=%s\n",
                    WiFi.localIP().toString().c_str(),
                    WiFi.RSSI(),
                    WiFi.softAPIP().toString().c_str());
    }
    return;
  }

  if (!wifiStaStarted && (int32_t)(nowMs - wifiStaStartAtMs) >= 0) {
    wifiStaStarted = true;
    beginStaConnection("boot-delay");
    return;
  }

  if ((uint32_t)(nowMs - wifiLastRetryMs) >= WIFI_RETRY_MS) {
    wifiLastRetryMs = nowMs;
    Serial.printf("[WIFI] retry ssid=%s\n", STA_SSID);
    beginStaConnection("retry");
  }
}

static void printHelp() {
  Serial.println();
  Serial.println(F("Commands:"));
  Serial.println(F("  motor on"));
  Serial.println(F("  motor off"));
  Serial.println(F("  pwm 0-1023"));
  Serial.println(F("  freq 1000|5000|10000|20000"));
  Serial.println(F("  invertpwm 0|1"));
  Serial.println(F("  tx on|off"));
  Serial.println(F("  motor sweep"));
  Serial.println(F("  motor sweep fine"));
  Serial.println(F("  motor sweep off"));
  Serial.println(F("  send $"));
  Serial.println(F("  send startlds$"));
  Serial.println(F("  send start"));
  Serial.println(F("  send stop"));
  Serial.println(F("  raw on|off"));
  Serial.println(F("  uart on|off"));
  Serial.println(F("  scan on|off"));
  Serial.println(F("  sniff on|off"));
  Serial.println(F("  base on|off"));
  Serial.println(F("  map"));
  Serial.println(F("  map on|off"));
  Serial.println(F("  status"));
  Serial.println(F("  help"));
}

static bool parseOnOff(const String &value) {
  return value == F("1") || value == F("on") || value == F("ON") || value == F("true");
}

static void handleCommand(const String &lineIn) {
  String line = lineIn;
  line.trim();
  if (line.isEmpty()) return;

  if (line == F("help")) {
    printHelp();
    return;
  }
  if (line == F("motor on")) {
    setMotorEnabled(true);
    return;
  }
  if (line == F("motor off")) {
    setMotorEnabled(false);
    return;
  }
  if (line.startsWith(F("pwm "))) {
    setPwmDuty((uint16_t)constrain(line.substring(4).toInt(), 0, 1023));
    return;
  }
  if (line.startsWith(F("freq "))) {
    setPwmFreq((uint32_t)line.substring(5).toInt());
    return;
  }
  if (line.startsWith(F("invertpwm "))) {
    setPwmInverted(line.substring(10).toInt() != 0);
    return;
  }
  if (line == F("motor sweep")) {
    startMotorSweep();
    return;
  }
  if (line == F("motor sweep fine")) {
    startMotorSweepFine();
    return;
  }
  if (line == F("motor sweep off")) {
    stopMotorSweep();
    return;
  }
  if (line.startsWith(F("tx "))) {
    lidarTxEnabled = parseOnOff(line.substring(3));
    lidarSerial.end();
    lidarStarted = false;
    Serial.printf("[LIDAR] tx=%u\n", lidarTxEnabled ? 1 : 0);
    return;
  }
  if (line == F("raw on")) {
    rawLogEnabled = true;
    Serial.println(F("[LIDAR] raw=on"));
    return;
  }
  if (line == F("raw off")) {
    rawLogEnabled = false;
    Serial.println(F("[LIDAR] raw=off"));
    return;
  }
  if (line == F("uart on")) {
    uartSummaryEnabled = true;
    Serial.println(F("[LIDAR] uart=on"));
    return;
  }
  if (line == F("uart off")) {
    uartSummaryEnabled = false;
    Serial.println(F("[LIDAR] uart=off"));
    return;
  }
  if (line == F("scan on")) {
    startScanCombo();
    return;
  }
  if (line == F("scan off")) {
    scanMode = false;
    lidarBaud = LIDAR_BAUD;
    lidarInverted = false;
    uartSummaryEnabled = true;
    uartMapStreaming = true;
    lidarSerial.end();
    lidarStarted = false;
    Serial.println(F("[SCAN] off"));
    return;
  }
  if (line == F("scan next")) {
    scanMode = true;
    finishScanCombo(millis());
    return;
  }
  if (line == F("sniff on")) {
    sniffMode = true;
    sniffLen = 0;
    rawLogEnabled = false;
    uartSummaryEnabled = false;
    uartMapStreaming = false;
    Serial.println(F("[SNIFF] on"));
    return;
  }
  if (line == F("sniff off")) {
    flushSniff();
    sniffMode = false;
    uartSummaryEnabled = true;
    uartMapStreaming = true;
    Serial.println(F("[SNIFF] off"));
    return;
  }
  if (line == F("tap on")) {
    tapMode = true;
    tapLen = 0;
    Serial.println(F("[TAP] on"));
    return;
  }
  if (line == F("tap off")) {
    flushTap();
    tapMode = false;
    Serial.println(F("[TAP] off"));
    return;
  }
  if (line == F("base on")) {
    baseExploreMode = true;
    baseLen = 0;
    rawLogEnabled = false;
    uartSummaryEnabled = false;
    uartMapStreaming = false;
    Serial.println(F("[BASE] on"));
    return;
  }
  if (line == F("base off")) {
    flushBaseExplore();
    baseExploreMode = false;
    Serial.println(F("[BASE] off"));
    return;
  }
  if (line == F("map")) {
    printUartMap();
    return;
  }
  if (line == F("map on")) {
    uartMapStreaming = true;
    lastMapMs = 0;
    Serial.println(F("[UARTMAP] stream=on"));
    return;
  }
  if (line == F("map off")) {
    uartMapStreaming = false;
    Serial.println(F("[UARTMAP] stream=off"));
    return;
  }
  if (line == F("send $")) {
    if (lidarTxEnabled) {
      lidarSerial.write('$');
      Serial.println(F("[LIDAR] TX=$"));
    } else {
      Serial.println(F("[LIDAR] TX disabled"));
    }
    return;
  }
  if (line == F("send startlds$")) {
    if (lidarTxEnabled) {
      lidarSerial.write("startlds$");
      Serial.println(F("[LIDAR] TX=startlds$"));
    } else {
      Serial.println(F("[LIDAR] TX disabled"));
    }
    return;
  }
  if (line == F("send start")) {
    if (lidarTxEnabled) {
      lidarSerial.write("start");
      Serial.println(F("[LIDAR] TX=start"));
    } else {
      Serial.println(F("[LIDAR] TX disabled"));
    }
    return;
  }
  if (line == F("send stop")) {
    if (lidarTxEnabled) {
      lidarSerial.write("stop");
      Serial.println(F("[LIDAR] TX=stop"));
    } else {
      Serial.println(F("[LIDAR] TX disabled"));
    }
    return;
  }
  if (line == F("status")) {
    const uint32_t nowMs = millis();
    Serial.printf("[STATUS] motor=%u stable=%u pwm=%u freq=%u baud=%u tx=%u invertuart=%u scan=%u windows=%u valid=%u invalid=%u best_delta=%u decoded=%u rejected=%u\n",
                  motorEnabled ? 1 : 0,
                  motorStable(nowMs) ? 1 : 0,
                  pwmDuty,
                  (unsigned)pwmFreq,
                  (unsigned)lidarBaud,
                  lidarTxEnabled ? 1 : 0,
                  lidarInverted ? 1 : 0,
                  scanMode ? 1 : 0,
                  (unsigned)scanWindowsDone,
                  (unsigned)scanValidPackets,
                  (unsigned)scanInvalidPackets,
                  (unsigned)scanBestChecksumDelta,
                  (unsigned)decodedPackets,
                  (unsigned)rejectedPackets);
    return;
  }

  Serial.printf("[CLI] unknown: %s\n", line.c_str());
}

static void pollUsb() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(usbLine);
      usbLine = "";
    } else if (usbLine.length() < 128) {
      usbLine += c;
    } else {
      usbLine = "";
    }
  }
}

static void pollLidar(uint32_t nowMs) {
  if (!motorStable(nowMs)) {
    if (lidarStarted) {
      lidarSerial.end();
      lidarStarted = false;
    }
    resetPacket();
    return;
  }

  if (!lidarStarted) {
    startLidarSerial();
  }

  while (lidarSerial.available() > 0) {
    const int raw = lidarSerial.read();
    if (raw < 0) break;
    lastRxByteMs = nowMs;
    if (tapMode) {
      if (tapLen == 0) tapLastMs = nowMs;
      if (tapLen < sizeof(tapBuf)) {
        tapBuf[tapLen++] = (uint8_t)raw;
      } else {
        flushTap();
        tapLastMs = nowMs;
        tapBuf[tapLen++] = (uint8_t)raw;
      }
      if (tapLen >= 32) {
        flushTap();
      }
    }
    feedBaseExplore((uint8_t)raw, nowMs);
    if (!baseExploreMode) {
      feedByte((uint8_t)raw);
    }
    yield();
  }

  if (packetLen > 0 && (uint32_t)(nowMs - lastRxByteMs) > RAW_FRAME_TIMEOUT_MS) {
    resetPacket();
  }
  if (tapMode && tapLen > 0 && (uint32_t)(nowMs - tapLastMs) > SNIFF_FLUSH_TIMEOUT_MS) {
    flushTap();
  }
  if (baseExploreMode && baseLen > 0 && (uint32_t)(nowMs - baseLastMs) > SNIFF_FLUSH_TIMEOUT_MS) {
    flushBaseExplore();
  }
  if (badChecksumStreak >= 20) {
    badChecksumStreak = 0;
    Serial.println(F("[LIDAR] checksum streak high"));
  }

  if (scanMode && (uint32_t)(nowMs - scanWindowStartedMs) >= SCAN_WINDOW_MS) {
    finishScanCombo(nowMs);
  }
}

static void pollUartMap(uint32_t nowMs) {
  if (!uartMapStreaming) return;
  if ((uint32_t)(nowMs - lastMapMs) < UART_MAP_PERIOD_MS) return;
  lastMapMs = nowMs;
  printUartMap();
}

void setup() {
  Serial.begin(WIFI_AP_BAUD);
  delay(300);
  Serial.println();
  Serial.println(F("[BOOT] LDS02RR / ESP32-S3 firmware"));
  Serial.printf("[PINOUT] PWM/MOT_EN=GPIO%d RX=GPIO%d TX(optional)=GPIO%d\n",
                PWM_PIN, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial.printf("[UART] U1RXD=GPIO18 U1TXD=GPIO17 on ESP32-S3 DevKitC-1\n");

  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  configureMotorPwm();
  markMotorChange();
  pinMode(LIDAR_RX_PIN, INPUT_PULLUP);
  pinMode(LIDAR_TX_PIN, OUTPUT);

  printHelp();
  setupWifi();
}

void loop() {
  const uint32_t nowMs = millis();
  pollUsb();
  pollLidar(nowMs);
  pollWifi(nowMs);
  pollMotorPwm(nowMs);
  pollMotorSweep(nowMs);
  pollMotorSweepFine(nowMs);
  pollUartMap(nowMs);
  webServer.handleClient();
  yield();
}
