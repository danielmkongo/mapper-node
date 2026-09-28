#include <Arduino.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include "DW1000Ranging.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <LoRa_E32.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Identity
//   NODE_ID is the "I" field in every report - the id the app / digital twin show.
//   Set it per wearable in platformio.ini (build_flags = -D NODE_ID=n).
// ---------------------------------------------------------------------------
#ifndef NODE_ID
#define NODE_ID 5
#endif

// LoRa radio address of this node (E32 fixed-transmission addressing).
#define LORA_ADDR 100

// How often a report goes out. UWB ranges ~5-10x per second; 1Hz keeps a walking
// person smooth on the map. Each report is ONE radio packet (~37 of the E32's 58 bytes,
// ~30ms on air at 19.2kbps), so the channel stays >95% idle. A random +/-150ms jitter
// keeps several nodes from locking into the same slot and colliding every second.
#define REPORT_MS 1000
#define REPORT_JITTER_MS 150

// A distance older than this is reported as 0 ("unknown") instead of silently reusing
// a stale value - otherwise one lost anchor freezes a wrong position on screen.
#define RANGE_STALE_MS 1500

// Loop watchdog: reboot if the main loop ever stops for this long.
#define LOOP_WDT_SECONDS 15

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
// E32 pins. The E32 has M0, M1 and AUX (no "M2"). These used to be named M0/M1/M2 and
// passed to LoRa_E32(serial, auxPin, m0Pin, m1Pin) in that order - so by position GPIO13
// has always been the library's AUX, 27 its M0 and 12 its M1. The gateway uses the same
// mapping. Names now match what the library actually does with each pin.
#define LORA_TX 25
#define LORA_RX 26
#define LORA_AUX 13
#define LORA_M0 27
#define LORA_M1 12
constexpr uint8_t LORA_CHANNEL = 0x19;

#define PIN_RST 16
#define PIN_SS 5
#define PIN_IRQ 17

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define BATTERY_PIN 36

// Uncomment to enable debug prints for motion & gesture
// #define DEBUG_MOTION

LoRa_E32 lora(&Serial1, LORA_AUX, LORA_M0, LORA_M1);
Adafruit_BMP280 bme;
Adafruit_MPU6050 mpu;
bool hasBmp = false;
bool hasMpu = false;
bool hasLora = false;

// ---------------------------------------------------------------------------
// Anchors - short addresses as reported by DW1000 (byte[1]*256 + byte[0] of each
// anchor EUI, see safe_anchor ANCHORS[]). Index = position in the "U" array: A, B, C.
// ---------------------------------------------------------------------------
const uint16_t ANCHOR_ADDR[3] = {0x2C9F, 0xEFAF, 0xEFBF};

struct RangeSlot
{
  float metres;
  uint32_t at; // millis() of the reading, 0 = never
};
RangeSlot ranges[3] = {};

// ------------------ KALMAN FILTER ------------------
struct KalmanFilter
{
  float q, r, x, p, k;
};

KalmanFilter kfAx = {0.02, 0.3, 0, 1, 0};
KalmanFilter kfAy = {0.02, 0.3, 0, 1, 0};
KalmanFilter kfAz = {0.02, 0.3, 0, 1, 0};
KalmanFilter kfGx = {0.02, 0.3, 0, 1, 0};
KalmanFilter kfGy = {0.02, 0.3, 0, 1, 0};
KalmanFilter kfGz = {0.02, 0.3, 0, 1, 0};

float kalmanUpdate(KalmanFilter *kf, float m)
{
  kf->p += kf->q;
  kf->k = kf->p / (kf->p + kf->r);
  kf->x += kf->k * (m - kf->x);
  kf->p *= (1 - kf->k);
  return kf->x;
}
// --------------------------------------------------

float accelMagLP = 1.0f;

void newRange()
{
  DW1000Device *dev = DW1000Ranging.getDistantDevice();
  if (!dev)
    return;

  const uint16_t addr = dev->getShortAddress();
  const float range_m = dev->getRange();
  if (isnan(range_m) || range_m <= 0)
    return;

  for (int i = 0; i < 3; i++)
  {
    if (addr == ANCHOR_ADDR[i])
    {
      ranges[i].metres = range_m;
      ranges[i].at = millis();
      Serial.printf("anchor %c (%04X): %.2f m\n", 'A' + i, addr, range_m);
      return;
    }
  }
}

void newDevice(DW1000Device *device) {}
void inactiveDevice(DW1000Device *device) {}

bool initLora()
{
  // The library drives M0/M1 itself (program mode for the config write, normal mode after)
  // and waits on AUX before each send, so the module is never written to while busy.
  Serial1.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  if (!lora.begin())
    return false;

  ResponseStructContainer c = lora.getConfiguration();
  if (c.status.code != 1)
  {
    c.close();
    return false;
  }
  Configuration config = *(Configuration *)c.data;
  c.close();

  config.ADDL = LORA_ADDR & 0xFF;
  config.ADDH = (LORA_ADDR >> 8) & 0xFF;
  config.CHAN = LORA_CHANNEL;

  config.OPTION.fec = FEC_1_ON;
  config.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
  config.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
  config.OPTION.transmissionPower = POWER_17;
  config.OPTION.wirelessWakeupTime = WAKE_UP_1250;

  config.SPED.airDataRate = AIR_DATA_RATE_101_192;
  config.SPED.uartBaudRate = UART_BPS_9600;
  config.SPED.uartParity = MODE_00_8N1;

  return lora.setConfiguration(config, WRITE_CFG_PWR_DWN_LOSE).code == 1;
}

// Report frame, sized to fit a single E32 air packet (58 bytes incl. the 3-byte address
// header of fixed transmission). Longer payloads get split in two, and losing either half
// loses the report - that, not the send rate, is what made fast reporting unreliable.
//
//   ~I,T,P,G,M,A,B,C,S*XX^
//     I node id   T temp C   P pressure hPa   G 1 standing / 0 lying   M 1 moving
//     A,B,C range to each anchor in whole cm (0 = no fresh range)
//     S sequence 0-255 (lets the hub measure packet loss)
//     XX XOR of every character between '~' and '*', 2 hex digits
//
// The gateway checks XX, drops bad frames, and republishes as the usual JSON on MQTT.
uint8_t frameChecksum(const char *body, size_t n)
{
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++)
    x ^= (uint8_t)body[i];
  return x;
}

float readBatteryVoltage()
{
  int raw = analogRead(BATTERY_PIN);
  return raw * (3.3f / 4095.0f); // adjust if a voltage divider is fitted
}

// Distance for the report: fresh reading, or 0 = unknown.
float freshRange(int i, uint32_t now)
{
  if (ranges[i].at == 0 || now - ranges[i].at > RANGE_STALE_MS)
    return 0.0f;
  return ranges[i].metres;
}

void setup()
{
  Serial.begin(115200);

  // A missing radio must not brick the tag: keep ranging (and printing) without it.
  for (int attempt = 0; attempt < 3 && !hasLora; attempt++)
    hasLora = initLora();
  randomSeed(esp_random());
  Serial.println(hasLora ? "LoRa initialized" : "LoRa init FAILED - running without uplink");

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, 33);

  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  pinMode(PIN_IRQ, INPUT_PULLDOWN); // see safe_anchor: keeps a marginal IRQ line quiet
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  static char tagEui[] = "7D:00:22:EA:82:60:3B:9C"; // startAsTag() wants a mutable char[]
  DW1000Ranging.startAsTag(tagEui, DW1000.MODE_LONGDATA_RANGE_LOWPOWER);

  hasBmp = bme.begin(0x76);
  hasMpu = mpu.begin();
  if (!hasBmp)
    Serial.println("BMP280 not found");
  if (!hasMpu)
    Serial.println("MPU6050 not found");

  Serial.printf("node %d ready\n", NODE_ID);

  esp_task_wdt_init(LOOP_WDT_SECONDS, true);
  esp_task_wdt_add(nullptr);
}

void loop()
{
  esp_task_wdt_reset();
  DW1000Ranging.loop();

  static uint32_t lastSend = 0;
  static uint32_t interval = REPORT_MS;
  static uint8_t seq = 0;
  const uint32_t now = millis();
  if (now - lastSend < interval)
    return;
  lastSend = now;
  interval = REPORT_MS + random(-REPORT_JITTER_MS, REPORT_JITTER_MS + 1);

  int gesture = 1; // 1=standing, 0=laying
  bool moving = false;
  if (hasMpu)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Kalman filter on accelerometer
    float Ax = kalmanUpdate(&kfAx, a.acceleration.x);
    float Ay = kalmanUpdate(&kfAy, a.acceleration.y);
    float Az = kalmanUpdate(&kfAz, a.acceleration.z);

    // Kalman filter on gyro (converted to degrees)
    float Gx = kalmanUpdate(&kfGx, g.gyro.x * 57.2958f);
    float Gy = kalmanUpdate(&kfGy, g.gyro.y * 57.2958f);
    float Gz = kalmanUpdate(&kfGz, g.gyro.z * 57.2958f);

    float gyroMag = sqrtf(Gx * Gx + Gy * Gy + Gz * Gz);
    float accelMag = sqrtf(Ax * Ax + Ay * Ay + Az * Az) / 9.80665f;

    accelMagLP = accelMagLP * 0.98f + accelMag * 0.02f;
    float accelChange = fabsf(accelMag - accelMagLP);

    moving = (gyroMag > 10.0f) || (accelChange > 0.12f);

    float normMag = sqrtf(Ax * Ax + Ay * Ay + Az * Az);
    if (normMag < 0.0001f)
      normMag = 0.0001f;
    gesture = (fabsf(Ay / normMag) > 0.85f) ? 1 : 0;

#ifdef DEBUG_MOTION
    Serial.println(gesture ? "Standing.." : "Laying..");
    if (moving)
      Serial.println("Moving!");
#endif
  }

  int T = 0, P = 0;
  if (hasBmp)
  {
    const float t = bme.readTemperature();
    const float p = bme.readPressure();
    if (!isnan(t))
      T = (int)lroundf(t);
    if (!isnan(p))
      P = (int)lroundf(p / 100.0f);
  }

  auto cm = [&](int i) { return (int)lroundf(freshRange(i, now) * 100.0f); };
  char body[48];
  const int n = snprintf(body, sizeof(body), "%d,%d,%d,%d,%d,%d,%d,%d,%u",
                         NODE_ID, T, P, gesture, moving ? 1 : 0, cm(0), cm(1), cm(2), seq++);
  char payload[56];
  snprintf(payload, sizeof(payload), "~%s*%02X^", body, frameChecksum(body, n));

  if (hasLora)
  {
    ResponseStatus rs = lora.sendBroadcastFixedMessage(LORA_CHANNEL, String(payload));
    if (rs.code != 1)
      Serial.printf("LoRa send failed: %s\n", rs.getResponseDescription().c_str());
  }
  Serial.println(payload);
}
