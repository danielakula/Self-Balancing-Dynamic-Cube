#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <MahonyAHRS.h> 

#define ISM_CS   16
#define SPI_SCK  37
#define SPI_MISO 39
#define SPI_MOSI 38

Adafruit_ISM330DHCX ism;
Mahony filter;

const int FILTER_HZ    = 500;
const int SERIAL_HZ    = 50;
const int FILTER_US    = 1000000 / FILTER_HZ;
const int SERIAL_US    = 1000000 / SERIAL_HZ;
const int CALIB_SAMPLES = 1000;  

const float SMOOTH = 0.1f;
float sq0 = 1.0f, sq1 = 0.0f, sq2 = 0.0f, sq3 = 0.0f;

// Gyro bias (rad/s)
float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;
// Accel bias (m/s^2)
float ax_bias = 0.0f, ay_bias = 0.0f;  

unsigned long lastFilter = 0;
unsigned long lastSerial = 0;

// --- MAHONY ADAPTIVE TUNING ---
const float BASE_KP = 0.5f;       // Base trust in accelerometer
const float BASE_KI = 0.005f;      // Base trust in gyro bias correction
const float GRAVITY = 9.80665f; 
const float ACCEL_TOLERANCE = 1.5f; 

void calibrate() {
  Serial.println("KEEP STILL - CALIBRATING...");
  delay(500);

  double sumGX = 0, sumGY = 0, sumGZ = 0;
  double sumAX = 0, sumAY = 0;
  int count = 0;

  while (count < CALIB_SAMPLES) {
    unsigned long now = micros();
    if ((now - lastFilter) >= FILTER_US) {
      lastFilter = now;
      sensors_event_t accel, gyro, temp;
      ism.getEvent(&accel, &gyro, &temp);

      sumGX += gyro.gyro.x; sumGY += gyro.gyro.y; sumGZ += gyro.gyro.z;
      //sumAX += accel.acceleration.x; sumAY += accel.acceleration.y;
      count++;
    }
  }

  gx_bias = sumGX / CALIB_SAMPLES;
  gy_bias = sumGY / CALIB_SAMPLES;
  gz_bias = sumGZ / CALIB_SAMPLES;
//   ax_bias = sumAX / CALIB_SAMPLES;
//   ay_bias = sumAY / CALIB_SAMPLES;

  Serial.println("Calibration done!");
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  if (!ism.begin_SPI(ISM_CS, SPI_SCK, SPI_MISO, SPI_MOSI)) {
    Serial.println("Failed to find ISM330DHCX chip!");
    while (1) delay(10);
  }

  ism.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  ism.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  ism.setAccelDataRate(LSM6DS_RATE_416_HZ);
  ism.setGyroDataRate(LSM6DS_RATE_416_HZ);

  lastFilter = micros();
  lastSerial = micros();

  calibrate();
}

void loop() {
  unsigned long now = micros();

  if ((now - lastFilter) >= FILTER_US) {
    // dt in seconds
    float dt = (now - lastFilter) / 1000000.0f;
    lastFilter = now;

    sensors_event_t accel, gyro, temp;
    ism.getEvent(&accel, &gyro, &temp);

    // Bias (Rad/s)
    float gx = gyro.gyro.x - gx_bias;
    float gy = gyro.gyro.y - gy_bias;
    float gz = gyro.gyro.z - gz_bias;

    // Deadband
    if (abs(gx) < 0.008f) gx = 0.0f;
    if (abs(gy) < 0.008f) gy = 0.0f;  
    if (abs(gz) < 0.008f) gz = 0.0f;

    // Accel Bias (m/s^2)
    float ax = accel.acceleration.x - ax_bias;
    float ay = accel.acceleration.y - ay_bias;
    float az = accel.acceleration.z;  

    // ADAPTIVE GAIN LOGIC 
    float accel_mag = sqrt(ax*ax + ay*ay + az*az);

    if (abs(accel_mag - GRAVITY) > ACCEL_TOLERANCE) {
      // High Acceleration: Turn off accelerometer trust, fly blind on Gyro
      filter.twoKp = 0.0f;
      filter.twoKi = 0.0f;
    } else {
      // Normal Gravity: Let accelerometer correct drift
      filter.twoKp = BASE_KP * 2.0f;
      filter.twoKi = BASE_KI * 2.0f;
    }

    // Update Filter
    filter.update(gx, gy, gz, ax, ay, az, dt);

    // Low pass filter 
    sq0 = (1.0f - SMOOTH) * sq0 + SMOOTH * filter.q0;
    sq1 = (1.0f - SMOOTH) * sq1 + SMOOTH * filter.q1;
    sq2 = (1.0f - SMOOTH) * sq2 + SMOOTH * filter.q2;
    sq3 = (1.0f - SMOOTH) * sq3 + SMOOTH * filter.q3;

    // Renormalise after smoothing
    float n = sqrt(sq0*sq0 + sq1*sq1 + sq2*sq2 + sq3*sq3);
    sq0 /= n; sq1 /= n; sq2 /= n; sq3 /= n;
  }

  if ((now - lastSerial) >= SERIAL_US) {
    lastSerial = now;

    // Print to Python Visualizer (Appending a dummy '0' for the encoder angle if not yet integrated)
    Serial.print(sq0, 4); Serial.print(",");
    Serial.print(sq1, 4); Serial.print(",");
    Serial.print(sq2, 4); Serial.print(",");
    Serial.print(sq3, 4); Serial.println();
  }
}