#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <MadgwickAHRS.h>

#define ISM_CS   16
#define SPI_SCK  37
#define SPI_MISO 39
#define SPI_MOSI 38

Adafruit_ISM330DHCX ism;
Madgwick filter;

const int FILTER_HZ    = 500;
const int SERIAL_HZ    = 50;
const int FILTER_US    = 1000000 / FILTER_HZ;
const int SERIAL_US    = 1000000 / SERIAL_HZ;
const int CALIB_SAMPLES = 1000;  // 2 seconds at 500Hz for better accuracy

const float SMOOTH = 0.1f;

float sq0 = 1.0f, sq1 = 0.0f, sq2 = 0.0f, sq3 = 0.0f;

// Gyro bias (rad/s)
float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;

// Accel bias (m/s^2)
float ax_bias = 0.0f, ay_bias = 0.0f;  // Z intentionally excluded (gravity)

unsigned long lastFilter = 0;
unsigned long lastSerial = 0;

const float BASE_BETA = 0.01f;   
const float GRAVITY = 9.80665f;    
const float ACCEL_TOLERANCE = 1.5f;

void calibrate() {
  Serial.println("KEEP STILL - CALIBRATING...");

  double sumGX = 0, sumGY = 0, sumGZ = 0;
  double sumAX = 0, sumAY = 0;
  int count = 0;

  delay(500);

  while (count < CALIB_SAMPLES) {
    unsigned long now = micros();
    if ((now - lastFilter) >= FILTER_US) {
      lastFilter = now;

      sensors_event_t accel, gyro, temp;
      ism.getEvent(&accel, &gyro, &temp);

      sumGX += gyro.gyro.x;
      sumGY += gyro.gyro.y;
      sumGZ += gyro.gyro.z;

      sumAX += accel.acceleration.x;
      sumAY += accel.acceleration.y;

      count++;
    }
  }

  gx_bias = sumGX / CALIB_SAMPLES;
  gy_bias = sumGY / CALIB_SAMPLES;
  gz_bias = sumGZ / CALIB_SAMPLES;

//   ax_bias = sumAX / CALIB_SAMPLES;
//   ay_bias = sumAY / CALIB_SAMPLES;

  Serial.print("Gyro bias  X: "); Serial.println(gx_bias, 6);
  Serial.print("Gyro bias  Y: "); Serial.println(gy_bias, 6);
  Serial.print("Gyro bias  Z: "); Serial.println(gz_bias, 6);
  Serial.print("Accel bias X: "); Serial.println(ax_bias, 6);
  Serial.print("Accel bias Y: "); Serial.println(ay_bias, 6);
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

  filter.begin(FILTER_HZ);
  filter.beta = 0.01f;

  lastFilter = micros();
  lastSerial = micros();

  calibrate();
}

void loop() {
  unsigned long now = micros();

  if ((now - lastFilter) >= FILTER_US) {
    lastFilter = now;

    sensors_event_t accel, gyro, temp;
    ism.getEvent(&accel, &gyro, &temp);

    // Bias & Deadband to Gyro
    float gx_raw = gyro.gyro.x - gx_bias;
    float gy_raw = gyro.gyro.y - gy_bias;
    float gz_raw = gyro.gyro.z - gz_bias;

    if (abs(gx_raw) < 0.008f) gx_raw = 0.0f;
    if (abs(gy_raw) < 0.008f) gy_raw = 0.0f;  
    if (abs(gz_raw) < 0.008f) gz_raw = 0.0f;

    float gx = gx_raw * 57.2958f;
    float gy = gy_raw * 57.2958f;
    float gz = gz_raw * 57.2958f;

    // Pure Accel Data 
    float ax = accel.acceleration.x - ax_bias;
    float ay = accel.acceleration.y - ay_bias;
    float az = accel.acceleration.z;  

    // Adaptive Beta Logic 
    float accel_mag = sqrt(ax*ax + ay*ay + az*az);
    if (abs(accel_mag - GRAVITY) > ACCEL_TOLERANCE) {
        filter.beta = 0.0f; 
    } else {
        filter.beta = BASE_BETA; 
    }

    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // Low pass filter
    sq0 = (1.0f - SMOOTH) * sq0 + SMOOTH * filter.q0;
    sq1 = (1.0f - SMOOTH) * sq1 + SMOOTH * filter.q1;
    sq2 = (1.0f - SMOOTH) * sq2 + SMOOTH * filter.q2;
    sq3 = (1.0f - SMOOTH) * sq3 + SMOOTH * filter.q3;

    // Renormalise
    float n = sqrt(sq0*sq0 + sq1*sq1 + sq2*sq2 + sq3*sq3);
    sq0 /= n; sq1 /= n; sq2 /= n; sq3 /= n;
  }

  if ((now - lastSerial) >= SERIAL_US) {
    lastSerial = now;
    
    // Output pure quaternions
    Serial.print(sq0, 4); Serial.print(",");
    Serial.print(sq1, 4); Serial.print(",");
    Serial.print(sq2, 4); Serial.print(",");
    Serial.print(sq3, 4);
    Serial.println();
  }
}