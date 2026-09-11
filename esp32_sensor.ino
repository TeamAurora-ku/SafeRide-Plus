/*
   ============================================================
              SafeRide+ - Complete ESP32 Sensor Code
   ============================================================

   Hardware:
   - ESP32 DevKit
   - MAX30102
   - MPU6500
   - Analog Light Sensor
   - Buzzer
   - Alert LED

   Functions:
   1. Heart Rate
   2. SpO2
   3. Acceleration
   4. Gyroscope
   5. Impact detection
   6. Light level detection
   7. Local warning through buzzer + LED

   I2C:
   SDA = GPIO 21
   SCL = GPIO 22

   NOTE:
   The motion sensor is MPU6500.
   WHO_AM_I should return 0x70.
*/

// ============================================================
// LIBRARIES
// ============================================================

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define SDA_PIN             21
#define SCL_PIN             22

#define LIGHT_SENSOR_PIN    34

#define BUZZER_PIN          33
#define ALERT_LED_PIN       26
#define MQ3_PIN             35

// ============================================================
// MAX30102
// ============================================================

MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];

byte rateSpot = 0;

long lastBeat = 0;

float beatsPerMinute = 0;
int beatAvg = 0;

// ============================================================
// MPU6500
// ============================================================

#define MPU6500_ADDR        0x68

#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1          0x6B

#define ACCEL_CONFIG        0x1C
#define GYRO_CONFIG         0x1B

#define ACCEL_XOUT_H        0x3B

// MPU6500 sensitivity
// ±2g = 16384 LSB/g
// ±250 deg/s = 131 LSB/(deg/s)

const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE  = 131.0;

// ============================================================
// IMPACT DETECTION
// ============================================================

// Initial prototype threshold.
// DO NOT consider this a final scientifically validated
// crash threshold.

const float IMPACT_G_THRESHOLD = 2.0;
const float GYRO_THRESHOLD = 100.0;

// ============================================================
// LIGHT LEVEL THRESHOLDS
// ============================================================

// These are prototype thresholds.
// Calibrate them using your actual sensor.

const int LOW_LIGHT_THRESHOLD = 1000;
const int BRIGHT_LIGHT_THRESHOLD = 2800;

// ============================================================
// SENSOR DATA VARIABLES
// ============================================================

float accelX = 0;
float accelY = 0;
float accelZ = 0;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

float totalAcceleration = 0;
float totalGyro = 0;

int lightRaw = 0;
float lightVoltage = 0;

String lightLevel = "UNKNOWN";

bool impactDetected = false;

// ============================================================
// 5-SECOND DATA COLLECTION
// ============================================================

const unsigned long SAMPLE_WINDOW = 5000;
unsigned long windowStartTime = 0;

// Counters
unsigned long sampleCount = 0;

// MAX30102
float sumHeartRate = 0;
unsigned long validHeartSamples = 0;

// MQ-3 Alcohol Sensor
int mq3Raw = 0;
float mq3Voltage = 0;

float sumMQ3Raw = 0;
float sumMQ3Voltage = 0;



bool alcoholDetected = false;

// Adjust this after testing your actual MQ-3
const int MQ3_THRESHOLD = 1800;

// MPU6500
float sumAccelX = 0;
float sumAccelY = 0;
float sumAccelZ = 0;
float sumTotalAcceleration = 0;

float sumGyroX = 0;
float sumGyroY = 0;
float sumGyroZ = 0;
float sumTotalGyro = 0;

// Light sensor
float sumLightRaw = 0;
float sumLightVoltage = 0;

// Impact
bool impactInWindow = false;

// ============================================================
// MPU6500 WRITE
// ============================================================

void writeMPU6500(byte reg, byte data)
{
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// ============================================================
// MPU6500 READ
// ============================================================

byte readMPU6500(byte reg)
{
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6500_ADDR, (uint8_t)1);

  if (Wire.available())
  {
    return Wire.read();
  }

  return 0xFF;
}

// ============================================================
// READ MPU6500 ACCEL + GYRO
// ============================================================

void readMPU6500()
{
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6500_ADDR, (uint8_t)14);

  if (Wire.available() < 14)
  {
    return;
  }

  int16_t rawAX = (Wire.read() << 8) | Wire.read();
  int16_t rawAY = (Wire.read() << 8) | Wire.read();
  int16_t rawAZ = (Wire.read() << 8) | Wire.read();

  // Temperature registers
  Wire.read();
  Wire.read();

  int16_t rawGX = (Wire.read() << 8) | Wire.read();
  int16_t rawGY = (Wire.read() << 8) | Wire.read();
  int16_t rawGZ = (Wire.read() << 8) | Wire.read();

  // Convert acceleration to g
  accelX = rawAX / ACCEL_SCALE;
  accelY = rawAY / ACCEL_SCALE;
  accelZ = rawAZ / ACCEL_SCALE;

  // Convert gyro to degrees/second
  gyroX = rawGX / GYRO_SCALE;
  gyroY = rawGY / GYRO_SCALE;
  gyroZ = rawGZ / GYRO_SCALE;

  // Total acceleration magnitude
  totalAcceleration = sqrt(
    accelX * accelX +
    accelY * accelY +
    accelZ * accelZ
  );

  // Total gyro magnitude
  totalGyro = sqrt(
    gyroX * gyroX +
    gyroY * gyroY +
    gyroZ * gyroZ
  );

  // ----------------------------------------------------------
  // IMPACT DETECTION
  // ----------------------------------------------------------

  if (
    totalAcceleration >= IMPACT_G_THRESHOLD ||
    totalGyro >= GYRO_THRESHOLD
  )
  {
    impactDetected = true;
  }
  else
  {
    impactDetected = false;
  }
}

// ============================================================
// READ LIGHT SENSOR
// ============================================================

void readLightSensor()
{
  lightRaw = analogRead(LIGHT_SENSOR_PIN);

  lightVoltage = (lightRaw / 4095.0) * 3.3;

  if (lightRaw < LOW_LIGHT_THRESHOLD)
  {
    lightLevel = "LOW LIGHT";
  }
  else if (lightRaw < BRIGHT_LIGHT_THRESHOLD)
  {
    lightLevel = "NORMAL LIGHT";
  }
  else
  {
    lightLevel = "BRIGHT LIGHT";
  }
}

// ============================================================
// ALERT FUNCTION
// ============================================================

void triggerAlert()
{
  digitalWrite(ALERT_LED_PIN, HIGH);

  tone(BUZZER_PIN, 2000);

  delay(300);

  noTone(BUZZER_PIN);

  digitalWrite(ALERT_LED_PIN, LOW);
}

// ============================================================
// PRINT MPU DATA
// ============================================================

void printMPUData()
{
  Serial.println();
  Serial.println("----- MOTION DATA -----");

  Serial.print("Accel X : ");
  Serial.print(accelX, 2);
  Serial.println(" g");

  Serial.print("Accel Y : ");
  Serial.print(accelY, 2);
  Serial.println(" g");

  Serial.print("Accel Z : ");
  Serial.print(accelZ, 2);
  Serial.println(" g");

  Serial.print("Total Acceleration : ");
  Serial.print(totalAcceleration, 2);
  Serial.println(" g");

  Serial.print("Gyro X : ");
  Serial.print(gyroX, 2);
  Serial.println(" deg/s");

  Serial.print("Gyro Y : ");
  Serial.print(gyroY, 2);
  Serial.println(" deg/s");

  Serial.print("Gyro Z : ");
  Serial.print(gyroZ, 2);
  Serial.println(" deg/s");

  Serial.print("Total Gyro : ");
  Serial.print(totalGyro, 2);
  Serial.println(" deg/s");

  Serial.print("Impact : ");

  if (impactDetected)
  {
    Serial.println("YES");
  }
  else
  {
    Serial.println("NO");
  }
}

// ============================================================
// PRINT LIGHT DATA
// ============================================================

void printLightData()
{
  Serial.println();
  Serial.println("----- LIGHT DATA -----");

  Serial.print("Raw ADC : ");
  Serial.println(lightRaw);

  Serial.print("Voltage : ");
  Serial.print(lightVoltage, 2);
  Serial.println(" V");

  Serial.print("Light Level : ");
  Serial.println(lightLevel);
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1500);

  Serial.println();
  Serial.println("================================================");
  Serial.println("             SAFERIDE+ ESP32 SYSTEM");
  Serial.println("================================================");

  // ----------------------------------------------------------
  // GPIO
  // ----------------------------------------------------------

  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(MQ3_PIN, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ALERT_LED_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(ALERT_LED_PIN, LOW);

  analogReadResolution(12);

  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  Serial.println("I2C started.");

  // ==========================================================
  // MPU6500 INITIALIZATION
  // ==========================================================

  Serial.println();
  Serial.println("Checking MPU6500...");

  byte whoAmI = readMPU6500(WHO_AM_I_REG);

  Serial.print("MPU WHO_AM_I = 0x");

  if (whoAmI < 16)
    Serial.print("0");

  Serial.println(whoAmI, HEX);

  if (whoAmI == 0x70)
  {
    Serial.println("MPU6500 DETECTED.");
  }
  else
  {
    Serial.println("WARNING: MPU6500 not detected.");
    Serial.println("Check SDA, SCL, VCC, GND and address.");
  }

  // Wake MPU6500
  writeMPU6500(PWR_MGMT_1, 0x00);

  delay(100);

  // Accelerometer ±2g
  writeMPU6500(ACCEL_CONFIG, 0x00);

  // Gyroscope ±250 deg/s
  writeMPU6500(GYRO_CONFIG, 0x00);

  Serial.println("MPU6500 configured.");

  // ==========================================================
  // MAX30102 INITIALIZATION
  // ==========================================================

  Serial.println();
  Serial.println("Checking MAX30102...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30102 NOT DETECTED.");

    Serial.println();
    Serial.println("Check:");
    Serial.println("VCC -> 3.3V");
    Serial.println("GND -> GND");
    Serial.println("SDA -> GPIO21");
    Serial.println("SCL -> GPIO22");

    while (1)
    {
      digitalWrite(ALERT_LED_PIN, !digitalRead(ALERT_LED_PIN));
      delay(500);
    }
  }

  Serial.println("MAX30102 detected.");

  // ----------------------------------------------------------
  // MAX30102 SETTINGS
  // ----------------------------------------------------------

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;       // Red + IR
  int sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(
    ledBrightness,
    sampleAverage,
    ledMode,
    sampleRate,
    pulseWidth,
    adcRange
  );

  particleSensor.setPulseAmplitudeRed(0x3F);
  particleSensor.setPulseAmplitudeIR(0x3F);

  Serial.println("MAX30102 configured.");

  // ==========================================================
  // READY
  // ==========================================================

  Serial.println();
  Serial.println("================================================");
  Serial.println("             SAFERIDE+ READY");
  Serial.println("================================================");

  Serial.println();
  Serial.println("Place your finger on MAX30102.");
  Serial.println("Move the MPU gently for testing.");
  Serial.println();
}

// ============================================================
// LOOP - COLLECT DATA FOR 5 SECONDS THEN DISPLAY RESULT
// ============================================================

void loop()
{
  // ==========================================================
  // MAX30102
  // ==========================================================

  long irValue = particleSensor.getIR();

  if (irValue > 50000)
  {
    if (checkForBeat(irValue))
    {
      long delta = millis() - lastBeat;

      if (delta > 0)
      {
        lastBeat = millis();

        beatsPerMinute = 60.0 / (delta / 1000.0);

        if (beatsPerMinute > 20 && beatsPerMinute < 255)
        {
          rates[rateSpot++] = (byte)beatsPerMinute;

          rateSpot %= RATE_SIZE;

          beatAvg = 0;

          for (byte x = 0; x < RATE_SIZE; x++)
          {
            beatAvg += rates[x];
          }

          beatAvg /= RATE_SIZE;
        }
      }
    }
  }
  else
  {
    beatsPerMinute = 0;
  }

  // ==========================================================
  // MPU6500
  // ==========================================================

  readMPU6500();

  // ==========================================================
  // LIGHT SENSOR
  // ==========================================================

  readLightSensor();
  // ==========================================================
// MQ-3 ALCOHOL SENSOR
// ==========================================================

mq3Raw = analogRead(MQ3_PIN);

mq3Voltage = (mq3Raw / 4095.0) * 3.3;

  // ==========================================================
  // COLLECT DATA
  // ==========================================================

  sampleCount++;

  // Heart Rate
  if (beatAvg > 0)
  {
    sumHeartRate += beatAvg;
    validHeartSamples++;
  }

  // MPU6500
  sumAccelX += accelX;
  sumAccelY += accelY;
  sumAccelZ += accelZ;
  sumTotalAcceleration += totalAcceleration;

  sumGyroX += gyroX;
  sumGyroY += gyroY;
  sumGyroZ += gyroZ;
  sumTotalGyro += totalGyro;

  // Light
  sumLightRaw += lightRaw;
  sumLightVoltage += lightVoltage;

  // Impact
  if (impactDetected)
  {
    impactInWindow = true;
  }

  // ==========================================================
  // CHECK IF 5 SECONDS ARE COMPLETE
  // ==========================================================

  if (millis() - windowStartTime >= SAMPLE_WINDOW)
  {
    // ========================================================
    // CALCULATE 5-SECOND AVERAGES
    // ========================================================

    float avgHeartRate = 0;

    if (validHeartSamples > 0)
    {
      avgHeartRate = sumHeartRate / validHeartSamples;
    }

    float avgAccelX = sumAccelX / sampleCount;
    float avgAccelY = sumAccelY / sampleCount;
    float avgAccelZ = sumAccelZ / sampleCount;
    float avgTotalAcceleration = sumTotalAcceleration / sampleCount;

    float avgGyroX = sumGyroX / sampleCount;
    float avgGyroY = sumGyroY / sampleCount;
    float avgGyroZ = sumGyroZ / sampleCount;
    float avgTotalGyro = sumTotalGyro / sampleCount;

    float avgLightRaw = sumLightRaw / sampleCount;
    float avgLightVoltage = sumLightVoltage / sampleCount;
    float avgMQ3Raw = sumMQ3Raw / sampleCount;
float avgMQ3Voltage = sumMQ3Voltage / sampleCount;

    // ========================================================
    // DETERMINE LIGHT LEVEL
    // ========================================================

    String avgLightLevel;

    if (avgLightRaw < LOW_LIGHT_THRESHOLD)
    {
      avgLightLevel = "LOW LIGHT";
    }
    else if (avgLightRaw < BRIGHT_LIGHT_THRESHOLD)
    {
      avgLightLevel = "NORMAL LIGHT";
    }
    else
    {
      avgLightLevel = "BRIGHT LIGHT";
    }
    if (avgMQ3Raw >= MQ3_THRESHOLD)
{
  alcoholDetected = true;
}
else
{
  alcoholDetected = false;
}
    // ========================================================
    // FINAL 5-SECOND OUTPUT
    // ========================================================

    Serial.println();
    Serial.println();
    Serial.println("================================================");
    Serial.println("       SAFERIDE+ - 5 SECOND ANALYSIS");
    Serial.println("================================================");

    Serial.print("Samples Collected : ");
    Serial.println(sampleCount);

    // --------------------------------------------------------
    // HEALTH DATA
    // --------------------------------------------------------

    Serial.println();
    Serial.println("----- HEALTH DATA -----");

    if (avgHeartRate > 0)
    {
      Serial.print("Average Heart Rate : ");
      Serial.print(avgHeartRate, 1);
      Serial.println(" BPM");
    }
    else
    {
      Serial.println("Average Heart Rate : No valid reading");
    }

    // --------------------------------------------------------
    // MOTION DATA
    // --------------------------------------------------------

    Serial.println();
    Serial.println("----- MOTION DATA -----");

    Serial.print("Average Accel X : ");
    Serial.print(avgAccelX, 2);
    Serial.println(" g");

    Serial.print("Average Accel Y : ");
    Serial.print(avgAccelY, 2);
    Serial.println(" g");

    Serial.print("Average Accel Z : ");
    Serial.print(avgAccelZ, 2);
    Serial.println(" g");

    Serial.print("Average Total Acceleration : ");
    Serial.print(avgTotalAcceleration, 2);
    Serial.println(" g");

    Serial.print("Average Gyro X : ");
    Serial.print(avgGyroX, 2);
    Serial.println(" deg/s");

    Serial.print("Average Gyro Y : ");
    Serial.print(avgGyroY, 2);
    Serial.println(" deg/s");

    Serial.print("Average Gyro Z : ");
    Serial.print(avgGyroZ, 2);
    Serial.println(" deg/s");

    Serial.print("Average Total Gyro : ");
    Serial.print(avgTotalGyro, 2);
    Serial.println(" deg/s");

    // --------------------------------------------------------
    // IMPACT
    // --------------------------------------------------------

    Serial.print("Impact During 5 Seconds : ");

    if (impactInWindow)
    {
      Serial.println("YES");
    }
    else
    {
      Serial.println("NO");
    }

    // --------------------------------------------------------
    // LIGHT DATA
    // --------------------------------------------------------

    Serial.println();
    Serial.println("----- LIGHT DATA -----");

    Serial.print("Average Light ADC : ");
    Serial.println(avgLightRaw, 0);

    Serial.print("Average Light Voltage : ");
    Serial.print(avgLightVoltage, 2);
    Serial.println(" V");

    Serial.print("Light Level : ");
    Serial.println(avgLightLevel);
    // --------------------------------------------------------
// ALCOHOL DATA
// --------------------------------------------------------

Serial.println();
Serial.println("----- ALCOHOL SENSOR DATA -----");

Serial.print("MQ-3 Average ADC : ");
Serial.println(avgMQ3Raw, 0);

Serial.print("MQ-3 Average Voltage : ");
Serial.print(avgMQ3Voltage, 2);
Serial.println(" V");

Serial.print("Alcohol Detection : ");

if (alcoholDetected)
{
  Serial.println("ALCOHOL DETECTED");
}
else
{
  Serial.println("NO ALCOHOL DETECTED");
}

    // --------------------------------------------------------
    // SYSTEM STATUS
    // --------------------------------------------------------

    Serial.println();
    Serial.println("----- SAFERIDE+ STATUS -----");
    if (alcoholDetected)
{
  Serial.println("!!! ALCOHOL DETECTED - RIDER NOT SAFE !!!");
}
else
{
  Serial.println("Alcohol Status : NORMAL");
}

    if (impactInWindow)
    {
      Serial.println("!!! POSSIBLE IMPACT DETECTED !!!");

      // Trigger alert ONCE after the 5-second analysis
      triggerAlert();
    }
    else
    {
      Serial.println("Motion Status : NORMAL");
    }

    if (avgLightLevel == "BRIGHT LIGHT")
    {
      Serial.println("Visor Mode : ANTI-GLARE");
    }
    else if (avgLightLevel == "LOW LIGHT")
    {
      Serial.println("Visor Mode : CLEAR / LOW LIGHT");
    }
    else
    {
      Serial.println("Visor Mode : NORMAL");
    }

    Serial.println("================================================");
    Serial.println("       5 SECOND ANALYSIS COMPLETE");
    Serial.println("================================================");

    // ========================================================
    // RESET FOR NEXT 5-SECOND WINDOW
    // ========================================================

    sampleCount = 0;

    sumHeartRate = 0;
    validHeartSamples = 0;

    sumAccelX = 0;
    sumAccelY = 0;
    sumAccelZ = 0;
    sumTotalAcceleration = 0;

    sumGyroX = 0;
    sumGyroY = 0;
    sumGyroZ = 0;
    sumTotalGyro = 0;

    sumLightRaw = 0;
    sumLightVoltage = 0;

    impactInWindow = false;

    // Start new 5-second window
    windowStartTime = millis();

    Serial.println();
    Serial.println("Collecting next 5 seconds...");
  }

  // Small delay to prevent excessive serial/sensor polling
  delay(20);
}
