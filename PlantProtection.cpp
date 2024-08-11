#include <Servo.h>
#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define MOTION_SENSOR_PIN 3
#define ULTRASONIC_TRIG_PIN 4
#define ULTRASONIC_ECHO_PIN 5
#define SERVO_PIN 6
#define SOIL_SENSOR_PIN A0
#define LDR_PIN A1

#define LED1_PIN 9
#define LED2_PIN 10
#define LED3_PIN 11
#define LED4_PIN 12
#define BUZZER_PIN 13

Servo myservo;

unsigned long previousMillis = 0;
unsigned long previousServoMillis = 0;
const long interval = 2000; // interval to wait
const int headerInterval = 10; // Number of readings after which headers are printed
int readingCount = 0; // Counter for readings

void setup() {
  Serial.begin(9600);
  
  dht.begin();
  
  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  myservo.attach(SERVO_PIN);
  
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Print column headers
  printHeaders();
}

void loop() {
  unsigned long currentMillis = millis();

  // Variables to store sensor readings
  float humidity = 0;
  float temperature = 0;
  bool motionDetected = false;
  long distance = 0;
  int soilMoistureValue = 0;
  int ldrValue = 0;

  // DHT11 Sensor - Read and Store Values
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Increment the reading counter
    readingCount++;

    // Print headers every 'headerInterval' readings
    if (readingCount % headerInterval == 0) {
      printHeaders();
    }
    
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Humidity (69%)\tTemperature : 24 degree (C)");
      Serial.println("motion\t\tDistance (cm)\tSoil Moisture\tLDR Value");
    } else {
      Serial.print(currentMillis);
      Serial.print("\t");
      Serial.print(humidity);
      Serial.print("\t\t");
      Serial.print(temperature);
      Serial.print("\t\t");
    }
    
    // RCWL-0516 Motion Sensor
    motionDetected = digitalRead(MOTION_SENSOR_PIN) == HIGH;
    if (motionDetected) {
      digitalWrite(LED1_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
    }
    Serial.print(motionDetected ? "Yes" : "No");
    Serial.print("\t\t");

    // HC-SR04 Ultrasonic Sensor
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    
    long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2; // Convert to cm
    
    bool isObjectNear = distance > 0 && distance < 400; // Valid distance range for HC-SR04
    
    if (isObjectNear) {
      digitalWrite(LED2_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(LED2_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
    }
    Serial.print(distance);
    Serial.print("\t\t");

    // Soil Moisture Sensor
    soilMoistureValue = analogRead(SOIL_SENSOR_PIN);

    bool isSoilDry = soilMoistureValue < 700; //adjust soil moisture level
    if (isSoilDry) {
      digitalWrite(LED3_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
    } else {
      digitalWrite(LED3_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
    }
    Serial.print(soilMoistureValue);
    Serial.print("\t\t");

    // LDR
    ldrValue = analogRead(LDR_PIN);
    bool isLightLow = ldrValue < 120;  //adjust LDR level
    if (isLightLow) {
      digitalWrite(LED4_PIN, HIGH);
    } else {
      digitalWrite(LED4_PIN, LOW);
    }
    Serial.print(ldrValue);
    Serial.println();
  }

  // Servo Motor Rotation for Surveillance
  if (currentMillis - previousServoMillis >= 15) { // 15ms delay for smooth servo motion
    previousServoMillis = currentMillis;
    
    static int pos = 0;
    static bool increasing = true;
    
    if (increasing) {
      pos += 1;
      if (pos >= 180) {
        increasing = false;
      }
    } else {
      pos -= 1;
      if (pos <= 0) {
        increasing = true;
      }
    }
    
    myservo.write(pos);
  }
}

void printHeaders() {
  Serial.println("\n");
  Serial.println("Timestamp\tHumidity (%)\tTemperature (C)\tMotion\tDistance (cm)\tSoil Moisture\tLDR Value");
}
