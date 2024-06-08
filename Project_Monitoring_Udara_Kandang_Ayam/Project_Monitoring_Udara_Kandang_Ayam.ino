// Include the libraries
#include <MQUnifiedsensor.h>
#include <ESP32Servo.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>

// Definitions
#define placa "ESP-32"
#define Voltage_Resolution 3.3
#define pin 34 // Analog input 34 of your ESP32
#define type "MQ-135" // MQ135
#define ADC_Bit_Resolution 12 // For ESP32
#define RatioMQ135CleanAir 3.6 // RS / R0 = 3.6 ppm
#define buzzerPin 14 // Pin for the buzzer
#define servoPin 15 // Pin for the servo
#define DHTPin 13 // Pin for the DHT22 sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)

// Declare Sensors
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
DHT dht(DHTPin, DHTTYPE);

// Declare Servo
Servo myServo;

void setup() {
  // Init the serial port communication - to debug the library
  Serial.begin(9600); // Init serial port

  // Set buzzer pin as output
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW); // Ensure buzzer is off initially

  // Attach servo to the specified pin
  myServo.attach(servoPin);
  myServo.write(0); // Set initial position to 0 degrees

  // Initialize DHT22 sensor
  dht.begin();

  // Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); // _PPM = a * ratio^b
  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration

  /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen   | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton   | 34.668 | -3.369
  */

  MQ135.init(); 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update(); // Update data, the Arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!");
  
  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1);
  }
  MQ135.serialDebug(true);
}

void loop() {
  // Read temperature and humidity from DHT22 sensor
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Print temperature and humidity to serial monitor
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");

  // Read CO2 concentration
  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration
  MQ135.update(); // Update data for CO2
  float co2_ppm = MQ135.readSensor(); // Sensor will read CO2 PPM concentration
  Serial.print("CO2: ");
  Serial.print(co2_ppm);
  Serial.println(" ppm");

  // Read NH3 concentration
  MQ135.setA(102.2); MQ135.setB(-2.473); // Configure the equation to calculate NH3 concentration
  MQ135.update(); // Update data for NH3
  float nh3_ppm = MQ135.readSensor(); // Sensor will read NH3 PPM concentration
  Serial.print("NH3: ");
  Serial.print(nh3_ppm);
  Serial.println(" ppm");

  // Check conditions for gas concentrations, temperature, and humidity
  bool gasAlert = (co2_ppm > 5) || (nh3_ppm > 5);
  bool tempHumidityAlert = (temperature > 28) && (humidity > 65);

  // Control buzzer
  if (gasAlert) {
    digitalWrite(buzzerPin, HIGH); // Turn on the buzzer
  } else {
    digitalWrite(buzzerPin, LOW); // Turn off the buzzer
  }

  // Control servo
  if (tempHumidityAlert) {
    myServo.write(90); // Turn servo to 90 degrees
  } else {
    myServo.write(0); // Return servo to 0 degrees
  }

  MQ135.serialDebug(); // Will print the table on the serial port
  delay(3000); // Sampling frequency
}
