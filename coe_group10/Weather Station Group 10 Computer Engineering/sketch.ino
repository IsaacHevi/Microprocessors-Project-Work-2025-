#define BLYNK_TEMPLATE_ID "TMPL2LkN5CqEI"
#define BLYNK_TEMPLATE_NAME "Weather Station"
#define BLYNK_AUTH_TOKEN "5UPoKFB8To5X8J5kMvaYzRZU-AJGTUqD"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Wi-Fi Credentials
char ssid[] = "Wokwi-GUEST";
char pass[] = "";

// Pin Definitions
#define DHTPIN 15          // DHT22 on GPIO 15
#define DHTTYPE DHT22      // DHT sensor type
#define POTPIN 34          // Potentiometer on GPIO 34
#define BUTTONPIN 13       // Push Button on GPIO 13
#define GREEN_LED 25       // Normal LED on GPIO 25
#define YELLOW_LED 26      // Warning LED on GPIO 26
#define RED_LED 27         // Danger LED on GPIO 27
#define BUZZER 32          // Buzzer on GPIO 32

// Blynk Virtual Pin Definitions (for LED widgets)
#define GREEN_LED_VPIN V3   // Green LED (Normal)
#define YELLOW_LED_VPIN V4  // Yellow LED (Warning)
#define RED_LED_VPIN V5     // Red LED (Danger)

// Sensor and Display Setup
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Button variables
int displayMode = 0;
bool lastButtonState = HIGH;
bool buttonHandled = false;

// Timing variables
unsigned long previousMillis = 0;
const long updateInterval = 1000;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Weather Station...");

  // Start Blynk connection
  Serial.println("Connecting to Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("Blynk connected!");

  // Initialize sensors and components
  dht.begin();
  Serial.println("DHT sensor initialized.");

  lcd.init();
  lcd.backlight();
  Serial.println("LCD initialized.");

  pinMode(BUTTONPIN, INPUT_PULLUP);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Welcome message
  lcd.setCursor(0, 0);
  lcd.print("Weather Station");
  delay(2000);
  lcd.clear();
}

void loop() {
  Blynk.run(); // Keep Blynk connected

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int pressureValue = analogRead(POTPIN);
  float pressure = map(pressureValue, 0, 4095, 900, 1100);

  // Debug sensor readings
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  // Check for invalid sensor readings
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Error: Failed to read from DHT sensor!");
    return; // Exit loop if sensor read fails
  }

  // Send data to Blynk (only if connected)
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, temperature);  // Send temperature to Blynk chart
    Blynk.virtualWrite(V1, humidity);     // Send humidity to Blynk chart
    Blynk.virtualWrite(V2, pressure);     // Send pressure to Blynk chart
    Serial.println("Data sent to Blynk.");
  } else {
    Serial.println("Blynk not connected!");
  }

  // Handle button toggle
  bool currentButtonState = digitalRead(BUTTONPIN);
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    if (!buttonHandled) {
      displayMode = (displayMode + 1) % 3;
      Serial.print("Button pressed. Display mode: ");
      Serial.println(displayMode);
      buttonHandled = true;
    }
  } else if (currentButtonState == HIGH) {
    buttonHandled = false;
  }
  lastButtonState = currentButtonState;

  // Update display every second
  if (millis() - previousMillis >= updateInterval) {
    previousMillis = millis();
    lcd.clear();

    if (displayMode == 0) {
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperature);
      lcd.print((char)223);
      lcd.print("C");
    } else if (displayMode == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print("%");
    } else if (displayMode == 2) {
      lcd.setCursor(0, 0);
      lcd.print("Pressure:");
      lcd.setCursor(0, 1);
      lcd.print(pressure);
      lcd.print(" hPa");
    }

    manageAlerts(temperature, humidity, pressure);
  }
}

// Manage LED and buzzer alerts
void manageAlerts(float temp, float hum, float press) {
  // Reset all LED states (both physical and Blynk LEDs)
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER, LOW);

  Blynk.virtualWrite(GREEN_LED_VPIN, 0);
  Blynk.virtualWrite(YELLOW_LED_VPIN, 0);
  Blynk.virtualWrite(RED_LED_VPIN, 0);

  // Temperature Alerts
  if (temp > 35) {
    digitalWrite(RED_LED, HIGH);
    Blynk.virtualWrite(RED_LED_VPIN, 255); // Turn ON Blynk red LED
    buzz(3, 200);
    Blynk.logEvent("high_temp_alert", "Temperature exceeded 35°C!");
    Serial.println("ALERT: High Temperature!");
  } else if (temp >= 31) {
    digitalWrite(YELLOW_LED, HIGH);
    Blynk.virtualWrite(YELLOW_LED_VPIN, 255);
    buzz(1, 500);
    Serial.println("WARNING: Elevated Temperature.");
  } else {
    digitalWrite(GREEN_LED, HIGH);
    Blynk.virtualWrite(GREEN_LED_VPIN, 255);
    Serial.println("Temperature Normal.");
  }

  // Humidity Alerts
  if (hum > 80 || hum < 20) {
    digitalWrite(RED_LED, HIGH);
    Blynk.virtualWrite(RED_LED_VPIN, 255);
    buzz(1, 1000);
    Blynk.logEvent("humidity_alert", "Humidity out of range!");
    Serial.println("ALERT: Humidity Out of Range!");
  }

  // Pressure Alerts
  if (press < 980) {
    digitalWrite(YELLOW_LED, HIGH);
    Blynk.virtualWrite(YELLOW_LED_VPIN, 255);
    buzz(1, 1500);
    Blynk.logEvent("low_pressure_alert", "Pressure dropped below 980 hPa!");
    Serial.println("WARNING: Low Pressure.");
  }
}

// Buzzer function
void buzz(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(duration);
    digitalWrite(BUZZER, LOW);
    delay(duration);
  }
}
