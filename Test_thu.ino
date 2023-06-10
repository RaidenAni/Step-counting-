#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <DNSServer.h> 
#include <WiFiManager.h>
#include <HTTPClient.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

// OLED display width, in pixels
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define BUTTON_PIN 5
#define BUTTON_PIN_2 4
int buttonPinPressed = 0;         // Variable to store the button state

unsigned long debounceDelay = 250;    // Debounce delay in milliseconds

//Define button state
int buttonState = HIGH;    // Current state of the button
int lastButtonState = HIGH; // Previous state of the button
unsigned long lastDebounceTime = 0;  // Time of the last button state change

//Define button state
int buttonState1 = HIGH;    // Current state of the button
int lastButtonState1 = HIGH; // Previous state of the button
unsigned long lastDebounceTime1 = 0;  // Time of the last button state change

//Define button state
int buttonState2 = HIGH;    // Current state of the button
int lastButtonState2 = HIGH; // Previous state of the button
unsigned long lastDebounceTime2 = 0;  // Time of the last button state change

//Define variables for heartbeat counting
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// Define variables for step counting
int steps = -1; // number of steps
float distance_onestep = 64; //distance for one step 
float Kcal = 0.03584; // == (0,7 m/step) * (56kcal/km) / (1000 m)
float distance;
float kcal;
 
// Define variables for algorithm
bool peak = false; // flag for peak detection
#define WINDOW_SIZE 10 // Define the window size
#define THRESHOLD_FACTOR 0.7 // Define the threshold factor
float Accel_window[WINDOW_SIZE]; // Declare an array to store the acceleration values
float Accel_threshold; // Declare a variable to store the threshold value
int index_x = 0; // Declare a variable to store the index of the array

#define MIN_STEP_INTERVAL 250 // Define the minimum step interval in milliseconds
unsigned long last_step_time = 0; // Declare a variable to store the last step time
unsigned long current_time; // Declare a variable to store the current time

template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

// Filter instance
LowPass<2> lp(3,50,true);

//Setup DatatoSheet
String server = "http://maker.ifttt.com";
String eventName = "Steps";
String IFTTT_Key = "eOMCLR5uZa7PRRR-IEeGZN3XbIywMyhRwvN9HtRek-X";
String IFTTTUrl="https://maker.ifttt.com/trigger/Steps/with/key/eOMCLR5uZa7PRRR-IEeGZN3XbIywMyhRwvN9HtRek-X";

int value1;
int value2;
int value3;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  pinMode(BUTTON_PIN_2, INPUT_PULLUP); // set ESP32 pin to input pull-up mode

  Wire.begin(21, 22);
  
  Wire.beginTransmission(0x53);
  Wire.write(0x2C); 
  Wire.write(0x08); 
  Wire.endTransmission();

  Wire.beginTransmission(0x53);
  Wire.write(0x31); 
  Wire.write(0x08); 
  Wire.endTransmission();
  
  Wire.beginTransmission(0x53);
  Wire.write(0x2D); 
  Wire.write(0x08); 
  Wire.endTransmission();
  
  //OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  // MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println(F("MAX30105 was not found. Please check wiring/power."));
    display.display();
    delay(2000);
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(F("Starting..."));
  display.display();
  delay(2000);
}

void loop() {
  displayStepDistanceCalories();
  Heartbeat();
  
  Wire.beginTransmission(0x53);
  Wire.write(0x32); 
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  byte x0 = Wire.read();
  
  Wire.beginTransmission(0x53);
  Wire.write(0x33); 
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  byte x1 = Wire.read();
  x1 = x1 & 0x03;
  
  uint16_t x = (x1 << 8) + x0;
  int16_t xf = x;
  if(xf > 511){
    xf = xf - 1024;
  }
  float xa = xf * 0.004;
  
  Wire.beginTransmission(0x53);
  Wire.write(0x34); 
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  byte y0 = Wire.read();
  
  Wire.beginTransmission(0x53);
  Wire.write(0x35); 
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  byte y1 = Wire.read();
  y1 = y1 & 0x03;
  
  uint16_t y = (y1 << 8) + y0;
  int16_t yf = y;
  if(yf > 511){
    yf = yf - 1024;
  }
  float ya = yf * 0.004;

  Wire.beginTransmission(0x53);
  Wire.write(0x36); 
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  byte z0 = Wire.read();
  
  Wire.beginTransmission(0x53);
  Wire.write(0x37); 
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  byte z1 = Wire.read();
  z1 = z1 & 0x03;
  
  uint16_t z = (z1 << 8) + z0;
  int16_t zf = z;
  if(zf > 511){
    zf = zf - 1024;
  }
  float za = zf * 0.004;
  
  float Accel = sqrt(xa*xa+ya*ya+za*za);
  float Accel_f = lp.filt(Accel);

  Serial.print("Before: ");
  Serial.print(Accel);
  Serial.print(" After: ");
  Serial.println(Accel_f);
  
  // Update the array with the new value
  Accel_window[index_x] = Accel_f;
  index_x = (index_x + 1) % WINDOW_SIZE; // increment and wrap around

  // Calculate the average or maximum value in the array
  float Accel_avg = 0;
  float Accel_max = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    Accel_avg += Accel_window[i];
      if (Accel_window[i] > Accel_max) {
        Accel_max = Accel_window[i];
  }
}
  Accel_avg /= WINDOW_SIZE;

  // Set the threshold value based on the average or maximum value
  Accel_threshold = Accel_avg * THRESHOLD_FACTOR; // or Accel_max * THRESHOLD_FACTOR

  // Apply the peak detection algorithm with the dynamic threshold
  if (Accel_f > Accel_threshold && !peak) {
    peak = true; // set the peak flag
    // Get the current time
    current_time = millis();

    // Check if the current time is greater than the last step time plus the minimum step interval
    if (current_time > last_step_time + MIN_STEP_INTERVAL) {
      steps++; // increment the step count
      last_step_time = current_time; // update the last step time
  }
}
  if (Accel_f < Accel_threshold && peak) {
    peak = false; // clear the peak flag
}
  distance = steps*distance_onestep/100;
  kcal = steps*Kcal;

  // Checking Wifi
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        // Button is pressed
        buttonPinPressed = 1;
        while (buttonPinPressed == 1) {
          displayConfirmationScreen();
          delay(75);
          ConfirmWifi();
          Cancelsending();
        }
      }
    }
  }
  lastButtonState = reading;
}

bool configureWiFi() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Connecting to WiFi...");
  display.display();

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(60);
  wifiManager.setConnectTimeout(10);
  wifiManager.setSaveConfigCallback([]() {
    display.println("Saving WiFi credentials...");
    display.display();
  });

  if (!wifiManager.startConfigPortal("Pedometer_2NDV")) {
    display.println("Failed to connect to WiFi");
    display.display();
    delay(2000);
    ESP.restart();
  }

  display.println("Connected to WiFi");
  display.println("IP address: " + WiFi.localIP().toString());
  display.display();
  delay(2000);
}

// Function to handle button press
void ConfirmWifi() {
  int reading1 = digitalRead(BUTTON_PIN);

  if (reading1 != lastButtonState1) {
    lastDebounceTime1 = millis();
  }

  if ((millis() - lastDebounceTime1) > debounceDelay) {
    if (reading1 != buttonState1) {
      buttonState1 = reading1;

      if (buttonState1 == LOW) {
        // Button is pressed
        configureWiFi(); // Call WiFi configuration function
        sendDataToSheet(); // Send data to Google Sheet
        buttonPinPressed = 0;
      }
    }
  }

  lastButtonState1 = reading1;
}

// Function to handle button press
void Cancelsending() {
  int reading2 = digitalRead(BUTTON_PIN_2);

  if (reading2 != lastButtonState2) {
    lastDebounceTime2 = millis();
  }

  if ((millis() - lastDebounceTime2) > debounceDelay) {
    if (reading2 != buttonState2) {
      buttonState2 = reading2;

      if (buttonState2 == LOW) {
        // Button is pressed
        displayStepDistanceCalories();
        buttonPinPressed = 0;
      }
    }
  }

  lastButtonState2 = reading2;
}

void sendDataToSheet() {
  WiFiClient client;
  HTTPClient http;

  String url = "http://maker.ifttt.com/trigger/" + eventName + "/with/key/" + IFTTT_Key;
  String payload = "value1=" + String(steps) + "&value2=" + String(distance) + "&value3=" + String(kcal);

  display.println("Sending data...");
  display.display();

  if (http.begin(client, url)) {
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    int httpCode = http.POST(payload);
    if (httpCode == HTTP_CODE_OK) {
      display.println("Complete!");
    } else {
      display.println("Failed to send data");
    }

    http.end();
  } else {
    display.println("Failed to connect to server");
  }

  display.display();
  delay(2000);
}

void displayConfirmationScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Do you want to send  data online?");
  display.println("");  
  display.println("1. Yes");
  display.println("2. No");
  display.println("");
  display.display();
}

void displayStepDistanceCalories() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Steps: " + String(steps));
  display.println("Distance: " + String(distance) + " m");
  display.println("Energy: " + String(kcal) + " Kcal");
  display.println("Heartrate: " + String(beatAvg) + " BPM");
  display.display();
}

void Heartbeat(){
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
}
