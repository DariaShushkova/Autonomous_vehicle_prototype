// Available pins for color sensors
#define S0_PIN A0
#define S1_PIN A1
#define S2_PIN A2
#define S3_PIN A3
#define SENSOR_OUT_PIN A5 // Renamed OUT to avoid confusion with 'OUTPUT' keyword

// These variables will store the raw frequency values read from the sensor for each color channel. 
// A lower frequency value a higher intensity for that color is being detected.
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

void setup() {

  Serial.begin(9600); 
  Serial.println("Color Sensor Test Initialized");

  // Configure the Pins
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  
  // The SENSOR_OUT_PIN reads the frequency from the sensor.
  pinMode(SENSOR_OUT_PIN, INPUT);
  
  // Frequency Scaling 100%
  // This provides the highest possible resolution for the readings.
  // S0=HIGH, S1=HIGH sets scaling to 100%.
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, HIGH);
}

void loop() {
  // Read the values from the three color channels
  readColorValues();

  // Print the raw frequency values
  Serial.print("R: ");
  Serial.print(redFrequency);
  Serial.print("  G: ");
  Serial.print(greenFrequency);
  Serial.print("  B: ");
  Serial.print(blueFrequency);

  // Print the detected color
  if (redFrequency < greenFrequency && redFrequency < blueFrequency) {
    Serial.println("  - RED");
  } else if (greenFrequency < redFrequency && greenFrequency < blueFrequency) {
    Serial.println("  - GREEN");
  } else if (blueFrequency < redFrequency && blueFrequency < greenFrequency) {
    Serial.println("  - BLUE");
  } else {
    Serial.println("  - Undefined/White/Black");
  }
  delay(500); 
}

// Reads the R, G, and B values from the sensor
void readColorValues() {
  // Set the sensor's filter to RED
  // S2=LOW, S3=LOW selects the red photodiode.
  digitalWrite(S2_PIN, LOW);
  digitalWrite(S3_PIN, LOW);
  redFrequency = pulseIn(SENSOR_OUT_PIN, LOW);
  delay(100);

  // Set the sensor's filter to GREEN
  // S2=HIGH, S3=HIGH selects the green photodiode.
  digitalWrite(S2_PIN, HIGH);
  digitalWrite(S3_PIN, HIGH);
  greenFrequency = pulseIn(SENSOR_OUT_PIN, LOW);
  delay(100);

  // Set the sensor's filter to BLUE
  // S2=LOW, S3=HIGH selects the blue photodiode.
  digitalWrite(S2_PIN, LOW);
  digitalWrite(S3_PIN, HIGH);
  blueFrequency = pulseIn(SENSOR_OUT_PIN, LOW);
  delay(100);
}
