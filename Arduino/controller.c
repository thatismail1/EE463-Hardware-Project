// Arduino Code for PI Controller and PWM Generation for Speed Control

// Constants
const int pwmPin = 9;           // PWM output pin for MOSFET driver
const int hallSensorPin = 2;    // Hall-effect sensor pin
const int pwmFrequency = 10000; // PWM frequency (10 kHz)
const float samplingInterval = 0.1; // Sampling interval for PI control (100 ms)
const int vref = 157;           // Reference speed in RPM (adjustable)

// PI Controller Constants
const float kp = 0.01;           // Proportional gain
const float ki = 0.02;           // Integral gain

// Variables
volatile unsigned int pulseCount = 0; // Pulse count from Hall sensor
float motorSpeed = 0;           // Measured speed in RPM
float error = 0;                // Speed error
float integral = 0;             // Integral term for PI controller
float controlOutput = 0;        // Output of PI controller
unsigned long lastTime = 0;     // Last control loop time

void setup() {
    // Initialize pins
    pinMode(pwmPin, OUTPUT);
    pinMode(hallSensorPin, INPUT_PULLUP);

    // Attach interrupt for Hall sensor
    attachInterrupt(digitalPinToInterrupt(hallSensorPin), countPulse, RISING);

    // Set PWM frequency
    analogWriteFrequency(pwmPin, pwmFrequency);

    // Start serial communication for debugging
    Serial.begin(9600);
}

void loop() {
    unsigned long currentTime = millis();

    // Run PI control loop at defined sampling interval
    if ((currentTime - lastTime) >= (samplingInterval * 1000)) {
        lastTime = currentTime;

        // Calculate motor speed in RPM
        noInterrupts();
        unsigned int pulses = pulseCount;
        pulseCount = 0;
        interrupts();
        motorSpeed = (pulses * 60.0) / (1.0 / samplingInterval); // RPM

        // Calculate PI controller output
        error = vref - motorSpeed;
        integral += error * samplingInterval;
        controlOutput = (kp * error) + (ki * integral);

        // Constrain control output to valid PWM range (0-255)
        controlOutput = constrain(controlOutput, 0, 255);

        // Output PWM signal
        analogWrite(pwmPin, (int)controlOutput);

        // Debugging output
        Serial.print("Speed (RPM): ");
        Serial.print(motorSpeed);
        Serial.print(" | Error: ");
        Serial.print(error);
        Serial.print(" | PWM: ");
        Serial.println((int)controlOutput);
    }
}

// Interrupt service routine for Hall sensor pulse counting
void countPulse() {
    pulseCount++;
}
