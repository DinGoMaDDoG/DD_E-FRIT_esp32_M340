/*
 * DD-E-FRIT speed control of a DC motor on an ESP32
 *
 * Every 100 ms the firmware measures the motor speed from a quadrature
 * encoder, runs one step of the I-PD controller (PIDcontroller.h) and writes
 * the new PWM duty cycle to the H-bridge. The setpoint steps through
 * 75 -> 125 -> 175 -> 225 RPM, 30 s each, then the motor is stopped.
 *
 * Two kinds of experiment are built from the same code (platformio.ini:
 * `pio run` for the first, `pio run -e dd_efrit` for the second):
 *   USE_DD_EFRIT 0  fixed I-PD gains. This "initial run" provides the
 *                   closed-loop data that the offline E-FRIT tuning
 *                   (algorithm/esp32_dd_efrit_tuning.ipynb) turns into the
 *                   gain database in dataset.h.
 *   USE_DD_EFRIT 1  before each control step the gains are looked up in the
 *                   database (k-nearest-neighbour query), and the time taken
 *                   by the lookup is printed.
 *
 * Serial output (115200 baud), one block per sample:
 *   >Setpoint:..,RPM:..,PWM:..    for the Teleplot / serial plotter view
 *   DATA,sp,rpm,pwm,kp,ki,kd      parsed by esp32/tools/parse_serial_log.py
 */

#include <Arduino.h>
#include <PIDcontroller.h>
#include <dataset.h>

#ifndef USE_DD_EFRIT
#define USE_DD_EFRIT 0
#endif

// H-bridge driver
int motorpin1=17; // IN1 (TX2)
int motorpin2=5;  // IN2 (D5)
int enablePin=16; // EN, PWM output (RX2)

// Quadrature encoder
int greenSignal=2;  // channel A, counted on the rising edge
int yellowSignal=4; // channel B, gives the direction

// PWM: 5 kHz, 8-bit duty cycle (0-255)
const int freq=5000;
const int pwmChannel=0;
const int resolution=8;

const unsigned long SAMPLE_PERIOD_MS=100;
// rpm = pulses per sample * (samples per minute) / (pulses per revolution)
const double SAMPLES_PER_MINUTE=600.0; // 60 s / 0.1 s
const double PULSES_PER_REV=231.0;     // encoder pulses per output-shaft revolution

volatile long pulse_count;
unsigned long previous_millis;
float rpm;
int newPWM;
unsigned long experiment_start_time=0;
bool experiment_running=true;

// Encoder interrupt: count up or down depending on channel B.
void ARDUINO_ISR_ATTR Pulse_counter () {
    if (digitalRead(yellowSignal)==HIGH){pulse_count++;}
    else {pulse_count--;}
}


// Initial gains Kp, Ki, Kd and the PWM output limits.
PIDController motorPID(0.8, 1.5, 0.05, 0, 255);

void setup() {
    pinMode(motorpin1, OUTPUT);
    pinMode(motorpin2, OUTPUT);
    pinMode(enablePin, OUTPUT);

    // Both direction inputs LOW, so the enable pulse does not turn the motor.
    digitalWrite(motorpin1, LOW);
    digitalWrite(motorpin2, LOW);
    ledcSetup(pwmChannel, freq, resolution);
    ledcAttachPin(enablePin, pwmChannel);
    ledcWrite(pwmChannel, 255);
    delay(3000);
    ledcWrite(pwmChannel, 0);

    experiment_running=true;
    motorPID.reset();

    noInterrupts();
    pulse_count=0;
    interrupts();

    previous_millis=0;

    pinMode(greenSignal, INPUT_PULLUP);
    pinMode(yellowSignal, INPUT_PULLUP);

    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(greenSignal), Pulse_counter, RISING);

    // Forward direction
    digitalWrite(motorpin1, HIGH);
    digitalWrite(motorpin2, LOW);

    // Wait for a key press so the serial log starts with the experiment.
    Serial.println("Send any key to start experiment...");
    while (!Serial.available()) { delay(10); }
    Serial.read();
    Serial.println("Starting!");

    motorPID.setPoint=75;
    experiment_start_time=millis();
    previous_millis=millis();
}


void loop() {
    if (!experiment_running) return;

    unsigned long current_millis=millis();

    if (current_millis-previous_millis>=SAMPLE_PERIOD_MS){
        float dt=(current_millis-previous_millis)/1000.0;
        previous_millis+=SAMPLE_PERIOD_MS;

        // Read and clear the pulse count of the last sample period.
        noInterrupts();
        long current_pulses=pulse_count;
        pulse_count=0;
        interrupts();

        rpm=(current_pulses*SAMPLES_PER_MINUTE)/PULSES_PER_REV;

        // Setpoint schedule: four 30 s steps, then stop the motor.
        unsigned long elapsed=current_millis-experiment_start_time;
        if (elapsed<30000) {
            motorPID.setPoint=75.0;
        } else if (elapsed<60000) {
            motorPID.setPoint=125.0;
        } else if (elapsed<90000) {
            motorPID.setPoint=175.0;
        } else if (elapsed<120000) {
            motorPID.setPoint=225.0;
        } else {
            digitalWrite(motorpin1, LOW);
            digitalWrite(motorpin2, LOW);
            ledcWrite(pwmChannel, 0);
            experiment_running=false;
            Serial.println("EXPERIMENT_COMPLETE");
            return;
        }

#if USE_DD_EFRIT
        // Look up the gains for the current operating point and time it.
        unsigned long start_us=micros();
        motorPID.updateWeightsandPID(motorPID.setPoint, rpm);
        unsigned long end_us=micros();
#endif

        newPWM=(int)motorPID.compute(rpm, dt);

        ledcWrite(pwmChannel, newPWM);

        Serial.print(">Setpoint:"); Serial.print(motorPID.setPoint);
        Serial.print(",RPM:");      Serial.print(rpm);
        Serial.print(",PWM:");      Serial.println(newPWM);
        Serial.print("DATA,");
        Serial.print(motorPID.setPoint); Serial.print(",");
        Serial.print(rpm); Serial.print(",");
        Serial.print(newPWM); Serial.print(",");
        Serial.print(motorPID.kp); Serial.print(",");
        Serial.print(motorPID.ki); Serial.print(",");
        Serial.println(motorPID.kd);
#if USE_DD_EFRIT
        Serial.print("Algorithm Time (us): ");
        Serial.println(end_us-start_us);
#endif
    }
}
