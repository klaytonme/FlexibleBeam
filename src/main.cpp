#include <Arduino.h>
// #include <Stepper.h>
#include <AccelStepper.h>
#include <VL53L0X.h>
#include <Wire.h>

// uint8_t STEP_DIR_PIN = 5, STEP_PULSE_PIN = 2, STEP_EN_PIN = 8;
uint8_t SENS_TH_XOUT_PIN = 6;

uint64_t pulseL = 0, pulseP = 20000;
uint64_t thL = 0, thP = 1000;

void  oscillate(void);
void  control(void);
void  measure(void);
float notchFilter(float, float);


bool stepPulseState = LOW;

VL53L0X sens_in;
VL53L0X sens_thresh;


const int	 stepsPerRevolution = 200; // change this to fit the number of steps per revolution
AccelStepper stepper1			= AccelStepper(AccelStepper::DRIVER, 2, 5);


void setup() {
	Serial.begin(115200);

	pinMode(SENS_TH_XOUT_PIN, OUTPUT);

	Wire.begin();

	stepper1.setEnablePin(8);
	stepper1.setPinsInverted(false, false, true);
	stepper1.enableOutputs();

	stepper1.setMaxSpeed(500);
	stepper1.setAcceleration(800);

	// turn off threshold sensor to initialize measurement sensor
	digitalWrite(SENS_TH_XOUT_PIN, LOW);
	delay(500);
	sens_in.setTimeout(500);
	if (!sens_in.init()) {
		Serial.println("In sensor init failed!");
		while (1)
			;
	}
	sens_in.setAddress(0x0F);
	sens_in.setMeasurementTimingBudget(20000);
	sens_in.startContinuous(); // non-blocking reads

	digitalWrite(SENS_TH_XOUT_PIN, HIGH);
	sens_thresh.setTimeout(500);
	if (!sens_thresh.init()) {
		Serial.println("Threshold sensor init failed!");
		while (1)
			;
	}
	sens_thresh.setMeasurementTimingBudget(20000);
	sens_thresh.startContinuous();

	control();
	// oscillate();
	// measure();
}

void loop() {}


void testBounds() {
	stepper1.setMaxSpeed(50);
	stepper1.setSpeed(50);
	stepper1.setCurrentPosition(stepper1.currentPosition());
	stepper1.setBound(50);

	while (1)
		stepper1.runSpeedBounded();
}


void measure() {
	while (1) {
		uint16_t in	 = sens_in.readRangeContinuousMillimeters();
		uint16_t set = sens_thresh.readRangeContinuousMillimeters();

		Serial.print("in: ");
		Serial.print(in);
		Serial.print("\tthresh: ");
		Serial.println(set);
	}
}


void oscillate() {
	int pos = 50;

	while (1) {
		stepper1.runSpeedToPosition();

		if (stepper1.distanceToGo() == 0) {
			pos = -pos;
			stepper1.moveTo(pos); // run "forever"
			stepper1.setSpeed(500);
			delay(1000);
		}
	}
}

// Steady-state Kalman filter — all matrices pre-solved offline.
// P has converged, so gain K is constant. The predict+update cycle
// reduces to a single matrix-vector multiply with baked-in coefficients.
//
// From Riccati solution (run once in Python, never again on the MCU):
//   K0 = 0.096984  (position correction gain)
//   K1 = 0.015938  (velocity correction gain)
//   A  = (I - K*H) * F  (combined predict+update matrix)

struct KalmanFilter {
	float pos = 209.0f; // estimated position (mm)
	float vel = 0.0f;	// estimated velocity (mm/s)

	void update(float measurement, float equilibrium) {
		float m		  = measurement - equilibrium;
		float p		  = pos - equilibrium;
		float new_pos = 0.891847f * p + 0.017670f * vel + 0.096984f * m;
		float new_vel = -1.242784f * p + 0.952652f * vel + 0.015938f * m;
		pos			  = constrain(new_pos + equilibrium, 0.0f, 500.0f);
		vel			  = constrain(new_vel, -2000.0f, 2000.0f);
	}
};

// ---- PID ----
struct PID {
	float kp, kd;

	PID(float p, float d, float ilim) : kp(p), kd(d) {}

	float compute(float error, float velocity, float dt) {
		// D term uses Kalman velocity — clean, no finite-difference noise
		return kp * error + kd * (-velocity);
	}
};

KalmanFilter kf;
PID			 pid(1.5f, 0.5f, 300.0f);

void control() {
	uint16_t	   setpoint = 200;
	const uint32_t calcP	= 20;
	const float	   dt		= 0.02f;

	stepper1.setMaxSpeed(500);
	stepper1.setSpeed(0);
	stepper1.setCurrentPosition(stepper1.currentPosition());
	stepper1.setBound(40);

	kf.pos		   = sens_in.readRangeContinuousMillimeters();
	uint64_t calcL = millis();

	while (1) {
		stepper1.runSpeedBounded();

		if (millis() >= calcL + calcP) {
			calcL += calcP;

			setpoint		= sens_thresh.readRangeContinuousMillimeters();
			uint16_t raw_in = sens_in.readRangeContinuousMillimeters();

			if (!sens_in.timeoutOccurred() && raw_in < 1000) {
				kf.update((float)raw_in, (float)setpoint);
			} else {
				// Let the physics model coast — don't corrupt state with 8191
				float new_pos = 0.891847f * (kf.pos - setpoint) + 0.017670f * kf.vel;
				float new_vel = -1.242784f * (kf.pos - setpoint) + 0.952652f * kf.vel;
				kf.pos		  = constrain(new_pos + setpoint, 0.0f, 500.0f);
				kf.vel		  = constrain(new_vel, -2000.0f, 2000.0f);
			}

			float error = (float)setpoint - kf.pos;
			float v		= pid.compute(error, kf.vel, dt);

			Serial.print(raw_in);
			Serial.print(", ");
			Serial.print(setpoint);
			Serial.print(", ");
			Serial.print(error);
			Serial.print(", ");
			Serial.print(kf.pos);
			Serial.print(", ");
			Serial.print(kf.vel);
			Serial.print(", ");
			Serial.println(v);

			stepper1.setSpeed((long)constrain(v, -500.0f, 500.0f));
		}
	}
}