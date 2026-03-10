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
void  pidControl(void);
void  measure(void);
void  justMove(void);
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

	justMove();
	// pidControl();
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


void justMove() {
	int set1, set2, out1, out2;

	int setpoint, in;

	stepper1.setMaxSpeed(500);
	stepper1.setCurrentPosition(stepper1.currentPosition());
	stepper1.setSpeed(-20);

	enum init { RANGING1, MOVING1, RANGING2, MOVING2, DONE };
	enum init state = RANGING1;

	do {
		setpoint = sens_thresh.readRangeContinuousMillimeters();
		in		 = sens_in.readRangeContinuousMillimeters();
		if (sens_thresh.timeoutOccurred() || sens_in.timeoutOccurred() || setpoint > 1000 || in > 1000) continue;

		switch (state) {
			case RANGING1:
				if (setpoint - in <= -60) state = MOVING1;
				break;
			case MOVING1:
				stepper1.runSpeed();
				if (in <= setpoint) {
					state = RANGING2;
					set1 = setpoint, out1 = stepper1.currentPosition();
				}
				break;
			case RANGING2:
				if (setpoint - in >= 120) {
					stepper1.setSpeed(20);
					state = MOVING2;
				}
				break;
			case MOVING2:
				stepper1.runSpeed();
				if (in >= setpoint) {
					set2 = setpoint, out2 = stepper1.currentPosition();
					state = DONE;
				}
				break;
		}
	} while (state != DONE);

	Serial.print(set1);
	Serial.print(", ");
	Serial.print(out1);
	Serial.print(", ");
	Serial.print(set2);
	Serial.print(", ");
	Serial.println(out2);


	int		 speed = 80;
	uint64_t calcP = 20, calcL = millis();

	while (1) {
		stepper1.runSpeed();

		if (millis() > calcL + calcP) {
			calcL += calcP;

			setpoint = sens_thresh.readRangeContinuousMillimeters();
			int pos	 = stepper1.currentPosition();
			int targ = map(setpoint, set1, set2, out1, out2);
			// in		 = sens_in.readRangeContinuousMillimeters();

			// if (in > setpoint)

			// stepper1.moveTo(map(setpoint, set1, set2, out1, out2));
			if (abs(pos - targ) > 10) stepper1.setSpeed(pos - targ > 0 ? -speed : speed);
			else stepper1.setSpeed(0);
		}
	}
}

// Precompute notch coefficients once — do this at startup
struct NotchCoeffs {
	float b0, b1, b2, a1, a2;
} notch;

void initNotchFilter(float f_notch, float f_sample) {
	float w0 = 2.0f * PI * f_notch / f_sample;
	float r	 = 0.95f; // Close to 1 = narrow, deep notch
	notch.b0 = 1.0f;
	notch.b1 = -2.0f * cos(w0);
	notch.b2 = 1.0f;
	notch.a1 = -2.0f * r * cos(w0);
	notch.a2 = r * r;
}

float notch_x1 = 0, notch_x2 = 0, notch_y1 = 0, notch_y2 = 0;
float notchFilter(float input) {
	float output =
		notch.b0 * input + notch.b1 * notch_x1 + notch.b2 * notch_x2 - notch.a1 * notch_y1 - notch.a2 * notch_y2;
	notch_x2 = notch_x1;
	notch_x1 = input;
	notch_y2 = notch_y1;
	notch_y1 = output;
	return output;
}

void pidControl() {
	long	 v		  = 0;
	uint16_t setpoint = 200, in;
	float	 eFiltered, eFilteredLast = 0;
	float	 d, dRaw;
	float	 integral = 0;

	// Tune these — start with kd=0, increase slowly
	float kp = 1.2f, ki = 0, kd = 0.0f;
	float integralLimit = 200.0f;

	uint64_t calcP = 20;

	const float dt = calcP / 1000.0f; // 0.02s

	// Init notch for 1.25Hz at 50Hz sample rate
	initNotchFilter(1.25f, 1000.0f / calcP);

	stepper1.setMaxSpeed(500);
	stepper1.setSpeed(500);
	stepper1.setCurrentPosition(stepper1.currentPosition());
	stepper1.setBound(40);

	uint64_t calcL = millis();

	while (1) {
		stepper1.runSpeedBounded();

		if (millis() > calcL + calcP) {
			calcL += calcP;

			setpoint = sens_thresh.readRangeContinuousMillimeters();
			in		 = sens_in.readRangeContinuousMillimeters();
			if (sens_in.timeoutOccurred() || in >= 8190) continue;

			float e = (float)(setpoint - in);

			// Filter THEN differentiate
			eFiltered	  = notchFilter(e);
			dRaw		  = (eFiltered - eFilteredLast) / dt;
			d			  = 0.7f * d + 0.3f * dRaw; // low-pass on derivative
			eFilteredLast = eFiltered;

			// Integral with anti-windup clamp
			integral = constrain(integral + eFiltered * dt, -integralLimit, integralLimit);

			v = (long)constrain(kp * eFiltered + ki * integral + kd * d, -400, 400);

			Serial.print(in);
			Serial.print(",");
			Serial.print(setpoint);
			Serial.print(",");
			Serial.print(e);
			Serial.print(",");
			Serial.println(v);

			stepper1.setSpeed(v);
		}
	}
}