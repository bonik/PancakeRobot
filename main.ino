#include <Servo.h>

// Pins
const byte TOP_SWITCH_PIN              = 42;
const byte BOTTOM_SWITCH_PIN           = 40;
const byte SPATULA_SERVO_PIN           = 7;
const byte CRANE_MOTOR_PIN             = 6;
const byte OPENING_ARM_RELAY_PIN       = 36;
const byte CLOSING_ARM_RELAY_PIN       = 28;
const byte OPENING_DISPENSER_RELAY_PIN = 32;
const byte CLOSING_DISPENSER_RELAY_PIN = 30;
const byte OPENING_SERVER_RELAY_PIN    = 4;
const byte CLOSING_SERVER_RELAY_PIN    = 26;
const byte BLENDER_PIN                 = 5;
const byte BUTTON_PIN                  = 46;

// Motor speeds
const byte MOTOR_UP_SPEED = 200;
const byte MOTOR_DOWN_SPEED = 180;
const byte MOTOR_ZERO_SPEED = 188;

// Timings
const unsigned long DISPENSE_DURATION    = 900;
const unsigned long COOK_SIDE_A_DURATION = 1000L * 75;
const unsigned long COOK_SIDE_B_DURATION = 1000L * 55;

// States
enum state {
    resetCrane,
    resetArm,
    resetServer,
    resetSpatula,
    standby,
    openingDispenser,
    closingDispenser,
    cookingSideA,
    loweringArm,
    openingArm,
    raisingArm,
    closingArm,
    cookingSideB,
    loweringArmAgain,
    openingArmAgain,
    raisingArmAgain,
    closingArmAgain,
    serving,
    dropping
};   

String stateNames[22] = {
    "resetCrane",
    "resetArm",
    "resetServer",
    "resetSpatula",
    "standby",
    "openingDispenser",
    "closingDispenser",
    "cookingSideA",
    "loweringArm",
    "openingArm",
    "raisingArm",
    "closingArm",
    "cookingSideB",
    "loweringArmAgain",
    "openingArmAgain",
    "raisingArmAgain",
    "closingArmAgain",
    "serving",
    "dropping"
};

state currentState = resetCrane;

Servo spatula;

unsigned long previousMillis = 0;
unsigned long previousDispenserMotorMillis = 0;

String lastMessage;

void setup() {
    Serial.begin(9600);
    Serial.println("Setting up");

    pinMode(CRANE_MOTOR_PIN ,OUTPUT);
    pinMode(SPATULA_SERVO_PIN, OUTPUT);
    pinMode(BLENDER_PIN, OUTPUT);
    pinMode(OPENING_DISPENSER_RELAY_PIN, OUTPUT);
    pinMode(CLOSING_DISPENSER_RELAY_PIN, OUTPUT);
    pinMode(OPENING_ARM_RELAY_PIN, OUTPUT);
    pinMode(CLOSING_ARM_RELAY_PIN, OUTPUT);
    pinMode(OPENING_SERVER_RELAY_PIN, OUTPUT);
    pinMode(CLOSING_SERVER_RELAY_PIN, OUTPUT);

    pinMode(TOP_SWITCH_PIN, INPUT);
    pinMode(BOTTOM_SWITCH_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT);

    analogWrite(CRANE_MOTOR_PIN, 188);
    analogWrite(BLENDER_PIN, 130);

    digitalWrite(SPATULA_SERVO_PIN, LOW);
    digitalWrite(OPENING_DISPENSER_RELAY_PIN, LOW);
    digitalWrite(CLOSING_DISPENSER_RELAY_PIN, LOW);
    digitalWrite(OPENING_ARM_RELAY_PIN, LOW);
    digitalWrite(CLOSING_ARM_RELAY_PIN, LOW);
    digitalWrite(OPENING_SERVER_RELAY_PIN, LOW);
    digitalWrite(CLOSING_SERVER_RELAY_PIN, LOW);

    spatula.attach(SPATULA_SERVO_PIN);

    previousMillis = millis();
}

// activateRelay activates the requested relay until the given duration passes.
// Returns true on completion.
bool activateRelay(byte pin, unsigned long duration) {
    if (millis() - previousMillis >= duration) {
        previousMillis = millis();
        digitalWrite(pin, LOW);
        return true;
    } else {
        digitalWrite(pin, HIGH);
        return false;
    }
}

void log(String msg) {
    if (msg != lastMessage) {
        Serial.println(msg);
        lastMessage = msg;
    }
}

bool lowerCrane() {
    unsigned int topSwitchState = digitalRead(TOP_SWITCH_PIN);

    if (topSwitchState == HIGH) {
        analogWrite(CRANE_MOTOR_PIN, MOTOR_ZERO_SPEED);
        return true;
    } else {
        analogWrite(CRANE_MOTOR_PIN, MOTOR_DOWN_SPEED);
        return false;
    }
}

bool raiseCrane() {
    unsigned int bottomSwitchState = digitalRead(BOTTOM_SWITCH_PIN);

    if (bottomSwitchState == HIGH) {
        analogWrite(CRANE_MOTOR_PIN, MOTOR_ZERO_SPEED);
        return true;
    } else {
        analogWrite(CRANE_MOTOR_PIN, MOTOR_UP_SPEED);
        return false;
    }
}

bool closeServer() {
    return activateRelay(CLOSING_SERVER_RELAY_PIN, 2000);
}

bool openDispenser() {
    return activateRelay(OPENING_DISPENSER_RELAY_PIN, DISPENSE_DURATION);
}

bool closeDispenser() {
    return activateRelay(CLOSING_DISPENSER_RELAY_PIN, 2000);
}

bool openArm() {
    return activateRelay(OPENING_ARM_RELAY_PIN, 2000);
}

bool closeArm() {
    return activateRelay(CLOSING_ARM_RELAY_PIN, 5000);
}

bool serve() {
    return activateRelay(OPENING_SERVER_RELAY_PIN, 5000);
}

bool rotateSpatula(byte angle) {
    spatula.write(angle);
}

void loop() {
    // Read switches state
    unsigned int topSwitchState = digitalRead(TOP_SWITCH_PIN);
    unsigned int bottomSwitchState = digitalRead(BOTTOM_SWITCH_PIN);
    unsigned int buttonState = digitalRead(BUTTON_PIN);

    log(stateNames[currentState]);

    switch (currentState) {
        case resetCrane:
            if (raiseCrane()) {
                currentState = resetArm;
            }
            break;
        case resetArm:
            if (closeArm()) {
                currentState = resetServer;
            }
            break;
        case resetServer:
            if (closeServer()) {
                currentState = resetSpatula;
            }
            break;
        case resetSpatula:
            rotateSpatula(180);
            if (millis() - previousMillis >= 2000) {
                previousMillis = millis();
                currentState = standby;
            }
        case standby:
            buttonState = digitalRead(BUTTON_PIN);
            if (buttonState == 1) {
                previousMillis = millis();
                currentState = openingDispenser;
            }
            break;
        case openingDispenser:
            if (openDispenser()) {
                currentState = closingDispenser;
            }
            break;
        case closingDispenser:
            if (closeDispenser()) {
                currentState = cookingSideA;
            }
            break;
        case cookingSideA:
            Serial.println(millis() - previousMillis);
            if (millis() - previousMillis >= COOK_SIDE_A_DURATION) {
                previousMillis = millis();
                currentState = loweringArm;
            }
            break;
        case loweringArm:
            if (lowerCrane()) {
                previousMillis = millis();
                currentState = openingArm;
            }
            break;
        case openingArm:
            if (openArm()) {
                currentState = raisingArm;
            }
            break;
        case raisingArm:
            if (raiseCrane()) {
                previousMillis = millis();
                currentState = closingArm;
            }
            break;
        case closingArm:
            rotateSpatula(0);
            if (closeArm()) {
                rotateSpatula(180);
                currentState = cookingSideB;
            }
            break;
        case cookingSideB:
            if (millis() - previousMillis >= COOK_SIDE_B_DURATION) {
                previousMillis = millis();
                currentState = loweringArmAgain;
            }
            break;
        case loweringArmAgain:
            if (lowerCrane()) {
                previousMillis = millis();
                currentState = openingArmAgain;
            }
            break;
        case openingArmAgain:
            if (openArm()) {
                currentState = raisingArmAgain;
            }
            break;
        case raisingArmAgain:
            if (raiseCrane()) {
                previousMillis = millis();
                currentState = closingArmAgain;
            }
            break;
        case closingArmAgain:
            if (closeArm()) {
                currentState = serving;
            }
            break;
        case serving:
            if (serve()) {
                currentState = dropping;
            }
            break;
        case dropping:
            rotateSpatula(0);
            if (millis() - previousMillis >= 5000) {
                previousMillis = millis();
                currentState = resetArm;
            }
            break;
    }
}
