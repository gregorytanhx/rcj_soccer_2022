#include <Adafruit_BNO055_t4.h>
#include <Arduino.h>
#include <Bluetooth.h>
#include <Common.h>
#include <Config.h>
#include <EEPROM.h>
#include <Light.h>
#include <Motor.h>
#include <PID.h>
#include <Pins.h>
#include <Point.h>
#include <Wire.h>
#include <utility/imumaths.h>

Light light;
LineData lineData;
MoveData moveData;
Motors motors;
BallData ballData;
Bluetooth bt;
BluetoothData btData;
BBox bbox;

float lastLineAngle = 0;
bool lineTrack = false;
bool lineAvoid = true;
bool hasBall = false;

TOFBuffer tof;
IMU imu;
float heading;
float robotAngle;
float frontTOF, backTOF, leftTOF, rightTOF;

Timer kickerTimer(500);
Timer bluetoothTimer(BLUETOOTH_UPDATE_TIME);

Role role = Role::undecided;
Role defaultRole;
uint8_t robotID;

Point botCoords(0, 0);
Point relBallCoords(0, 0);
Point absBallCoords(0, 0);

PID coordPID(COORD_KP, COORD_KI, COORD_KD);
PID goaliePID(GOALIE_KP, GOALIE_KI, GOALIE_KD);


// initialise neutral point coordinates
// Point neutralPoints[] = {
//     Point(10, 10);
//     ...
// };

Role currentRole() {
    // will tell the robot if its supposed to attack or defend
    if (bt.isConnected) {
        // if role is undecided, pick default, else return the role that has been previously assigned
        return Role::undecided ? defaultRole : role;
    } else if (bt.previouslyConnected) {
        // robot will come back in as goalie
        return Role::defend;
    } else {
        // if robot is only one on field, default to striker
        return Role::attack
    }
}

void dribble() { 
    // use pwm to control dribbler
    analogWrite(DRIBBLER_PIN, 64); 
}

void kick() {
    if (kickerTimer.timeHasPassed()) 
        digitalWriteFast(KICKER_PIN, HIGH);
    else 
        digitalWriteFast(KICKER_PIN, LOW);
}

void setMove(float speed, float angle, float rotation) {
    moveData.speed.val = speed;
    moveData.angle.val = angle;
    moveData.rotation.val = rotation;
}

void sendLayer1() {
    L1Serial.write(LAYER1_REC_SYNC_BYTE);
    L1Serial.write(moveData.speed.b, 4);
    L1Serial.write(moveData.angle.b, 4);
    L1Serial.write(moveData.rotation.b, 4);
    L1Serial.write(lineTrack);
    L1Serial.write(lineAvoid);
}

void readLayer1() {
    while (L1Serial.available() >= LAYER1_SEND_PACKET_SIZE) {
        uint8_t syncByte = L1Serial.read();
        if (syncByte == LAYER1_REC_SYNC_BYTE) {
            for (int i = 0; i < 4; i++) {
                lineData.lineAngle.b[i] = L1Serial.read();
            }
            for (int i = 0; i < 4; i++) {
                lineData.chordLength.b[i] = L1Serial.read();
            }
            lineData.onLine = L1Serial.read();
        }
    }
}

void readLayer4() {
    while (L4Serial.available() >= LAYER4_PACKET_SIZE) {
        uint8_t syncByte = L4Serial.read();
        if (syncByte == LAYER4_SYNC_BYTE) {
            for (int i = 0; i < LAYER4_PACKET_SIZE - 1; i++) {
                tof.b[i] = L4Serial.read();
            }
        }
    }
}

void processTOF() {
    // TODO: convert TOF distances to bounding box of robot on field
    // use kalman filter to detect if TOF is temporarily blocked?
    bbox.update(tof);
    botCoords.x = bbox.x;
    botCoords.y = bbox.y; 
}

void trackBall() {
    float ballOffset;
    if (ballData.angle < 180)
        ballOffset = fmin(ballData.angle * 0.96, 90);
    else 
        ballOffset = max((360 - ballData.angle) * 0.96, -90);
    
    float factor = 1 - ballData.dist / 520;
    float ballMult = fmin(1, 0.0134 * exp(factor * 2.6));

    robotAngle = ballData.angle + ballMult * ballOffset;
    setMove(SPEED, robotAngle, 0);
     
}

bool readLightGate() {
    return analogRead(LIGHT_GATE_PIN) >= LIGHT_GATE_THRESH;
} 

void updateBallData() {    
    ballData.visible = camera.ballVisible;
    ballData.captured = readLightGate();
    if (ballData.visible) {
        relBallCoords = Point(camera.ballAngle, camera.ballDist);
        absBallCoords = relBallCoords + botCoords;
        ballData.angle = camera.ballAngle;
        ballData.dist = camera.ballDist;
        ballData.x = absBallCoords.x;
        ballData.y = absBallCoords.y;

    } else if (bt.otherData.ballData.visible) {
        // find ball position based on other robot's data
        // derive angle and distance of ball relative to robot based on its absolute coordinates

        absBallCoords = Point(bt.otherData.ballData.x, bt.otherData.ballData.y);
        relBallCoords = absBallCoords - botCoords;
        ballData.angle = relBallCoords.getAngle();
        ballData.dist = relBallCoords.getDist();
        // treat ball as visible 
        ballData.visible = true;
    } 
}

void trackGoal() {
    float goalOffset;
    if (camera.oppAngle < 180) 
        goalOffset = fmin(camera.oppAngle, 90);
    else 
        goalOffset = max((360 - camera.oppAngle), -90);

    float goalMult = 1.5;
    robotAngle = camera.oppAngle + goalMult * goalOffset;
    setMove(SPEED, robotAngle, 0);
}

void defend() {
    // align robot to x-coordinate of ball while tracking line 
    if (lineData.onLine()) {

        if (abs(ballData.x) < GOALIE_LEEWAY_DIST) {
            // stop once ball is within certain horizontal distance
            float moveSpeed = 0;
        } else {
            float moveSpeed = max(goaliePID.update(abs(ballData.x)), MIN_SPEED);
        }
            
        float moveAngle = (ballData.angle > 180) ? -90 : 90;
        lineTrack = true;
    } else {
        Point target = Point(ballData.X, GOALIE_HOME_Y);
        goTo(target);
    }
}

void goTo(Point target) {
    // go to point based on robot's coordinates
    Point moveVector = target - botCoords;
    if (moveVector.getDist() < COORD_LEEWAY_DIST) {
        // stop once robot is close to target position
        float moveSpeed = 0;
    } else {
        float moveSpeed = max(coordPID.update(moveVector.getDist()), MIN_SPEED);
    }
    // adjust speed based on confidence in position along each axis
    float moveAngle = moveVector.getAngle();
    moveSpeed = distance(moveSpeed * sin(rad2deg(moveAngle)) * bbox.Xconfidence, 
                         moveSpeed * cos(rad2deg(moveAngle)) * bbox.Yconfidence); 
    setMove(moveSpeed, moveAngle, 0);
}

void goToWithCam(Point target) {
    // go to point based on vector calculations
    Point oppGoalVec(Camera.oppGoalAngle, Camera.oppGoalDistance);
    Point ownGoalVec(Camera.ownGoalAngle, Camera.ownGoalDistance);

    // vector pointing to the centre of the field
    Point centre = oppGoalVec + ownGoalVec;
    centre.distance /= 2;
    Point moveVector = centre + target;
    float moveSpeed = (moveVector.getDist() < COORD_LEEWAY_DIST) ? 0 : max(coordPID.update(moveVector.getDist()), MIN_SPEED);
    float moveAngle = moveVector.getAngle();
    setMove(moveSpeed, moveAngle, 0);
}

void updateBluetooth() {
    btData = BluetoothData(ballData, botCoords, role, onField);
    bt.update(btData);
}

bool shouldSwitchRoles() { 
    // switch roles if goalie has ball or ball is much closer to goalie or striker went out out field
    return (role == Role::defend && ballData.dist <= SWITCH_ROLE_BALL_DIST_THRESH );
}

void angleCorrect() {
    moveData.rotation.val = IMU.read();
}

// void getCameraCoords() {
//   Point oppGoalVec(Camera.oppGoalAngle, Camera.oppGoalDistance);
//   Point ownGoalVec(Camera.ownGoalAngle, Camera.ownGoalDistance);

//   int vecX = (oppGoalVec.x + ownGoalVec.x) / 2;
//   int vecY = (oppGoalVec.y + oppGoalVec.y) / 2;

//   Point centre(vecX, vecY);
// }



void setup() {

#ifdef SET_ID
    EEPROM.write(EEPROM_ID_ADDR, ID);
#else
    robotID = EEPROM.read(EEPROM_ID_ADDR);
#endif
    defaultPlayMode = robotID == 0 ? Role::attack : Role::defend;

#ifdef DEBUG
    Serial.begin(9600);
#endif
    L1Serial.begin(STM32_BAUD);
    L4Serial.begin(STM32_BAUD);
    CamSerial.begin(CAMERA_BAUD);
    BluetoothSerial.begin(BLUETOOTH_BAUD);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    imu.init();

    pinMode(KICKER_PIN, OUTPUT);
    pinMode(DRIBBLER_PIN, OUTPUT);
    analogWriteFrequency(DRIBBLER_PIN, 1000);
    analogWrite(DRIBBLER_PIN, 32);
    delay(DRIBBLER_WAIT);
}

void loop() {
    // put your main code here, to run repeatedly:
    camera.read();
    camera.process();
    heading = imu.read();
    readLayer4();
    processTOF();
    updateBallData();

    if (currentRole() == Role::attack) {
        if (ballData.captured) {
            trackGoal();
        } else if (ballData.visible) {
            trackBall();
        } else {
            goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
        }
    } else {
        if (ballData.visible) {
            defend();
        } else {
            goTo(Point(GOALIE_HOME_X, GOALIE_HOME_Y));
        }
    }
    angleCorrect();
    sendLayer1();
    readLayer1();
}