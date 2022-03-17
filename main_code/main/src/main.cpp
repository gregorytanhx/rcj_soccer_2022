#include <Adafruit_BNO055_t4.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BBox.h>
#include <Bluetooth.h>
#include <Camera.h>
#include <Common.h>
#include <Config.h>
#include <EEPROM.h>
#include <IMU.h>
#include <PID.h>
#include <Pins.h>
#include <Point.h>
#include <Role.h>
#include <Wire.h>
#include <utility/imumaths.h>

LineData lineData;

MoveData moveData(0, 0, 0);
BallData ballData;
Bluetooth bt;
BluetoothData btData;
Camera camera;
BBox bbox;
LightBuffer lightVals;

float lastLineAngle = 0;
bool lineTrack = false;
bool lineAvoid = true;
bool hasBall = false;
bool calibrate = false;
bool doneCalibrating = false;

TOFBuffer tof;
IMU cmp(&Wire1);
float heading;
float robotAngle;
float frontTOF, backTOF, leftTOF, rightTOF;

MyTimer kickerTimer(500);
MyTimer bluetoothTimer(BLUETOOTH_UPDATE_TIME);
MyTimer dribblerTimer(2000);

// decide if bots will switch roles mid match (while both still in)
bool roleSwitching = false;
bool movingSideways = false;
bool onField = true;
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
        return Role::attack;
    }
}

void controlDribbler() { 
    // use pwm to control dribbler
    bool dribble = ballData.dist <= BALL_DRIBBLE_THRESH;
    if (dribble || !dribblerTimer.timeHasPassed()){
        analogWrite(DRIBBLER_PIN, DRIBBLER_UPPER_LIMIT);
        dribblerTimer.update();
    } else {
        analogWrite(DRIBBLER_PIN, DRIBBLER_LOWER_LIMIT);
    }
       
}

void kick() {
    digitalWriteFast(KICKER_PIN, LOW);
    kickerTimer.update();       
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
    L1Serial.write(calibrate);
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

void calibrateLight() {
    calibrate = true;
    bool printNormalVals = false;
    sendLayer1();
   
    if (!doneCalibrating) {
        Serial.println("Calibrating...");
        while (L1Serial.available() >= LIGHT_PACKET_SIZE) {
            doneCalibrating = true;
            uint8_t syncByte = L1Serial.read();
            if (syncByte == LAYER1_REC_SYNC_BYTE) {
                Serial.println("Calibration done!");
                Serial.println("Thresh: ");
                for (int i = 0; i < 32; i++) {
                    for (int j = 0; j < 2; j++) {
                        lightVals.b[j] = L1Serial.read();
                    }
                    Serial.print(lightVals.vals[i]);
                    Serial.print(" ");
                }
            }
            Serial.println();
        }
    } else {
        if (Serial.available()) {
            printNormalVals = true;
        }
        if (printNormalVals) {
            while (L1Serial.available() >= LIGHT_PACKET_SIZE) {
                uint8_t syncByte = L1Serial.read();
                if (syncByte == LAYER1_REC_SYNC_BYTE) {
                    Serial.println("Vals: ");
                    for (int i = 0; i < 32; i++) {
                        for (int j = 0; j < 2; j++) {
                            lightVals.b[j] = L1Serial.read();
                        }
                        Serial.print(lightVals.vals[i]);
                        Serial.print(" ");
                    }
                }
            }
            Serial.println();
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

void updatePosition() {
    // TODO: combine camera data with TOF data
    // TODO: use light sensors to confirm robot's position along x-axis

    bbox.update(tof, lineData, moveData.rotation.val);
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
        ballData.dist = relBallCoords.getDistance();
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

void goTo(Point target) {
    // Point target: vector pointing from centre of field to point on field

    // Use cam by default if both goals are visible
    // otherwise use a combination of camera and TOF coords
    Point moveVector;
    float moveSpeed;
    if (camera.oppVisible && camera.ownVisible) {
        // go to point based on vector calculations
        Point oppGoalVec(camera.oppAngle, camera.oppDist);
        Point ownGoalVec(camera.ownAngle, camera.ownDist);

        // vector pointing to the centre of the field
        Point centre = oppGoalVec + ownGoalVec;
        centre.distance /= 2;
        moveVector = centre + target;
       
    } else {
        // go to point based on robot's coordinates
        moveVector = target - botCoords;
    }

    if (moveVector.getDistance() < COORD_LEEWAY_DIST) {
        // stop once robot is close to target position
        moveSpeed = 0;
    } else {
        moveSpeed =
            max(coordPID.update(moveVector.getDistance()), MIN_SPEED);
    }

    // if using coordinates, adjust speed based on confidence in position along
    // each axis
    if (!(camera.oppVisible && camera.ownVisible)) {
        float moveSpeed = distance(
            moveSpeed * sin(rad2deg(moveVector.getAngle())) * bbox.Xconfidence,
            moveSpeed * cos(rad2deg(moveVector.getAngle())) * bbox.Yconfidence);
    } 

    setMove(moveSpeed, moveVector.getAngle(), 0);
}

void guardGoal() {
    // align robot to x-coordinate of ball while tracking line
    // TODO: slowdown nearer to edges
    if (lineData.onLine) {
        if (abs(ballData.x) < GOALIE_LEEWAY_DIST) {
            // stop once ball is within certain horizontal distance
            float moveSpeed = 0;
        } else {
            float moveSpeed = max(goaliePID.update(abs(ballData.x)), MIN_SPEED);
        }

        float moveAngle = (ballData.angle > 180) ? -90 : 90;
        lineTrack = true;
    } else {
        Point target = Point(ballData.x, GOALIE_HOME_Y);
        goTo(target);
    }
}

bool shouldSwitchRoles(BluetoothData attackerData, BluetoothData defenderData) { 
    // switch roles if goalie has ball or ball is much closer to goalie or striker went out out field
    return 0;
}

void updateRole() {
    Role previousRole = role;

    if (role == Role::undecided) {
        // Undecided role, pick default unless the other robot has a role

        if (bt.otherData.role == Role::undecided) {
            role = defaultRole;
        } else {
            // take the other role based on the other robot's role
            role = bt.otherData.role == Role::attack ? Role::defend : Role::attack;
        }
    } else if (robotID == 1) {
        // Robot ID 1 (default defender) decides on role

        BluetoothData attackerData = role == Role::attack ? btData : bt.otherData;
        BluetoothData defenderData = role == Role::defend ? btData : bt.otherData;

        if (shouldSwitchRoles(attackerData, defenderData)) {
            role = role == Role::attack ? Role::defend : Role::attack;
        }
    } else {
        // Robot ID 0 is always the the opposite of the other robot
        role = bt.otherData.role == Role::attack ? Role::defend : Role::attack;
    }

    if (role != previousRole &&
        previousRole == Role::attack) {
        // If switched to defender, move to the side to prevent collision
        movingSideways = true;
        Point sidewaysCoordinate = Point(40 * sign(botCoords.x), botCoords.y);
    } else {
        movingSideways = false;
    }
}

void updateBluetooth() {
    btData = BluetoothData(ballData, botCoords, role, onField);
    bt.update(btData);
    if (bt.isConnected && roleSwitching) {
        updateRole();
    } else if (bt.previouslyConnected) {
        role = Role::defend;
    }
}

void angleCorrect() {
    moveData.rotation.val = cmp.readQuat() * 0.1;
}

// TODO: triangulate position based on coords of both goals
// TODO: if only one goal visible, use that goal
// TODO: if goalie, use own goal only 

void getCameraCoords() {
  Point oppGoalVec = Point(camera.oppAngle, camera.oppDist);
  Point ownGoalVec = Point(camera.ownAngle, camera.ownDist);

  int vecX = (oppGoalVec.x + ownGoalVec.x) / 2;
  int vecY = (oppGoalVec.y + oppGoalVec.y) / 2;

  Point centre(vecX, vecY);
}

void setup() {

#ifdef SET_ID
    EEPROM.write(EEPROM_ID_ADDR, ID);
#else
    robotID = EEPROM.read(EEPROM_ID_ADDR);
#endif
    // defaultRole = robotID == 0 ? Role::attack : Role::defend;

#ifdef DEBUG
    Serial.begin(9600);
#endif
    L1Serial.begin(STM32_BAUD);
    L4Serial.begin(STM32_BAUD);
    // camera.init();
    // BTSerial.begin(BLUETOOTH_BAUD);

    cmp.init();

    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, HIGH);
    pinMode(DRIBBLER_PIN, OUTPUT);
    // analogWriteFrequency(DRIBBLER_PIN, 1000);
    // analogWrite(DRIBBLER_PIN, DRIBBLER_LOWER_LIMIT);
    // delay(DRIBBLER_WAIT);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

MyTimer circleTimer(5);
int dir = 0;

void loop() {
 
    if (circleTimer.timeHasPassed()) {
        dir++;
        dir %= 360;
    }
    setMove(50, dir, 0);
    angleCorrect();
    sendLayer1();
    
    

    // // controlDribbler();
    // cmp.printCalib();
    // Serial.print("Euler: ");
    // Serial.println(cmp.readEuler());
    // Serial.print("Quaternion: ");
    // Serial.println(cmp.readQuat());

    // Serial.println(cmp.readRaw());
    //cmp.printAllData();

    // readLayer1();
    // if (lineData.onLine) {
    //     Serial.println("Line Detected");
    //     Serial.print("Angle: ");
    //     Serial.print(lineData.lineAngle.val);
    //     Serial.print(" Chord Length: ");
    //     Serial.println(lineData.chordLength.val);
    // }

    // camera.read();

    // heading = cmp.read();
    // readLayer4();
    // updatePosition();

    // if (bluetoothTimer.timeHasPassed()) {
    //     updateBluetooth();
    // }

    // updateBallData();
    
    // if (currentRole() == Role::attack) {
    //     if (ballData.captured) {
    //         trackGoal();
    //         if (camera.oppDist <= KICK_DISTANCE_THRES) {
    //             kick();
    //         }
    //     } else if (ballData.visible) {
    //         trackBall();

           
    //     } else {
    //         goTo(Point(STRIKER_HOME_X, STRIKER_HOME_Y));
    //     }
    // } else {
    //     if (ballData.visible) {
    //         guardGoal();
    //     } else {
    //         goTo(Point(GOALIE_HOME_X, GOALIE_HOME_Y));
    //     }
    // }


    // if (kickerTimer.timeHasPassed()) digitalWriteFast(KICKER_PIN, HIGH);

    // angleCorrect();
    // sendLayer1();
    // readLayer1();
}