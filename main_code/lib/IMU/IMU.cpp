#include "IMU.h"

IMU::IMU(TwoWire* theWire) {
    // bno = Adafruit_BNO055(WIRE2_BUS, -1, 0x29, I2C_MASTER, I2C_PINS_3_4,
    //                       I2C_PULLUP_EXT, I2C_RATE_100, I2C_OP_MODE_ISR);
    bno = Adafruit_BNO055(55, 0x29, theWire);
}

float IMU::readEuler() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    heading = euler.x();
    heading -= eulerOffset;
    if (heading < 0) heading += 360;
    if (heading > 180) heading -= 360;
    return heading;
}

float IMU::read() {
    bno.getEvent(&event);
    heading = event.orientation.x;
    heading -= eulerOffset;
    return heading;
}

void IMU::init() {
    
    if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS)) {
#ifdef DEBUG
        Serial.println("No BNO055 detected");
#endif
        while (1);
    }

    delay(1000);
    bno.setExtCrystalUse(true);
    delay(100);
    uint8_t system, gyro, accel, mag = 0;
    //loadCalib();
    while (gyro != 3) {
        
        bno.getCalibration(&system, &gyro, &accel, &mag);
        
        bno.getEvent(&event);
#ifdef DEBUG
        Serial.println("Uncalibrated!");
        Serial.print("X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);

        /* Optional: Display calibration status */
        printCalib();

        /* New line for the next sample */
        Serial.println("");
#endif
        /* Wait the specified delay before requesting new data */
        delay(100);
    }

    eulerOffset = readEuler();
    quatOffset = readQuat();
}

void IMU::loadCalib(){
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    // displaySensorOffsets(calibrationData);
#ifdef DEBUG
    Serial.println("\n\nRestoring Calibration data to the BNO055...");
#endif
    bno.setSensorOffsets(calibrationData);

    displaySensorOffsets(calibrationData);
#ifdef DEBUG
    Serial.println("\n\nCalibration data loaded into BNO055");
#endif
}

void IMU::printCalib() {
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.println();
    Serial.print("Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);

    Serial.println("--");
    delay(10);
}
void IMU::printEvent(sensors_event_t* event) {
    double x = -1000000, y = -1000000,
           z = -1000000;  // dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
        Serial.print("Accl:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    } else if (event->type == SENSOR_TYPE_ORIENTATION) {
        Serial.print("Orient:");
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
    } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
        Serial.print("Mag:");
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
        Serial.print("Gyro:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
        Serial.print("Rot:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
        Serial.print("Linear:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    } else if (event->type == SENSOR_TYPE_GRAVITY) {
        Serial.print("Gravity:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    } else {
        Serial.print("Unk:");
    }

    Serial.print("\tx= ");
    Serial.print(x);
    Serial.print(" |\ty= ");
    Serial.print(y);
    Serial.print(" |\tz= ");
    Serial.println(z);
}

void IMU::printAllData() {
    sensors_event_t orientationData, angVelocityData, linearAccelData,
        magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    printEvent(&orientationData);
    printEvent(&angVelocityData);
    printEvent(&linearAccelData);
    printEvent(&magnetometerData);
    printEvent(&accelerometerData);
    printEvent(&gravityData);

    int8_t boardTemp = bno.getTemp();
    Serial.println();
    Serial.print(F("temperature: "));
    Serial.println(boardTemp);

    printCalib();
}

double IMU::readQuat() {
    imu::Quaternion quat = bno.getQuat();
    /* Display the quat data */

    double yy = quat.y() * quat.y();  // 2 Uses below

    heading = 57.2958 * atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()),
                              1 - 2 * (yy + quat.z() * quat.z()));

    heading *= -1;
    heading -= quatOffset;
    if (heading < 0) heading += 360;
    if (heading > 180) heading -= 360;
    return heading;
}

void IMU::displaySensorDetails() {
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" xxx");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" xxx");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void IMU::displaySensorOffsets(const adafruit_bno055_offsets_t& calibData) {
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x);
    Serial.print(" ");
    Serial.print(calibData.accel_offset_y);
    Serial.print(" ");
    Serial.print(calibData.accel_offset_z);
    Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x);
    Serial.print(" ");
    Serial.print(calibData.gyro_offset_y);
    Serial.print(" ");
    Serial.print(calibData.gyro_offset_z);
    Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x);
    Serial.print(" ");
    Serial.print(calibData.mag_offset_y);
    Serial.print(" ");
    Serial.print(calibData.mag_offset_z);
    Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void IMU::calibrate(){
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);

    /* Optional: Display calibration status */
    printCalib();

    /* Optional: Display sensor status (debug only) */
    // displaySensorStatus();

    /* New line for the next sample */
    Serial.println("");

    if (bno.isFullyCalibrated()) {
        Serial.println("\nFully calibrated!");
        Serial.println("--------------------------------");
        Serial.println("Calibration Results: ");
        adafruit_bno055_offsets_t newCalib;
        bno.getSensorOffsets(newCalib);
        displaySensorOffsets(newCalib);

        Serial.println("\n\nStoring calibration data to EEPROM...");

        eeAddress += sizeof(long);
        EEPROM.put(eeAddress, newCalib);
        Serial.println("Data stored to EEPROM.");

        Serial.println("\n--------------------------------\n");
    }

    /* Wait the specified delay before requesting new data */
    delay(100);
}