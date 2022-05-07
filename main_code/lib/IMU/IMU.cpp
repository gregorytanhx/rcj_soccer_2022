#include "IMU.h"

IMU::IMU(TwoWire* theWire) { bno = Adafruit_BNO055(55, 0x29, theWire); }

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
    heading -= angleOffset;
   
    if (heading < 0) heading += 360;
    if (heading > 180) heading -= 360;
    return heading;
}

void IMU::sendCalib() {
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial2.write(IMU_SYNC_BYTE);
    Serial2.write(system);
    Serial2.write(gyro);
    Serial2.write(accel);
    Serial2.write(mag);
}

void IMU::begin(bool useMag) {
   
    if (useMag) {
        if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
            Serial.println("No BNO055 detected");
            while (1)
                ;
        }

    } else {
        if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS)) {
            Serial.println("No BNO055 detected");
            while (1)
                ;
        }
    }

    uint8_t system, gyro, accel, mag = 0;
    loadCalib();
    delay(1000);
    bno.setExtCrystalUse(false);

    if (useMag) {
        while (gyro != 3 || mag != 3) {
            bno.getCalibration(&system, &gyro, &accel, &mag);
            printCalib();
        }
    } else {
        while (gyro != 3) {
            bno.getCalibration(&system, &gyro, &accel, &mag);
            printCalib();
        }
    }
    delay(100);
    angleOffset = read();
    // for (int i = 0; i < 10; i++) {
    //     angleOffset += read();
    //     delay(10);
    // }

    // angleOffset /= 10;
}

void IMU::loadCalib() {
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

void IMU::printData() {
    Serial.print("Euler: ");
    Serial.println(readEuler());
    Serial.print("Quaternion: ");
    Serial.println(readQuat());

    printCalib();
}

// double IMU::read() {
//     sensors_event_t event;
//     bno.getEvent(&event);
//     heading = -(event.orientation.x - angleOffset);
//     return nonReflex(heading);
// }

double IMU::readQuat() {
    imu::Quaternion quat = bno.getQuat();
    /* Display the quat data */

    double yy = quat.y() * quat.y();  // 2 Uses below

    heading = 57.2958 * atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()),
                              1 - 2 * (yy + quat.z() * quat.z()));

    heading *= -1;
    heading -= angleOffset;

    return nonReflex(heading);
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

void IMU::calibrate() {
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
        // write calibration to eeprom memory
        eeAddress += sizeof(long);
        EEPROM.put(eeAddress, newCalib);

        Serial.println("Data stored to EEPROM.");

        Serial.println("\n--------------------------------\n");
    }

    /* Wait the specified delay before requesting new data */
    delay(100);
}