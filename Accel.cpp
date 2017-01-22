#include "Accel.h"

#define MOVING_AVERAGE_INTERVALS 20
#define ACCEL_THRESHOLD 1
#define COMPASS_ROTATE_FRACTION 0.05
#define ACCELEROMETER_FRACTION 0.1
#define ACCELEROMETER_CALIBRATE 0
#define MAG_CALIBRATE_X 0.7
#define MAG_CALIBRATE_Y -0.5
#define MAG_INVERT_Z -1.0

bool Accel::begin() {
    // Try to initialise and warn if we couldn't detect the chip
    if (!_lsm.begin()) {
      Serial.println("LSM!");
      while (1);
    }
    // _lsm.setupAccel(_lsm.LSM9DS0_ACCELRANGE_4G);
    // _lsm.setupMag(_lsm.LSM9DS0_MAGGAIN_2GAUSS);
    _lsm.setupGyro(_lsm.LSM9DS0_GYROSCALE_500DPS);
        
    return true;
}

bool Accel::Update() {
    int newMillis = millis();
    int elaspedMillis = newMillis - _lastUpdateMS;
    if ( elaspedMillis > _intervalMS + 10) {
        Serial.print("E: accel time: "); Serial.println(elaspedMillis);
    }
    if (elaspedMillis < _intervalMS) {
        return false;
    }
    _lastUpdateMS = newMillis;

    sensors_event_t accelEvent;
    sensors_event_t magEvent;
    sensors_event_t gyroEvent;
    //sensors_event_t tempEvent;
    _lsm.getEvent(&accelEvent, &magEvent, &gyroEvent, NULL);

    float magx = magEvent.magnetic.x + MAG_CALIBRATE_X;
    float magy = magEvent.magnetic.y + MAG_CALIBRATE_Y;
    float magz = magEvent.magnetic.z * MAG_INVERT_Z; // For some reason z points the opposite of north.

    // Gyro is in degrees per second, so we change to rads and mult by dt to get rotation.
    float degPerSec = Quaternion(gyroEvent.gyro.x, gyroEvent.gyro.y, gyroEvent.gyro.z).norm();
    float degPerSecToRads = PI/180.0 * elaspedMillis / 1000.0;
    float gyrox = gyroEvent.gyro.x * degPerSecToRads;
    float gyroy = gyroEvent.gyro.y * degPerSecToRads;
    float gyroz = gyroEvent.gyro.z * degPerSecToRads;

    // Rotate by the gyro. This line takes 2ms to convert and multiply.
    _q *= Quaternion::from_euler_rotation_approx(gyrox, gyroy, gyroz);
    _q.normalize();

    Quaternion gravity(accelEvent.acceleration.x, accelEvent.acceleration.y, accelEvent.acceleration.z);
    _currentAccel = gravity.norm() - SENSORS_GRAVITY_EARTH + ACCELEROMETER_CALIBRATE;
    float currentAbsAccel = abs(_currentAccel);
    _avgAbsAccel = (_avgAbsAccel * (MOVING_AVERAGE_INTERVALS - 1) + currentAbsAccel)/MOVING_AVERAGE_INTERVALS;

    // Ignore gravity if it isn't around G.  We only want to update based on the accelrometer if we aren't bouncing.
    // We also want to ignore gravity if the gyro is moving a lot.
    // TODO: carrino: make these defines at the top
    if (degPerSec < 90 && currentAbsAccel < 1) {
        // cal expected gravity takes 1ms
        Quaternion expected_gravity = _q.conj().rotate(Quaternion(0, 0, 1));
        if (_count++ % 2 == 0) {
            // This chunk of code takes 3ms
            gravity.normalize();
            Quaternion toRotateG = gravity.rotation_between_vectors(expected_gravity);

            _q = _q * toRotateG.fractional(ACCELEROMETER_FRACTION);
        } else {
            // This code path of code takes 5ms

            // We want to subtract gravity from the magnetic reading.
            // mag readings point into the earth quite a bit, but gravity is handled by accelerometer ok.
            // We just want to use mag for rotation around the gravity axis.
            // https://en.wikipedia.org/wiki/Earth%27s_magnetic_field#Inclination
            Quaternion expected_north = _q.conj().rotate(Quaternion(1, 0, 0));
            expected_north += expected_gravity * (-expected_gravity.dot_product(expected_north));
            expected_north.normalize();

            Quaternion mag(magx, magy, magz);
            mag += expected_gravity * (-expected_gravity.dot_product(mag));
            mag.normalize();

            Quaternion toRotateMag = mag.rotation_between_vectors(expected_north);
            _q = _q * toRotateMag.fractional(COMPASS_ROTATE_FRACTION);
        }
    }

    //int lastMillis = millis();
    //Serial.print("Accel time: "); Serial.println(lastMillis - newMillis);
    return true;
}

const Quaternion Accel::getDeviceOrientation(const Quaternion &absolutePosition) const {
    return _q.conj().rotate(absolutePosition);
}

const Quaternion Accel::getAbsoluteOrientation(const Quaternion &deviceVector) const {
    return _q.rotate(deviceVector);
}

float Accel::getLinearAcceleration() const {
    return _currentAccel;
}

bool Accel::isDancing() const {
    return  _avgAbsAccel > ACCEL_THRESHOLD;
}

