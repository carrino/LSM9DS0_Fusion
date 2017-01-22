#ifndef ACCEL_H
#define ACCEL_H

#include <Adafruit_LSM9DS0.h>
#include "Quaternion.h"

class Accel {
    protected:
        Adafruit_LSM9DS0 _lsm;
        uint32_t _lastUpdateMS;
        uint32_t _intervalMS;
        float _avgAbsAccel;
        float _currentAccel;
        int _count;
        Quaternion _q;
    public:
        Accel(uint32_t intervalMS) { _intervalMS = intervalMS; }
        bool begin();
        bool Update();
        bool isDancing() const;
        float getLinearAcceleration() const;

        // This method takes a vector in 3 space (a == 0).
        // X corresponds to north, Y is west, Z to up.
        // This returns a vector in device space that points in the direction of v.
        // For example if I want to see which axis of the device is pointing north I would pass
        // in (1, 0, 0) and I would get back a vector of what was pointing north.
        const Quaternion getDeviceOrientation(const Quaternion &absolutePosition) const;

        // This method takes a vector in 3 space (a == 0).
        // X corresponds to north, Y is west, Z to up.
        // This returns a vector in absolute space where this device is pointing.
        // For example if I want to know where the top of my device is pointing
        // I'd call this method with in (0, 0, 1) and I would get back a vector
        // of which absolute direction the top of the device was pointing.
        const Quaternion getAbsoluteOrientation(const Quaternion &deviceVector) const;
};

#endif // ACCEL_H
