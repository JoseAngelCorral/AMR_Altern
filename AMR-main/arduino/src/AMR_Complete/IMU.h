#pragma once

class IMU {
public:
    void init();
    void read(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
};