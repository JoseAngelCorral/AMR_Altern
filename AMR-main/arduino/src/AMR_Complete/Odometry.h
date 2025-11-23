#pragma once

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "Encoder.h"

// Configuración del robot
// IMPORTANTE: actualizar a la distancia centro-a-centro real entre ruedas
#ifndef WHEEL_BASE_CM
#define WHEEL_BASE_CM 63.5   // Distancia entre ruedas en cm (centro a centro)
#endif

class Odometry {
private:
    Encoder* encoder;
    
    // Posición actual del robot
    float x, y;           // Posición en cm
    float theta;          // Orientación en radianes
    
    // Lecturas anteriores del encoder
    long lastLeftPulses;
    long lastRightPulses;
    
    // Conversiones
    float radiansToDegrees(float radians);
    float degreesToRadians(float degrees);
    
public:
    // Constructor
    Odometry(Encoder* enc);
    
    // Inicialización
    void init(float startX = 0.0, float startY = 0.0, float startTheta = 0.0);
    
    // Actualización de odometría
    void update();
    
    // Getters de posición
    float getX() { return x; }
    float getY() { return y; }
    float getTheta() { return theta; }
    float getThetaDegrees() { return radiansToDegrees(theta); }
    
    // Setters de posición (para corrección)
    void setPosition(float newX, float newY, float newTheta);
    void resetPosition();
    
    // Utilidades
    void printPosition();
    float getDistanceFromOrigin();
    
    // Control de movimiento con odometría
    bool moveForwardDistance(float distanceCm);
    bool turnAngle(float angleDegrees);
};

#endif // ODOMETRY_H