#pragma once

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// Pines para control de motores BTS7960 - Configuración real
// Motor Izquierdo (BTS7960 #1)
#define MOTOR_LEFT_RPWM 5   // PWM Adelante motor izquierdo
#define MOTOR_LEFT_LPWM 6   // PWM Atrás motor izquierdo  
// REN y LEN: Alimentación externa (siempre HIGH)

// Motor Derecho (BTS7960 #2)  
#define MOTOR_RIGHT_RPWM 10   // PWM Adelante motor derecho
#define MOTOR_RIGHT_LPWM 11   // PWM Atrás motor derecho
// REN y LEN: Alimentación externa (siempre HIGH)

// Velocidades para BTS7960 (más potente que L298N)
// Ajustadas para que las velocidades automáticas coincidan con las manuales
// Usamos porcentajes del valor máximo (MAX_SPEED = 255):
// - Avance/retroceso manual/automático = 40% -> 255 * 0.40 ≈ 102
// - Giro manual/automático = 20% -> 255 * 0.20 ≈ 51
#define MAX_SPEED 255        // Velocidad máxima (PWM 0..255)
#define DEFAULT_SPEED 102    // Velocidad por defecto para avance (≈40% de MAX)
#define TURN_SPEED 51        // Velocidad para giros automáticos (≈20% de MAX)
#define MIN_SPEED 80         // Velocidad mínima para superar fricción

class MotorDriver {
private:
    // --- PID velocity control members ---
    bool velocityControlEnabled = false;
    unsigned long lastPIDMillis = 0;
    unsigned int pidIntervalMs = 50; // PID update interval (ms)

    // Target speed (pulses per second) - único valor para ambos motores
    float targetPps = 0.0f;
    float appliedPps = 0.0f;  // Setpoint con rampa aplicada

    // PID state - un solo controlador para ambos motores
    float Kp = 0.08f; // initial guess, tune on hardware
    float Ki = 0.02f;
    float Kd = 0.002f;
    float integral = 0.0f;
    float prevError = 0.0f;
    float integralClamp = 500.0f; // anti-windup

    // Soft-start ramp time in ms (time to go from 0 -> target)
    unsigned long rampTimeMs = 800;
    
    // Factor de compensación para motor derecho (corrige curva a la derecha)
    static constexpr float RIGHT_MOTOR_COMPENSATION = 1.1f;
    
public:
    void init();
    
    // Control básico de movimiento
    void moveForward(int speed = DEFAULT_SPEED);
    void moveBackward(int speed = DEFAULT_SPEED);
    void turnLeft(int speed = TURN_SPEED);
    void turnRight(int speed = TURN_SPEED);
    void stop();
    
    // Control individual de motores
    void setLeftMotor(int speed);   // speed: -255 a 255
    void setRightMotor(int speed);  // speed: -255 a 255
    void setBothMotors(int leftSpeed, int rightSpeed);
    
    // Funciones de diagnóstico
    void testMotors();              // Test automático de motores
    
    // --- Velocity PID API ---
    // Enable/disable closed-loop velocity control
    void enableVelocityControl(bool en);
    bool isVelocityControlEnabled();

    // Set PID gains (tune on hardware)
    void setPIDGains(float kp, float ki, float kd);

    // Set PID update interval
    void setPIDInterval(unsigned int ms);

    // Set ramp time (ms) for soft-start
    void setRampTime(unsigned long ms);

    // Set target speed in pulses per second (encoder pulses/sec) - valor único para ambos motores
    void setTargetPulsesPerSecond(float pps);

    // Must be called periodically (from main loop) with encoder deltas and dt
    // Interpola lecturas de ambos encoders y usa un solo PID
    void updateVelocityControl(long leftDeltaPulses, long rightDeltaPulses, unsigned long dtMs);
};

#endif // MOTOR_DRIVER_H