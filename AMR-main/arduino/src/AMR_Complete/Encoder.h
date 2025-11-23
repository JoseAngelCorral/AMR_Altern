#pragma once

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Configuración del encoder
// Observación: durante la inspección y la prueba "Avanzar 1 vuelta" (comando 'V') los
// contadores mostraron un valor consistente cercano a 3418 pulsos por revolución.
// Actualizamos el valor por defecto para reflejar esta medición. Si vuelves a
// calibrar con la prueba 'V', ajusta este número.
#define DEFAULT_PULSES_PER_REVOLUTION 3418  // Valor por defecto (ajustado tras medición)
#define WHEEL_DIAMETER_CM 15.50       // Diámetro de la rueda en centímetros
#define WHEEL_CIRCUMFERENCE_CM (PI * WHEEL_DIAMETER_CM)  // Circunferencia en cm

// Wheel base (centro a centro). If not defined elsewhere, provide a default
// so encoder init can compute deg/pulse for diagnostics. The canonical
// definition lives in Odometry.h; this is only a safe fallback for builds.
#ifndef WHEEL_BASE_CM
#define WHEEL_BASE_CM 63.5
#endif

// Pines para Arduino Uno - Configuración personalizada
// Asignación solicitada:
// Encoder IZQ: A = 8, B = 2
// Encoder DER: A = 9, B = 3
// Nota: en Arduino UNO sólo los pines 2 y 3 soportan interrupciones externas
// (INT0/INT1). Por eso las señales B deben ir a 2 o 3.
#define ENCODER_LEFT_A_PIN 8         // Pin A del encoder izquierdo
#define ENCODER_LEFT_B_PIN 2         // Pin B del encoder izquierdo (INT0)
#define ENCODER_RIGHT_A_PIN 9        // Pin A del encoder derecho
#define ENCODER_RIGHT_B_PIN 3        // Pin B del encoder derecho (INT1)

class Encoder {
private:
    static volatile long leftPulses;     // Contador de pulsos del encoder izquierdo
    static volatile long rightPulses;    // Contador de pulsos del encoder derecho
    // Pulsos por revolución (ajustable en runtime)
    static int pulsesPerRevolution;
    // Flags para invertir el sentido de conteo si el encoder está cableado al revés
    static bool leftInverted;
    static bool rightInverted;
    
public:
    // Inicialización del encoder
    void init();
    
    // Lectura de pulsos
    long readLeft();
    long readRight();
    
    // Reset de contadores
    void resetLeft();
    void resetRight();
    void resetBoth();
    
    // Conversión de pulsos a distancia
    float pulsesToCentimeters(long pulses);
    float pulsesToRevolutions(long pulses);
    
    // Cálculo de distancia diferencial para cada rueda
    float getLeftDistanceCm();
    float getRightDistanceCm();
    
    // Funciones de interrupción estáticas
    static void leftEncoderISR();
    static void rightEncoderISR();

    // Ajuste del sentido (runtime)
    static void setLeftInverted(bool inv);
    static void setRightInverted(bool inv);
    static bool isLeftInverted();
    static bool isRightInverted();
    
    // Getters/Setters para configuración (runtime ajustable)
    static int getPulsesPerRevolution();
    static void setPulsesPerRevolution(int v);
    static float getWheelDiameter() { return WHEEL_DIAMETER_CM; }
};

#endif // ENCODER_H