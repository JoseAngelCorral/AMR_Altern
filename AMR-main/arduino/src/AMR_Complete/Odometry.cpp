#include "Odometry.h"
#include <math.h>

Odometry::Odometry(Encoder* enc) {
    encoder = enc;
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    lastLeftPulses = 0;
    lastRightPulses = 0;
}

void Odometry::init(float startX, float startY, float startTheta) {
    x = startX;
    y = startY;
    theta = degreesToRadians(startTheta);
    
    // Inicializar lecturas previas
    lastLeftPulses = encoder->readLeft();
    lastRightPulses = encoder->readRight();
    
    Serial.println(F("Odo OK"));
}

void Odometry::update() {
    // Leer pulsos actuales
    long currentLeftPulses = encoder->readLeft();
    long currentRightPulses = encoder->readRight();
    
    // Calcular diferencias desde la última actualización
    long deltaLeftPulses = currentLeftPulses - lastLeftPulses;
    long deltaRightPulses = currentRightPulses - lastRightPulses;
    
    // Convertir a distancia
    float deltaLeftCm = encoder->pulsesToCentimeters(deltaLeftPulses);
    float deltaRightCm = encoder->pulsesToCentimeters(deltaRightPulses);
    
    // Calcular movimiento del robot
    float deltaDistance = (deltaLeftCm + deltaRightCm) / 2.0;
    float deltaTheta = (deltaRightCm - deltaLeftCm) / WHEEL_BASE_CM;

    // Guardar theta previo y calcular nueva orientación
    float prevTheta = theta;
    float newTheta = theta + deltaTheta;

    // Normalizar nueva orientación entre -π y π
    while (newTheta > PI) newTheta -= 2 * PI;
    while (newTheta < -PI) newTheta += 2 * PI;

    // Para integrar posición correctamente durante giros parciales, usar el
    // ángulo medio (prevTheta + deltaTheta/2). Además, si no hay un avance
    // significativo (p.ej. giro en sitio), no actualizar X/Y.
    const float DIST_EPS_CM = 0.001f; // umbral para considerar movimiento nulo
    if (fabs(deltaDistance) > DIST_EPS_CM) {
        float midTheta = prevTheta + deltaTheta * 0.5f;
        // normalizar midTheta en rango [-PI,PI]
        while (midTheta > PI) midTheta -= 2 * PI;
        while (midTheta < -PI) midTheta += 2 * PI;
        x += deltaDistance * cos(midTheta);
        y += deltaDistance * sin(midTheta);
    }

    // Finalmente asignar la nueva orientación normalizada
    theta = newTheta;
    
    // Guardar lecturas para próxima iteración
    lastLeftPulses = currentLeftPulses;
    lastRightPulses = currentRightPulses;
}

float Odometry::radiansToDegrees(float radians) {
    return radians * 180.0 / PI;
}

float Odometry::degreesToRadians(float degrees) {
    return degrees * PI / 180.0;
}

void Odometry::setPosition(float newX, float newY, float newTheta) {
    x = newX;
    y = newY;
    theta = degreesToRadians(newTheta);
}

void Odometry::resetPosition() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    encoder->resetBoth();
    lastLeftPulses = 0;
    lastRightPulses = 0;
}

void Odometry::printPosition() {
    Serial.print(F("("));
    Serial.print(x, 1);
    Serial.print(F(","));
    Serial.print(y, 1);
    Serial.print(F(") "));
    Serial.print(radiansToDegrees(theta), 0);
    Serial.println(F("°"));
}

float Odometry::getDistanceFromOrigin() {
    return sqrt(x*x + y*y);
}