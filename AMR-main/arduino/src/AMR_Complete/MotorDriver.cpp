#include "MotorDriver.h"

void MotorDriver::init() {
    // Configurar solo pines PWM como salida (enables externos)
    // Motor Izquierdo
    pinMode(MOTOR_LEFT_RPWM, OUTPUT);  // PWM Adelante
    pinMode(MOTOR_LEFT_LPWM, OUTPUT);  // PWM Atrás
    
    // Motor Derecho
    pinMode(MOTOR_RIGHT_RPWM, OUTPUT); // PWM Adelante
    pinMode(MOTOR_RIGHT_LPWM, OUTPUT); // PWM Atrás
    
    // Los enables están alimentados externamente (siempre HIGH)
    // No necesitamos configurar REN/LEN
    
    // Inicializar en estado parado
    stop();
    
    Serial.println(F("=== BTS7960 Init ==="));
    Serial.println(F("MotIzq:10,11 MotDer:5,6"));
    Serial.println(F("Enables externos. Usa 'T'"));
}

void MotorDriver::moveForward(int speed) {
    // Limitar velocidad mínima y máxima
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    // Motor izquierdo hacia adelante (LPWM activo - corregido)
    analogWrite(MOTOR_LEFT_RPWM, 0);       // PWM adelante = 0
    analogWrite(MOTOR_LEFT_LPWM, speed);   // PWM atrás
    
    // Motor derecho hacia adelante (LPWM activo - corregido)
    analogWrite(MOTOR_RIGHT_RPWM, 0);      // PWM adelante = 0
    analogWrite(MOTOR_RIGHT_LPWM, speed);  // PWM atrás
}

void MotorDriver::moveBackward(int speed) {
    // Limitar velocidad mínima y máxima
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    // Motor izquierdo hacia atrás (RPWM activo - corregido)
    analogWrite(MOTOR_LEFT_RPWM, speed);   // PWM adelante
    analogWrite(MOTOR_LEFT_LPWM, 0);       // PWM atrás = 0
    
    // Motor derecho hacia atrás (RPWM activo - corregido)
    analogWrite(MOTOR_RIGHT_RPWM, speed);  // PWM adelante
    analogWrite(MOTOR_RIGHT_LPWM, 0);      // PWM atrás = 0
}

void MotorDriver::turnLeft(int speed) {
    // Allow manual turn speeds below MIN_SPEED: constrain to 0..MAX_SPEED
    // This enables low-speed manual turning (e.g. 20% PWM) even if MIN_SPEED
    // (used for overcoming static friction in linear motion) is higher.
    speed = constrain(speed, 0, MAX_SPEED);

    Serial.print(F("TurnL: speed="));
    Serial.println(speed);

    if (speed == 0) {
        // stop motors if zero requested
        analogWrite(MOTOR_LEFT_RPWM, 0);
        analogWrite(MOTOR_LEFT_LPWM, 0);
        analogWrite(MOTOR_RIGHT_RPWM, 0);
        analogWrite(MOTOR_RIGHT_LPWM, 0);
        return;
    }

    // Motor izquierdo hacia atrás (giro en su lugar)
    analogWrite(MOTOR_LEFT_RPWM, speed);   // PWM adelante
    analogWrite(MOTOR_LEFT_LPWM, 0);       // PWM atrás = 0
    Serial.print(F("IzqR="));
    Serial.println(speed);

    // Motor derecho hacia adelante
    analogWrite(MOTOR_RIGHT_RPWM, 0);      // PWM adelante = 0
    analogWrite(MOTOR_RIGHT_LPWM, speed);  // PWM atrás
    Serial.print(F("DerL="));
    Serial.println(speed);
}

void MotorDriver::turnRight(int speed) {
    // Allow manual turn speeds below MIN_SPEED: constrain to 0..MAX_SPEED
    speed = constrain(speed, 0, MAX_SPEED);

    Serial.print(F("TurnR: speed="));
    Serial.println(speed);

    if (speed == 0) {
        analogWrite(MOTOR_LEFT_RPWM, 0);
        analogWrite(MOTOR_LEFT_LPWM, 0);
        analogWrite(MOTOR_RIGHT_RPWM, 0);
        analogWrite(MOTOR_RIGHT_LPWM, 0);
        return;
    }

    // Motor izquierdo hacia adelante
    analogWrite(MOTOR_LEFT_RPWM, 0);       // PWM adelante = 0
    analogWrite(MOTOR_LEFT_LPWM, speed);   // PWM atrás
    Serial.print(F("IzqL="));
    Serial.println(speed);

    // Motor derecho hacia atrás
    analogWrite(MOTOR_RIGHT_RPWM, speed);  // PWM adelante
    analogWrite(MOTOR_RIGHT_LPWM, 0);      // PWM atrás = 0
    Serial.print(F("DerR="));
    Serial.println(speed);
}

void MotorDriver::stop() {
    // Parar ambos motores BTS7960 (todos los PWM a 0)
    analogWrite(MOTOR_LEFT_RPWM, 0);
    analogWrite(MOTOR_LEFT_LPWM, 0);
    analogWrite(MOTOR_RIGHT_RPWM, 0);
    analogWrite(MOTOR_RIGHT_LPWM, 0);
}

void MotorDriver::setLeftMotor(int speed) {
    // Limitar velocidad entre -255 y 255
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    
    if (speed > 0) {
        // Hacia adelante - usar LPWM (corregido)
        if (speed < MIN_SPEED) speed = MIN_SPEED; // Velocidad mínima
        analogWrite(MOTOR_LEFT_RPWM, 0);
        analogWrite(MOTOR_LEFT_LPWM, speed);
    } else if (speed < 0) {
        // Hacia atrás - usar RPWM (corregido)
        int absSpeed = -speed;
        if (absSpeed < MIN_SPEED) absSpeed = MIN_SPEED;
        analogWrite(MOTOR_LEFT_RPWM, absSpeed);
        analogWrite(MOTOR_LEFT_LPWM, 0);
    } else {
        // Parado
        analogWrite(MOTOR_LEFT_RPWM, 0);
        analogWrite(MOTOR_LEFT_LPWM, 0);
    }
}

void MotorDriver::setRightMotor(int speed) {
    // Limitar velocidad entre -255 y 255
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    
    if (speed > 0) {
        // Hacia adelante - usar LPWM (corregido)
        if (speed < MIN_SPEED) speed = MIN_SPEED;
        analogWrite(MOTOR_RIGHT_RPWM, 0);
        analogWrite(MOTOR_RIGHT_LPWM, speed);
    } else if (speed < 0) {
        // Hacia atrás - usar RPWM (corregido)
        int absSpeed = -speed;
        if (absSpeed < MIN_SPEED) absSpeed = MIN_SPEED;
        analogWrite(MOTOR_RIGHT_RPWM, absSpeed);
        analogWrite(MOTOR_RIGHT_LPWM, 0);
    } else {
        // Parado
        analogWrite(MOTOR_RIGHT_RPWM, 0);
        analogWrite(MOTOR_RIGHT_LPWM, 0);
    }
}

void MotorDriver::setBothMotors(int leftSpeed, int rightSpeed) {
    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
}

void MotorDriver::testMotors() {
    Serial.println(F("=== TEST MOTORES ==="));
    
    // Test Motor Izquierdo Adelante
    Serial.println(F("Izq+"));
    analogWrite(MOTOR_LEFT_RPWM, 0);
    analogWrite(MOTOR_LEFT_LPWM, 150);
    delay(1000);
    stop();
    delay(300);
    
    // Test Motor Izquierdo Atrás  
    Serial.println(F("Izq-"));
    analogWrite(MOTOR_LEFT_RPWM, 150);
    analogWrite(MOTOR_LEFT_LPWM, 0);
    delay(1000);
    stop();
    delay(300);
    
    // Test Motor Derecho Adelante
    Serial.println(F("Der+"));
    analogWrite(MOTOR_RIGHT_RPWM, 0);
    analogWrite(MOTOR_RIGHT_LPWM, 150);
    delay(1000);
    stop();
    delay(300);
    
    // Test Motor Derecho Atrás
    Serial.println(F("Der-"));
    analogWrite(MOTOR_RIGHT_RPWM, 150);
    analogWrite(MOTOR_RIGHT_LPWM, 0);
    delay(1000);
    stop();
    delay(300);
    
    Serial.println(F("Test OK"));
}

// --------------------
// PID velocity control
// --------------------
void MotorDriver::enableVelocityControl(bool en) {
    velocityControlEnabled = en;
    if (!en) {
        // reset integrators and stop applying setpoints
        integral = 0.0f;
        prevError = 0.0f;
        appliedPps = 0.0f;
        // ensure motors are stopped or left under direct control
    } else {
        lastPIDMillis = millis();
    }
    Serial.print(F("VelocityControl "));
    Serial.println(en ? F("ENABLED") : F("DISABLED"));
}

bool MotorDriver::isVelocityControlEnabled() {
    return velocityControlEnabled;
}

void MotorDriver::setTargetPulsesPerSecond(float pps) {
    targetPps = pps;
    // when first setting, don't instantly jump applied setpoint if off
    if (!velocityControlEnabled) {
        appliedPps = targetPps;
    }
}

void MotorDriver::setPIDGains(float kp, float ki, float kd) {
    Kp = kp; Ki = ki; Kd = kd;
}

void MotorDriver::setPIDInterval(unsigned int ms) {
    if (ms < 5) ms = 5;
    pidIntervalMs = ms;
}

void MotorDriver::setRampTime(unsigned long ms) {
    rampTimeMs = ms;
}

// updateVelocityControl: called from main loop with encoder delta counts and elapsed ms
// Interpola lecturas de ambos encoders para generar un valor único y aplicar un solo PID
void MotorDriver::updateVelocityControl(long leftDeltaPulses, long rightDeltaPulses, unsigned long dtMs) {
    if (!velocityControlEnabled) return;
    if (dtMs == 0) return;

    // Only run at configured interval
    unsigned long now = millis();
    if (now - lastPIDMillis < pidIntervalMs) return;
    unsigned long elapsed = now - lastPIDMillis;
    lastPIDMillis = now;

    float dt = (float)elapsed / 1000.0f; // seconds

    // INTERPOLACIÓN: Promedio de lecturas de ambos encoders para generar valor único
    float avgDeltaPulses = ((float)leftDeltaPulses + (float)rightDeltaPulses) / 2.0f;
    float measPps = avgDeltaPulses / dt; // Velocidad medida promedio (pulses per second)

    // Soft-start ramp applied setpoint towards target
    if (rampTimeMs > 0) {
        float maxDelta = (abs(targetPps) / (float)rampTimeMs) * (float)elapsed; // pps per this interval

        // adjust appliedPps towards targetPps
        if (appliedPps < targetPps) {
            appliedPps += maxDelta;
            if (appliedPps > targetPps) appliedPps = targetPps;
        } else if (appliedPps > targetPps) {
            appliedPps -= maxDelta;
            if (appliedPps < targetPps) appliedPps = targetPps;
        }
    } else {
        appliedPps = targetPps;
    }

    // PID único usando el valor interpolado
    float error = appliedPps - measPps;
    integral += error * dt;
    
    // Clamp integral (anti-windup)
    if (integral > integralClamp) integral = integralClamp;
    if (integral < -integralClamp) integral = -integralClamp;
    
    float derivative = (error - prevError) / dt;
    prevError = error;
    
    // Cálculo de salida PID
    float pidOutput = Kp * error + Ki * integral + Kd * derivative;

    // Map PID output to PWM command (asumiendo que los gains están ajustados)
    int pwmBase = (int)round(pidOutput);
    pwmBase = constrain(pwmBase, -MAX_SPEED, MAX_SPEED);

    // Aplicar PWM base al motor izquierdo
    int pwmLeft = pwmBase;
    
    // Aplicar factor de compensación 1.1 al motor derecho (corrige curva a la derecha)
    int pwmRight = (int)round((float)pwmBase * RIGHT_MOTOR_COMPENSATION);
    pwmRight = constrain(pwmRight, -MAX_SPEED, MAX_SPEED); // Limitar para no exceder MAX_SPEED

    // Aplicar PWM a ambos motores
    setBothMotors(pwmLeft, pwmRight);

    // Optional debug print (comment/uncomment for tuning)
    // Serial.print(F("PID dt:")); Serial.print(dt, 3);
    // Serial.print(F(" avgPps:")); Serial.print(measPps, 1);
    // Serial.print(F(" target:")); Serial.print(appliedPps, 1);
    // Serial.print(F(" error:")); Serial.print(error, 1);
    // Serial.print(F(" PWM L:")); Serial.print(pwmLeft);
    // Serial.print(F(" R:")); Serial.println(pwmRight);
}