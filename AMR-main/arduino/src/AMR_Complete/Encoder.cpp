#include "Encoder.h"

// Inicialización de variables estáticas
volatile long Encoder::leftPulses = 0;
volatile long Encoder::rightPulses = 0;
// Pulses per revolution (runtime adjustable). Start from default measured value.
int Encoder::pulsesPerRevolution = DEFAULT_PULSES_PER_REVOLUTION;
// Invert flags (default: not inverted)
bool Encoder::leftInverted = false;
bool Encoder::rightInverted = true;

void Encoder::init() {
    // Configurar pines como entrada con pull-up interno
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);
    
    // Configurar interrupciones para Arduino Uno
    // Usar digitalPinToInterrupt para enlazar ISRs a los pines B configurados.
    int intLeft = digitalPinToInterrupt(ENCODER_LEFT_B_PIN);
    int intRight = digitalPinToInterrupt(ENCODER_RIGHT_B_PIN);
    // digitalPinToInterrupt may return a negative value on cores that don't support
    // interrupts on those pins. Instead of relying on NOT_AN_INTERRUPT macro (which
    // may not be defined on all cores), check for a negative value and only attach
    // the interrupt if valid.
    if (intLeft < 0 || intRight < 0) {
        Serial.print(F("Warning: encoder B pin(s) not an interrupt: L_B=")); Serial.print(ENCODER_LEFT_B_PIN);
        Serial.print(F(" R_B=")); Serial.println(ENCODER_RIGHT_B_PIN);
    } else {
        attachInterrupt(intLeft, leftEncoderISR, CHANGE);
        attachInterrupt(intRight, rightEncoderISR, CHANGE);
    }
    
    // Reset contadores
    resetBoth();
    
    Serial.print(F("Enc OK "));
    Serial.print(Encoder::pulsesPerRevolution);
    Serial.print(F(" PPR "));
    Serial.print(F("Pins L_A:")); Serial.print(ENCODER_LEFT_A_PIN);
    Serial.print(F(" L_B:")); Serial.print(ENCODER_LEFT_B_PIN);
    Serial.print(F(" R_A:")); Serial.print(ENCODER_RIGHT_A_PIN);
    Serial.print(F(" R_B:")); Serial.println(ENCODER_RIGHT_B_PIN);

    // Imprimir información de calibración útil
    float cmPerPulse = WHEEL_CIRCUMFERENCE_CM / (float)Encoder::pulsesPerRevolution;
    Serial.print(F("cm/pulse:")); Serial.println(cmPerPulse, 6);
    // grados por tic cuando solo una rueda avanza (rad = cm_per_pulse / wheel_base)
    float degPerPulseSingle = (cmPerPulse / (float)WHEEL_BASE_CM) * 180.0 / PI;
    Serial.print(F("deg/pulse(single):")); Serial.println(degPerPulseSingle, 6);
    // grados por par de tics opuestos (una rueda adelante, otra atras)
    float degPerPulsePair = (2.0 * cmPerPulse / (float)WHEEL_BASE_CM) * 180.0 / PI;
    Serial.print(F("deg/pulse(pair):")); Serial.println(degPerPulsePair, 6);
}

long Encoder::readLeft() {
    long temp;
    noInterrupts();
    temp = leftPulses;
    interrupts();
    return temp;
}

long Encoder::readRight() {
    long temp;
    noInterrupts();
    temp = rightPulses;
    interrupts();
    return temp;
}

void Encoder::resetLeft() {
    noInterrupts();
    leftPulses = 0;
    interrupts();
}

void Encoder::resetRight() {
    noInterrupts();
    rightPulses = 0;
    interrupts();
}

void Encoder::resetBoth() {
    noInterrupts();
    leftPulses = 0;
    rightPulses = 0;
    interrupts();
}

float Encoder::pulsesToCentimeters(long pulses) {
    // Convertir pulsos a distancia en centímetros
    // Distancia = (pulsos / pulsos_por_revolución) * circunferencia
    float revolutions = (float)pulses / (float)Encoder::pulsesPerRevolution;
    return revolutions * WHEEL_CIRCUMFERENCE_CM;
}

float Encoder::pulsesToRevolutions(long pulses) {
    // Convertir pulsos a revoluciones
    return (float)pulses / (float)Encoder::pulsesPerRevolution;
}

// Getter/Setter for runtime pulses/revolution
int Encoder::getPulsesPerRevolution() { return pulsesPerRevolution; }
void Encoder::setPulsesPerRevolution(int v) { if (v > 0) pulsesPerRevolution = v; }

float Encoder::getLeftDistanceCm() {
    return pulsesToCentimeters(readLeft());
}

float Encoder::getRightDistanceCm() {
    return pulsesToCentimeters(readRight());
}

// Función de interrupción para encoder izquierdo
void Encoder::leftEncoderISR() {
    // Leer estados de ambos canales
    bool A = digitalRead(ENCODER_LEFT_A_PIN);
    bool B = digitalRead(ENCODER_LEFT_B_PIN);

    // Determinar dirección usando cuadratura
    int delta = (A == B) ? -1 : 1; // default: A==B -> -1, else +1
    if (Encoder::leftInverted) delta = -delta;
    leftPulses += delta;
}

// Función de interrupción para encoder derecho
void Encoder::rightEncoderISR() {
    // Leer estados de ambos canales
    bool A = digitalRead(ENCODER_RIGHT_A_PIN);
    bool B = digitalRead(ENCODER_RIGHT_B_PIN);

    // Determinar dirección usando cuadratura
    int delta = (A == B) ? -1 : 1;
    if (Encoder::rightInverted) delta = -delta;
    rightPulses += delta;
}

void Encoder::setLeftInverted(bool inv) { leftInverted = inv; }
void Encoder::setRightInverted(bool inv) { rightInverted = inv; }
bool Encoder::isLeftInverted() { return leftInverted; }
bool Encoder::isRightInverted() { return rightInverted; }