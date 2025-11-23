# Análisis Profundo: Máquina de Estados del Movimiento Automático
## Líneas 929-1070 del archivo AMR_Complete.ino

---

## 📋 RESUMEN EJECUTIVO

La máquina de estados implementa un sistema de navegación automática con evasión de obstáculos para un robot móvil (AMR). El código maneja la ejecución de rutas predefinidas con capacidad de detección y evasión de obstáculos en tiempo real.

---

## 🔄 ESTRUCTURA DE LA MÁQUINA DE ESTADOS

### Estados Principales (RouteState enum):
1. **ROUTE_IDLE** (0): Inactivo
2. **ROUTE_WAITING** (1): Esperando delay inicial o confirmación
3. **ROUTE_TURNING** (2): Realizando giro hacia el siguiente waypoint
4. **ROUTE_MOVING** (3): Moviéndose hacia el waypoint objetivo
5. **ROUTE_DONE** (4): Ruta completada

### Estados de Evasión de Obstáculos (obstacleState):
1. **0 (IDLE)**: Sin evasión activa
2. **1 (TURN)**: Giro inicial de 90° hacia el lado elegido
3. **2 (FORWARD)**: Avance lateral mientras se monitorea el sensor opuesto
4. **3 (TURNBACK)**: Giro de retorno de 90° hacia la dirección original
5. **4 (CROSS_FORWARD)**: Avance final para cruzar el obstáculo
6. **5 (DONE)**: Evasión completada

---

## 🔍 ANÁLISIS DETALLADO POR SECCIÓN

### **Línea 930: handleAutoTurn()**
```cpp
handleAutoTurn();
```
- **Función**: Gestiona la finalización de giros automáticos
- **Responsabilidades**:
  - Monitorea encoders para detectar cuando un giro ha completado
  - Transiciona estados de evasión de obstáculos (1→2, 3→4)
  - Maneja giros post-finalización de ruta

---

### **Líneas 933-939: Estado ROUTE_WAITING**
```cpp
if (routeExec.state == ROUTE_WAITING) {
    if (now - routeExec.requestMillis >= routeExec.delayMs) {
        beginNextWaypoint();
    }
}
```
**Análisis**:
- ✅ **Correcto**: Espera el tiempo de delay antes de iniciar
- ✅ **Correcto**: Llama a `beginNextWaypoint()` cuando expira el delay
- ⚠️ **Observación**: No verifica si `routeExec.awaitingConfirm` está activo, pero esto se maneja en otras partes del código

---

### **Líneas 940-961: Estado ROUTE_TURNING**
```cpp
else if (routeExec.state == ROUTE_TURNING) {
    if (!turningInProgress) {
        // Calcula distancia al objetivo
        float dx = routeExec.targetX - odometry.getX();
        float dy = routeExec.targetY - odometry.getY();
        float dist = sqrtf(dx*dx + dy*dy);
        // Calcula pulsos requeridos
        float pulsesF = (dist / (float)WHEEL_CIRCUMFERENCE_CM) * (float)encoders.getPulsesPerRevolution();
        routeExec.moveTargetPulses = (long)(pulsesF + 0.5f);
        routeExec.moveStartLeft = encoders.readLeft();
        routeExec.moveStartRight = encoders.readRight();
        if (routeExec.moveTargetPulses <= 0) {
            // Nada que mover, avanzar al siguiente waypoint
            routeExec.currentPoint++;
            beginNextWaypoint();
        } else {
            motors.moveForward();
            routeExec.state = ROUTE_MOVING;
        }
    }
}
```
**Análisis**:
- ✅ **Correcto**: Espera a que `turningInProgress` sea false
- ✅ **Correcto**: Calcula la distancia y pulsos necesarios
- ✅ **Correcto**: Maneja el caso donde no hay distancia a recorrer
- ✅ **Correcto**: Guarda posición inicial de encoders antes de moverse
- ⚠️ **Observación**: El cálculo de distancia se hace después del giro, lo cual es correcto ya que la posición puede haber cambiado

---

### **Líneas 962-1067: Estado ROUTE_MOVING**

Este es el estado más complejo, con lógica anidada para detección y evasión de obstáculos.

#### **Fase 1: Detección de Obstáculos (Líneas 963-1006)**

**Líneas 963-966: Lectura de Sensores**
```cpp
float dFL = distanciaSamples(IR_FRONT_LEFT_PIN, 3, NULL);
float dFR = distanciaSamples(IR_FRONT_RIGHT_PIN, 3, NULL);
float frontMin = min(dFL, dFR);
```
- ✅ **Correcto**: Lee ambos sensores frontales y toma el mínimo
- ✅ **Correcto**: Usa muestreo (3 muestras) para reducir ruido

**Líneas 970-973: Inicio de Período de Espera**
```cpp
if (!routeExec.obstacleActive && !routeExec.obstacleWaitActive && frontMin <= OBSTACLE_THRESHOLD_CM) {
    routeExec.obstacleWaitActive = true;
    routeExec.obstacleWaitStartMillis = millis();
}
```
- ✅ **Correcto**: Implementa un período de confirmación para evitar falsas alarmas
- ✅ **Correcto**: Solo activa si no hay evasión en progreso
- ✅ **Correcto**: Usa umbral de 30cm (OBSTACLE_THRESHOLD_CM)

**Líneas 974-1006: Confirmación de Obstáculo**
```cpp
else if (!routeExec.obstacleActive && routeExec.obstacleWaitActive) {
    if (millis() - routeExec.obstacleWaitStartMillis >= OBSTACLE_DETECTION_DELAY_MS) {
        // Re-muestra para confirmar
        float frontMin2 = min(dFL2, dFR2);
        if (frontMin2 <= OBSTACLE_THRESHOLD_CM) {
            // Confirma obstáculo y activa evasión
            routeExec.obstacleSide = (dL > dR) ? +1 : -1;
            routeExec.obstacleActive = true;
            routeExec.obstacleState = 1; // TURN
            startAutoTurn(routeExec.obstacleSide * 90.0f);
        }
    }
}
```
- ✅ **Correcto**: Espera 2 segundos (OBSTACLE_DETECTION_DELAY_MS) antes de confirmar
- ✅ **Correcto**: Re-muestra los sensores para confirmar
- ✅ **Correcto**: Elige el lado con más espacio libre (izquierda si dL > dR)
- ✅ **Correcto**: Configura el pin de sonda opuesto al giro
- ✅ **Correcto**: Calcula límite de seguridad (AVOID_MAX_STEP_CM = 200cm)
- ⚠️ **Observación**: Si el obstáculo desaparece durante la espera, continúa normalmente (línea 1004)

---

#### **Fase 2: Manejo de Evasión Activa (Líneas 1007-1054)**

**Líneas 1011-1025: Estado 2 (FORWARD) - Avance Lateral**
```cpp
if (routeExec.obstacleState == 2) {
    float probeDist = distanciaSamples(routeExec.obstacleProbePin, 3, NULL);
    long maxm = max(dl, dr); // pulsos máximos movidos
    if (probeDist >= (OBSTACLE_THRESHOLD_CM + AVOID_CLEAR_MARGIN_CM) || maxm >= routeExec.obstacleMoveMaxPulses) {
        motors.stop();
        routeExec.obstacleState = 3; // TURNBACK
        startAutoTurn(-routeExec.obstacleSide * 90.0f);
    }
}
```
- ✅ **Correcto**: Monitorea el sensor opuesto para detectar cuando el obstáculo ha sido superado
- ✅ **Correcto**: Usa umbral de 38cm (30cm + 8cm de margen)
- ✅ **Correcto**: Tiene límite de seguridad de 200cm para evitar avance infinito
- ✅ **Correcto**: Gira en dirección opuesta para volver a la dirección original

**Líneas 1026-1053: Estado 4 (CROSS_FORWARD) - Avance Final**
```cpp
else if (routeExec.obstacleState == 4) {
    long maxm = max(dl, dr);
    if (maxm >= routeExec.obstacleMoveTargetPulses) {
        motors.stop();
        routeExec.obstacleState = 5; // DONE
        routeExec.obstacleActive = false;
        // Recalcula movimiento hacia el waypoint original
        // ...
    }
}
```
- ✅ **Correcto**: Avanza una distancia fija (AVOID_STEP_CM = 30cm)
- ✅ **Correcto**: Al finalizar, recalcula la distancia al waypoint objetivo
- ✅ **Correcto**: Si ya está en el waypoint (distancia <= 0), avanza al siguiente
- ✅ **Correcto**: Si no, continúa moviéndose hacia el waypoint original

---

#### **Fase 3: Movimiento Normal (Líneas 1055-1067)**
```cpp
else {
    // Normal movement completion check (no obstacle active)
    long maxm = max(dl, dr);
    if (maxm >= routeExec.moveTargetPulses) {
        motors.stop();
        routeExec.currentPoint++;
        delay(80);
        beginNextWaypoint();
    }
}
```
- ✅ **Correcto**: Solo se ejecuta cuando no hay evasión activa
- ✅ **Correcto**: Verifica si se alcanzaron los pulsos objetivo
- ✅ **Correcto**: Avanza al siguiente waypoint al completar
- ⚠️ **Observación**: El `else` en línea 1055 corresponde al `if (routeExec.obstacleActive)` de línea 1007, lo cual es correcto

---

## ⚠️ PROBLEMAS IDENTIFICADOS

### **1. PROBLEMA CRÍTICO: Falta de Manejo de Estados 1 y 3 en ROUTE_MOVING**

**Ubicación**: Líneas 1007-1054

**Descripción**: 
- Los estados `obstacleState == 1` (TURN) y `obstacleState == 3` (TURNBACK) NO se manejan en el bloque `ROUTE_MOVING`
- Estos estados se manejan en `handleAutoTurn()` (líneas 1456-1475), lo cual es correcto
- **PERO**: Cuando `obstacleActive == true` y `obstacleState == 1` o `3`, el código entra al bloque `else if (routeExec.obstacleActive)` pero no hace nada porque solo verifica estados 2 y 4

**Impacto**: 
- ⚠️ **BAJO**: No es un bug crítico porque `handleAutoTurn()` maneja las transiciones 1→2 y 3→4
- Sin embargo, durante los giros (estados 1 y 3), el código no verifica si el obstáculo frontal desapareció

**Recomendación**: 
- Agregar verificación opcional durante estados 1 y 3 para cancelar evasión si el obstáculo desaparece

---

### **2. PROBLEMA MENOR: Indentación Inconsistente**

**Ubicación**: Líneas 1007-1054

**Descripción**: 
- La indentación del bloque `if (routeExec.obstacleActive)` no es consistente
- Línea 1007: `} else if (routeExec.obstacleActive) {`
- Línea 1008-1010: Comentarios con indentación extra
- Línea 1011: `if (routeExec.obstacleState == 2) {` con indentación incorrecta

**Impacto**: 
- ⚠️ **MUY BAJO**: Solo afecta legibilidad

**Recomendación**: 
- Corregir indentación para mejorar legibilidad

---

### **3. PROBLEMA POTENCIAL: Condición del else en Línea 1055**

**Ubicación**: Línea 1055

**Descripción**: 
- El `else` en línea 1055 corresponde al `if (routeExec.obstacleActive)` de línea 1007
- Esto significa que el movimiento normal solo se verifica cuando `obstacleActive == false`
- **PERO**: También debe ser `false` cuando `obstacleWaitActive == true`

**Análisis**:
- ✅ **Correcto**: La estructura es:
  ```
  if (!obstacleActive && !obstacleWaitActive && obstáculo detectado) → inicia espera
  else if (!obstacleActive && obstacleWaitActive) → confirma obstáculo
  else if (obstacleActive) → maneja evasión
  else → movimiento normal
  ```
- El `else` solo se ejecuta cuando `obstacleActive == false` Y no estamos en espera de confirmación, lo cual es correcto

**Impacto**: 
- ✅ **NINGUNO**: La lógica es correcta

---

### **4. PROBLEMA MENOR: Falta de Verificación de Obstáculo Durante Evasión**

**Descripción**: 
- Durante la evasión (estados 2 y 4), no se verifica si aparece un nuevo obstáculo frontal
- Si aparece un obstáculo durante la evasión, el robot podría chocar

**Impacto**: 
- ⚠️ **MEDIO**: En entornos dinámicos, podría ser problemático

**Recomendación**: 
- Agregar verificación de obstáculo frontal durante estados 2 y 4
- Si se detecta, detener y reiniciar evasión o implementar evasión anidada

---

## ✅ ASPECTOS POSITIVOS

1. **Sistema de Confirmación de Obstáculos**: 
   - Implementa un delay de 2 segundos para evitar falsas alarmas
   - Re-muestra los sensores antes de confirmar

2. **Límites de Seguridad**: 
   - `AVOID_MAX_STEP_CM = 200cm` previene avance infinito
   - Verificación de pulsos máximos en cada paso

3. **Recálculo de Ruta**: 
   - Después de evasión, recalcula la distancia al waypoint original
   - Permite continuar la ruta desde la nueva posición

4. **Selección Inteligente de Lado**: 
   - Elige el lado con más espacio libre para evasión

5. **Manejo de Casos Especiales**: 
   - Maneja correctamente el caso donde no hay distancia a recorrer
   - Maneja el caso donde el waypoint se alcanza durante la evasión

---

## 🔧 RECOMENDACIONES DE MEJORA

### **1. Agregar Verificación de Obstáculo Durante Evasión**
```cpp
// En estado 2 (FORWARD) y 4 (CROSS_FORWARD)
if (routeExec.obstacleState == 2 || routeExec.obstacleState == 4) {
    float dFL = distanciaSamples(IR_FRONT_LEFT_PIN, 3, NULL);
    float dFR = distanciaSamples(IR_FRONT_RIGHT_PIN, 3, NULL);
    float frontMin = min(dFL, dFR);
    if (frontMin <= OBSTACLE_THRESHOLD_CM * 0.7) { // Umbral más estricto
        // Detener y reiniciar evasión o implementar evasión anidada
        motors.stop();
        // ... lógica de reinicio
    }
}
```

### **2. Mejorar Manejo de Estados 1 y 3**
```cpp
else if (routeExec.obstacleActive) {
    if (routeExec.obstacleState == 1 || routeExec.obstacleState == 3) {
        // Durante giros, verificar si obstáculo desapareció
        float dFL = distanciaSamples(IR_FRONT_LEFT_PIN, 3, NULL);
        float dFR = distanciaSamples(IR_FRONT_RIGHT_PIN, 3, NULL);
        float frontMin = min(dFL, dFR);
        if (frontMin > OBSTACLE_THRESHOLD_CM + 10.0f) {
            // Obstáculo desapareció, cancelar evasión
            motors.stop();
            routeExec.obstacleActive = false;
            routeExec.obstacleState = 0;
            // Continuar movimiento normal
            motors.moveForward();
        }
    } else if (routeExec.obstacleState == 2) {
        // ... código existente
    } else if (routeExec.obstacleState == 4) {
        // ... código existente
    }
}
```

### **3. Corregir Indentación**
- Ajustar indentación del bloque `if (routeExec.obstacleActive)` para consistencia

### **4. Agregar Logging Adicional**
- Agregar más mensajes Serial para debugging durante evasión
- Incluir información de sensores en cada estado

---

## 📊 DIAGRAMA DE FLUJO

```
ROUTE_MOVING
    │
    ├─→ Lectura sensores frontales
    │
    ├─→ ¿Obstáculo detectado? (frontMin <= 30cm)
    │   │
    │   ├─→ NO → Verificar completitud de movimiento normal
    │   │         │
    │   │         └─→ ¿Pulsos >= objetivo? → Siguiente waypoint
    │   │
    │   └─→ SÍ → ¿Ya en espera de confirmación?
    │       │
    │       ├─→ NO → Iniciar período de espera (2 seg)
    │       │
    │       └─→ SÍ → ¿Tiempo expirado?
    │           │
    │           ├─→ NO → Continuar esperando
    │           │
    │           └─→ SÍ → Re-muestrear sensores
    │               │
    │               ├─→ ¿Obstáculo confirmado?
    │               │   │
    │               │   ├─→ NO → Continuar movimiento normal
    │               │   │
    │               │   └─→ SÍ → Iniciar evasión
    │               │           │
    │               │           ├─→ Estado 1 (TURN): Giro 90°
    │               │           │   └─→ handleAutoTurn() → Estado 2
    │               │           │
    │               │           ├─→ Estado 2 (FORWARD): Avance lateral
    │               │           │   └─→ ¿Sensor opuesto libre? → Estado 3
    │               │           │
    │               │           ├─→ Estado 3 (TURNBACK): Giro -90°
    │               │           │   └─→ handleAutoTurn() → Estado 4
    │               │           │
    │               │           └─→ Estado 4 (CROSS_FORWARD): Avance final
    │               │               └─→ ¿Pulsos >= objetivo? → Estado 5
    │               │                   └─→ Recalcular ruta → Continuar
```

---

## 🎯 CONCLUSIÓN

La máquina de estados está **bien implementada** en general, con una lógica sólida para:
- ✅ Navegación hacia waypoints
- ✅ Detección de obstáculos con confirmación
- ✅ Evasión de obstáculos en 4 pasos
- ✅ Recálculo de ruta después de evasión

**Problemas identificados**:
- ⚠️ Falta verificación de obstáculos durante evasión (riesgo medio)
- ⚠️ Indentación inconsistente (riesgo bajo, solo legibilidad)
- ⚠️ No se puede cancelar evasión si obstáculo desaparece durante giros (riesgo bajo)

**Recomendación general**: El código es funcional y robusto, pero se beneficiaría de las mejoras sugeridas para entornos más dinámicos.

