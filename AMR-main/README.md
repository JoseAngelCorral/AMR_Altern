# 🤖 AMR (Autonomous Mobile Robot) - Sistema de Control Completo

## 📋 Descripción del Proyecto

Sistema completo de control para robot móvil autónomo basado en Arduino Uno (o UNO R4 WiFi) con navegación automática, evasión de obstáculos, interfaz web y control por comandos serie. El robot incluye encoders de alta resolución, drivers de motor BTS7960, sensores IR y capacidades de tracking de posición en tiempo real.

### Características Principales

- ✅ **Navegación Automática**: Sistema de rutas predefinidas con waypoints
- ✅ **Evasión de Obstáculos**: Sistema inteligente con confirmación de 2 segundos
- ✅ **Seguimiento de Pared**: Modo automático para seguir pared izquierda o derecha
- ✅ **Máquina de Estados Robusta**: Control preciso de ejecución de rutas
- ✅ **Interfaz Web Dashboard**: Visualización en tiempo real con gráficos
- ✅ **Control PID de Velocidad**: Sistema opcional con interpolación de encoders
- ✅ **Odometría Precisa**: Tracking de posición basado en encoders
- ✅ **Sensores IR**: 5 sensores para detección de obstáculos
- ✅ **Control Dual**: Por comandos serie y por interfaz web

## 🛠️ Hardware Requerido

### Componentes Principales:
- **Arduino Uno / UNO R4 WiFi** - Microcontrolador principal
- **Encoder E386G5** - Calibrado a ~3418 PPR (pulsos por revolución)
- **Driver BTS7960** (x2) - Control de motores DC (hasta 43A cada uno)
- **Motores DC** (x2) - Con ruedas de 15.50cm de diámetro
- **Sensores IR Analógicos** (x5) - Detección de obstáculos
- **Fuente de alimentación** - Para motores (12V/24V recomendado)

### Especificaciones Técnicas:
- **Resolución de encoder**: ~3418 pulsos por revolución (calibrable)
- **Diámetro de rueda**: 15.50cm
- **Base de ruedas**: 63.5cm (centro a centro)
- **Velocidad máxima**: 0-255 PWM (configurable)
- **Velocidad por defecto**: 102 PWM (~40% de máximo)
- **Velocidad de giro**: 51 PWM (~20% de máximo)

## 🔌 Conexiones de Hardware

### Resumen de Pines

**Encoders:**
- Encoder Izquierdo: A = `PIN 8`, B = `PIN 2` (INT0)
- Encoder Derecho: A = `PIN 9`, B = `PIN 3` (INT1)
- Nota: Los pines 2 y 3 son interrupciones externas usadas por las ISRs del encoder

**Drivers de Motor (BTS7960):**
- Motor Izquierdo (BTS7960 #1):
  - RPWM = `PIN 10` (PWM - sentido "atrás")
  - LPWM = `PIN 11` (PWM - sentido "adelante")
- Motor Derecho (BTS7960 #2):
  - RPWM = `PIN 5` (PWM - sentido "atrás")
  - LPWM = `PIN 6` (PWM - sentido "adelante")
- REN / LEN (enables): Alimentación externa (siempre HIGH)

**Sensores IR Analógicos:**
- LEFT_SIDE → `A5` (Lateral izquierdo)
- FRONT_LEFT → `A4` (Frontal izquierdo)
- BACK_CENTER → `A2` (Trasero central)
- FRONT_RIGHT → `A1` (Frontal derecho)
- RIGHT_SIDE → `A0` (Lateral derecho)

### ⚠️ IMPORTANTE - Alimentación y Seguridad

- Los BTS7960 controlan la alimentación de los motores y **deben alimentarse desde una fuente externa** (12V o 24V según tus motores)
- **NUNCA** alimentes los motores desde la salida 5V del Arduino
- Conectar las masas (GND) del Arduino y de la fuente de motor: **GND común**
- REN / LEN (enables) de cada BTS7960 deben estar a nivel HIGH (VCC) para permitir el driver

## 📡 Interfaz Web

El sistema incluye una interfaz web completa accesible cuando el robot está en modo Access Point:

- **SSID**: `AMR_Robot_AP`
- **Contraseña**: `12345678` (configurable en código)
- **URL Principal**: `http://<robot_ip>/` (Dashboard)
- **URL Rutas**: `http://<robot_ip>/routes_ui` (Control de rutas)

### Características del Dashboard:
- Visualización de trayectoria recorrida en tiempo real
- Brújula con orientación actual
- Gráficos de sensores IR con distancias en cm
- Control D-pad para movimiento manual
- Botones de pruebas rápidas

### Características de la Interfaz de Rutas:
- Selección de ruta y modo (Ida/Retorno)
- Control de ejecución con delay configurable
- **Seguimiento de Pared**: Botones para iniciar seguimiento de pared izquierda o derecha
- Vista previa de waypoints (implementación futura)

## ⌨️ Comandos Serie (115200 baudios)

### Comandos de Movimiento:
- **W** - Adelante (hold, mantén presionado)
- **S** - Atrás (hold)
- **A** - Giro izquierda 90° (automático por encoder)
- **D** - Giro derecha 90° (automático por encoder)
- **Q** - Giro izquierda continuo (hasta X)
- **E** - Giro derecha continuo (hasta X)
- **X** - Parar todos los motores

### Comandos de Utilidad:
- **R** - Reset posición odométrica (vuelve a 0,0,0°)
- **P** - Mostrar posición actual (x, y, theta)
- **H** - Mostrar ayuda (lista de comandos)

### Comandos de Prueba:
- **T** - Test completo de motores (secuencia automática)
- **V** - Avanzar exactamente 1 vuelta (calibración encoder)
- **I** - Inspección continua (muestra encoders y sensores IR cada 250ms)

## 🗺️ Sistema de Navegación

### Rutas Predefinidas

El sistema incluye 4 rutas predefinidas (configurables en código):
- **Ruta A**: `{0,0} → {30,0} → {30,30}`
- **Ruta B**: `{-10,5} → {0,10} → {10,5} → {0,0}`
- **Ruta C**: `{-10,5} → {0,10} → {10,5} → {0,0}`
- **Ruta D**: `{0,0} → {5,0} → {10,5}`

### Máquina de Estados

El sistema de navegación implementa una máquina de estados con 5 estados principales:

1. **ROUTE_IDLE**: Inactivo, sin ruta en ejecución
2. **ROUTE_WAITING**: Esperando delay inicial o confirmación del operador
3. **ROUTE_TURNING**: Realizando giro hacia el siguiente waypoint
4. **ROUTE_MOVING**: Moviéndose hacia el waypoint objetivo (con evasión de obstáculos)
5. **ROUTE_DONE**: Ruta completada

### Modos de Ejecución:
- **Ida**: Ejecuta la ruta desde el primer waypoint al último
- **Retorno**: Ejecuta la ruta en sentido inverso
- **Confirmación**: Requiere confirmación del operador antes de iniciar retorno

## 🚧 Sistema de Evasión de Obstáculos

### Características:
- **Confirmación de 2 segundos**: Evita falsas alarmas con objetos transitorios
- **Selección automática de lado**: Elige el lado con más espacio libre
- **Máquina de estados de 5 fases**:
  1. **TURN**: Giro inicial de 90° hacia el lado elegido
  2. **FORWARD**: Avance lateral mientras se monitorea el sensor opuesto
  3. **TURNBACK**: Giro de retorno de 90° hacia la dirección original
  4. **CROSS_FORWARD**: Avance final para cruzar el obstáculo
  5. **DONE**: Evasión completada, recalcula ruta hacia waypoint original

### Parámetros Configurables:
- `OBSTACLE_THRESHOLD_CM = 30.0cm` - Distancia mínima para considerar obstáculo
- `OBSTACLE_DETECTION_DELAY_MS = 2000ms` - Tiempo de confirmación
- `AVOID_STEP_CM = 30.0cm` - Distancia de avance por paso
- `AVOID_CLEAR_MARGIN_CM = 8.0cm` - Margen adicional para considerar objeto superado
- `AVOID_MAX_STEP_CM = 200.0cm` - Límite de seguridad para avance máximo

## 🧱 Sistema de Seguimiento de Pared

### Características:
- **Modos disponibles**: Seguimiento de pared izquierda o derecha
- **Control automático de velocidad**: Ajusta velocidad proporcionalmente según distancia a la pared seguida
- **Detección de topes**: Usa sensores frontales para detectar obstáculos y girar
- **Manejo de esquinas externas**: Gira automáticamente 90° hacia el exterior cuando detecta esquina
- **Detección completa**: Se detiene cuando todos los sensores (frontales + laterales) detectan pared
- **Timeout automático**: Finaliza si todas las paredes se detectan por más de 10 segundos
- **Modo independiente**: Completamente aislado de las rutas automáticas (no hay combinación entre ambos modos)

### Control Automático de Velocidad:
El sistema implementa un **control proporcional de velocidad** basado en la distancia a la pared seguida:

- **Muy cerca (<15cm)**: Reduce velocidad a 60-100% de la base y se aleja ligeramente
  - La velocidad es proporcional: más cerca = más lento
  - Ajusta dirección para alejarse de la pared
  
- **Rango ideal (15-25cm)**: Mantiene velocidad base constante
  - Velocidad óptima para seguimiento estable
  
- **Lejos (>25cm)**: Aumenta velocidad hasta 120% de la base y se acerca ligeramente
  - La velocidad aumenta proporcionalmente con la distancia
  - Ajusta dirección para acercarse a la pared

**Ventajas**: Este sistema permite un seguimiento más suave y estable, adaptándose automáticamente a las condiciones del entorno sin intervención manual.

### Manejo de Esquinas Externas:
Cuando el robot llega a una **esquina externa** (la pared que sigue se aleja formando un ángulo de 90° hacia afuera):

- **Condición detectada**: 
  - Los sensores frontales **NO** detectan pared
  - El sensor lateral que está siguiendo **deja de detectar** la pared seguida
  
- **Acción automática**:
  - El robot realiza un **giro de 90° hacia el exterior** (hacia donde estaba la pared seguida)
  - Esto permite continuar siguiendo la pared alrededor de la esquina
  - El giro se realiza automáticamente sin necesidad de intervención

**Ejemplo**: Si estás siguiendo una pared izquierda y llegas a una esquina donde la pared gira 90° hacia la izquierda, el sensor lateral izquierdo dejará de detectar la pared, pero no habrá pared al frente. El robot girará 90° hacia la izquierda para seguir la nueva dirección de la pared.

### Estados del Sistema:
1. **FOLLOWING**: Siguiendo la pared activamente
2. **TURNING**: Realizando giro para evitar obstáculo frontal o seguir esquina externa
3. **STOPPED**: Todas las paredes detectadas (esperando timeout o cambio de condiciones)

### Condiciones de Finalización:
El seguimiento de pared se detiene automáticamente cuando:
- **Todas las paredes detectadas por más de 10 segundos**: Timeout automático
- **Comando manual**: El usuario presiona "Detener Seguimiento" en la interfaz web
- **Inicio de ruta automática**: Si se inicia una ruta, el seguimiento de pared se detiene automáticamente

### Parámetros Configurables:
- `WALL_FOLLOW_THRESHOLD_CM = 30.0cm` - Distancia para considerar pared detectada
- `ALL_WALLS_TIMEOUT_MS = 10000ms` - Tiempo máximo con todas las paredes detectadas antes de finalizar
- `WALL_FOLLOW_SPEED = 100` - Velocidad base PWM para seguimiento
- `WALL_FOLLOW_TURN_SPEED = 80` - Velocidad PWM para giros durante seguimiento

### Uso desde Interfaz Web:
1. Acceder a la interfaz de rutas: `http://<robot_ip>/routes_ui`
2. En la sección "Seguimiento de Pared", seleccionar:
   - **Seguir Pared Izquierda**: Inicia seguimiento usando sensor lateral izquierdo
   - **Seguir Pared Derecha**: Inicia seguimiento usando sensor lateral derecho
   - **Detener Seguimiento**: Cancela el modo de seguimiento

### Endpoints HTTP:
- `GET /wall_follow?side=left|right` - Inicia seguimiento de pared
- `GET /stop_wall_follow` - Detiene seguimiento de pared

### Notas Importantes:
- **Modos completamente independientes**: El seguimiento de pared y las rutas automáticas son **mutuamente excluyentes** y **completamente aislados**
- **No hay combinación**: Es una opción O la otra, nunca ambas al mismo tiempo
- **Al iniciar seguimiento de pared**: Cualquier ruta activa se detiene automáticamente
- **Al iniciar una ruta**: Cualquier seguimiento de pared activo se detiene automáticamente
- **Uso independiente**: El seguimiento de pared funciona de forma autónoma sin necesidad de tener una ruta programada

## ⚙️ Sistema PID de Velocidad

### Características:
- **Interpolación de encoders**: Promedia lecturas de ambos encoders para generar un valor único
- **Control único**: Un solo controlador PID para ambos motores
- **Factor de compensación**: Motor derecho con factor 1.1 para corregir curva a la derecha
- **Rampa suave**: Soft-start configurable (por defecto 800ms)

### Parámetros PID (ajustables):
- **Kp = 0.08** - Ganancia proporcional
- **Ki = 0.02** - Ganancia integral
- **Kd = 0.002** - Ganancia derivativa
- **Integral Clamp = 500.0** - Límite anti-windup

### Uso:
El sistema PID es opcional y se activa mediante la API de `MotorDriver`. Actualmente no hay comando serie para activarlo (se removió el comando 'Y').

## 📊 Tabla de Velocidades (PWM)

| Modo | Acción | Valor PWM | % de MAX | Comentarios |
|------|--------|-----------|----------|-------------|
| Manual | Adelante/Atrás (hold - `W`/`S`) | 102 | 40% | `MAX_SPEED * 0.40f` |
| Manual | Giro en sitio (hold - `Q`/`E`) | 51 | 20% | `MAX_SPEED * 0.20f` |
| Automático | Avance por defecto | 102 | 40% | `DEFAULT_SPEED` |
| Automático | Giro automático 90° (`A`/`D`) | 51 | 20% | `TURN_SPEED` |
| Sistema | Velocidad mínima | 80 | 31.4% | `MIN_SPEED` (supera fricción) |
| Sistema | Velocidad máxima | 255 | 100% | `MAX_SPEED` |

**Nota**: Los valores en PWM son enteros 0..255. La convención es: LPWM activa movimiento "adelante" y RPWM activa "atrás".

## ✅ Tests y Calibración

### Comando `T` - Test Completo de Motores
Ejecuta una secuencia automática de pruebas:
1. Motor izquierdo adelante (1 segundo)
2. Motor izquierdo atrás (1 segundo)
3. Motor derecho adelante (1 segundo)
4. Motor derecho atrás (1 segundo)

**Uso**: Verificar que todos los motores funcionan correctamente en ambos sentidos.

### Comando `V` - Calibración de Encoder
Avanza exactamente una revolución de rueda y actualiza el valor de `pulsesPerRevolution` en runtime.

**Proceso**:
1. Lee contadores iniciales de encoders
2. Avanza hasta que la rueda que más ha girado alcance el objetivo
3. Calcula pulsos medidos por revolución
4. Actualiza configuración en runtime

**Uso**: Calibrar el encoder para mejorar precisión de odometría. Repetir varias veces y promediar para mejor precisión.

### Comando `I` - Inspección Continua
Muestra periódicamente (cada 250ms):
- Pulsos de encoders (izquierdo y derecho)
- Distancias de sensores IR en cm (5 sensores)

**Uso**: Monitorear comportamiento del robot en tiempo real. Detener con comando `X`.

## 🔧 Estructura del Código

El código está organizado en secciones claras:

1. **Librerías e includes**: MotorDriver, Encoder, Odometry, WiFi
2. **Configuración WiFi**: Access Point y HTML embebido (PROGMEM)
3. **Definición de rutas**: Estructuras de datos para waypoints
4. **Instancias globales**: motors, encoders, odometry
5. **Máquina de estados**: RouteExecution con evasión de obstáculos
6. **Seguimiento de pared**: Sistema WallFollow con estados y reanudación
7. **Variables de control**: Timing, logging, flags
8. **Sensores IR**: Lectura, conversión a distancia, detección
9. **Setup**: Inicialización de hardware y WiFi
10. **Loop principal**: Ejecución de tareas no bloqueantes
11. **Procesamiento de comandos**: Interfaz serie
12. **Giros automáticos**: Sistema basado en encoders
13. **Servidor WiFi**: Dashboard y API HTTP

## 📝 Notas de Desarrollo

### Optimizaciones Implementadas:
- Uso de PROGMEM para strings grandes (HTML)
- Sistema de logging circular para evitar desbordamiento
- Máquina de estados no bloqueante
- Lecturas de sensores con promediado para reducir ruido

### Mejoras Futuras Sugeridas:
- Implementar cache de respuestas HTTP estáticas
- Optimizar construcción de JSON (usar char[] en lugar de String)
- Agregar verificación de obstáculos durante evasión
- Implementar cancelación de evasión si obstáculo desaparece

## 📚 Documentación Adicional

- **Análisis de Código Completo**: Ver `ANALISIS_CODIGO_COMPLETO.md`
- **Análisis de Máquina de Estados**: Ver `ANALISIS_MAQUINA_ESTADOS.md`
- **Recomendaciones de Optimización**: Ver `OPTIMIZACIONES_RECOMENDADAS.md`

## 🐛 Solución de Problemas

### El robot no avanza en línea recta:
- Verificar calibración de encoders (comando `V`)
- Ajustar factor de compensación del motor derecho en `MotorDriver.h`
- Verificar que ambos motores tengan la misma carga

### El robot no detecta obstáculos:
- Verificar conexiones de sensores IR
- Ajustar `OBSTACLE_THRESHOLD_CM` según condiciones de iluminación
- Verificar que los sensores estén limpios

### La odometría es imprecisa:
- Recalibrar encoder (comando `V` varias veces y promediar)
- Verificar que las ruedas no patinen
- Ajustar `WHEEL_BASE_CM` si la distancia entre ruedas es diferente

### El WiFi no funciona:
- Verificar que uses Arduino UNO R4 WiFi o placa compatible
- Verificar que la librería WiFiS3 esté instalada
- Revisar que el SSID y contraseña sean correctos

## 📄 Licencia

Este proyecto está disponible para uso educativo y de desarrollo.

## 👥 Contribuciones

Las mejoras y sugerencias son bienvenidas. Por favor, documenta cualquier cambio significativo.

---

**Versión**: 1.0  
**Última actualización**: 2024  
**Plataforma**: Arduino Uno / UNO R4 WiFi
