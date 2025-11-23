/*
 * ========================================
 *          AMR CONTROL SYSTEM
 * ========================================
 * 
 * Sistema completo de control para robot móvil autónomo (AMR) con navegación
 * automática, evasión de obstáculos, interfaz web y control por comandos serie.
 * 
 * HARDWARE:
 * - Arduino Uno / UNO R4 WiFi
 * - Encoder E386G5 (calibrado a ~3418 PPR por defecto)
 * - Driver BTS7960 para motores (43A máx, 2 unidades)
 * - Ruedas 15.50cm diámetro
 * - Sensores IR analógicos (5 sensores: frontal, laterales, trasero)
 * 
 * CARACTERÍSTICAS PRINCIPALES:
 * - Navegación automática por waypoints con rutas predefinidas
 * - Sistema de evasión de obstáculos con confirmación (2 segundos)
 * - Máquina de estados robusta para ejecución de rutas
 * - Interfaz web dashboard con visualización en tiempo real
 * - Control PID de velocidad (opcional, con interpolación de encoders)
 * - Odometría precisa basada en encoders
 * 
 * CONTROLES SERIE (115200 baudios):
 * W = Adelante (hold)          S = Atrás (hold)
 * A = Giro izquierda 90°       D = Giro derecha 90° (por encoder)
 * Q = Giro izquierda continuo  E = Giro derecha continuo (hasta X)
 * X = Parar todos los motores
 * R = Reset posición odométrica
 * P = Mostrar posición actual
 * T = Test completo de motores
 * V = Avanzar exactamente 1 vuelta (calibración encoder)
 * I = Inspección continua (sensores IR y encoders)
 * 
 * VELOCIDADES (definidas en MotorDriver.h):
 * - DEFAULT_SPEED = 102 (~40% PWM) - Velocidad por defecto de avance
 * - TURN_SPEED    = 51  (~20% PWM) - Velocidad para giros automáticos
 * - MIN_SPEED     = 80  (~31% PWM) - Velocidad mínima para superar fricción
 * - MAX_SPEED     = 255 (100% PWM) - Velocidad máxima
 * 
 * CONEXIONES:
 * Encoder Izq:  A=Pin8, B=Pin2 (INT0)
 * Encoder Der:  A=Pin9, B=Pin3 (INT1)  
 * Motor Izq:    RPWM=Pin10, LPWM=Pin11 (LPWM=Adelante)
 * Motor Der:    RPWM=Pin5, LPWM=Pin6 (LPWM=Adelante)
 * Sensores IR:  LEFT_SIDE=A5, FRONT_LEFT=A4, BACK=A2, FRONT_RIGHT=A1, RIGHT_SIDE=A0
 * Enables:      Alimentación externa (siempre HIGH)
 * 
 * NOTAS:
 * - El sistema PID de velocidad interpola lecturas de ambos encoders para
 *   generar un valor único y aplicar un solo controlador PID.
 * - El motor derecho tiene un factor de compensación de 1.1 para corregir
 *   la tendencia a curvar hacia la derecha.
 * - La evasión de obstáculos implementa un sistema de confirmación de 2 segundos
 *   para evitar falsas alarmas con objetos transitorios.
 */

// ========================================
//             LIBRERÍAS
// ========================================
#include <avr/pgmspace.h>  // Para PROGMEM
#include "MotorDriver.h"
#include "Encoder.h"
#include "Odometry.h"
// WiFi (UNO R4 WiFi - WiFiS3 core)
#include <WiFiS3.h>
#include <WiFiServer.h>
#include <math.h>

// ======== CONFIGURACIÓN WIFI AP ========
const char* AP_SSID = "AMR_Robot_AP";
const char* AP_PASS = "12345678"; // puedes cambiarla

WiFiServer server(80);

// Forward declare types/functions that are referenced in generated prototypes
struct IRSensors;
void startAutoTurn(float angleDelta);

// ----------------------
// Routes data (for dropdown UI)
// ----------------------
struct Point { float x; float y; };

// Example routes (adjust/add as needed)
const Point route0[] = { {0.0f,0.0f}, {30.0f,0.0f}, {30.0f,30.0f} };
const Point route1[] = { {-10.0f,5.0f}, {0.0f,10.0f}, {10.0f,5.0f}, {0.0f,0.0f} };
const Point route2[] = { {-10.0f,5.0f}, {0.0f,10.0f}, {10.0f,5.0f}, {0.0f,0.0f} };
const Point route3[] = { {0.0f,0.0f}, {5.0f,0.0f}, {10.0f,5.0f} };

const char* routeNames[] = { "Ruta A", "Ruta B","Ruta C", "Ruta D" };
const Point* routesPoints[] = { route0, route1, route2, route3 };
const int routesCounts[] = { sizeof(route0)/sizeof(route0[0]), sizeof(route1)/sizeof(route1[0]), sizeof(route2)/sizeof(route1[0]), sizeof(route3)/sizeof(route3[0]) };
const int ROUTE_COUNT = 4;

// ----------------------
// Routes UI page (serves the dropdown HTML)
// Access it at: http://<robot_ip>/routes_ui
const char routesPageHTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="es">
<head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width,initial-scale=1" />
    <title>Control de Rutas - Robot</title>
    <style>
        /* Dashboard-like dark theme for Routes UI */
        html, body {
            background: #0b0b0b;
            color: #eaeaea;
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 12px;
            -webkit-user-select: none;
            -moz-user-select: none;
            -ms-user-select: none;
            user-select: none;
            -webkit-touch-callout: none;
            -webkit-tap-highlight-color: rgba(0,0,0,0);
            touch-action: manipulation;
        }
        .container { max-width: 780px; margin: 0 auto; }
    .card { background: #111; border: 1px solid #222; border-radius: 8px; padding: 12px; box-shadow: 0 2px 6px rgba(0,0,0,0.6); display:block; }
        .headerRow { display:flex; align-items:center; gap:10px; margin-bottom:10px; }
        .headerRow h2 { margin:0; font-size:1.2em; color:#7CFC00; }
        .headerRow button { background:transparent; border:0; color:#7CFC00; font-size:20px; width:40px; height:40px; border-radius:6px; cursor:pointer; }
        .headerRow button:active { transform: translateY(1px); }

    label { display:block; margin: 8px 0 6px 0; color:#ccc; text-align:center; }
    select, input[type=number] { width:90%; max-width:540px; padding:8px; border-radius:6px; border:1px solid #333; background:#0d0d0d; color:#eee; margin:0 auto; }
        select option { background:#111; color:#eee; }
        pre { background:#0e0e0e; color:#cfcfcf; padding:8px; border-radius:6px; }

    .row { display:flex; gap:18px; align-items:flex-start; width:100%; }
        .row > * { flex: 1 1 auto; }
    .controls { display:flex; gap:8px; margin-top:8px; flex-wrap:wrap; justify-content:center; }
    .controls button { background:#222; color:#7CFC00; border:none; padding:10px 12px; margin:4px 0; font-size:16px; border-radius:6px; cursor:pointer; }
        .controls button:hover { background:#7CFC00; color:#000; }
        
        /* Estilos para elementos deshabilitados (preview) */
        select:disabled {
            opacity: 0.6;
            cursor: not-allowed;
            background: #1a1a1a !important;
        }
        select:disabled option {
            color: #666;
        }

        @media (max-width:520px) {
            .container { padding:6px; }
            .headerRow h2 { font-size:1.0em; }
            .controls button { padding:12px 14px; font-size:18px; }
        }
    </style>
</head>
<body>
    <div class="container">
    <div class="card">
    <div class="headerRow">
        <button id="backToDash" title="Volver al dashboard" onclick="location.href='/'">←</button>
        <h2>Control de Rutas Automáticas</h2>
    </div>
    
    <div>
        <label for="routeSelect">Selecciona la ruta:</label>
        <select id="routeSelect"></select>
    </div>

    <div>
        <label for="waypointSelect">Waypoints de la ruta <span style="color:#888; font-size:0.85em;">(Preview - Implementación futura)</span>:</label>
        <select id="waypointSelect" disabled style="opacity:0.6; cursor:not-allowed; background:#1a1a1a;"></select>
    </div>

    <div>
        <label for="allPointsSelect">Todos los puntos <span style="color:#888; font-size:0.85em;">(Preview - Implementación futura)</span>:</label>
        <select id="allPointsSelect" size="6" disabled style="opacity:0.6; cursor:not-allowed; background:#1a1a1a;"></select>
    </div>

    <div class="row">
        <div>
            <label for="delaySec">Delay antes de iniciar (segundos):</label>
            <input id="delaySec" type="number" value="10" min="0" style="width:100%" />
        </div>
        <div style="display:flex;flex-direction:column;gap:6px;">
            <div class="controls">
                <button id="startIda">Iniciar (IDA)</button>
                <button id="startRet">Iniciar (RETORNO)</button>
                <button id="stopRoute">Detener</button>
            </div>
            <div class="controls">
                <button id="confirmStart" disabled>Confirmar inicio</button>
                <div style="padding:6px; background:#222; color:#0f0; border-radius:4px; text-align:center;">Cuenta regresiva: <span id="countdown">--</span></div>
            </div>
            <div style="margin-top:12px; padding-top:12px; border-top:1px solid #333;">
                <div style="text-align:center; color:#7CFC00; margin-bottom:8px; font-size:0.9em;">Seguimiento de Pared</div>
                <div class="controls">
                    <button id="wallFollowLeft" style="background:#444; color:#7CFC00;">Seguir Pared Izquierda</button>
                    <button id="wallFollowRight" style="background:#444; color:#7CFC00;">Seguir Pared Derecha</button>
                    <button id="stopWallFollow" style="background:#666; color:#fff;">Detener Seguimiento</button>
                </div>
            </div>
            
        </div>
    </div>

    </div> <!-- .card -->
    </div> <!-- .container -->

    <div>
        <strong>Estado:</strong>
        <pre id="status">Inactivo</pre>
    </div>

    <div>
        <strong>Información de la ruta:</strong>
        <pre id="selected">Selecciona una ruta para ver sus waypoints</pre>
    </div>

    <script>
        const ROUTES_URL = '/routes';
        const START_URL = '/start_route';
        const STOP_URL = '/stop_route';
        const STATUS_URL = '/route_status';
        const CONFIRM_URL = '/confirm_route';
        const WALL_FOLLOW_URL = '/wall_follow';
        const STOP_WALL_FOLLOW_URL = '/stop_wall_follow';

        let routes = [];
        const routeSelect = document.getElementById('routeSelect');
        const waypointSelect = document.getElementById('waypointSelect');
        const allPointsSelect = document.getElementById('allPointsSelect');
        const selectedPre = document.getElementById('selected');
        const statusPre = document.getElementById('status');
        const countdownSpan = document.getElementById('countdown');
        const confirmBtn = document.getElementById('confirmStart');

        async function loadRoutes() {
            try {
                const resp = await fetch(ROUTES_URL, { cache: 'no-store' });
                if (!resp.ok) throw new Error('Error ' + resp.status);
                routes = await resp.json();
                populateRouteSelect();
                populateAllPoints();
                if (routes.length > 0) { 
                    routeSelect.selectedIndex = 0; 
                    populateWaypointsForRoute(0); 
                    // Mostrar información inicial de la primera ruta
                    const firstRoute = routes[0];
                    if (firstRoute && firstRoute.points && firstRoute.points.length > 0) {
                        selectedPre.textContent = `Ruta: ${firstRoute.name || 'Ruta 0'}\nWaypoints: ${firstRoute.points.length}\nPrimer punto: (${firstRoute.points[0].x}, ${firstRoute.points[0].y})\nÚltimo punto: (${firstRoute.points[firstRoute.points.length-1].x}, ${firstRoute.points[firstRoute.points.length-1].y})`;
                    }
                }
            } catch (err) {
                statusPre.textContent = 'No se pudieron cargar rutas: ' + err;
            }
        }

        function populateRouteSelect() {
            routeSelect.innerHTML = '';
            routes.forEach((r, idx) => { const opt=document.createElement('option'); opt.value=idx; opt.textContent = `Ruta: ${r.name||('#'+idx)}`; routeSelect.appendChild(opt); });
        }
        function populateWaypointsForRoute(routeIndex) {
            waypointSelect.innerHTML = '';
            const waypoints = (routes[routeIndex] && routes[routeIndex].points) || [];
            waypoints.forEach((pt,pIndex)=>{ 
                const opt=document.createElement('option'); 
                opt.value=`${routeIndex}|${pIndex}`; 
                opt.textContent=`Waypoint ${pIndex + 1}: (${pt.x}, ${pt.y})`; 
                waypointSelect.appendChild(opt); 
            });
            if (waypoints.length===0){ 
                const opt=document.createElement('option'); 
                opt.textContent='(sin waypoints)'; 
                opt.value=''; 
                waypointSelect.appendChild(opt);
            }
            // Actualizar información mostrada
            if (waypoints.length > 0) {
                const routeName = routes[routeIndex]?.name || `Ruta ${routeIndex}`;
                selectedPre.textContent = `Ruta: ${routeName}\nWaypoints: ${waypoints.length}\nPrimer punto: (${waypoints[0].x}, ${waypoints[0].y})\nÚltimo punto: (${waypoints[waypoints.length-1].x}, ${waypoints[waypoints.length-1].y})`;
            }
        }
        function populateAllPoints(){ allPointsSelect.innerHTML=''; routes.forEach((r,ri)=>{ (r.points||[]).forEach((pt,pi)=>{ const opt=document.createElement('option'); opt.value=`${ri}|${pi}`; opt.textContent=`(${pt.x}, ${pt.y}, ${ri}, ${pi}) — ${r.name||''}`; allPointsSelect.appendChild(opt); }); }); }
        function showSelectedFromOptionValue(value){ 
            if(!value){ 
                selectedPre.textContent='Selecciona una ruta para ver sus waypoints'; 
                return; 
            } 
            const [r,p]=value.split('|').map(n=>parseInt(n,10)); 
            if(isNaN(r)||isNaN(p)||!routes[r]||!routes[r].points[p]){ 
                selectedPre.textContent='Valor inválido'; 
                return; 
            } 
            const pt=routes[r].points[p]; 
            const routeName = routes[r].name || `Ruta ${r}`;
            selectedPre.textContent=`Ruta: ${routeName}\nWaypoint ${p + 1} de ${routes[r].points.length}\nCoordenadas: x=${pt.x}, y=${pt.y}\n\n(Preview - Selección de waypoints disponible en futuras versiones)`;
        }

        // Actualizar waypoints cuando cambia la ruta (solo para visualización)
        routeSelect.addEventListener('change', ()=>{ 
            const idx=parseInt(routeSelect.value,10); 
            populateWaypointsForRoute(idx); 
            if(waypointSelect.options.length>0){ 
                waypointSelect.selectedIndex=0; 
                // Mostrar información del primer waypoint como preview
                const firstWaypoint = waypointSelect.options[0];
                if (firstWaypoint && firstWaypoint.value) {
                    showSelectedFromOptionValue(firstWaypoint.value);
                } else {
                    selectedPre.textContent = `Ruta: ${routes[idx]?.name || 'N/A'}\nWaypoints: ${routes[idx]?.points?.length || 0}`;
                }
            }
        });
        
        // Los waypoints están deshabilitados, pero mantenemos los listeners para futuro
        waypointSelect.addEventListener('change', ()=> {
            if (!waypointSelect.disabled) {
                showSelectedFromOptionValue(waypointSelect.value);
            }
        });
        
        allPointsSelect.addEventListener('change', ()=>{ 
            if (!allPointsSelect.disabled) {
                showSelectedFromOptionValue(allPointsSelect.value); 
                const [r,p]=allPointsSelect.value.split('|').map(n=>parseInt(n,10)); 
                if(!isNaN(r)){ 
                    routeSelect.value = r; 
                    populateWaypointsForRoute(r); 
                    if(!isNaN(p) && waypointSelect.options[p]) waypointSelect.selectedIndex = p; 
                }
            }
        });

        async function startRoute(dir) {
            // Solo usar la ruta seleccionada, ignorar waypoints (están deshabilitados)
            const ridx = parseInt(routeSelect.value || '0', 10);
            if (isNaN(ridx) || ridx < 0) {
                statusPre.textContent = 'Error: Selecciona una ruta válida';
                return;
            }
            const delaySec = parseFloat(document.getElementById('delaySec').value || '0');
            const delayMs = Math.max(0, Math.round(delaySec*1000));
            const url = `${START_URL}?route=${ridx}&dir=${dir}&delay=${delayMs}`;
            try {
                statusPre.textContent = `Programando ruta ${routes[ridx]?.name || ridx} (${dir})...`;
                const r = await fetch(url);
                if (!r.ok) throw new Error('HTTP '+r.status);
                statusPre.textContent = `Ruta programada: ${routes[ridx]?.name || ridx} (${dir})`;
                // start polling status
            } catch (e) { statusPre.textContent = 'Error: '+e; }
        }

        async function stopRoute(){ try { const r = await fetch(STOP_URL); if(!r.ok) throw new Error('HTTP '+r.status); statusPre.textContent = 'Detenido'; } catch(e){ statusPre.textContent = 'Error stop: '+e; } }

        async function confirmStart(){ try { const r = await fetch(CONFIRM_URL); if(!r.ok) throw new Error('HTTP '+r.status); statusPre.textContent = 'Inicio confirmado'; } catch(e){ statusPre.textContent = 'Error confirm: '+e; } }

        document.getElementById('startIda').addEventListener('click', ()=> startRoute('ida'));
        document.getElementById('startRet').addEventListener('click', ()=> startRoute('retorno'));
        document.getElementById('stopRoute').addEventListener('click', ()=> stopRoute());
        confirmBtn.addEventListener('click', ()=> confirmStart());
        
        // Seguimiento de pared
        async function startWallFollow(side) {
            try {
                const r = await fetch(`${WALL_FOLLOW_URL}?side=${side}`);
                if (!r.ok) throw new Error('HTTP '+r.status);
                statusPre.textContent = `Seguimiento de pared ${side === 'left' ? 'izquierda' : 'derecha'} iniciado`;
            } catch(e) { statusPre.textContent = 'Error: '+e; }
        }
        
        async function stopWallFollow() {
            try {
                const r = await fetch(STOP_WALL_FOLLOW_URL);
                if (!r.ok) throw new Error('HTTP '+r.status);
                statusPre.textContent = 'Seguimiento de pared detenido';
            } catch(e) { statusPre.textContent = 'Error stop: '+e; }
        }
        
        document.getElementById('wallFollowLeft').addEventListener('click', ()=> startWallFollow('left'));
        document.getElementById('wallFollowRight').addEventListener('click', ()=> startWallFollow('right'));
        document.getElementById('stopWallFollow').addEventListener('click', ()=> stopWallFollow());

        // Poll route status and update UI
        async function pollStatus(){
            try {
                const r = await fetch(STATUS_URL, { cache: 'no-store' });
                if (!r.ok) throw new Error('Status HTTP '+r.status);
                const j = await r.json();
                // update state
                if (j.active) {
                            statusPre.textContent = `Active. state:${j.state} route:${j.routeIndex} pt:${j.currentPoint} awaitingConfirm:${j.awaitingConfirm} obstacle:${j.obstacleActive?1:0} obState:${j.obstacleState}`;
                    // disable start buttons while active
                    document.getElementById('startIda').disabled = true;
                    document.getElementById('startRet').disabled = true;
                    document.getElementById('stopRoute').disabled = false;
                } else {
                    statusPre.textContent = 'Inactivo';
                    document.getElementById('startIda').disabled = false;
                    document.getElementById('startRet').disabled = false;
                    document.getElementById('stopRoute').disabled = true;
                }
                // awaiting confirm
                if (j.awaitingConfirm) {
                    confirmBtn.disabled = false;
                } else {
                    confirmBtn.disabled = true;
                }
                // countdown
                if (j.remainingDelayMs && j.remainingDelayMs > 0) {
                    const s = Math.ceil(j.remainingDelayMs/1000);
                    countdownSpan.textContent = s + ' s';
                } else {
                    countdownSpan.textContent = '--';
                }
            } catch (e) {
                // ignore
            }
        }

        // initial load and start polling
        loadRoutes();
        setInterval(pollStatus, 800);
    </script>
    <script>
        // Evitar que se seleccione texto en el UI por long-press
        document.addEventListener('selectstart', function(e) {
            const t = e.target && e.target.tagName;
            if (t !== 'INPUT' && t !== 'TEXTAREA' && t !== 'SELECT') e.preventDefault();
        }, false);
        // Prevenir el menu contextual por long-press en botones (solo en botones)
        document.addEventListener('contextmenu', function(e){ if (e.target && e.target.tagName === 'BUTTON') e.preventDefault(); }, false);
    </script>
</body>
</html>
)rawliteral";

// Full dashboard page: user-provided layout completed with responsive CSS and JS
const char dashboardHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AMR Dashboard</title>
<style>
    /* Evitar selección y resaltado táctil en botones/áreas no editables */
    html, body {
        background:#111; color:#eee; font-family:Arial; margin:0; padding:8px;
        -webkit-user-select: none; /* Safari */
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none; /* Evita selección de texto por long-press */
        -webkit-touch-callout: none; /* iOS long-press */
        -webkit-tap-highlight-color: rgba(0,0,0,0); /* quitar highlight */
        touch-action: manipulation; /* mejora la interacción táctil */
    }
    /* Permitir selección en campos editables */
    select, input, textarea { -webkit-user-select: text; user-select: text; }
    h1 { color:#0f0; text-align:center; margin:8px 0; }
    #charts { display:flex; justify-content:space-around; flex-wrap:wrap; gap:10px; align-items:center; padding:6px; }
    /* Ensure chart panels center their content and have matching center lines */
    .chart { display:flex; flex-direction:column; align-items:center; justify-content:center; min-height:160px; }
    canvas { background:#222; border:1px solid #444; border-radius:6px; display:block; width:100%; height:auto; }
    .map { max-width:260px; }
    .compass { max-width:140px; }
    .ir { max-width:300px; }
    .controls { margin-top:14px; display:flex; justify-content:center; gap:8px; flex-wrap:wrap; }
    .controls button { background:#333; color:#0f0; border:none; padding:10px 16px; margin:4px; font-size:18px; border-radius:6px; }
    .controls button:hover { background:#0f0; color:#000; }
    /* Tests panel (compact buttons to trigger built-in tests) */
    .tests-grid { display:flex; gap:8px; justify-content:center; flex-wrap:wrap; margin-top:10px; }
    /* separation and header for controls section */
    .section-title { color:#7CFC00; text-align:center; margin:12px 0 6px; font-size:1.02rem; }
    .control-section { display:flex; justify-content:center; gap:18px; align-items:flex-start; margin-top:10px; padding:8px 6px; border-top:1px solid #222; flex-wrap:wrap; }
    /* D-pad (cruceta) layout: use previous button aesthetic but arranged in cruceta */
    .dpad { display:grid; grid-template-columns: 1fr 1fr 1fr; grid-auto-rows: auto; gap:10px; justify-content:center; align-items:center; width:100%; max-width:360px; margin:0 auto; }
    .dpad-btn { background:#333; color:#0f0; border:none; border-radius:8px; padding:10px 16px; font-size:16px; min-width:110px; min-height:44px; cursor:pointer; }
    .dpad-btn:hover { background:#7CFC00; color:#000; }
    .dpad-btn:active { transform: translateY(1px); }
    .dpad .spacer { background:transparent; box-shadow:none; border:none; }
    .dpad .stop { background:#444; color:#fff; font-weight:bold; }
    .route-btn { background:#222; color:#7CFC00; border:1px solid #333; padding:10px 14px; border-radius:8px; font-size:16px; cursor:pointer; }
    .test-btn { background:#222; color:#7CFC00; border:1px solid #333; padding:8px 10px; border-radius:6px; font-size:14px; cursor:pointer; min-width:110px; }
    .test-btn:hover { background:#7CFC00; color:#000; border-color:#7CFC00; }
    .status { text-align:center; margin-top:10px; color:#ccc }

    @media (max-width:520px) {
        .map { max-width:96%; }
        .compass { max-width:80%; }
        .ir { max-width:96%; }
        .controls button { padding:12px 18px; font-size:20px; }
        /* cruceta buttons use default sizing (no zoom) */
        .controls > div:first-child button { font-size:18px; padding:10px 16px; }
    }
</style>
</head>
<body>
    <h1> AMR CONTROL DASHBOARD</h1>
    <div id="charts">
        <div class="chart map"><canvas id="map"></canvas><small>Trayectoria Recorrida</small></div>
        <div class="chart compass"><canvas id="compass"></canvas><small>Brújula</small></div>
        <div class="chart ir"><canvas id="irChart"></canvas><small>Sensores IR</small></div>
    </div>

    <!-- Section title separating charts and controls -->
    <h2 class="section-title">Panel de Control</h2>

    <!-- Controls section: two-column layout for main controls + route button -->
    <div class="control-section">
        <div>
            <div class="dpad">
                <button class="dpad-btn spacer" aria-hidden="true"></button>
                <button id="btnW" class="dpad-btn up" onmousedown="startHold('W')" onmouseup="stopHold()" onmouseleave="stopHold()" ontouchstart="startHold('W')" ontouchend="stopHold()">↑ Adelante</button>
                <button class="dpad-btn spacer" aria-hidden="true"></button>

                <button class="dpad-btn left" onmousedown="startHold('Q')" onmouseup="stopHold()" onmouseleave="stopHold()" ontouchstart="startHold('Q')" ontouchend="stopHold()">← Izq</button>
                <button class="dpad-btn stop" onclick="sendCmd('X')">⏹ Stop</button>
                <button class="dpad-btn right" onmousedown="startHold('E')" onmouseup="stopHold()" onmouseleave="stopHold()" ontouchstart="startHold('E')" ontouchend="stopHold()">Der →</button>

                <button class="dpad-btn spacer" aria-hidden="true"></button>
                <button id="btnS" class="dpad-btn down" onmousedown="startHold('S')" onmouseup="stopHold()" onmouseleave="stopHold()" ontouchstart="startHold('S')" ontouchend="stopHold()">↓ Atrás</button>
                <button class="dpad-btn spacer" aria-hidden="true"></button>
            </div>
        </div>

        <div style="display:flex;align-items:center;justify-content:center;min-width:140px;">
            <button class="route-btn" onclick="location.href='/routes_ui'">Control Rutas</button>
        </div>
    </div>

    <!-- Tests block moved below controls, visually grouped and with a small header -->
    <div style="text-align:center;margin-top:8px;color:#ccc;font-size:0.95rem;">Pruebas Rápidas</div>
    <div id="tests" class="tests-grid" style="margin-top:8px;">
        <button class="test-btn" onclick="sendCmd('T')">Test Motores (T)</button>
        <button class="test-btn" onclick="sendCmd('V')">Avanzar 1 vuelta (V)</button>
        <button class="test-btn" onclick="sendCmd('I')">Inspección (I)</button>
        <button class="test-btn" onclick="sendCmd('R')">Reset Pos (R)</button>
    </div>

    <div class="status"><strong>Estado:</strong> <span id="statusText">-</span></div>

    <script>
    // Responsive canvas setup + lightweight renderer
    const mapCanvas = document.getElementById('map');
    const compassCanvas = document.getElementById('compass');
    const irCanvas = document.getElementById('irChart');
    const statusText = document.getElementById('statusText');
    let lastPositions = [];

    function setDPR(c) {
        const dpr = window.devicePixelRatio || 1;
        const rect = c.getBoundingClientRect();
        c.width = Math.max(1, Math.floor(rect.width * dpr));
        // keep a square-ish compass, map similar
        let cssH = rect.width; if (c===irCanvas) cssH = rect.width * 0.65; if (c===compassCanvas) cssH = rect.width;
        c.height = Math.max(1, Math.floor(cssH * dpr));
        const ctx = c.getContext('2d'); ctx.setTransform(dpr,0,0,dpr,0,0);
    }

    function resizeAll(){ [mapCanvas, compassCanvas, irCanvas].forEach(setDPR); }

    function drawMap(x,y){ const ctx = mapCanvas.getContext('2d'); const w = mapCanvas.clientWidth, h = mapCanvas.clientHeight; ctx.clearRect(0,0,mapCanvas.width,mapCanvas.height); ctx.fillStyle='#111'; ctx.fillRect(0,0,w,h); ctx.strokeStyle='#333'; const step = Math.max(16, Math.round(Math.min(w,h)/10)); for(let i=0;i<=w;i+=step){ ctx.beginPath(); ctx.moveTo(i,0); ctx.lineTo(i,h); ctx.stroke(); } for(let i=0;i<=h;i+=step){ ctx.beginPath(); ctx.moveTo(0,i); ctx.lineTo(w,i); ctx.stroke(); } lastPositions.push({x,y}); if(lastPositions.length>80) lastPositions.shift(); ctx.strokeStyle='lime'; ctx.beginPath(); lastPositions.forEach((p,i)=>{ const sx=w/2 + p.x, sy=h/2 - p.y; if(i==0) ctx.moveTo(sx,sy); else ctx.lineTo(sx,sy); }); ctx.stroke(); ctx.fillStyle='red'; ctx.beginPath(); ctx.arc(w/2 + x, h/2 - y, Math.max(3, Math.min(8, w*0.02)), 0, 2*Math.PI); ctx.fill(); }

    function drawCompass(th){ const ctx = compassCanvas.getContext('2d'); const w = compassCanvas.clientWidth, h = compassCanvas.clientHeight; ctx.clearRect(0,0,compassCanvas.width,compassCanvas.height); const cx=w/2, cy=h/2, r=Math.min(w,h)/2-6; ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.strokeStyle='#555'; ctx.stroke(); ctx.save(); ctx.translate(cx,cy); ctx.rotate(th*Math.PI/180); ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(0,-r*0.8); ctx.strokeStyle='red'; ctx.stroke(); ctx.restore(); ctx.fillStyle='#eee'; ctx.fillText(Math.round(th)+'°',cx-10,cy+Math.round(r*0.5)); }

    function drawIR(values){
        const ctx = irCanvas.getContext('2d');
        const w = irCanvas.clientWidth, h = irCanvas.clientHeight;
        ctx.clearRect(0,0,irCanvas.width,irCanvas.height);
        const padding = 8, gap = 8;
        const bottomLabelArea = 18; // reserve space for sensor labels under bars
        const barAreaW = w - padding*2;
        const barWidth = (barAreaW - gap*(values.length-1)) / values.length;
        // Display scale in centimeters (user requested max 100 cm for margin)
        const maxValCm = 100.0;
        // adaptive font sizes
        const valFontSize = Math.max(10, Math.round(h * 0.08));
        const nameFontSize = Math.max(10, Math.round(h * 0.06));
        ctx.textAlign = 'center';
        for (let i = 0; i < values.length; i++) {
            const raw = Number(values[i]);
            const maxBarHeight = h - padding - bottomLabelArea;

            // Convert ADC/raw value to centimeters using the calibrated formula
            // distancia_cm = 17569.7 * adc^-1.2062
            let dist = 0.0;
            if (raw <= 0) {
                dist = maxValCm; // if invalid reading, show as far
            } else {
                dist = 17569.7 * Math.pow(raw, -1.2062);
            }
            if (!isFinite(dist)) dist = maxValCm;
            if (dist < 2.0) dist = 2.0;
            if (dist > maxValCm) dist = maxValCm;

            // Height proportional to distance (0..maxValCm mapped to 0..maxBarHeight)
            const barH = Math.max(2, (dist / maxValCm) * maxBarHeight);
            const x = padding + i * (barWidth + gap);
            const y = h - bottomLabelArea - barH;

            // color: red if very close (e.g., < 20 cm), otherwise lime
            ctx.fillStyle = dist < 20.0 ? 'red' : 'lime';
            ctx.fillRect(x, y, barWidth, barH);

            // draw numeric value above the bar (distance in cm with 1 decimal)
            ctx.fillStyle = '#fff';
            ctx.font = valFontSize + 'px Arial';
            const valueX = x + barWidth / 2;
            const valueY = Math.max(12, y - 6);
            ctx.fillText(String(dist.toFixed(1)) + ' cm', valueX, valueY);

            // draw sensor short name centered below the bar
            ctx.fillStyle = '#eee';
            ctx.font = nameFontSize + 'px Arial';
            const names = ['L','FL','B','FR','R'];
            const name = (names[i] || 'S') + '';
            const nameY = h - 4;
            ctx.fillText(name, x + barWidth / 2, nameY);
        }
        // reset textAlign to default left for other drawings
        ctx.textAlign = 'left';
    }

    function startHold(cmd){ if(window._holdInterval) clearInterval(window._holdInterval); fetch('/cmd?c='+cmd).catch(()=>{}); window._holdInterval = setInterval(()=>fetch('/cmd?c='+cmd).catch(()=>{}),200); }
    function stopHold(){ if(window._holdInterval) { clearInterval(window._holdInterval); window._holdInterval = null; } fetch('/cmd?c=X').catch(()=>{}); }
    function sendCmd(cmd){ fetch('/cmd?c='+cmd).catch(()=>{}); }

    function updateLoop(){ fetch('/data').then(r=>r.json()).then(j=>{ drawMap(j.x,j.y); drawCompass(j.th); drawIR(j.ir); statusText.textContent = `x:${j.x.toFixed(2)} y:${j.y.toFixed(2)} th:${j.th.toFixed(0)}°`; }).catch(()=>{ statusText.textContent='No telemetría'; }); setTimeout(updateLoop,500); }

    window.addEventListener('load', ()=>{ resizeAll(); window.addEventListener('resize', resizeAll); updateLoop(); });
    window.addEventListener('orientationchange', ()=> setTimeout(resizeAll,250));
    </script>
    <script>
        // Evitar selección por long-press en áreas no editables, pero permitir en inputs/selects
        document.addEventListener('selectstart', function(e) {
            const t = e.target && e.target.tagName;
            if (t !== 'INPUT' && t !== 'TEXTAREA' && t !== 'SELECT') e.preventDefault();
        }, false);
        // Evitar el menú contextual por long-press en botones (útil en móviles)
        document.addEventListener('contextmenu', function(e){ if (e.target && e.target.tagName === 'BUTTON') e.preventDefault(); }, false);
        // Quitar highlight táctil adicional en todos los botones (estilo redundante)
        Array.from(document.querySelectorAll('button')).forEach(b=>{ b.style.webkitTapHighlightColor = 'transparent'; });
    </script>
</body>
</html>
)rawliteral";
// ========================================
// ESTRUCTURA DEL CÓDIGO
// ========================================
// Este sketch está organizado en secciones claras:
// 1. Librerías e includes
// 2. Configuración WiFi y HTML embebido (PROGMEM)
// 3. Definición de rutas y estructuras de datos
// 4. Instancias globales (motors, encoders, odometry)
// 5. Máquina de estados de ejecución de rutas
// 6. Variables de control y logging
// 7. Sistema de sensores IR
// 8. Setup e inicialización
// 9. Loop principal con máquina de estados
// 10. Procesamiento de comandos serie
// 11. Sistema de giros automáticos
// 12. Servidor WiFi y manejo HTTP
//
// Tests disponibles: 'T' (motores), 'V' (calibración), 'I' (inspección)

// ========================================
//         INSTANCIAS GLOBALES
// ========================================
MotorDriver motors;
Encoder encoders;
Odometry odometry(&encoders);

// ----------------------
// MÁQUINA DE ESTADOS DE EJECUCIÓN DE RUTAS
// ----------------------
// Estados principales de la máquina de estados:
// - ROUTE_IDLE: Inactivo, sin ruta en ejecución
// - ROUTE_WAITING: Esperando delay inicial o confirmación del operador
// - ROUTE_TURNING: Realizando giro hacia el siguiente waypoint
// - ROUTE_MOVING: Moviéndose hacia el waypoint objetivo (con evasión de obstáculos)
// - ROUTE_DONE: Ruta completada
//
// Sistema de evasión de obstáculos integrado:
// - Estados: 0=idle, 1=TURN (90°), 2=FORWARD (avance lateral), 
//            3=TURNBACK (-90°), 4=CROSS_FORWARD (cruzar), 5=DONE
// - Confirmación de 2 segundos antes de iniciar evasión
// - Selección automática del lado con más espacio libre
enum RouteState { ROUTE_IDLE=0, ROUTE_WAITING, ROUTE_TURNING, ROUTE_MOVING, ROUTE_DONE };
struct RouteExecution {
    bool active = false;
    int routeIndex = 0;
    int direction = 1; // 1 = ida, -1 = retorno
    unsigned long requestMillis = 0;
    unsigned long delayMs = 0;
    int currentPoint = 0; // 0..(n-1)
    RouteState state = ROUTE_IDLE;
    bool awaitingConfirm = false;
    bool waitingForReturnConfirm = false; // waiting confirmation after finishing ida
    bool returnModeActive = false; // true when executing return leg
    bool postFinishTurn = false; // indicates we've started the final 180° turn
    // Obstacle avoidance state
    bool obstacleActive = false; // flag indicating avoidance in progress
    int obstacleSide = 0; // +1 = left, -1 = right (chosen side to go around)
    int obstacleState = 0; // 0=idle,1=TURN,2=FORWARD,3=TURNBACK,4=CROSS_FORWARD,5=DONE
    long obstacleMoveStartLeft = 0;
    long obstacleMoveStartRight = 0;
    long obstacleMoveTargetPulses = 0;
    long obstacleMoveMaxPulses = 0; // safety cap if sensor never clears
    int obstacleProbePin = -1; // pin to sample for clearance (opposite sensor)
    bool obstacleWaitActive = false; // waiting a short time to confirm obstacle isn't transient
    unsigned long obstacleWaitStartMillis = 0;
    // movement bookkeeping
    long moveStartLeft = 0;
    long moveStartRight = 0;
    long moveTargetPulses = 0;
    float targetX = 0.0f;
    float targetY = 0.0f;
} routeExec;

// ========================================
//     SEGUIMIENTO DE PARED
// ========================================
// Estados: 0=idle, 1=following, 2=turning, 3=stopped (all walls detected)
struct WallFollow {
    bool active = false;
    int side = 0; // +1 = izquierda, -1 = derecha
    int state = 0; // 0=idle, 1=following, 2=turning, 3=stopped
    unsigned long allWallsDetectedStart = 0; // timestamp cuando se detectaron todas las paredes
    bool routeResumePending = false; // flag para reanudar ruta después de seguimiento
    int resumeAction = 0; // 0=none, 1=turn, 2=forward
} wallFollow;

const float WALL_FOLLOW_THRESHOLD_CM = 30.0f; // distancia para considerar pared detectada
const unsigned long ALL_WALLS_TIMEOUT_MS = 10000; // 10 segundos para finalizar si todas las paredes detectadas
const int WALL_FOLLOW_SPEED = 100; // velocidad base para seguimiento de pared
const int WALL_FOLLOW_TURN_SPEED = 80; // velocidad para giros durante seguimiento

// Helper: stop/abort execution
void stopRouteExecution() {
    if (!routeExec.active) return;
    routeExec.active = false;
    routeExec.state = ROUTE_IDLE;
    motors.stop();
    Serial.println(F("Route execution aborted"));
}

// Helper: stop wall following
void stopWallFollowing() {
    if (!wallFollow.active) return;
    wallFollow.active = false;
    wallFollow.state = 0;
    motors.stop();
    Serial.println(F("Wall following stopped"));
}

// Helper: start wall following
void startWallFollowing(int side) {
    if (wallFollow.active) stopWallFollowing();
    // Detener cualquier ruta activa antes de iniciar seguimiento de pared
    if (routeExec.active) {
        stopRouteExecution();
        Serial.println(F("Ruta detenida para iniciar seguimiento de pared."));
    }
    wallFollow.active = true;
    wallFollow.side = side;
    wallFollow.state = 1; // following
    wallFollow.allWallsDetectedStart = 0;
    wallFollow.routeResumePending = false;
    wallFollow.resumeAction = 0;
    Serial.print(F("Wall following started: "));
    Serial.println(side == 1 ? F("LEFT") : F("RIGHT"));
}

// helper to normalize angle to [-180,180]
float normalizeAngle(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// begin the next waypoint (calculate turn and start it)
// NOTA: Omite el primer waypoint en IDA (siempre está en 0,0) y el último en RETORNO (ya está ahí)
void beginNextWaypoint() {
    int idx = routeExec.routeIndex;
    if (idx < 0 || idx >= ROUTE_COUNT) { stopRouteExecution(); return; }
    int count = routesCounts[idx];
    
    // Si la ruta tiene solo 1 waypoint, no hay nada que visitar (se omite en ambos sentidos)
    if (count <= 1) {
        Serial.println(F("Ruta tiene solo 1 waypoint. No hay waypoints a visitar (se omite)."));
        if (!routeExec.returnModeActive) {
            routeExec.postFinishTurn = true;
            startAutoTurn(180);
            routeExec.state = ROUTE_TURNING;
        } else {
            routeExec.active = false;
            routeExec.state = ROUTE_DONE;
            Serial.println(F("Ruta completada (sin waypoints a visitar)."));
        }
        return;
    }
    
    // Calcular el número efectivo de waypoints a visitar (omitiendo primero/último según dirección)
    int effectiveCount = count - 1; // Restamos 1 porque omitimos un waypoint
    
    // Verificar si hemos completado todos los waypoints efectivos
    if (routeExec.currentPoint >= effectiveCount) {
        // reached end of route
        if (!routeExec.returnModeActive) {
            // IDA finished: perform 180° turn to face return direction, then wait for confirmation
            Serial.println(F("Ruta de IDA completada. Girando 180° para preparar retorno..."));
            routeExec.postFinishTurn = true;
            startAutoTurn(180);
            routeExec.state = ROUTE_TURNING;
            // El giro se completará en handleAutoTurn y luego esperará confirmación
            return;
        } else {
            // RETORNO finished: perform final 180° then finish route
            Serial.println(F("Ruta de RETORNO completada. Girando 180° para finalizar..."));
            routeExec.postFinishTurn = true;
            startAutoTurn(180);
            routeExec.state = ROUTE_TURNING;
            Serial.println(F("Retorno finalizado. Ruta completa terminada."));
            return;
        }
    }

    // Calcular índice del waypoint a visitar (omitiendo primero en IDA, último en RETORNO)
    int pIndex;
    if (routeExec.direction == 1) {
        // IDA: omitir waypoint 0 (índice 0), empezar desde waypoint 1 (índice 1)
        // currentPoint 0 → waypoint índice 1
        // currentPoint 1 → waypoint índice 2
        // etc.
        pIndex = routeExec.currentPoint + 1;
    } else {
        // RETORNO: omitir último waypoint (índice count-1), visitar todos los demás en orden inverso
        // Ejemplo con 4 waypoints [0,0], [30,0], [30,30], [0,30]:
        // - Omite: [0,30] (índice 3, último)
        // - Visita: [30,30] (índice 2) → [30,0] (índice 1) → [0,0] (índice 0)
        // currentPoint 0 → waypoint índice count-2 (penúltimo)
        // currentPoint 1 → waypoint índice count-3
        // currentPoint 2 → waypoint índice count-4 = 0 (primero, [0,0])
        // etc.
        pIndex = count - 2 - routeExec.currentPoint;
    }
    
    // Validar que el índice esté en rango
    if (pIndex < 0 || pIndex >= count) {
        Serial.print(F("Error: índice de waypoint fuera de rango: "));
        Serial.println(pIndex);
        stopRouteExecution();
        return;
    }
    
    routeExec.targetX = routesPoints[idx][pIndex].x;
    routeExec.targetY = routesPoints[idx][pIndex].y;

    // compute heading and angle delta
    float curX = odometry.getX();
    float curY = odometry.getY();
    float curTh = odometry.getThetaDegrees();
    float dx = routeExec.targetX - curX;
    float dy = routeExec.targetY - curY;
    float desired = atan2f(dy, dx) * 180.0f / PI;
    float delta = normalizeAngle(desired - curTh);

    // Log información del waypoint actual
    int effectiveCount = (count > 1) ? count - 1 : count;
    Serial.print(F("Waypoint "));
    Serial.print(routeExec.currentPoint + 1);
    Serial.print(F("/"));
    Serial.print(effectiveCount);
    Serial.print(F(" de ruta "));
    Serial.print(routeNames[idx]);
    Serial.print(F(" (índice "));
    Serial.print(pIndex);
    Serial.print(F("): ("));
    Serial.print(routeExec.targetX);
    Serial.print(F(","));
    Serial.print(routeExec.targetY);
    Serial.print(F(") - Giro: "));
    Serial.print(delta, 1);
    Serial.println(F("°"));

    // start turn using existing routine
    startAutoTurn(delta);
    routeExec.state = ROUTE_TURNING;
}

// schedule route execution
bool startRouteExecution(int routeIndex, bool retorno, unsigned long delayMilliseconds) {
    if (routeExec.active) return false;
    if (routeIndex < 0 || routeIndex >= ROUTE_COUNT) return false;
    routeExec.active = true;
    routeExec.routeIndex = routeIndex;
    routeExec.direction = retorno ? -1 : 1;
    routeExec.requestMillis = millis();
    routeExec.delayMs = delayMilliseconds;
    routeExec.currentPoint = 0; // Empezará desde 0, pero beginNextWaypoint omitirá el primer/último waypoint
    routeExec.state = (delayMilliseconds > 0) ? ROUTE_WAITING : ROUTE_IDLE;
    // require operator confirmation by default; will auto-start when delay expires
    routeExec.awaitingConfirm = true;

    int totalWaypoints = routesCounts[routeIndex];
    int effectiveWaypoints = (totalWaypoints > 1) ? totalWaypoints - 1 : totalWaypoints;
    
    Serial.print(F("Ruta programada: "));
    Serial.print(routeNames[routeIndex]);
    Serial.print(F(" - Modo: "));
    Serial.print(retorno ? F("RETORNO") : F("IDA"));
    Serial.print(F(" - Waypoints a visitar: "));
    Serial.print(effectiveWaypoints);
    Serial.print(F(" de "));
    Serial.print(totalWaypoints);
    Serial.print(F(" totales"));
    if (retorno) {
        Serial.print(F(" (omitiendo último waypoint)"));
    } else {
        Serial.print(F(" (omitiendo primer waypoint 0,0)"));
    }
    Serial.print(F(" - Delay: "));
    Serial.print(delayMilliseconds);
    Serial.println(F(" ms"));

    if (delayMilliseconds == 0) beginNextWaypoint();
    return true;
}

// --------------------------------------------------------------------------------
// INSTANCIAS GLOBALES - Descripción:
// 
// `motors`: Controlador de motores BTS7960
//   - Funciones de movimiento: moveForward(), moveBackward(), turnLeft(), turnRight()
//   - Control individual: setLeftMotor(), setRightMotor(), setBothMotors()
//   - Sistema PID de velocidad opcional con interpolación de encoders
//   - Factor de compensación 1.1 para motor derecho (corrige curva)
// 
// `encoders`: Gestión de encoders E386G5
//   - Contadores de pulsos por rueda (left/right)
//   - Calibración de pulsos por revolución (por defecto ~3418 PPR)
//   - Utilidades: getPulsesPerRevolution(), readLeft(), readRight()
// 
// `odometry`: Cálculo de posición y orientación
//   - Actualización continua basada en lecturas de encoders
//   - Posición (x, y) en centímetros
//   - Orientación (theta) en radianes/grados
//   - Funciones: getX(), getY(), getThetaDegrees(), resetPosition()
// --------------------------------------------------------------------------------

// ========================================
//         VARIABLES DE CONTROL
// ========================================
unsigned long lastPositionUpdate = 0;
const unsigned int POSITION_UPDATE_INTERVAL = 50;   // Actualizar cada 50ms

// Variables para giros automáticos
bool turningInProgress = false;
float targetAngle = 0;
unsigned long turnStartTime = 0;
const unsigned int MAX_TURN_TIME = 4000; // 4 segundos máximo para girar
// Variables para control de giros por encoder
long turnStartLeft0 = 0;
long turnStartRight0 = 0;
long turnTargetPulses = 0;

// Variables para imprimir tics mientras se avanza con 'W'
bool printTicksWhileMoving = false;
unsigned long lastTickPrintMillis = 0;
const unsigned int TICK_PRINT_INTERVAL = 100; // ms
long tickPrintLeft0 = 0;
long tickPrintRight0 = 0;


// ======== MUESTREO CONTINUO IR (caso 'K') ========
struct IRSampler {
    int samplesPerRead;      // número de lecturas por sensor por ciclo
    unsigned long intervalMs; // intervalo entre lecturas
    unsigned long lastMillis; // última vez que se hizo la lectura
    bool running;
};

IRSampler* irSampler = nullptr;

// Inspección continua (comando 'I')
bool inspectionActive = false;
unsigned long inspectionLastMillis = 0;
const unsigned long INSPECTION_INTERVAL_MS = 250; // intervalo para inspección continua

// Simple circular log buffer to capture important messages (also mirrored to Serial)
const int LOG_LINES = 64;
String logBuffer[LOG_LINES];
int logIndex = 0; // next insertion index

void addLogLine(const String &s) {
    logBuffer[logIndex] = s;
    logIndex++;
    if (logIndex >= LOG_LINES) logIndex = 0;
}

// Helpers that print to Serial and also store to buffer (overloads for flash strings)
void logPrint(const __FlashStringHelper *fs) { String tmp = String(fs); Serial.print(fs); addLogLine(tmp); }
void logPrintln(const __FlashStringHelper *fs) { String tmp = String(fs); Serial.println(fs); addLogLine(tmp); }
void logPrint(const String &s) { Serial.print(s); addLogLine(s); }
void logPrintln(const String &s) { Serial.println(s); addLogLine(s); }

String getLogsText() {
    String out = "";
    // start from the oldest message
    for (int i = 0; i < LOG_LINES; ++i) {
        int idx = (logIndex + i) % LOG_LINES;
        if (logBuffer[idx].length() > 0) {
            out += logBuffer[idx];
            out += '\n';
        }
    }
    return out;
}

// -------------------------------------------------------------------------------
// VARIABLES DE CONTROL - Descripción:
// 
// Odometría:
// - `POSITION_UPDATE_INTERVAL` (50ms): Frecuencia de actualización de odometría.
//   Valores más cortos aumentan precisión pero consumen más CPU.
// 
// Giros automáticos:
// - `turningInProgress`: Flag que indica si hay un giro en progreso
// - `turnTargetPulses`: Pulsos objetivo calculados para el giro
// - `turnStartLeft0/Right0`: Valores iniciales de encoders al iniciar giro
// - `MAX_TURN_TIME` (4000ms): Timeout de seguridad para giros
// 
// Utilidades:
// - `printTicksWhileMoving`: Activa impresión periódica de tics con comando 'W'
// - `inspectionActive`: Modo de inspección continua (comando 'I')
// - `irSampler`: Muestreo continuo de sensores IR (comando 'K', no implementado)
// 
// Logging:
// - `logBuffer[]`: Buffer circular de 64 líneas para almacenar logs
// - Funciones logPrint/logPrintln: Imprimen a Serial y almacenan en buffer
// -------------------------------------------------------------------------------

// ========================================
//       SENSORES INFRARROJOS (ANALÓGICOS)
// ========================================
// Sistema de 5 sensores IR analógicos para detección de obstáculos y navegación.
// 
// Funcionamiento:
// - Lectura analógica 0-1023 (ADC de 10 bits)
// - Promediado de múltiples muestras para reducir ruido (IR_NUM_SAMPLES = 6)
// - Conversión a distancia en cm usando fórmula calibrada: 
//   distancia_cm = 17569.7 * adc^-1.2062
// - Detección booleana basada en umbral configurable (IR_THRESHOLD = 150)
// 
// Uso en evasión de obstáculos:
// - Sensores frontales (FL, FR) para detección inicial
// - Sensores laterales (L, R) para elegir lado de evasión
// - Sensor opuesto al giro como "sonda" durante evasión lateral
// - Confirmación de 2 segundos (OBSTACLE_DETECTION_DELAY_MS) antes de evadir

// Mapeo de pines (ajustado: swap L<->R)
// LEFT_SIDE (lateral izquierdo)  -> A5 (antes A0)
// FRONT_LEFT (frontal izquierdo) -> A4 (antes A1)
// BACK_CENTER (trasero central)  -> A2 (sin cambios)
// FRONT_RIGHT (frontal derecho)  -> A1 (antes A4)
// RIGHT_SIDE (lateral derecho)   -> A0 (antes A5)
const int IR_LEFT_SIDE_PIN   = A5; // Lateral izquierdo (swapped)
const int IR_FRONT_LEFT_PIN  = A4; // Frontal izquierdo (swapped)
const int IR_BACK_CENTER_PIN = A2; // Trasero central
const int IR_FRONT_RIGHT_PIN = A1; // Frontal derecho (swapped)
const int IR_RIGHT_SIDE_PIN  = A0; // Lateral derecho (swapped)

// Parámetros de lectura
const int IR_NUM_SAMPLES = 6;      // número de lecturas para promediar
int IR_THRESHOLD = 150;            // umbral por defecto (0-255). Ajustar por calibración
// Obstacle avoidance parameters
const float OBSTACLE_THRESHOLD_CM = 30.0f; // if front distance below this, consider obstacle
const float AVOID_STEP_CM = 30.0f; // how far to advance when circumventing (per step)
const float AVOID_CLEAR_MARGIN_CM = 8.0f; // extra margin to consider object cleared
const float AVOID_MAX_STEP_CM = 200.0f; // maximum allowed advance during avoidance (safety)
// When an obstacle is detected, wait this many ms before starting avoidance
const unsigned long OBSTACLE_DETECTION_DELAY_MS = 2000; // 2 seconds

// Estructura para devolver lecturas
struct IRSensors {
    int rawLeft;
    int rawFrontLeft;
    int rawBack;
    int rawFrontRight;
    int rawRight;
    bool left;
    bool frontLeft;
    bool back;
    bool frontRight;
    bool right;
};

// Inicializar pines analógicos (no es necesario pinMode para analogRead,
// pero dejamos una función para futura configuración y documentación)
void setupIRSensors() {
    // No es necesario configurar A0..A5 con pinMode para analogRead en Arduino,
    // pero si los sensores necesitan alimentación o referencias externas, eso se
    // debe hacer en el cableado físico.
    // Añadir una pequeña espera para estabilizar sensores si es necesario
    delay(20);
}

// Leer un pin IR con promedio de N muestras
int readIRRaw(int pin) {
    long acc = 0;
    for (int i = 0; i < IR_NUM_SAMPLES; ++i) {
        acc += analogRead(pin);
        delay(4);
    }
    int avg = (int)(acc / IR_NUM_SAMPLES);
    return avg;
}

IRSensors readIRSensors() {
    IRSensors s;
    s.rawLeft = readIRRaw(IR_LEFT_SIDE_PIN);
    s.rawFrontLeft = readIRRaw(IR_FRONT_LEFT_PIN);
    s.rawBack = readIRRaw(IR_BACK_CENTER_PIN);
    s.rawFrontRight = readIRRaw(IR_FRONT_RIGHT_PIN);
    s.rawRight = readIRRaw(IR_RIGHT_SIDE_PIN);

    // Detección booleana (suponer HIGH -> mayor valor -> detectado)
    s.left = s.rawLeft >= IR_THRESHOLD;
    s.frontLeft = s.rawFrontLeft >= IR_THRESHOLD;
    s.back = s.rawBack >= IR_THRESHOLD;
    s.frontRight = s.rawFrontRight >= IR_THRESHOLD;
    s.right = s.rawRight >= IR_THRESHOLD;
    return s;
}

// Convertir lectura raw del sensor IR (0-1023) a distancia aproximada en cm.
// Conversión calibrada (modelo empírico) de ADC -> cm.
// Esta fórmula proviene de la calibración específica mostrada:
// distancia_cm = 17569.7 * pow(adc, -1.2062)
// Usamos esa relación porque da mejor precisión para el sensor empleado.
float irRawToCentimeters(int raw) {
    if (raw <= 0) return 1000.0f;
    float adc = (float)raw;
    float d = 17569.7f * powf(adc, -1.2062f);
    if (d < 2.0f) d = 2.0f;
    if (d > 1000.0f) d = 1000.0f;
    return d;
}

// Medir distancia en un pin con 'n' muestras y opcionalmente devolver el tiempo
// en milisegundos mediante el parámetro de salida 'outTimeMs' (puede ser NULL).
float distanciaSamples(int pin, int n, unsigned long *outTimeMs = NULL) {
    unsigned long t0 = millis();
    long suma = 0;
    for (int i = 0; i < n; ++i) {
        suma += analogRead(pin);
        // pequeña espera entre muestras para estabilizar si se desea
        delay(5);
    }
    float adc = (float)suma / (float)n;
    float dist = 17569.7f * powf(adc, -1.2062f);
    if (dist < 2.0f) dist = 2.0f;
    if (dist > 1000.0f) dist = 1000.0f;
    if (outTimeMs) *outTimeMs = millis() - t0;
    return dist;
}

// Enviar telemetría simple por Serial
void sendIRTelemetry(const IRSensors &s) {
    Serial.print(F("IR: "));
    Serial.print(F("L:")); Serial.print(s.left ? 1 : 0); Serial.print(F("(")); Serial.print(s.rawLeft); Serial.print(F(") "));
    Serial.print(F("FL:")); Serial.print(s.frontLeft ? 1 : 0); Serial.print(F("(")); Serial.print(s.rawFrontLeft); Serial.print(F(") "));
    Serial.print(F("B:")); Serial.print(s.back ? 1 : 0); Serial.print(F("(")); Serial.print(s.rawBack); Serial.print(F(") "));
    Serial.print(F("FR:")); Serial.print(s.frontRight ? 1 : 0); Serial.print(F("(")); Serial.print(s.rawFrontRight); Serial.print(F(") "));
    Serial.print(F("R:")); Serial.print(s.right ? 1 : 0); Serial.print(F("(")); Serial.print(s.rawRight); Serial.println(F(")"));
}


// ========================================
//              SETUP
// ========================================
// Forward declarations used because large PROGMEM literals can confuse
// the automatic prototype generator. Keep prototypes for functions
// referenced before their definitions.
void setupWiFi();
void handleClient(WiFiClient client);
void handleWiFiServer();
void processCommand(char cmd);
void handleAutoTurn();
void startAutoTurn(float angleDelta);
void showHelp();

void setup() {
    Serial.begin(115200);
    
    // Banner de inicio
    Serial.println(F("=== AMR SYSTEM ==="));
    Serial.println(F("Enc:3850PPR Ruedas:15.5cm"));
    Serial.println(F("W/S:Adelante/Atras"));
    Serial.println(F("A/D:Izq/Der X:Stop"));
    Serial.println(F("P:Pos R:Reset T:Test"));
    
    // Inicializar hardware
    motors.init();
    encoders.init();
    odometry.init(0.0, 0.0, 0.0);
    // Inicializar sensores IR analógicos (A0..A5)
    setupIRSensors();
    
    Serial.println(F("LISTO! Pos:(0,0)"));

    // Iniciar Access Point y servidor web (UNO R4 WiFi)
    setupWiFi();
}

// ========================================
//                SETUP NOTES
// ========================================
// Secuencia de inicialización:
// 1. Serial a 115200 baudios (consola y control por USB)
// 2. Banner de inicio con información del sistema
// 3. Inicialización de hardware:
//    - MotorDriver: Configura pines PWM y detiene motores
//    - Encoder: Inicializa contadores y configura interrupciones
//    - Odometry: Inicializa posición en (0, 0, 0)
//    - Sensores IR: Pequeño delay para estabilización
// 4. WiFi Access Point: Crea red "AMR_Robot_AP" y servidor HTTP en puerto 80
//
// Nota: El WiFi requiere Arduino UNO R4 WiFi o placa compatible con WiFiS3.


// ========================================
//            LOOP PRINCIPAL
// ========================================
// El loop principal ejecuta las siguientes tareas en cada iteración:
// 1. Actualización de odometría (cada 50ms)
// 2. Procesamiento de comandos serie (si hay datos disponibles)
// 3. Manejo de giros automáticos (verifica finalización)
// 4. Máquina de estados de ejecución de rutas (si hay ruta activa)
//    - ROUTE_WAITING: Espera delay o confirmación
//    - ROUTE_TURNING: Espera finalización de giro
//    - ROUTE_MOVING: Movimiento hacia waypoint con evasión de obstáculos
// 5. Impresión de tics mientras avanza (comando 'W')
// 6. Inspección continua (comando 'I')
// 7. Manejo de servidor WiFi (dashboard y API HTTP)
//
// Nota: delay(5) al final proporciona estabilidad y evita saturación de CPU
void loop() {
    // Actualizar odometría frecuentemente
    if (millis() - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
        odometry.update();
        lastPositionUpdate = millis();
    }
    
    // Procesar comandos serie
    if (Serial.available()) {
        char command = Serial.read();
        processCommand(command);
        
        // Limpiar buffer serie
        while (Serial.available()) {
            Serial.read();
        }
    }
    
    // Manejar giros automáticos
    handleAutoTurn();

    // Route execution state machine
    if (routeExec.active) {
        unsigned long now = millis();
        if (routeExec.state == ROUTE_WAITING) {
            if (now - routeExec.requestMillis >= routeExec.delayMs) {
                // start now
                beginNextWaypoint();
            }
        } else if (routeExec.state == ROUTE_TURNING) {
            // wait for turningInProgress to finish (handled by handleAutoTurn)
            if (!turningInProgress) {
                // start moving towards target
                float dx = routeExec.targetX - odometry.getX();
                float dy = routeExec.targetY - odometry.getY();
                float dist = sqrtf(dx*dx + dy*dy);
                // compute pulses required
                float pulsesF = (dist / (float)WHEEL_CIRCUMFERENCE_CM) * (float)encoders.getPulsesPerRevolution();
                routeExec.moveTargetPulses = (long)(pulsesF + 0.5f);
                routeExec.moveStartLeft = encoders.readLeft();
                routeExec.moveStartRight = encoders.readRight();
                if (routeExec.moveTargetPulses <= 0) {
                    // Ya está en el waypoint, avanzar al siguiente
                    Serial.print(F("Waypoint alcanzado. Avanzando al siguiente..."));
                    routeExec.currentPoint++;
                    beginNextWaypoint();
                } else {
                    motors.moveForward();
                    routeExec.state = ROUTE_MOVING;
                    Serial.print(F("Avanzando hacia waypoint: "));
                    Serial.print(routeExec.moveTargetPulses);
                    Serial.println(F(" pulsos"));
                }
            }
        } else if (routeExec.state == ROUTE_MOVING) {
                // First, check for obstacles using IR sensors (sampled in cm)
                float dFL = distanciaSamples(IR_FRONT_LEFT_PIN, 3, NULL);
                float dFR = distanciaSamples(IR_FRONT_RIGHT_PIN, 3, NULL);
                float frontMin = min(dFL, dFR);
                // If we detect a front obstacle we first wait briefly to avoid reacting
                // to transient objects (e.g., a person or a passing object). Only after
                // OBSTACLE_DETECTION_DELAY_MS will we commit to avoidance.
                if (!routeExec.obstacleActive && !routeExec.obstacleWaitActive && frontMin <= OBSTACLE_THRESHOLD_CM) {
                    routeExec.obstacleWaitActive = true;
                    routeExec.obstacleWaitStartMillis = millis();
                    Serial.print(F("Obstacle seen briefly (waiting to confirm). frontMin=")); Serial.println(frontMin);
                } else if (!routeExec.obstacleActive && routeExec.obstacleWaitActive) {
                    // check if wait period expired
                    if (millis() - routeExec.obstacleWaitStartMillis >= OBSTACLE_DETECTION_DELAY_MS) {
                        // re-sample front to confirm
                        float dFL2 = distanciaSamples(IR_FRONT_LEFT_PIN, 3, NULL);
                        float dFR2 = distanciaSamples(IR_FRONT_RIGHT_PIN, 3, NULL);
                        float frontMin2 = min(dFL2, dFR2);
                        routeExec.obstacleWaitActive = false;
                        if (frontMin2 <= OBSTACLE_THRESHOLD_CM) {
                            // Confirmed obstacle: trigger avoidance
                            float dL = distanciaSamples(IR_LEFT_SIDE_PIN, 3, NULL);
                            float dR = distanciaSamples(IR_RIGHT_SIDE_PIN, 3, NULL);
                            routeExec.obstacleSide = (dL > dR) ? +1 : -1; // prefer left if more clearance
                            routeExec.obstacleActive = true;
                            routeExec.obstacleState = 1; // TURN
                            // stop current motion and initiate a 90deg turn toward chosen side
                            motors.stop();
                            delay(30);
                            // choose probe pin: sensor opposite to the turn (as requested)
                            routeExec.obstacleProbePin = (routeExec.obstacleSide == +1) ? IR_RIGHT_SIDE_PIN : IR_LEFT_SIDE_PIN;
                            // compute a safety cap in pulses for the sensor-based forward (AVOID_MAX_STEP_CM)
                            {
                                float pulsesF = (AVOID_MAX_STEP_CM / (float)WHEEL_CIRCUMFERENCE_CM) * (float)encoders.getPulsesPerRevolution();
                                routeExec.obstacleMoveMaxPulses = (long)(pulsesF + 0.5f);
                            }
                            startAutoTurn(routeExec.obstacleSide * 90.0f);
                            Serial.print(F("Obstacle confirmed. side=")); Serial.print(routeExec.obstacleSide);
                            Serial.print(F(" frontMin=")); Serial.println(frontMin2);
                        } else {
                            // false alarm; continue moving
                            Serial.print(F("Obstacle cleared during wait. frontMin=")); Serial.println(frontMin2);
                        }
                    }
                } else if (routeExec.obstacleActive) {
                // Obstacle avoidance in progress; handle via obstacle state machine below
                // The actual transitions are handled in handleAutoTurn() and this loop
                // by monitoring encoders when moving forward for avoidance steps.
                if (routeExec.obstacleState == 2) {
                        // moving forward step: sensor-driven completion (probe opposite sensor)
                        // read probe sensor distance
                        float probeDist = distanciaSamples(routeExec.obstacleProbePin, 3, NULL);
                        long dl = labs(encoders.readLeft() - routeExec.obstacleMoveStartLeft);
                        long dr = labs(encoders.readRight() - routeExec.obstacleMoveStartRight);
                        long maxm = (dl > dr) ? dl : dr;
                        // If probe reports clearance beyond threshold+margin, or we hit safety cap, stop
                        if (probeDist >= (OBSTACLE_THRESHOLD_CM + AVOID_CLEAR_MARGIN_CM) || maxm >= routeExec.obstacleMoveMaxPulses) {
                            motors.stop();
                            delay(30);
                            // after forward step, initiate turn back (opposite 90°)
                            routeExec.obstacleState = 3; // TURNBACK
                            startAutoTurn(-routeExec.obstacleSide * 90.0f);
                        }
                    } else if (routeExec.obstacleState == 4) {
                    // crossing forward step: similar completion check
                    long dl = labs(encoders.readLeft() - routeExec.obstacleMoveStartLeft);
                    long dr = labs(encoders.readRight() - routeExec.obstacleMoveStartRight);
                    long maxm = (dl > dr) ? dl : dr;
                    if (maxm >= routeExec.obstacleMoveTargetPulses) {
                        motors.stop();
                        delay(30);
                        // finish avoidance
                        routeExec.obstacleState = 5; // DONE
                        routeExec.obstacleActive = false;
                        Serial.println(F("Obstacle avoidance finished."));
                        // recompute movement towards same waypoint from new pose
                        float dx = routeExec.targetX - odometry.getX();
                        float dy = routeExec.targetY - odometry.getY();
                        float dist = sqrtf(dx*dx + dy*dy);
                        float pulsesF = (dist / (float)WHEEL_CIRCUMFERENCE_CM) * (float)encoders.getPulsesPerRevolution();
                        routeExec.moveTargetPulses = (long)(pulsesF + 0.5f);
                        routeExec.moveStartLeft = encoders.readLeft();
                        routeExec.moveStartRight = encoders.readRight();
                        if (routeExec.moveTargetPulses <= 0) {
                            // Ya alcanzó el waypoint después de evasión, avanzar al siguiente
                            Serial.println(F("Waypoint alcanzado después de evasión. Avanzando al siguiente..."));
                            routeExec.currentPoint++;
                            beginNextWaypoint();
                        } else {
                            // Continuar hacia el mismo waypoint desde nueva posición
                            Serial.print(F("Continuando hacia waypoint desde nueva posición: "));
                            Serial.print(routeExec.moveTargetPulses);
                            Serial.println(F(" pulsos"));
                            motors.moveForward();
                            routeExec.state = ROUTE_MOVING;
                        }
                    }
                }
            } else {
                // Normal movement completion check (no obstacle active)
                long dl = labs(encoders.readLeft() - routeExec.moveStartLeft);
                long dr = labs(encoders.readRight() - routeExec.moveStartRight);
                long maxm = (dl > dr) ? dl : dr;
                if (maxm >= routeExec.moveTargetPulses) {
                    // Waypoint alcanzado, avanzar al siguiente
                    motors.stop();
                    Serial.print(F("Waypoint alcanzado. Total waypoints visitados: "));
                    Serial.print(routeExec.currentPoint + 1);
                    Serial.print(F("/"));
                    Serial.println(routesCounts[routeExec.routeIndex]);
                    routeExec.currentPoint++;
                    // small pause before next waypoint
                    delay(80);
                    beginNextWaypoint();
                }
            }
        }
    }

    // ========================================
    //     SEGUIMIENTO DE PARED
    // ========================================
    // Nota: El seguimiento de pared y las rutas son mutuamente excluyentes
    if (wallFollow.active && !routeExec.active) {
        // Leer sensores
        float dFL = distanciaSamples(IR_FRONT_LEFT_PIN, 3, NULL);
        float dFR = distanciaSamples(IR_FRONT_RIGHT_PIN, 3, NULL);
        float dL = distanciaSamples(IR_LEFT_SIDE_PIN, 3, NULL);
        float dR = distanciaSamples(IR_RIGHT_SIDE_PIN, 3, NULL);
        
        // Determinar qué sensores están detectando pared
        bool frontLeftWall = dFL <= WALL_FOLLOW_THRESHOLD_CM;
        bool frontRightWall = dFR <= WALL_FOLLOW_THRESHOLD_CM;
        bool leftWall = dL <= WALL_FOLLOW_THRESHOLD_CM;
        bool rightWall = dR <= WALL_FOLLOW_THRESHOLD_CM;
        bool allWalls = frontLeftWall && frontRightWall && leftWall && rightWall;
        
        // Sensor lateral de la pared que estamos siguiendo
        float followSideDist = (wallFollow.side == 1) ? dL : dR;
        bool followSideWall = followSideDist <= WALL_FOLLOW_THRESHOLD_CM;
        
        // Sensor lateral opuesto a la pared que estamos siguiendo
        float oppositeSideDist = (wallFollow.side == 1) ? dR : dL;
        bool oppositeSideWall = oppositeSideDist <= WALL_FOLLOW_THRESHOLD_CM;
        
        // Sensor frontal (mínimo de ambos frontales)
        float frontMin = (dFL < dFR) ? dFL : dFR;
        bool frontWall = frontMin <= WALL_FOLLOW_THRESHOLD_CM;
        
        if (wallFollow.state == 3) {
            // Estado: detenido (todas las paredes detectadas)
            if (allWalls) {
                // Verificar timeout de 10 segundos
                if (wallFollow.allWallsDetectedStart == 0) {
                    wallFollow.allWallsDetectedStart = millis();
                } else if (millis() - wallFollow.allWallsDetectedStart >= ALL_WALLS_TIMEOUT_MS) {
                    Serial.println(F("Timeout: todas las paredes detectadas por más de 10s. Finalizando seguimiento."));
                    stopWallFollowing();
                }
            } else {
                // Ya no todas las paredes detectadas, volver a seguir
                wallFollow.state = 1;
                wallFollow.allWallsDetectedStart = 0;
                Serial.println(F("Reanudando seguimiento de pared."));
            }
        } else if (wallFollow.state == 2) {
            // Estado: girando (esperando que termine el giro automático)
            if (!autoTurnActive) {
                // Giro completado
                if (wallFollow.routeResumePending && wallFollow.resumeAction == 1) {
                    // Este giro era para reanudar la ruta, detener seguimiento y reanudar ruta
                    if (routeExec.active) {
                        // Recalcular movimiento hacia el waypoint objetivo desde la posición actual
                        float dx = routeExec.targetX - odometry.getX();
                        float dy = routeExec.targetY - odometry.getY();
                        float dist = sqrtf(dx*dx + dy*dy);
                        float pulsesF = (dist / (float)WHEEL_CIRCUMFERENCE_CM) * (float)encoders.getPulsesPerRevolution();
                        routeExec.moveTargetPulses = (long)(pulsesF + 0.5f);
                        routeExec.moveStartLeft = encoders.readLeft();
                        routeExec.moveStartRight = encoders.readRight();
                        if (routeExec.moveTargetPulses > 0) {
                            motors.moveForward();
                            routeExec.state = ROUTE_MOVING;
                            Serial.print(F("Giro completado. Reanudando ejecución de ruta. Movimiento: "));
                            Serial.print(routeExec.moveTargetPulses);
                            Serial.println(F(" pulsos"));
                        } else {
                            // Ya está en el waypoint, avanzar al siguiente
                            routeExec.currentPoint++;
                            beginNextWaypoint();
                        }
                    }
                    stopWallFollowing();
                } else {
                    // Giro normal durante seguimiento, volver a seguir
                    wallFollow.state = 1;
                    Serial.println(F("Giro completado. Reanudando seguimiento."));
                }
            }
        } else {
            // Estado: siguiendo pared (state == 1)
            
            // Verificar condición de reanudación de ruta
            // Prioridad 1: Si deja de detectar los 2 frontales Y el lateral opuesto → reanudar de frente
            if (!frontLeftWall && !frontRightWall && !oppositeSideWall && !wallFollow.routeResumePending) {
                wallFollow.routeResumePending = true;
                wallFollow.resumeAction = 2; // forward
                Serial.println(F("No hay frontales ni lateral opuesto. Reanudando ruta de frente."));
            }
            // Prioridad 2: Si deja de detectar los 2 frontales → reanudar de frente
            else if (!frontLeftWall && !frontRightWall && !wallFollow.routeResumePending) {
                wallFollow.routeResumePending = true;
                wallFollow.resumeAction = 2; // forward
                Serial.println(F("No hay paredes frontales. Reanudando ruta de frente."));
            }
            // Prioridad 3: Si deja de detectar el lateral opuesto → hacer giro (pues tendría pared al frente)
            else if (!oppositeSideWall && !wallFollow.routeResumePending) {
                wallFollow.routeResumePending = true;
                wallFollow.resumeAction = 1; // turn
                Serial.println(F("Lateral opuesto ya no detecta pared. Realizando giro para reanudar ruta (pared al frente probable)."));
            }
            
            // Si hay ruta pendiente de reanudar, ejecutar acción
            if (wallFollow.routeResumePending) {
                if (wallFollow.resumeAction == 1) {
                    // Realizar giro
                    wallFollow.state = 2; // turning
                    startAutoTurn(90.0f * wallFollow.side); // girar hacia la pared que seguíamos
                    wallFollow.routeResumePending = false;
                    wallFollow.resumeAction = 0;
                } else if (wallFollow.resumeAction == 2) {
                    // Reanudar ruta de frente
                    if (routeExec.active) {
                        // Recalcular movimiento hacia el waypoint objetivo desde la posición actual
                        float dx = routeExec.targetX - odometry.getX();
                        float dy = routeExec.targetY - odometry.getY();
                        float dist = sqrtf(dx*dx + dy*dy);
                        float pulsesF = (dist / (float)WHEEL_CIRCUMFERENCE_CM) * (float)encoders.getPulsesPerRevolution();
                        routeExec.moveTargetPulses = (long)(pulsesF + 0.5f);
                        routeExec.moveStartLeft = encoders.readLeft();
                        routeExec.moveStartRight = encoders.readRight();
                        if (routeExec.moveTargetPulses > 0) {
                            motors.moveForward();
                            routeExec.state = ROUTE_MOVING;
                            Serial.print(F("Reanudando ejecución de ruta. Movimiento: "));
                            Serial.print(routeExec.moveTargetPulses);
                            Serial.println(F(" pulsos"));
                        } else {
                            // Ya está en el waypoint, avanzar al siguiente
                            routeExec.currentPoint++;
                            beginNextWaypoint();
                        }
                    }
                    stopWallFollowing();
                }
            }
            
            // Verificar si todas las paredes están detectadas
            if (allWalls) {
                motors.stop();
                wallFollow.state = 3; // stopped
                wallFollow.allWallsDetectedStart = millis();
                Serial.println(F("Todas las paredes detectadas. Deteniendo seguimiento."));
            } else if (frontWall) {
                // Hay pared al frente, girar
                wallFollow.state = 2; // turning
                motors.stop();
                // Girar hacia el lado opuesto a la pared que seguimos (alejarse de la pared seguida)
                startAutoTurn(-90.0f * wallFollow.side);
                Serial.println(F("Pared al frente detectada. Girando..."));
            } else {
                // Seguir la pared ajustando velocidad según distancia
                int baseSpeed = WALL_FOLLOW_SPEED;
                int leftSpeed = baseSpeed;
                int rightSpeed = baseSpeed;
                
                // Ajustar velocidad según distancia a la pared seguida
                if (followSideDist < 15.0f) {
                    // Muy cerca de la pared, alejarse ligeramente
                    if (wallFollow.side == 1) {
                        // Siguiendo pared izquierda, girar ligeramente a la derecha
                        rightSpeed = baseSpeed - 20;
                        leftSpeed = baseSpeed;
                    } else {
                        // Siguiendo pared derecha, girar ligeramente a la izquierda
                        leftSpeed = baseSpeed - 20;
                        rightSpeed = baseSpeed;
                    }
                } else if (followSideDist > 25.0f) {
                    // Lejos de la pared, acercarse ligeramente
                    if (wallFollow.side == 1) {
                        // Siguiendo pared izquierda, girar ligeramente a la izquierda
                        leftSpeed = baseSpeed - 20;
                        rightSpeed = baseSpeed;
                    } else {
                        // Siguiendo pared derecha, girar ligeramente a la derecha
                        rightSpeed = baseSpeed - 20;
                        leftSpeed = baseSpeed;
                    }
                }
                
                motors.setBothMotors(leftSpeed, rightSpeed);
            }
        }
    }

    // Si estamos en modo impresión de tics mientras avanzamos (comando 'W')
    if (printTicksWhileMoving && millis() - lastTickPrintMillis >= TICK_PRINT_INTERVAL) {
        long dl = encoders.readLeft() - tickPrintLeft0;
        long dr = encoders.readRight() - tickPrintRight0;
        if (dl < 0) dl = 0;
        if (dr < 0) dr = 0;
        long avg = (dl + dr) / 2;
        Serial.print(F("Ticks L:")); Serial.print(dl);
        Serial.print(F(" R:")); Serial.print(dr);
        Serial.print(F(" Avg:")); Serial.println(avg);
        lastTickPrintMillis = millis();
    }
    
    // Inspección continua (comando 'I') - ejecuta las mismas acciones que la
    // inspección rápida pero de forma periódica hasta que se envíe 'X'.
    if (inspectionActive && (millis() - inspectionLastMillis >= INSPECTION_INTERVAL_MS)) {
        inspectionLastMillis = millis();
        // Single-line output: pulses and raw IR values, each field width 6 for alignment
        // Read raws (for telemetry) and compute distances in cm using calibrated function
        IRSensors ir = readIRSensors();
        long pL = encoders.readLeft();
        long pR = encoders.readRight();
        unsigned long tTotal = 0;
        float dL = distanciaSamples(IR_LEFT_SIDE_PIN, IR_NUM_SAMPLES, &tTotal);
        float dFL = distanciaSamples(IR_FRONT_LEFT_PIN, IR_NUM_SAMPLES, &tTotal);
        float dB = distanciaSamples(IR_BACK_CENTER_PIN, IR_NUM_SAMPLES, &tTotal);
        float dFR = distanciaSamples(IR_FRONT_RIGHT_PIN, IR_NUM_SAMPLES, &tTotal);
        float dR = distanciaSamples(IR_RIGHT_SIDE_PIN, IR_NUM_SAMPLES, &tTotal);
        char buf[160];
        // Single-line: pulses (width 6) and distances in cm with 1 decimal (width 6)
        snprintf(buf, sizeof(buf), "[I] Pulses L:%6ld R:%6ld  Dist cm: L:%6.1f FL:%6.1f B:%6.1f FR:%6.1f R:%6.1f",
             pL, pR, dL, dFL, dB, dFR, dR);
        Serial.println(buf);
    }

    delay(5); // Pequeña pausa para estabilidad

    // manejar cliente WiFi (dashboard server)
    handleWiFiServer();

    // Manejo de muestreo IR continuo (si está activo por comando 'K')
    if (irSampler != nullptr && irSampler->running) {
        unsigned long now = millis();
        if (now - irSampler->lastMillis >= irSampler->intervalMs) {
            irSampler->lastMillis = now;
            // Leer sensores IR y calcular distancias
            IRSensors s = readIRSensors();
            float dL = irRawToCentimeters(s.rawLeft);
            float dFL = irRawToCentimeters(s.rawFrontLeft);
            float dB = irRawToCentimeters(s.rawBack);
            float dFR = irRawToCentimeters(s.rawFrontRight);
            float dR = irRawToCentimeters(s.rawRight);
            // Enviar por Serial (puedes cambiar por otro canal si lo deseas)
            Serial.print(F("K IR: L:")); Serial.print(dL,1);
            Serial.print(F(" FL:")); Serial.print(dFL,1);
            Serial.print(F(" B:")); Serial.print(dB,1);
            Serial.print(F(" FR:")); Serial.print(dFR,1);
            Serial.print(F(" R:")); Serial.println(dR,1);
        }
    }
}

// ========================================
//                LOOP NOTES
// ========================================
// El loop principal implementa un sistema no bloqueante que ejecuta múltiples
// tareas de forma cooperativa. El delay(5) al final proporciona:
// - Estabilidad en lecturas de sensores analógicos
// - Prevención de saturación de CPU
// - Mejor responsividad del sistema WiFi
// 
// Tareas ejecutadas (en orden de prioridad):
// 1. Odometría (alta frecuencia, 50ms)
// 2. Comandos serie (alta prioridad, inmediato)
// 3. Giros automáticos (verificación continua)
// 4. Máquina de estados de rutas (si activa)
// 5. Utilidades (impresión tics, inspección)
// 6. Servidor WiFi (baja prioridad, no bloqueante)

// ========================================
//         PROCESAMIENTO COMANDOS
// ========================================
// Procesa comandos recibidos por Serial (115200 baudios).
// Los comandos se agrupan en categorías:
// - Movimiento: W (adelante), S (atrás), A/D (giro 90°), Q/E (giro continuo)
// - Utilidades: X (stop), R (reset posición), P (mostrar posición), H (ayuda)
// - Tests: T (test motores), V (calibración encoder), I (inspección continua)
//
// Nota: Durante giros automáticos, solo se acepta el comando 'X' para cancelar.
void processCommand(char cmd) {
    cmd = toupper(cmd);
    
    // Prevenir comandos durante giro automático (excepto 'X' para cancelar)
    if (turningInProgress && cmd != 'X') {
        Serial.println(F("Girando..."));
        return;
    }
    switch (cmd) {
    // ---------------------------
    // MOVIMIENTO: Adelante / Atrás / Giros
    // ---------------------------
    case 'W':
            Serial.println(F("Adelante (manual hold)"));
            // manual forward at 40% PWM while holding
            {
                int manualFwdSpeed = (int)(MAX_SPEED * 0.40f);
                // Iniciar impresión de tics mientras avanzamos
                tickPrintLeft0 = encoders.readLeft();
                tickPrintRight0 = encoders.readRight();
                lastTickPrintMillis = millis();
                printTicksWhileMoving = true;
                Serial.print(F("Imprimiendo tics cada "));
                Serial.print(TICK_PRINT_INTERVAL);
                Serial.println(F(" ms"));
                motors.moveForward(manualFwdSpeed);
            }
            break;
            
        case 'S':
            Serial.println(F("Atras (manual hold)"));
            {
                int manualBackSpeed = (int)(MAX_SPEED * 0.40f);
                motors.moveBackward(manualBackSpeed);
            }
            break;
            
    case 'A':
            Serial.println(F("Izq 90"));
            startAutoTurn(-90);
            break;
            
        case 'D':
            Serial.println(F("Der 90"));
            startAutoTurn(90);
            break;
            
        case 'Q':
            // Giro continuo a la izquierda (manual, while-pressed). Use 20% PWM
            {
                int manualTurnSpeed = (int)(MAX_SPEED * 0.20f);
                Serial.print(F("Giro Izq manual (hold). speed=")); Serial.println(manualTurnSpeed);
                motors.turnLeft(manualTurnSpeed);
            }
            break;
                        
        case 'E':
            // Giro continuo a la derecha (manual, while-pressed). Use 20% PWM
            {
                int manualTurnSpeed = (int)(MAX_SPEED * 0.20f);
                Serial.print(F("Giro Der manual (hold). speed=")); Serial.println(manualTurnSpeed);
                motors.turnRight(manualTurnSpeed);
            }
            break;

    // ---------------------------
    // TEST: Avanzar 1 vuelta (calibración de encoder)
    // ---------------------------
    case 'V':
            // Avanzar exactamente una revolución de rueda (ambas ruedas, promedio de encoders)
            Serial.println(F("Avanzar 1 vuelta"));
            {
                // Leer contadores iniciales
                long left0 = encoders.readLeft();
                long right0 = encoders.readRight();
                int target = encoders.getPulsesPerRevolution();
                unsigned long lastPrint = millis();

                Serial.print(F("Target pulses: "));
                Serial.println(target);

                // Arrancar motores hacia adelante
                motors.moveForward();

                // Esperar hasta alcanzar el objetivo (basado en la rueda que más avance)
                while (true) {
                    long dl = encoders.readLeft() - left0;
                    long dr = encoders.readRight() - right0;
                    if (dl < 0) dl = 0; // proteger contra lecturas invertidas momentáneas
                    if (dr < 0) dr = 0;
                    long maxv = (dl > dr) ? dl : dr;
                    if (maxv >= target) break;

                    // Imprimir tics periódicamente para ver progreso
                    if (millis() - lastPrint >= 100) {
                        long avg = (dl + dr) / 2;
                        Serial.print(F("Ticks L:")); Serial.print(dl);
                        Serial.print(F(" R:")); Serial.print(dr);
                        Serial.print(F(" Avg:")); Serial.print(avg);
                        Serial.print(F(" Max:")); Serial.println(maxv);
                        lastPrint = millis();
                    }

                    delay(20);
                }

                motors.stop();
                // Mostrar conteo final
                long finalL = encoders.readLeft() - left0;
                long finalR = encoders.readRight() - right0;
                Serial.print(F("Final L:")); Serial.print(finalL);
                Serial.print(F(" R:")); Serial.println(finalR);
                // Calcular pulso medido por vuelta (usar la rueda que más pulses registró)
                long measured = abs(finalL) > abs(finalR) ? abs(finalL) : abs(finalR);
                Serial.print(F("Measured pulses/rev:")); Serial.println(measured);
                // Actualizar configuración runtime
                encoders.setPulsesPerRevolution((int)measured);
                Serial.print(F("Pulses_per_rev updated to: ")); Serial.println(encoders.getPulsesPerRevolution());
                Serial.println(F("Hecho: 1 vuelta"));
            }
            break;
            
    // ---------------------------
    // UTILERÍAS / CONTROL
    // ---------------------------
    case 'X':
            Serial.println(F("Stop"));
            motors.stop();
            turningInProgress = false;
            // Detener impresión de tics si estaba activa
            printTicksWhileMoving = false;
            // Detener inspección continua si está activa
            if (inspectionActive) {
                inspectionActive = false;
                Serial.println(F("Inspeccion continua detenida."));
            }
            // Si hay muestreo IR en curso, detenerlo y liberar recursos
            if (irSampler) {
                Serial.println(F("Deteniendo muestreo IR (K)..."));
                delete irSampler;
                irSampler = nullptr;
            }
            break;
            
    case 'R':
            Serial.println(F("Reset"));
            motors.stop();
            odometry.resetPosition();
            turningInProgress = false;
            break;
            
    case 'P':
            odometry.printPosition();
            break;
            
    case 'H':
            showHelp();
            break;
            
    // ---------------------------
    // TEST: Test completo de motores
    // ---------------------------
        case 'T':
            Serial.println(F("Test"));
            motors.testMotors();
            break;

        case 'I':
            // Start continuous inspection mode: will run until 'X' is sent
            if (!inspectionActive) {
                inspectionActive = true;
                inspectionLastMillis = 0;
                Serial.println(F("Inspeccion continua iniciada. Enviar 'X' para detener."));
            } else {
                Serial.println(F("Inspeccion ya en ejecución."));
            }
            break;
            
    case '\r':
        case '\n':
            // Ignorar caracteres de nueva línea
            break;
            
        default:
            if (isPrintable(cmd)) {
                Serial.print(F("? "));
                Serial.println(cmd);
            }
            break;
    }
}

// ========================================
//            GIROS AUTOMÁTICOS
// ========================================
// Sistema de giros precisos basado en encoders con cálculo geométrico.
// 
// Funcionamiento:
// 1. Calcula pulsos necesarios según ángulo y geometría del robot
// 2. Inicia movimiento de giro en sitio (motores en sentido opuesto)
// 3. Monitorea encoders hasta alcanzar pulsos objetivo
// 4. Timeout de seguridad (4 segundos) para prevenir bloqueos
//
// Integración con máquina de estados:
// - Se usa en navegación automática (beginNextWaypoint)
// - Se usa en evasión de obstáculos (giros de 90°)
// - Maneja giros post-finalización de ruta (180°)
void startAutoTurn(float angleDelta) {
    // Preparar giro basado en encoders
    turningInProgress = true;
    targetAngle = odometry.getThetaDegrees() + angleDelta;

    // Normalizar ángulo objetivo
    while (targetAngle > 180) targetAngle -= 360;
    while (targetAngle < -180) targetAngle += 360;

    // Calcular pulsos necesarios para este ángulo (por rueda)
    float pulsesF = (abs(angleDelta) * (float)encoders.getPulsesPerRevolution() * (float)WHEEL_BASE_CM) / (360.0 * (float)WHEEL_DIAMETER_CM);
    turnTargetPulses = (long)(pulsesF + 0.5);

    // Guardar contadores de inicio
    turnStartLeft0 = encoders.readLeft();
    turnStartRight0 = encoders.readRight();
    turnStartTime = millis();

    // Iniciar movimiento: sentido según signo del ángulo
    if (angleDelta > 0) {
        // Giro derecha: motor izquierdo adelante, motor derecho atrás
        motors.setBothMotors(TURN_SPEED, -TURN_SPEED);
    } else {
        // Giro izquierda: motor izquierdo atrás, motor derecho adelante
        motors.setBothMotors(-TURN_SPEED, TURN_SPEED);
    }

    Serial.print(F("Obj:"));
    Serial.print(targetAngle, 0);
    Serial.print(F(" targetPulses:")); Serial.println(turnTargetPulses);
}

// ========================================
//            GIROS AUTOMÁTICOS (Notas)
// ========================================
// - `startAutoTurn` calcula la cantidad de pulsos de encoder necesarios para
//   girar el ángulo solicitado y arranca los motores en sentido opuesto para
//   producir un giro en sitio. El seguimiento se realiza en `handleAutoTurn`.
// - Los cálculos usan WHEEL_BASE_CM y WHEEL_DIAMETER_CM definidos en Encoder.h
//   o en la configuración del proyecto.

void handleAutoTurn() {
    if (!turningInProgress) return;

    // Comprobar avance por encoders
    long dl = encoders.readLeft() - turnStartLeft0;
    long dr = encoders.readRight() - turnStartRight0;
    long adl = abs(dl);
    long adr = abs(dr);
    long maxMoved = (adl > adr) ? adl : adr;

    // Mostrar progreso ocasionalmente
    // Si alcanzamos la cantidad de pulsos objetivo, paramos
    if (maxMoved >= turnTargetPulses) {
        motors.stop();
        turningInProgress = false;
        Serial.println(F("OK"));
        // If this was a post-finish 180° turn, handle waiting/finishing logic
        if (routeExec.postFinishTurn) {
            routeExec.postFinishTurn = false;
            if (!routeExec.returnModeActive) {
                // We completed the 180° after IDA: wait for confirmation to start return
                routeExec.awaitingConfirm = true;
                routeExec.waitingForReturnConfirm = true;
                routeExec.state = ROUTE_WAITING;
                Serial.println(F("Giro de 180° completado. Esperando confirmación para iniciar retorno..."));
                return;
            } else {
                // We completed the 180° after RETORNO: finish route
                routeExec.active = false;
                routeExec.state = ROUTE_DONE;
                Serial.println(F("Giro de 180° completado. Ruta y retorno finalizados. Listo para nueva ruta."));
                return;
            }
        }

        return;
    }
        // If an obstacle avoidance sequence was waiting for this turn to finish,
        // advance the obstacle state machine: after the initial TURN we should
        // start the forward step; after the TURNBACK we should start the crossing step.
        if (routeExec.obstacleActive) {
            if (routeExec.obstacleState == 1) {
                // Completed initial 90° turn; start forward step
                routeExec.obstacleState = 2; // FORWARD
                // compute pulses for AVOID_STEP_CM
                float pulsesF = (AVOID_STEP_CM / (float)WHEEL_CIRCUMFERENCE_CM) * (float)encoders.getPulsesPerRevolution();
                routeExec.obstacleMoveTargetPulses = (long)(pulsesF + 0.5f);
                routeExec.obstacleMoveStartLeft = encoders.readLeft();
                routeExec.obstacleMoveStartRight = encoders.readRight();
                motors.moveForward();
                Serial.print(F("Avoidance: forward step pulses:")); Serial.println(routeExec.obstacleMoveTargetPulses);
            } else if (routeExec.obstacleState == 3) {
                // Completed turn back toward original heading; start crossing forward step
                routeExec.obstacleState = 4; // CROSS_FORWARD
                float pulsesF = (AVOID_STEP_CM / (float)WHEEL_CIRCUMFERENCE_CM) * (float)encoders.getPulsesPerRevolution();
                routeExec.obstacleMoveTargetPulses = (long)(pulsesF + 0.5f);
                routeExec.obstacleMoveStartLeft = encoders.readLeft();
                routeExec.obstacleMoveStartRight = encoders.readRight();
                motors.moveForward();
                Serial.print(F("Avoidance: cross forward pulses:")); Serial.println(routeExec.obstacleMoveTargetPulses);
            }
            return;
        }

    // Verificar timeout
    if (millis() - turnStartTime > MAX_TURN_TIME) {
        motors.stop();
        turningInProgress = false;
        Serial.println(F("Timeout"));
        return;
    }
}

// ========================================
//            FUNCIONES AYUDA
// ========================================
void showHelp() {
    Serial.println(F("=== COMANDOS ==="));
    Serial.println(F("W:Adelante (imprime tics) / S:Atras"));
    Serial.println(F("A/D:Izq/Der 90"));
    Serial.println(F("X:Stop P:Pos R:Reset"));
    Serial.println(F("T:Test (motores) V:Avanzar 1 vuelta I:Inspeccionar"));
    odometry.printPosition();
}

// ========================================
//               AYUDA / DOCUMENTACIÓN
// ========================================
// - `showHelp()` imprime en Serial los comandos disponibles.
// - Mantén esta función actualizada si agregas/quitas comandos.

// ========================================
//            UTILIDADES
// ========================================
bool isPrintable(char c) {
    return (c >= 32 && c <= 126);
}

// ======== SERVIDOR Y RESPUESTAS ========
void setupWiFi() {
    WiFi.disconnect();
    WiFi.beginAP(AP_SSID, AP_PASS);
    Serial.print(F("Iniciando AP... "));
    Serial.println(AP_SSID);
    IPAddress ip = WiFi.localIP();
    Serial.print(F("IP local: ")); Serial.println(ip);
    server.begin();
}

void handleClient(WiFiClient client) {
    String req = client.readStringUntil('\r');
    client.flush();

    // Serve routes JSON
    if (req.indexOf("GET /routes ") >= 0 || req.indexOf("GET /routes\r") >= 0 || req.indexOf("GET /routes?") >= 0) {
        // Build JSON array of routes
        String json = "[";
        for (int i = 0; i < ROUTE_COUNT; ++i) {
            if (i) json += ",";
            json += "{";
            json += "\"name\":\"" + String(routeNames[i]) + "\",";
            json += "\"points\":[";
            for (int j = 0; j < routesCounts[i]; ++j) {
                if (j) json += ",";
                float px = routesPoints[i][j].x;
                float py = routesPoints[i][j].y;
                json += "{";
                json += "\"x\":" + String(px, 3) + ",";
                json += "\"y\":" + String(py, 3);
                json += "}";
            }
            json += "]}";
        }
        json += "]";

        client.println(F("HTTP/1.1 200 OK"));
        client.println(F("Content-Type: application/json"));
        client.println(F("Connection: close"));
        client.println();
        client.print(json);
        return;
    }

    // Serve the routes UI page
    if (req.indexOf("GET /routes_ui") >= 0) {
        client.println(F("HTTP/1.1 200 OK"));
        client.println(F("Content-Type: text/html"));
        client.println(F("Connection: close"));
        client.println();
        client.print(reinterpret_cast<const __FlashStringHelper*>(routesPageHTML));
        return;
    }

    // Return route execution status JSON
    if (req.indexOf("GET /route_status") >= 0) {
        String json = "{";
        json += "\"active\":" + String(routeExec.active ? 1 : 0) + ",";
        json += "\"state\":" + String((int)routeExec.state) + ",";
        json += "\"routeIndex\":" + String(routeExec.routeIndex) + ",";
        json += "\"direction\":" + String(routeExec.direction) + ",";
        json += "\"currentPoint\":" + String(routeExec.currentPoint) + ",";
        json += "\"awaitingConfirm\":" + String(routeExec.awaitingConfirm ? 1 : 0) + ",";
        json += "\"targetX\":" + String(routeExec.targetX,3) + ",";
        json += "\"targetY\":" + String(routeExec.targetY,3) + ",";
        json += "\"obstacleActive\":" + String(routeExec.obstacleActive ? 1 : 0) + ",";
        json += "\"obstacleState\":" + String(routeExec.obstacleState) + ",";
        unsigned long remaining = 0;
        if (routeExec.state == ROUTE_WAITING) {
            unsigned long elapsed = millis() - routeExec.requestMillis;
            if (elapsed < routeExec.delayMs) remaining = routeExec.delayMs - elapsed;
        }
        json += "\"remainingDelayMs\":" + String(remaining);
        json += "}";
        client.println(F("HTTP/1.1 200 OK"));
        client.println(F("Content-Type: application/json"));
        client.println(F("Connection: close"));
        client.println();
        client.print(json);
        return;
    }

    // Confirm scheduled route start early: /confirm_route
    if (req.indexOf("GET /confirm_route") >= 0) {
        if (routeExec.active && routeExec.state == ROUTE_WAITING && routeExec.awaitingConfirm) {
            // If we are waiting specifically to start the return, enable return mode
            if (routeExec.waitingForReturnConfirm) {
                // Operator confirmed return: enable return mode and start return
                // NOTA: Ya se hizo el giro de 180° al terminar la IDA, así que no necesitamos girar de nuevo
                routeExec.awaitingConfirm = false;
                routeExec.waitingForReturnConfirm = false;
                routeExec.returnModeActive = true;
                routeExec.direction = -1; // run return
                routeExec.currentPoint = 0; // start return from first waypoint (omitiendo el último)
                Serial.println(F("Retorno confirmado. Iniciando ruta de retorno..."));
                beginNextWaypoint(); // Iniciar directamente el retorno (ya está orientado correctamente)
                client.println(F("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nCONFIRMED_RETURN"));
            } else {
                // normal confirm to start scheduled route
                routeExec.awaitingConfirm = false;
                beginNextWaypoint();
                client.println(F("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nCONFIRMED"));
            }
        } else {
            client.println(F("HTTP/1.1 409 Conflict\r\nConnection: close\r\n\r\nNO_SCHEDULE"));
        }
        return;
    }

    // Start route execution: /start_route?route=0&dir=ida|retorno&delay=ms
    if (req.indexOf("GET /start_route") >= 0) {
        int rIdx = 0;
        bool retorno = false;
        unsigned long delayMs = 0;
        int p;
        p = req.indexOf("route=");
        if (p >= 0) {
            p += 6;
            String num;
            while (p < req.length()) {
                char ch = req[p];
                if (ch >= '0' && ch <= '9') { num += ch; p++; } else break;
            }
            rIdx = num.toInt();
        }
        p = req.indexOf("dir=");
        if (p >= 0) {
            p += 4;
            String d;
            while (p < req.length()) {
                char ch = req[p];
                if (ch == '&' || ch == ' ' || ch == '\r') break;
                d += ch; p++;
            }
            d.toLowerCase();
            if (d.indexOf("ret") >= 0) retorno = true;
        }
        p = req.indexOf("delay=");
        if (p >= 0) {
            p += 6;
            String num;
            while (p < req.length()) {
                char ch = req[p];
                if (ch >= '0' && ch <= '9') { num += ch; p++; } else break;
            }
            delayMs = (unsigned long)num.toInt();
        }

        bool ok = startRouteExecution(rIdx, retorno, delayMs);
        if (ok) {
            client.println(F("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK"));
        } else {
            client.println(F("HTTP/1.1 409 Conflict\r\nConnection: close\r\n\r\nBUSY"));
        }
        return;
    }

    // Stop route execution: /stop_route
    if (req.indexOf("GET /stop_route") >= 0) {
        stopRouteExecution();
        client.println(F("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nSTOPPED"));
        return;
    }

    // Start wall following: /wall_follow?side=left|right
    if (req.indexOf("GET /wall_follow") >= 0) {
        int sideIdx = req.indexOf("side=");
        if (sideIdx >= 0) {
            sideIdx += 5;
            String sideStr = req.substring(sideIdx, sideIdx + 5);
            int side = (sideStr.indexOf("left") >= 0) ? 1 : -1;
            startWallFollowing(side);
            client.println(F("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nWALL_FOLLOW_STARTED"));
        } else {
            client.println(F("HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\nMISSING_SIDE"));
        }
        return;
    }

    // Stop wall following: /stop_wall_follow
    if (req.indexOf("GET /stop_wall_follow") >= 0) {
        stopWallFollowing();
        client.println(F("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nWALL_FOLLOW_STOPPED"));
        return;
    }

    if (req.indexOf("GET /data") >= 0) {
        IRSensors s = readIRSensors();
        float x = odometry.getX();
        float y = odometry.getY();
        float th = odometry.getThetaDegrees();

        String json = "{";
        json += "\"x\":" + String(x, 2) + ",";
        json += "\"y\":" + String(y, 2) + ",";
        json += "\"th\":" + String(th, 1) + ",";
        json += "\"ir\":[" + String(s.rawLeft) + "," + String(s.rawFrontLeft) + "," + String(s.rawBack) + "," + String(s.rawFrontRight) + "," + String(s.rawRight) + "]";
        json += "}";

        client.println(F("HTTP/1.1 200 OK"));
        client.println(F("Content-Type: application/json"));
        client.println(F("Connection: close"));
        client.println();
        client.print(json);
        return;
    }

    if (req.indexOf("GET /cmd?c=") >= 0) {
        int idx = req.indexOf("c=") + 2;
        char c = req[idx];
        processCommand(c);
        client.println(F("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK"));
        return;
    }

    // Página principal (dashboard)
    client.println(F("HTTP/1.1 200 OK"));
    client.println(F("Content-Type: text/html"));
    client.println(F("Connection: close"));
    client.println();
    // `dashboardHTML` is stored in flash (PROGMEM). Cast to
    // __FlashStringHelper so Print::print handles it correctly
    // and the full HTML is streamed to the client without truncation.
    client.print(reinterpret_cast<const __FlashStringHelper*>(dashboardHTML));
}

void handleWiFiServer() {
    WiFiClient client = server.available();
    if (client) {
        handleClient(client);
        delay(1);
        client.stop();
    }
}