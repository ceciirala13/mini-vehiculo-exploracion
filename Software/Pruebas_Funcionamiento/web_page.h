#ifndef WEB_PAGE_H
#define WEB_PAGE_H

static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Rover Explorer - Control Hub & Estación Ambiental BME690</title>
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-primary: #0a0e17;
            --bg-card: rgba(18, 26, 43, 0.75);
            --border-card: rgba(0, 240, 255, 0.15);
            --accent-cyan: #00f0ff;
            --accent-magenta: #ff0055;
            --accent-green: #00ff88;
            --accent-amber: #ffaa00;
            --text-primary: #f0f6fc;
            --text-secondary: #8b949e;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
            font-family: 'Outfit', sans-serif;
            user-select: none;
            -webkit-user-select: none;
        }

        body {
            background-color: var(--bg-primary);
            background-image: 
                radial-gradient(circle at 15% 15%, rgba(0, 240, 255, 0.08) 0%, transparent 40%),
                radial-gradient(circle at 85% 85%, rgba(255, 0, 85, 0.08) 0%, transparent 40%);
            color: var(--text-primary);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 1.5rem;
        }

        header {
            width: 100%;
            max-width: 1100px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 1.5rem;
            padding-bottom: 1rem;
            border-bottom: 1px solid var(--border-card);
        }

        .brand {
            display: flex;
            align-items: center;
            gap: 12px;
        }

        .brand-logo {
            width: 42px;
            height: 42px;
            background: linear-gradient(135deg, var(--accent-cyan), var(--accent-magenta));
            border-radius: 12px;
            display: flex;
            align-items: center;
            justify-content: center;
            box-shadow: 0 0 15px rgba(0, 240, 255, 0.4);
            font-weight: 700;
            font-size: 1.3rem;
            color: #000;
        }

        .brand h1 {
            font-size: 1.4rem;
            font-weight: 700;
            letter-spacing: 1px;
            background: linear-gradient(90deg, #fff, var(--text-secondary));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }

        .status-badge {
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 6px 14px;
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid var(--border-card);
            border-radius: 20px;
            font-size: 0.85rem;
            backdrop-filter: blur(10px);
        }

        .status-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background-color: #555;
            transition: background-color 0.3s, box-shadow 0.3s;
        }

        .status-dot.online {
            background-color: var(--accent-green);
            box-shadow: 0 0 10px var(--accent-green);
        }

        .main-container {
            width: 100%;
            max-width: 1100px;
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 1.5rem;
        }

        .full-width {
            grid-column: 1 / -1;
        }

        @media (max-width: 850px) {
            .main-container {
                grid-template-columns: 1fr;
            }
        }

        .card {
            background: var(--bg-card);
            border: 1px solid var(--border-card);
            border-radius: 20px;
            padding: 1.5rem;
            backdrop-filter: blur(12px);
            box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37);
            display: flex;
            flex-direction: column;
            gap: 1.2rem;
        }

        .card-title {
            font-size: 1.05rem;
            font-weight: 600;
            color: var(--accent-cyan);
            text-transform: uppercase;
            letter-spacing: 1px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .telemetry-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
            gap: 1rem;
        }

        .metric-box {
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 14px;
            padding: 1rem;
            text-align: center;
            position: relative;
            overflow: hidden;
        }

        .metric-icon {
            font-size: 1.2rem;
            margin-bottom: 4px;
            display: block;
        }

        .metric-label {
            font-size: 0.75rem;
            color: var(--text-secondary);
            text-transform: uppercase;
            letter-spacing: 0.5px;
            margin-bottom: 4px;
        }

        .metric-value {
            font-size: 1.6rem;
            font-weight: 700;
            color: #fff;
        }

        .metric-unit {
            font-size: 0.85rem;
            color: var(--text-secondary);
            margin-left: 2px;
        }

        .iaq-badge {
            display: inline-block;
            padding: 3px 8px;
            border-radius: 8px;
            font-size: 0.75rem;
            font-weight: 600;
            margin-top: 4px;
        }

        .iaq-excellent { background: rgba(0, 255, 136, 0.2); color: var(--accent-green); border: 1px solid var(--accent-green); }
        .iaq-good      { background: rgba(0, 240, 255, 0.2); color: var(--accent-cyan); border: 1px solid var(--accent-cyan); }
        .iaq-moderate  { background: rgba(255, 170, 0, 0.2); color: var(--accent-amber); border: 1px solid var(--accent-amber); }
        .iaq-poor      { background: rgba(255, 0, 85, 0.2); color: var(--accent-magenta); border: 1px solid var(--accent-magenta); }

        .rover-vis {
            width: 100%;
            height: 180px;
            background: rgba(0, 0, 0, 0.3);
            border-radius: 14px;
            border: 1px dashed rgba(255, 255, 255, 0.1);
            position: relative;
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .rover-chassis {
            width: 70px;
            height: 110px;
            background: linear-gradient(180deg, #1e293b, #0f172a);
            border: 2px solid var(--accent-cyan);
            border-radius: 12px;
            position: relative;
            box-shadow: 0 0 15px rgba(0, 240, 255, 0.2);
            transition: transform 0.1s ease-out;
        }

        .front-wheels {
            position: absolute;
            top: -8px;
            width: 100%;
            display: flex;
            justify-content: space-between;
            padding: 0 4px;
            transition: transform 0.15s ease-out;
            transform-origin: center center;
        }

        .rear-wheels {
            position: absolute;
            bottom: -8px;
            width: 100%;
            display: flex;
            justify-content: space-between;
            padding: 0 4px;
        }

        .wheel {
            width: 12px;
            height: 24px;
            background: #475569;
            border-radius: 4px;
            border: 1px solid #94a3b8;
            transition: background 0.2s, box-shadow 0.2s;
        }

        .wheel.active {
            background: var(--accent-green);
            box-shadow: 0 0 10px var(--accent-green);
        }

        .wheel.reverse {
            background: var(--accent-magenta);
            box-shadow: 0 0 10px var(--accent-magenta);
        }

        .dpad-container {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            grid-template-rows: repeat(3, 1fr);
            gap: 10px;
            width: 220px;
            height: 220px;
            margin: 0 auto;
        }

        .btn-ctrl {
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid var(--border-card);
            border-radius: 14px;
            color: var(--text-primary);
            font-size: 1.2rem;
            font-weight: 700;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            cursor: pointer;
            transition: all 0.15s ease;
            outline: none;
            touch-action: manipulation;
        }

        .btn-ctrl:hover {
            background: rgba(0, 240, 255, 0.15);
            border-color: var(--accent-cyan);
            box-shadow: 0 0 12px rgba(0, 240, 255, 0.3);
        }

        .btn-ctrl:active, .btn-ctrl.pressed {
            background: var(--accent-cyan);
            color: #000;
            box-shadow: 0 0 20px var(--accent-cyan);
            transform: scale(0.95);
        }

        .btn-ctrl.reverse:active, .btn-ctrl.reverse.pressed {
            background: var(--accent-magenta);
            color: #fff;
            box-shadow: 0 0 20px var(--accent-magenta);
        }

        .btn-ctrl.stop {
            background: rgba(255, 0, 85, 0.15);
            border-color: var(--accent-magenta);
            color: var(--accent-magenta);
        }

        .btn-ctrl.stop:active, .btn-ctrl.stop.pressed {
            background: var(--accent-magenta);
            color: #fff;
        }

        .btn-subtext {
            font-size: 0.65rem;
            font-weight: 400;
            opacity: 0.7;
        }

        .up    { grid-area: 1 / 2 / 2 / 3; }
        .left  { grid-area: 2 / 1 / 3 / 2; }
        .stop  { grid-area: 2 / 2 / 3 / 3; }
        .right { grid-area: 2 / 3 / 3 / 4; }
        .down  { grid-area: 3 / 2 / 4 / 3; }

        .slider-group {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }

        .slider-header {
            display: flex;
            justify-content: space-between;
            font-size: 0.85rem;
            color: var(--text-secondary);
        }

        .custom-slider {
            width: 100%;
            height: 8px;
            border-radius: 4px;
            background: #1e293b;
            outline: none;
            -webkit-appearance: none;
            cursor: pointer;
        }

        .custom-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 22px;
            height: 22px;
            border-radius: 50%;
            background: var(--accent-cyan);
            cursor: pointer;
            box-shadow: 0 0 10px var(--accent-cyan);
            transition: transform 0.1s;
        }

        .custom-slider::-webkit-slider-thumb:hover {
            transform: scale(1.2);
        }

        .kbd-badge {
            background: rgba(255, 255, 255, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
            border-radius: 4px;
            padding: 2px 6px;
            font-family: monospace;
            font-size: 0.75rem;
        }

        .speed-presets {
            display: flex;
            gap: 8px;
        }

        .preset-btn {
            flex: 1;
            padding: 6px;
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid var(--border-card);
            border-radius: 8px;
            color: #fff;
            font-size: 0.8rem;
            cursor: pointer;
            transition: all 0.2s;
        }

        .preset-btn.active, .preset-btn:hover {
            background: rgba(0, 240, 255, 0.2);
            border-color: var(--accent-cyan);
            color: var(--accent-cyan);
        }

        .fault-alert {
            display: none;
            background: rgba(255, 0, 85, 0.2);
            border: 1px solid var(--accent-magenta);
            border-radius: 12px;
            padding: 10px 14px;
            color: #ff88a8;
            font-size: 0.85rem;
            align-items: center;
            gap: 10px;
        }

        .fault-alert.show {
            display: flex;
        }
    </style>
</head>
<body>
    <header>
        <div class="brand">
            <div class="brand-logo">R</div>
            <div>
                <h1>ROVER EXPLORER</h1>
                <p style="font-size: 0.75rem; color: var(--text-secondary);">Control ESP32-S3 + Estación Ambiental BME690 (I2C: IO5/IO6)</p>
            </div>
        </div>
        <div class="status-badge">
            <div class="status-dot" id="statusDot"></div>
            <span id="statusText">Conectando...</span>
        </div>
    </header>

    <div class="main-container">
        <!-- SECCIÓN BME690: Datos Ambientales -->
        <div class="card full-width">
            <div class="card-title">
                Sensor Ambiental Bosch BME690 (I2C: SDA=IO5, SCL=IO6)
                <span style="font-size: 0.75rem; font-weight: 400; color: var(--accent-green);">● Lecturas Activas (1 Hz)</span>
            </div>

            <div class="telemetry-grid">
                <div class="metric-box">
                    <span class="metric-icon">🌡️</span>
                    <div class="metric-label">Temperatura</div>
                    <div class="metric-value" id="bmeTemp">--<span class="metric-unit">°C</span></div>
                </div>

                <div class="metric-box">
                    <span class="metric-icon">💧</span>
                    <div class="metric-label">Humedad Relativa</div>
                    <div class="metric-value" id="bmeHum">--<span class="metric-unit">%</span></div>
                </div>

                <div class="metric-box">
                    <span class="metric-icon">⏲️</span>
                    <div class="metric-label">Presión Atmosférica</div>
                    <div class="metric-value" id="bmePress">--<span class="metric-unit">hPa</span></div>
                </div>

                <div class="metric-box">
                    <span class="metric-icon">🍃</span>
                    <div class="metric-label">Resistencia de Gas</div>
                    <div class="metric-value" id="bmeGas">--<span class="metric-unit">kΩ</span></div>
                    <div class="iaq-badge iaq-good" id="bmeIaqBadge">Calidad: Leyendo...</div>
                </div>
            </div>
        </div>

        <!-- Columna Izquierda: Telemetría de Tracción y Vista del Vehículo -->
        <div class="card">
            <div class="card-title">
                Telemetría de Tracción
                <span style="font-size: 0.75rem; font-weight: 400; color: var(--text-secondary);">DRV8873 Dual</span>
            </div>

            <div class="fault-alert" id="faultAlert">
                ⚠️ <strong>ALERTA DE FALLA:</strong> ¡Pin nFAULT detectado en bajo! Motores detenidos.
            </div>

            <div class="telemetry-grid">
                <div class="metric-box">
                    <div class="metric-label">Tracción / Velocidad</div>
                    <div class="metric-value" id="valSpeed">0<span class="metric-unit">%</span></div>
                </div>
                <div class="metric-box">
                    <div class="metric-label">Dirección / Ángulo</div>
                    <div class="metric-value" id="valAngle">0<span class="metric-unit">°</span></div>
                </div>
            </div>

            <div class="rover-vis">
                <div class="rover-chassis" id="roverChassis">
                    <div class="front-wheels" id="frontWheels">
                        <div class="wheel" id="wFL"></div>
                        <div class="wheel" id="wFR"></div>
                    </div>
                    <div class="rear-wheels">
                        <div class="wheel" id="wRL"></div>
                        <div class="wheel" id="wRR"></div>
                    </div>
                </div>
            </div>

            <div class="slider-group">
                <div class="slider-header">
                    <span>Límite de Velocidad Máxima</span>
                    <span id="speedLimitText">40%</span>
                </div>
                <div class="speed-presets">
                    <button class="preset-btn" onclick="setSpeedLimit(20)">20%</button>
                    <button class="preset-btn active" onclick="setSpeedLimit(40)">40%</button>
                    <button class="preset-btn" onclick="setSpeedLimit(60)">60%</button>
                    <button class="preset-btn" onclick="setSpeedLimit(80)">80%</button>
                    <button class="preset-btn" onclick="setSpeedLimit(100)">100%</button>
                </div>
            </div>
        </div>

        <!-- Columna Derecha: Controles del Vehículo -->
        <div class="card">
            <div class="card-title">
                Panel de Control
                <span>
                    <span class="kbd-badge">W</span> <span class="kbd-badge">A</span> <span class="kbd-badge">S</span> <span class="kbd-badge">D</span>
                </span>
            </div>

            <!-- D-PAD INTERACTIVO -->
            <div class="dpad-container">
                <button class="btn-ctrl up" id="btnW" 
                    onmousedown="handlePress('W')" onmouseup="handleRelease('W')"
                    ontouchstart="handlePress('W'); event.preventDefault();" ontouchend="handleRelease('W'); event.preventDefault();">
                    ▲
                    <span class="btn-subtext">W (Avance)</span>
                </button>

                <button class="btn-ctrl left" id="btnA" 
                    onmousedown="handlePress('A')" onmouseup="handleRelease('A')"
                    ontouchstart="handlePress('A'); event.preventDefault();" ontouchend="handleRelease('A'); event.preventDefault();">
                    ◀
                    <span class="btn-subtext">A (Izq)</span>
                </button>

                <button class="btn-ctrl stop" id="btnStop" onclick="emergencyStop()">
                    ■
                    <span class="btn-subtext">PARAR</span>
                </button>

                <button class="btn-ctrl right" id="btnD" 
                    onmousedown="handlePress('D')" onmouseup="handleRelease('D')"
                    ontouchstart="handlePress('D'); event.preventDefault();" ontouchend="handleRelease('D'); event.preventDefault();">
                    ▶
                    <span class="btn-subtext">D (Der)</span>
                </button>

                <button class="btn-ctrl down reverse" id="btnS" 
                    onmousedown="handlePress('S')" onmouseup="handleRelease('S')"
                    ontouchstart="handlePress('S'); event.preventDefault();" ontouchend="handleRelease('S'); event.preventDefault();">
                    ▼
                    <span class="btn-subtext">S (Reversa)</span>
                </button>
            </div>

            <!-- Control Deslizante Manual de Servo -->
            <div class="slider-group" style="margin-top: 10px;">
                <div class="slider-header">
                    <span>Ángulo de Servomotor</span>
                    <span id="servoSliderVal">0°</span>
                </div>
                <input type="range" min="-45" max="45" value="0" class="custom-slider" id="servoSlider" oninput="onServoSliderChange(this.value)">
                <div style="display: flex; justify-content: space-between; font-size: 0.75rem; color: var(--text-secondary);">
                    <span>-45° (Izquierda)</span>
                    <span style="cursor: pointer; color: var(--accent-cyan);" onclick="recenterServo()">[ Centrar 0° ]</span>
                    <span>+45° (Derecha)</span>
                </div>
            </div>
        </div>
    </div>

    <script>
        let ws = null;
        let useHttpFallback = false;
        let maxSpeedLimit = 40;
        let activeKeys = { W: false, S: false, A: false, D: false };
        let currentSpeed = 0;
        let currentAngle = 0;
        let sendingHttp = false;

        function updateBME690UI(temp, hum, press, gas) {
            if (temp !== undefined) document.getElementById('bmeTemp').innerHTML = `${temp.toFixed(1)}<span class="metric-unit">°C</span>`;
            if (hum !== undefined) document.getElementById('bmeHum').innerHTML = `${hum.toFixed(1)}<span class="metric-unit">%</span>`;
            if (press !== undefined) document.getElementById('bmePress').innerHTML = `${press.toFixed(1)}<span class="metric-unit">hPa</span>`;
            
            if (gas !== undefined) {
                const gasKohm = gas > 1000 ? (gas / 1000.0) : gas;
                document.getElementById('bmeGas').innerHTML = `${gasKohm.toFixed(1)}<span class="metric-unit">kΩ</span>`;
                
                const badge = document.getElementById('bmeIaqBadge');
                if (gasKohm > 100) {
                    badge.className = 'iaq-badge iaq-excellent';
                    badge.innerText = 'Calidad: Excelente (Limpio)';
                } else if (gasKohm > 50) {
                    badge.className = 'iaq-badge iaq-good';
                    badge.innerText = 'Calidad: Buena';
                } else if (gasKohm > 20) {
                    badge.className = 'iaq-badge iaq-moderate';
                    badge.innerText = 'Calidad: Moderada';
                } else {
                    badge.className = 'iaq-badge iaq-poor';
                    badge.innerText = 'Calidad: Contaminada (COV/Gas)';
                }
            }
        }

        function pollTelemetry() {
            fetch('/api/telemetry')
                .then(r => r.json())
                .then(data => {
                    if (data.temp !== undefined) {
                        updateBME690UI(data.temp, data.hum, data.press, data.gas);
                    }
                    if (data.fault !== undefined) {
                        const alert = document.getElementById('faultAlert');
                        if (data.fault) alert.classList.add('show');
                        else alert.classList.remove('show');
                    }
                })
                .catch(() => {});
        }

        function connectWebSocket() {
            const host = window.location.host || '192.168.4.1';
            try {
                ws = new WebSocket('ws://' + host + '/ws');

                ws.onopen = () => {
                    useHttpFallback = false;
                    document.getElementById('statusDot').className = 'status-dot online';
                    document.getElementById('statusText').innerText = 'CONECTADO (WebSocket)';
                };

                ws.onclose = () => {
                    useHttpFallback = true;
                    document.getElementById('statusDot').className = 'status-dot online';
                    document.getElementById('statusText').innerText = 'CONECTADO (HTTP REST)';
                };

                ws.onerror = () => {
                    useHttpFallback = true;
                    document.getElementById('statusDot').className = 'status-dot online';
                    document.getElementById('statusText').innerText = 'CONECTADO (HTTP REST)';
                };

                ws.onmessage = (event) => {
                    try {
                        const data = JSON.parse(event.data);
                        if (data.temp !== undefined) {
                            updateBME690UI(data.temp, data.hum, data.press, data.gas);
                        }
                        if (data.fault !== undefined) {
                            const alert = document.getElementById('faultAlert');
                            if (data.fault) alert.classList.add('show');
                            else alert.classList.remove('show');
                        }
                    } catch(e){}
                };
            } catch(e) {
                useHttpFallback = true;
                document.getElementById('statusDot').className = 'status-dot online';
                document.getElementById('statusText').innerText = 'CONECTADO (HTTP REST)';
            }
        }

        function sendControl(speed, angle) {
            currentSpeed = speed;
            currentAngle = angle;

            document.getElementById('valSpeed').innerHTML = `${speed}<span class="metric-unit">%</span>`;
            document.getElementById('valAngle').innerHTML = `${angle}<span class="metric-unit">°</span>`;
            document.getElementById('servoSlider').value = angle;
            document.getElementById('servoSliderVal').innerText = `${angle}°`;

            const frontWheels = document.getElementById('frontWheels');
            frontWheels.style.transform = `rotate(${angle}deg)`;

            const wheels = [document.getElementById('wFL'), document.getElementById('wFR'), document.getElementById('wRL'), document.getElementById('wRR')];
            wheels.forEach(w => {
                w.className = 'wheel';
                if (speed > 0) w.classList.add('active');
                else if (speed < 0) w.classList.add('reverse');
            });

            if (ws && ws.readyState === WebSocket.OPEN && !useHttpFallback) {
                ws.send(JSON.stringify({ speed: speed, angle: angle }));
            } else {
                if (!sendingHttp) {
                    sendingHttp = true;
                    fetch(`/api/control?speed=${speed}&angle=${angle}`)
                        .then(r => r.json())
                        .then(data => {
                            if (data.fault) document.getElementById('faultAlert').classList.add('show');
                            else document.getElementById('faultAlert').classList.remove('show');
                        })
                        .catch(() => {})
                        .finally(() => { sendingHttp = false; });
                }
            }
        }

        function updateStateFromKeys() {
            let targetSpeed = 0;
            let targetAngle = 0;

            if (activeKeys.W && !activeKeys.S) {
                targetSpeed = maxSpeedLimit;
            } else if (activeKeys.S && !activeKeys.W) {
                targetSpeed = -maxSpeedLimit;
            }

            if (activeKeys.A && !activeKeys.D) {
                targetAngle = -35;
            } else if (activeKeys.D && !activeKeys.A) {
                targetAngle = 35;
            }

            sendControl(targetSpeed, targetAngle);
        }

        function handlePress(key) {
            key = key.toUpperCase();
            if (activeKeys.hasOwnProperty(key)) {
                activeKeys[key] = true;
                const btn = document.getElementById('btn' + key);
                if (btn) btn.classList.add('pressed');
                updateStateFromKeys();
            }
        }

        function handleRelease(key) {
            key = key.toUpperCase();
            if (activeKeys.hasOwnProperty(key)) {
                activeKeys[key] = false;
                const btn = document.getElementById('btn' + key);
                if (btn) btn.classList.remove('pressed');
                updateStateFromKeys();
            }
        }

        function emergencyStop() {
            activeKeys = { W: false, S: false, A: false, D: false };
            document.querySelectorAll('.btn-ctrl').forEach(b => b.classList.remove('pressed'));
            sendControl(0, 0);
        }

        function setSpeedLimit(limit) {
            maxSpeedLimit = limit;
            document.getElementById('speedLimitText').innerText = `${limit}%`;
            document.querySelectorAll('.preset-btn').forEach(btn => {
                btn.classList.toggle('active', btn.innerText === `${limit}%`);
            });
            updateStateFromKeys();
        }

        function onServoSliderChange(val) {
            const angle = parseInt(val);
            sendControl(currentSpeed, angle);
        }

        function recenterServo() {
            sendControl(currentSpeed, 0);
        }

        window.addEventListener('keydown', (e) => {
            if (e.repeat) return;
            const k = e.key.toUpperCase();
            if (k === 'W' || k === 'ARROWUP') handlePress('W');
            else if (k === 'S' || k === 'ARROWDOWN') handlePress('S');
            else if (k === 'A' || k === 'ARROWLEFT') handlePress('A');
            else if (k === 'D' || k === 'ARROWRIGHT') handlePress('D');
            else if (e.code === 'Space') {
                e.preventDefault();
                emergencyStop();
            }
        });

        window.addEventListener('keyup', (e) => {
            const k = e.key.toUpperCase();
            if (k === 'W' || k === 'ARROWUP') handleRelease('W');
            else if (k === 'S' || k === 'ARROWDOWN') handleRelease('S');
            else if (k === 'A' || k === 'ARROWLEFT') handleRelease('A');
            else if (k === 'D' || k === 'ARROWRIGHT') handleRelease('D');
        });

        window.onload = () => {
            connectWebSocket();
            setInterval(pollTelemetry, 2000);
        };
    </script>
</body>
</html>
)rawliteral";

#endif // WEB_PAGE_H
