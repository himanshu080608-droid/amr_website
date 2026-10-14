/**
 * Ros_lidar_bot — Interactive Hardware Architecture & Telemetry Specification
 * Logic controller: Radar simulation, Rig Explorer, Packet Inspector,
 * Kinematics Calculators, BOM Filter/Export, and Bring-up copy tools.
 */

document.addEventListener('DOMContentLoaded', () => {
  initRadarSimulation();
  initRigExplorer();
  initSubsystemTabs();
  initPacketInspector();
  initGimbalTester();
  initKinematicsCalculator();
  initPowerCalculator();
  initBomTable();
  initAccordion();
  initCopyButtons();
});

/* ==========================================================================
   1. HERO RADAR & LIDAR SIMULATION (CANVAS)
   ========================================================================== */
function initRadarSimulation() {
  const canvas = document.getElementById('heroRadarCanvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  let width = canvas.width;
  let height = canvas.height;
  let centerX = width / 2;
  let centerY = height / 2;
  let sweepAngle = 0;
  let mode = 'dense'; // 'dense' or 'raw'
  let simulateObstacle = false;
  let obstacleDistance = 0.35; // meters (inside safety stop < 0.40m)

  // Generate simulated dense point cloud data (360 degrees)
  const baseRanges = [];
  for (let i = 0; i < 360; i++) {
    // create a pseudo room perimeter (walls at ~1.5m - 2.8m)
    let rad = (i * Math.PI) / 180;
    let dist = 1.8 + 0.6 * Math.sin(rad * 4) + 0.3 * Math.cos(rad * 2) + (Math.random() * 0.05);
    baseRanges.push(dist);
  }

  const btnMode = document.getElementById('btnRadarMode');
  const btnObstacle = document.getElementById('btnSimulateObstacle');

  if (btnMode) {
    btnMode.addEventListener('click', () => {
      mode = (mode === 'dense') ? 'sparse' : 'dense';
      btnMode.querySelector('span').textContent = mode === 'dense' ? 'DenseBoost (~3200 pts)' : 'Standard (~500 pts)';
    });
  }

  if (btnObstacle) {
    btnObstacle.addEventListener('click', () => {
      simulateObstacle = !simulateObstacle;
      btnObstacle.classList.toggle('active', simulateObstacle);
      btnObstacle.textContent = simulateObstacle ? 'Clear Obstacle (Safe)' : 'Simulate Obstacle (<0.40m)';
      
      const statusPill = document.getElementById('system-status-pill');
      if (statusPill) {
        if (simulateObstacle) {
          statusPill.style.borderColor = 'rgba(239, 68, 68, 0.4)';
          statusPill.style.background = 'rgba(239, 68, 68, 0.12)';
          statusPill.querySelector('.status-dot').style.background = '#ef4444';
          statusPill.querySelector('.status-dot').style.boxShadow = '0 0 10px #ef4444';
          statusPill.querySelector('.status-label').textContent = 'SAFETY STOP ENGAGED';
          statusPill.querySelector('.status-label').style.color = '#ef4444';
        } else {
          statusPill.style.borderColor = 'rgba(0, 255, 136, 0.25)';
          statusPill.style.background = 'rgba(0, 255, 136, 0.08)';
          statusPill.querySelector('.status-dot').style.background = '#00ff88';
          statusPill.querySelector('.status-dot').style.boxShadow = '0 0 10px #00ff88';
          statusPill.querySelector('.status-label').textContent = 'HARDWARE NOMINAL';
          statusPill.querySelector('.status-label').style.color = '#00ff88';
        }
      }
    });
  }

  // Pixels per meter scale
  const scale = 75; // 75px = 1 meter

  function drawRadar() {
    ctx.clearRect(0, 0, width, height);

    // 1. Grid Rings
    ctx.lineWidth = 1;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
    const rings = [0.5, 1.0, 1.5, 2.0, 2.5];
    rings.forEach(r => {
      ctx.beginPath();
      ctx.arc(centerX, centerY, r * scale, 0, Math.PI * 2);
      ctx.stroke();
    });

    // 2. Crosshairs
    ctx.strokeStyle = 'rgba(0, 242, 254, 0.15)';
    ctx.beginPath();
    ctx.moveTo(centerX, 20);
    ctx.lineTo(centerX, height - 20);
    ctx.moveTo(20, centerY);
    ctx.lineTo(width - 20, centerY);
    ctx.stroke();

    // 3. Safety Stop Envelope Ring (0.40 m)
    ctx.strokeStyle = simulateObstacle ? 'rgba(239, 68, 68, 0.8)' : 'rgba(255, 152, 0, 0.4)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.arc(centerX, centerY, 0.40 * scale, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);

    // 4. Chassis Envelope Ring (0.28 m)
    ctx.fillStyle = 'rgba(12, 21, 36, 0.85)';
    ctx.strokeStyle = 'rgba(0, 242, 254, 0.5)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(centerX, centerY, 0.28 * scale, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Draw wheels on chassis
    ctx.fillStyle = '#ff9800';
    // Left wheel (at x = -0.33/2 = -0.165m)
    let wheelW = 6;
    let wheelH = 0.1007 * scale; // 100.7mm
    let wheelOffset = (0.33 / 2) * scale;
    ctx.fillRect(centerX - wheelOffset - (wheelW/2), centerY - (wheelH/2), wheelW, wheelH);
    // Right wheel
    ctx.fillRect(centerX + wheelOffset - (wheelW/2), centerY - (wheelH/2), wheelW, wheelH);

    // Heading arrow (forward is Up = -Y)
    ctx.fillStyle = '#00f2fe';
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - 14);
    ctx.lineTo(centerX - 6, centerY - 2);
    ctx.lineTo(centerX + 6, centerY - 2);
    ctx.closePath();
    ctx.fill();

    // 5. Sweep Ray Fan
    const sweepLength = 2.4 * scale;
    const sweepEndAngle = sweepAngle;
    const sweepStartAngle = sweepAngle - (mode === 'dense' ? 0.35 : 0.2);

    const grad = ctx.createRadialGradient(centerX, centerY, 10, centerX, centerY, sweepLength);
    grad.addColorStop(0, 'rgba(0, 242, 254, 0.35)');
    grad.addColorStop(1, 'rgba(0, 242, 254, 0)');

    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.arc(centerX, centerY, sweepLength, sweepStartAngle, sweepEndAngle);
    ctx.closePath();
    ctx.fill();

    // 6. Draw Simulated Point Cloud
    const numPoints = (mode === 'dense') ? 220 : 60;
    const step = 360 / numPoints;

    for (let deg = 0; deg < 360; deg += step) {
      let rad = (deg * Math.PI) / 180;
      let dist = baseRanges[Math.floor(deg) % 360];

      // Insert obstacle in front if activated
      if (simulateObstacle && deg > 250 && deg < 290) {
        dist = obstacleDistance; // inside safety buffer!
      }

      let px = centerX + Math.cos(rad) * dist * scale;
      let py = centerY + Math.sin(rad) * dist * scale;

      // Point color
      if (dist <= 0.40) {
        ctx.fillStyle = '#ef4444'; // Safety violation point
      } else {
        ctx.fillStyle = (mode === 'dense') ? '#00f2fe' : '#4facfe';
      }

      ctx.beginPath();
      ctx.arc(px, py, (mode === 'dense') ? 1.5 : 2.5, 0, Math.PI * 2);
      ctx.fill();
    }

    // Safety Alert Visual on Radar
    if (simulateObstacle) {
      ctx.fillStyle = 'rgba(239, 68, 68, 0.9)';
      ctx.font = '700 11px JetBrains Mono';
      ctx.textAlign = 'center';
      ctx.fillText('SAFETY STOP: OBSTACLE 0.35m', centerX, centerY + (0.55 * scale));
    }

    // Advance sweep angle (10 Hz rotation effect)
    sweepAngle += 0.045;
    if (sweepAngle > Math.PI * 2) sweepAngle = 0;

    requestAnimationFrame(drawRadar);
  }

  requestAnimationFrame(drawRadar);
}

/* ==========================================================================
   2. INTERACTIVE RIG EXPLORER (HOTSPOT MODULE VIEWER)
   ========================================================================== */
const RIG_MODULE_DATA = {
  'left-motor': {
    chip: 'POWERTRAIN // LEFT ACTUATOR',
    title: 'Left DDSM115 Direct-Drive Smart Motor',
    desc: 'Brushless smart hub motor integrated into the left 100.7mm drive wheel. Communicates over half-duplex RS485 using ID 0x01. Powered directly from the main battery bus (12V-24V).',
    specs: [
      { k: 'Bus Address', v: '0x01 (Left Wheel Joint)' },
      { k: 'Port / Protocol', v: 'RS485 half-duplex → /dev/ttyACM0 (115200)' },
      { k: 'Wheel Geometry', v: '⌀100.7 mm, 0.33 m track center' },
      { k: 'Stall Guard', v: 'Silent stall below 5-6 RPM (requires feedforward)' },
      { k: 'Topic Flow', v: 'Subscribes /cmd_vel_safe, Publishes /encoder' }
    ]
  },
  'right-motor': {
    chip: 'POWERTRAIN // RIGHT ACTUATOR',
    title: 'Right DDSM115 Direct-Drive Smart Motor',
    desc: 'Brushless smart hub motor integrated into the right 100.7mm drive wheel. Communicates over half-duplex RS485 using ID 0x02 with a 2ms guard timing offset to eliminate bus collisions.',
    specs: [
      { k: 'Bus Address', v: '0x02 (Right Wheel Joint)' },
      { k: 'Port / Protocol', v: 'RS485 half-duplex → /dev/ttyACM0 (115200)' },
      { k: 'Wheel Geometry', v: '⌀100.7 mm, 0.33 m track center' },
      { k: 'Feedback Query', v: '0x74 binary query packet returns RPM, current, pos' },
      { k: 'Topic Flow', v: 'Subscribes /cmd_vel_safe, Publishes /encoder' }
    ]
  },
  'rpi5': {
    chip: 'CENTRAL SBC // COMPUTE CORE',
    title: 'Raspberry Pi 5 Single Board Computer',
    desc: 'Quad-Core 2.4 GHz Broadcom BCM2712 processor running Ubuntu 24.04 and ROS 2 Jazzy Jalisco. Orchestrates SLAM Toolbox, Nav2, EKF sensor fusion, and the onboard web dashboard.',
    specs: [
      { k: 'Processor', v: 'Broadcom BCM2712 Quad Cortex-A76 @ 2.4GHz' },
      { k: 'RAM & Storage', v: '8 GB LPDDR4X-4267, NVMe PCIe Gen2 SSD' },
      { k: 'OS / ROS', v: 'Ubuntu 24.04 LTS (64-bit) / ROS 2 Jazzy' },
      { k: 'Power Rail', v: '5.0V / 5.0A DC via dedicated high-current buck' },
      { k: 'Thermal Alert', v: 'Active cooler with dynamic PWM fan curve' }
    ]
  },
  'rplidar': {
    chip: 'PRIMARY PERCEPTION // ETHERNET LIDAR',
    title: 'Slamtec RPLidar S2E (DenseBoost Ethernet)',
    desc: 'Industrial-grade 2D LiDAR connected via Gigabit Ethernet UDP (port 8089). Delivers ~3200 points per revolution at 10 Hz without consuming USB serial bus bandwidth.',
    specs: [
      { k: 'Physical Link', v: 'RJ45 Ethernet → eth0 (192.168.11.1/24)' },
      { k: 'Target Address', v: '192.168.11.2:8089 (UDP Socket)' },
      { k: 'Sample Density', v: 'DenseBoost mode (~3200 sample rays/rev @ 10Hz)' },
      { k: 'Chassis Filter', v: '0.28m chassis radius blind-zone filter' },
      { k: 'Topic Flow', v: 'Publishes /scan (sensor_msgs/LaserScan)' }
    ]
  },
  'bno055': {
    chip: 'INERTIAL FUSION // HIGH-SPEED IMU',
    title: 'Bosch BNO055 IMU + Arduino Mega Bridge',
    desc: '9-DOF intelligent sensor with internal fusion coprocessor. Connected to an Arduino Mega 2560 that samples quaternions and angular velocity, streaming to Pi at 500,000 baud.',
    specs: [
      { k: 'Serial Port', v: '/dev/ttyACM1 (500,000 baud, 8N1)' },
      { k: 'Sensors', v: 'Triaxial 14-bit Accel, 16-bit Gyro, Geomagnetic' },
      { k: 'Publish Rate', v: '50 Hz jitter-free stream to /imu' },
      { k: 'Sensor Fusion', v: 'Fuses with /odom_raw in ekf.yaml' },
      { k: 'Calibration', v: 'On-chip auto-calibration (Sys/G/A/M : 3/3/3/3)' }
    ]
  },
  'camera-head': {
    chip: 'VISION & ALARM // ACTIVE 2-DOF GIMBAL',
    title: 'Camera Head (Pan/Tilt) & Proximity Buzzer',
    desc: 'Autonomous pan/tilt tracking head driven by an Arduino Uno over UART (/dev/ttyUSB0). Houses Sony IMX708 Camera Module 3 and a hardware PWM proximity buzzer.',
    specs: [
      { k: 'Pan Servo', v: 'OT5320M 20kg Continuous (Uno Pin D6, 7.4V ext)' },
      { k: 'Tilt Servo', v: 'SG90 Micro Positional (Uno Pin D5, 5V ext)' },
      { k: 'Acoustic Alert', v: 'Piezo Buzzer on Pin D9 (Timer 1 Hardware PWM)' },
      { k: 'Optical Sensor', v: 'RPi Camera Module 3 (Sony IMX708, 66° HFOV)' },
      { k: 'Command Link', v: '115,200 baud ASCII (e.g. P1500, T1400, B120)' }
    ]
  }
};

function initRigExplorer() {
  const hotspots = document.querySelectorAll('.hotspot-group');
  const chipEl = document.getElementById('detailModuleChip');
  const titleEl = document.getElementById('detailModuleTitle');
  const descEl = document.getElementById('detailModuleDesc');
  const specsEl = document.getElementById('detailModuleSpecs');

  if (!hotspots.length || !titleEl) return;

  function loadModule(modKey) {
    const data = RIG_MODULE_DATA[modKey];
    if (!data) return;

    chipEl.textContent = data.chip;
    titleEl.textContent = data.title;
    descEl.textContent = data.desc;

    let specsHtml = '';
    data.specs.forEach(s => {
      specsHtml += `
        <div class="spec-row">
          <span class="spec-k">${s.k}</span>
          <span class="spec-v code-font">${s.v}</span>
        </div>
      `;
    });
    specsEl.innerHTML = specsHtml;
  }

  hotspots.forEach(hs => {
    hs.addEventListener('click', () => {
      const modKey = hs.getAttribute('data-module');
      loadModule(modKey);
    });

    hs.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' || e.key === ' ') {
        e.preventDefault();
        const modKey = hs.getAttribute('data-module');
        loadModule(modKey);
      }
    });
  });
}

/* ==========================================================================
   3. SUBSYSTEM TABS
   ========================================================================== */
function initSubsystemTabs() {
  const tabs = document.querySelectorAll('.tab-btn');
  const contents = document.querySelectorAll('.tab-content');

  tabs.forEach(tab => {
    tab.addEventListener('click', () => {
      tabs.forEach(t => t.classList.remove('active'));
      contents.forEach(c => c.classList.remove('active'));

      tab.classList.add('active');
      const targetId = tab.getAttribute('data-tab');
      const targetContent = document.getElementById(targetId);
      if (targetContent) targetContent.classList.add('active');
    });
  });
}

/* ==========================================================================
   4. DDSM115 10-BYTE BINARY PACKET INSPECTOR & CRC-8/MAXIM
   ========================================================================== */
/**
 * Exact CRC-8/MAXIM implementation matching driver_node.py:
 * Polynomial = 0x8C reversed, Init = 0x00.
 */
function crc8Maxim(byteArray) {
  let crc = 0x00;
  for (let i = 0; i < byteArray.length; i++) {
    crc ^= byteArray[i];
    for (let j = 0; j < 8; j++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0x8C;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc & 0xFF;
}

function initPacketInspector() {
  const motorIdSelect = document.getElementById('packetMotorId');
  const rpmSlider = document.getElementById('packetRpmInput');
  const rpmDisplay = document.getElementById('rpmDisplayVal');
  const rampInput = document.getElementById('packetRamp');
  const hexDisplay = document.getElementById('fullPacketHexDisplay');
  const crcDisplay = document.getElementById('crcDisplayHex');
  const btnCopy = document.getElementById('btnCopyHex');

  if (!motorIdSelect || !rpmSlider) return;

  function updatePacket() {
    const motorId = parseInt(motorIdSelect.value, 10);
    const rpm = parseInt(rpmSlider.value, 10);
    const ramp = parseInt(rampInput.value, 10) || 0;

    rpmDisplay.textContent = `${rpm} RPM`;

    // Convert signed 16-bit RPM to Big-Endian bytes
    let clampedRpm = Math.max(-32768, Math.min(32767, rpm));
    let rawRpm = (clampedRpm < 0) ? (0x10000 + clampedRpm) : clampedRpm;
    let rpmHigh = (rawRpm >> 8) & 0xFF;
    let rpmLow = rawRpm & 0xFF;

    // First 9 bytes of payload
    const payload9 = [
      motorId,       // Byte 0: Motor ID (0x01 or 0x02)
      0x64,          // Byte 1: Drive Command code
      rpmHigh,       // Byte 2: RPM High byte
      rpmLow,        // Byte 3: RPM Low byte
      0x00,          // Byte 4: Reserved
      0x00,          // Byte 5: Reserved
      0x00,          // Byte 6: Reserved
      0x00,          // Byte 7: Brake flag (0xFF for active brake)
      ramp & 0xFF    // Byte 8: Acceleration/Deceleration ramp
    ];

    // Compute CRC-8/MAXIM check byte
    const crc = crc8Maxim(payload9);
    const full10 = [...payload9, crc];

    // Update Byte Box elements
    for (let i = 0; i < 9; i++) {
      const bEl = document.getElementById(`b${i}`);
      if (bEl) {
        bEl.querySelector('.b-val').textContent = '0x' + full10[i].toString(16).padStart(2, '0').toUpperCase();
      }
    }

    if (crcDisplay) {
      crcDisplay.textContent = '0x' + crc.toString(16).padStart(2, '0').toUpperCase();
    }

    // Format full string
    const hexStr = full10.map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
    if (hexDisplay) hexDisplay.textContent = hexStr;
  }

  motorIdSelect.addEventListener('change', updatePacket);
  rpmSlider.addEventListener('input', updatePacket);
  rampInput.addEventListener('input', updatePacket);

  if (btnCopy) {
    btnCopy.addEventListener('click', () => {
      const text = hexDisplay.textContent;
      navigator.clipboard.writeText(text).then(() => {
        btnCopy.textContent = 'Copied!';
        setTimeout(() => { btnCopy.textContent = 'Copy Hex'; }, 2000);
      });
    });
  }

  updatePacket();
}

/* ==========================================================================
   5. CAMERA HEAD & BUZZER TESTER (ASCII + WEB AUDIO API)
   ========================================================================== */
function initGimbalTester() {
  const panSlider = document.getElementById('sliderPanUs');
  const tiltSlider = document.getElementById('sliderTiltUs');
  const buzzerSlider = document.getElementById('sliderBuzzer');
  const panVal = document.getElementById('valPanUs');
  const tiltVal = document.getElementById('valTiltUs');
  const buzzerVal = document.getElementById('valBuzzerDuty');
  const outputCode = document.getElementById('asciiStreamOutput');
  const buzzerLabel = document.getElementById('buzzerStatusLabel');
  const btnBeep = document.getElementById('btnPlayBeep');

  if (!panSlider || !outputCode) return;

  function updateGimbalStream() {
    const pan = panSlider.value;
    const tilt = tiltSlider.value;
    const buzzer = buzzerSlider.value;

    panVal.textContent = `${pan} µs`;
    tiltVal.textContent = `${tilt} µs`;
    buzzerVal.textContent = (buzzer == 0) ? '0 (Silent)' : `${buzzer} / 255`;

    outputCode.textContent = `P${pan}\\nT${tilt}\\nB${buzzer}\\n`;

    if (buzzer > 0) {
      // Approximate mapped obstacle distance from duty cycle
      // 0 = >=45cm, 255 = <=30cm
      const dist = (45 - (buzzer / 255) * 15).toFixed(1);
      buzzerLabel.textContent = `ALERT: Obstacle detected at ~${dist} cm (Duty ${buzzer})`;
      buzzerLabel.style.color = '#ff9800';
    } else {
      buzzerLabel.textContent = 'BUZZER SILENT (Distance > 45cm)';
      buzzerLabel.style.color = '#94a3b8';
    }
  }

  panSlider.addEventListener('input', updateGimbalStream);
  tiltSlider.addEventListener('input', updateGimbalStream);
  buzzerSlider.addEventListener('input', updateGimbalStream);

  // Web Audio API simulated buzzer preview
  if (btnBeep) {
    btnBeep.addEventListener('click', () => {
      try {
        const audioCtx = new (window.AudioContext || window.webkitAudioContext)();
        const osc = audioCtx.createOscillator();
        const gain = audioCtx.createGain();

        const duty = parseInt(buzzerSlider.value, 10);
        const volume = (duty === 0) ? 0.3 : Math.min(1.0, duty / 255);

        osc.type = 'square'; // Piezo transducer sound
        osc.frequency.setValueAtTime(2700, audioCtx.currentTime); // 2.7 kHz resonant pitch

        gain.gain.setValueAtTime(volume * 0.25, audioCtx.currentTime);
        gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + 0.25);

        osc.connect(gain);
        gain.connect(audioCtx.destination);

        osc.start();
        osc.stop(audioCtx.currentTime + 0.25);
      } catch (err) {
        console.warn('Audio preview error:', err);
      }
    });
  }

  updateGimbalStream();
}

/* ==========================================================================
   6. DIFFERENTIAL DRIVE KINEMATICS CALCULATOR
   ========================================================================== */
function initKinematicsCalculator() {
  const vxInput = document.getElementById('calcVx');
  const wzInput = document.getElementById('calcWz');
  const leftRpmEl = document.getElementById('resLeftRpm');
  const rightRpmEl = document.getElementById('resRightRpm');
  const leftSpeedEl = document.getElementById('resLeftSpeed');
  const rightSpeedEl = document.getElementById('resRightSpeed');
  const warningEl = document.getElementById('deadzoneWarning');

  if (!vxInput || !wzInput) return;

  const WHEEL_RADIUS = 0.05035; // 100.7mm dia / 2
  const WHEEL_BASE = 0.33;      // 330mm

  function calculateKinematics() {
    const vx = parseFloat(vxInput.value) || 0;
    const wz = parseFloat(wzInput.value) || 0;

    // Differential kinematics equations:
    // v_l = vx - (wz * wheelbase / 2)
    // v_r = vx + (wz * wheelbase / 2)
    const vLeft = vx - (wz * WHEEL_BASE / 2.0);
    const vRight = vx + (wz * WHEEL_BASE / 2.0);

    // RPM = (v / (2 * PI * r)) * 60
    const rpmLeft = (vLeft / (2 * Math.PI * WHEEL_RADIUS)) * 60.0;
    const rpmRight = (vRight / (2 * Math.PI * WHEEL_RADIUS)) * 60.0;

    leftRpmEl.textContent = `${rpmLeft.toFixed(1)} RPM`;
    rightRpmEl.textContent = `${rpmRight.toFixed(1)} RPM`;
    leftSpeedEl.textContent = `${vLeft.toFixed(3)} m/s`;
    rightSpeedEl.textContent = `${vRight.toFixed(3)} m/s`;

    // Deadzone warning: DDSM115 motors stall under load when RPM is between -6 and +6 (and not zero)
    const absLeft = Math.abs(rpmLeft);
    const absRight = Math.abs(rpmRight);
    const inDeadzone = (absLeft > 0.05 && absLeft < 6.0) || (absRight > 0.05 && absRight < 6.0);

    if (warningEl) {
      warningEl.style.display = inDeadzone ? 'block' : 'none';
    }
  }

  vxInput.addEventListener('input', calculateKinematics);
  wzInput.addEventListener('input', calculateKinematics);

  calculateKinematics();
}

/* ==========================================================================
   7. BATTERY ENDURANCE & POWER CALCULATOR
   ========================================================================== */
function initPowerCalculator() {
  const voltSelect = document.getElementById('batteryVoltage');
  const capInput = document.getElementById('batteryCapacity');
  const modeSelect = document.getElementById('robotMissionMode');
  const powerEl = document.getElementById('resPowerDraw');
  const runtimeEl = document.getElementById('resRuntime');
  const currentEl = document.getElementById('resCurrentDraw');

  if (!voltSelect || !capInput) return;

  function calculatePower() {
    const voltage = parseFloat(voltSelect.value) || 14.8;
    const capacityMah = parseFloat(capInput.value) || 5000;
    const mode = modeSelect.value;

    // Power draw in Watts:
    // Base Pi 5 + Fans + Microcontrollers + S2E LiDAR ≈ 17W
    let totalWatts = 17.0;

    if (mode === 'active_mapping') {
      // Adding continuous driving at ~0.3 m/s (12W motors)
      totalWatts += 11.5;
    } else if (mode === 'heavy_vision') {
      // Adding YOLO-World neural inference on Pi 5 + pan/tilt servos + driving
      totalWatts += 18.0;
    } else if (mode === 'idle_bench') {
      // SBC idle + LiDAR spinning only
      totalWatts = 13.5;
    }

    // Usable capacity considering 80% discharge safe cutoff
    const energyWh = (capacityMah / 1000.0) * voltage * 0.80;
    const hours = energyWh / totalWatts;
    const currentA = totalWatts / voltage;

    powerEl.textContent = `${totalWatts.toFixed(1)} W`;
    runtimeEl.textContent = `${hours.toFixed(1)} Hours (~${Math.round(hours * 60)} mins)`;
    currentEl.textContent = `${currentA.toFixed(2)} A @ ${voltage}V`;
  }

  voltSelect.addEventListener('change', calculatePower);
  capInput.addEventListener('input', calculatePower);
  modeSelect.addEventListener('change', calculatePower);

  calculatePower();
}

/* ==========================================================================
   8. BOM FILTERING & CSV EXPORT
   ========================================================================== */
function initBomTable() {
  const searchInput = document.getElementById('bomSearchInput');
  const filterChips = document.querySelectorAll('.filter-chip');
  const table = document.getElementById('bomTable');
  const btnExport = document.getElementById('btnExportBom');

  if (!table) return;

  let activeCategory = 'all';

  function applyFilter() {
    const query = (searchInput ? searchInput.value.toLowerCase() : '').trim();
    const rows = table.querySelectorAll('tbody tr');

    rows.forEach(row => {
      const category = row.getAttribute('data-category');
      const text = row.textContent.toLowerCase();

      const matchCat = (activeCategory === 'all' || category === activeCategory);
      const matchQuery = (query === '' || text.includes(query));

      row.style.display = (matchCat && matchQuery) ? '' : 'none';
    });
  }

  filterChips.forEach(chip => {
    chip.addEventListener('click', () => {
      filterChips.forEach(c => c.classList.remove('active'));
      chip.classList.add('active');
      activeCategory = chip.getAttribute('data-category');
      applyFilter();
    });
  });

  if (searchInput) {
    searchInput.addEventListener('input', applyFilter);
  }

  // Export visible rows to CSV
  if (btnExport) {
    btnExport.addEventListener('click', () => {
      const rows = table.querySelectorAll('tr');
      let csvContent = 'data:text/csv;charset=utf-8,';

      rows.forEach(row => {
        if (row.style.display !== 'none') {
          const cols = row.querySelectorAll('th, td');
          const rowData = [];
          cols.forEach(col => {
            let cellText = col.innerText.replace(/"/g, '""').replace(/\n/g, ' ');
            rowData.push(`"${cellText}"`);
          });
          csvContent += rowData.join(',') + '\r\n';
        }
      });

      const encodedUri = encodeURI(csvContent);
      const link = document.createElement('a');
      link.setAttribute('href', encodedUri);
      link.setAttribute('download', 'Ros_lidar_bot_Hardware_BOM.csv');
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
    });
  }
}

/* ==========================================================================
   9. ACCORDION (DIAGNOSTICS)
   ========================================================================== */
function initAccordion() {
  const items = document.querySelectorAll('.accordion-item');

  items.forEach(item => {
    const header = item.querySelector('.accordion-header');
    if (header) {
      header.addEventListener('click', () => {
        const isOpen = item.classList.contains('open');
        items.forEach(i => i.classList.remove('open'));
        if (!isOpen) item.classList.add('open');
      });
    }
  });
}

/* ==========================================================================
   10. COPY BUTTONS FEEDBACK
   ========================================================================== */
function initCopyButtons() {
  // Command buttons
  document.querySelectorAll('.btn-copy-cmd').forEach(btn => {
    btn.addEventListener('click', () => {
      const cmd = btn.getAttribute('data-cmd');
      if (cmd) {
        navigator.clipboard.writeText(cmd).then(() => {
          const original = btn.textContent;
          btn.textContent = 'Copied!';
          setTimeout(() => { btn.textContent = original; }, 2000);
        });
      }
    });
  });

  // Udev block
  const btnUdev = document.getElementById('btnCopyUdev');
  if (btnUdev) {
    btnUdev.addEventListener('click', () => {
      const code = document.getElementById('udevCodeBlock');
      if (code) {
        navigator.clipboard.writeText(code.innerText).then(() => {
          btnUdev.textContent = 'Copied!';
          setTimeout(() => { btnUdev.textContent = 'Copy'; }, 2000);
        });
      }
    });
  }
}
