const DEVICE_NAME = "eSag_Sensor";
const SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const FREEFALL_G = 0.25;
const LIVE_WINDOW_MS = 400;
const RATE_WINDOW_MS = 1000;
const CALIBRATION_HOLD_MS = 2200;
const RANGE_NOTICE_MS = 2000;
const MAG_RANGE_KEY = "esag_mag_range";
const BIKE_MODEL_KEY = "esag_bike_model";
const SENSOR_DIR_KEY = "esag_sensor_dir";

const CHARACTERISTICS = {
  avg10: "beb5483e-36e1-4688-b7f5-ea07361b26a8",
  avg50: "beb5483e-36e1-4688-b7f5-ea07361b26a9",
  avg100: "beb5483e-36e1-4688-b7f5-ea07361b26aa",
  imu: "beb5483e-36e1-4688-b7f5-ea07361b26ab",
  command: "beb5483e-36e1-4688-b7f5-ea07361b26ac",
};

const OTA_SERVICE_UUID = "fb1e4001-54ae-4a28-9f74-dfccb248601d";
const OTA_CONTROL_UUID = "fb1e4002-54ae-4a28-9f74-dfccb248601d";
const OTA_DATA_UUID = "fb1e4003-54ae-4a28-9f74-dfccb248601d";
const OTA_CHUNK_SIZE = 240;
const OTA_READY_TIMEOUT_MS = 5000;
const OTA_OK_TIMEOUT_MS = 30000;

const textDecoder = new TextDecoder("utf-8");
const textEncoder = new TextEncoder();

const connectBtn = document.getElementById("connect-btn");
const statusEl = document.getElementById("status");
const zeroBtn = document.getElementById("zero-btn");
const calibrateBtn = document.getElementById("calibrate-btn");
const resetPeakBtn = document.getElementById("reset-peak-btn");
const calNoticeEl = document.getElementById("cal-notice");
const sagPercentEl = document.getElementById("sag-percent");
const sagMmEl = document.getElementById("sag-mm");
const travelBarEl = document.getElementById("travel-bar");
const travelFillEl = document.getElementById("travel-fill");
const peakValueEl = document.getElementById("peak-value");
const shockLengthEl = document.getElementById("shock-length");
const deltaThetaEl = document.getElementById("delta-theta");
const relAngleEl = document.getElementById("rel-angle");
const absAngleEl = document.getElementById("abs-angle");
const pitchEl = document.getElementById("pitch");
const pitchGradeEl = document.getElementById("pitch-grade");
const rollEl = document.getElementById("roll");
const rollSideEl = document.getElementById("roll-side");
const gyroZEl = document.getElementById("gyro-z");
const accMagEl = document.getElementById("acc-mag");
const accMetricEl = document.getElementById("acc-metric");
const accFlagEl = document.getElementById("acc-flag");
const packetHzEl = document.getElementById("packet-hz");
const liveDotEl = document.getElementById("live-dot");
const bikeModelSelect = document.getElementById("bike-model-select");
const sensorDirSelect = document.getElementById("sensor-dir-select");
const otaFileInput = document.getElementById("ota-file-input");
const otaUploadBtn = document.getElementById("ota-upload-btn");
const otaProgressEl = document.getElementById("ota-progress");
const otaProgressFillEl = document.getElementById("ota-progress-fill");
const otaStatusEl = document.getElementById("ota-status");
const svgBikePitchGroup = document.getElementById("svg-bike-pitch-group");
const svgRearTriangle = document.getElementById("svg-rear-triangle");
const svgShockPiston = document.getElementById("svg-shock-piston");
const svgShockEye = document.getElementById("svg-shock-eye");
const svgBikeRearView = document.getElementById("svg-bike-rear-view");
const svgHudSag = document.getElementById("svg-hud-sag");
const svgHudMm = document.getElementById("svg-hud-mm");
const svgHudPitch = document.getElementById("svg-hud-pitch");
const svgHudRoll = document.getElementById("svg-hud-roll");
const svgHudRollSide = document.getElementById("svg-hud-roll-side");
const windowButtons = Array.from(document.querySelectorAll("[data-window]"));
const rangeButtons = Array.from(document.querySelectorAll("[data-range]"));

/** @type {BluetoothDevice | null} */
let bluetoothDevice = null;

/** @type {BluetoothRemoteGATTServer | null} */
let gattServer = null;

/** @type {BluetoothRemoteGATTCharacteristic | null} */
let commandCharacteristic = null;

/** @type {BluetoothRemoteGATTCharacteristic | null} */
let otaControlCharacteristic = null;

/** @type {BluetoothRemoteGATTCharacteristic | null} */
let otaDataCharacteristic = null;

/** @type {WakeLockSentinel | null} */
let screenWakeLock = null;

/** @type {{ characteristic: BluetoothRemoteGATTCharacteristic, handler: (event: Event) => void }[]} */
let activeSubscriptions = [];

let connectInFlight = false;
let averageWindow = 50;
let wantWakeLock = false;
let wakeLockEpoch = 0;
let imuPacketCount = 0;
let rateWindowStart = performance.now();
let displayHz = 0;
let lastImuAt = 0;
let calibrateTimer = 0;
let rangeNoticeTimer = 0;
let otaSupported = false;
let otaUploading = false;
let otaRun = 0;
let otaStatusLocked = false;

/** @type {{ expect: string, resolve: (value: string) => void, reject: (error: Error) => void } | null} */
let otaWaiter = null;
let peakTravelMm = 0;
let peakPercent = 0;
let bikeModelKey = loadBikeModel();
let sensorDirection = loadSensorDirection();

const painted = {
  sagPercent: "",
  sagMm: "",
  peak: "",
  shock: "",
  delta: "",
  rel: "",
  abs: "",
  pitch: "",
  grade: "",
  roll: "",
  side: "",
  sideMuted: true,
  gyroZ: "",
  accMag: "",
  hz: "",
  travel: "",
  band: "",
  live: false,
  freefall: false,
  hudSag: "",
  hudMm: "",
  hudPitch: "",
  hudRoll: "",
  hudSide: "",
};

const visualPose = {
  pitch: "",
  rear: "",
  shock: "",
  shockColor: "",
  roll: "",
};

const SHOCK_TIP_EXTENDED = 71;
const SHOCK_TIP_COMPRESSED = 36;
const SHOCK_COLORS = {
  low: "#3b82f6",
  optimal: "#00c853",
  warn: "#ffb300",
  bottom: "#ff5252",
};

const warnedFrames = new Set();

window.telemetryState = {
  avg10: { abs: null, rel: null },
  avg50: { abs: null, rel: null },
  avg100: { abs: null, rel: null },
  imu: { pitch: null, roll: null, gyroZ: null, accMag: null },
};

applyBikeSettings();
applyMagneticRangeButtons(loadMagneticRange());

connectBtn.addEventListener("click", onConnectClick);
zeroBtn.addEventListener("click", onZeroClick);
calibrateBtn.addEventListener("click", onCalibrateClick);
resetPeakBtn.addEventListener("click", onResetPeakClick);
document.addEventListener("visibilitychange", onVisibilityChange);

for (const button of windowButtons) {
  button.addEventListener("click", onAverageWindowClick);
}

for (const button of rangeButtons) {
  button.addEventListener("click", onMagneticRangeClick);
}

bikeModelSelect.addEventListener("change", onBikeModelChange);
sensorDirSelect.addEventListener("change", onSensorDirectionChange);

otaFileInput.addEventListener("change", onOtaFileChange);
otaUploadBtn.addEventListener("click", onOtaUploadClick);

requestAnimationFrame(renderFrame);

async function onConnectClick() {
  if (connectInFlight) {
    return;
  }

  if (bluetoothDevice?.gatt?.connected) {
    disconnect();
    return;
  }

  await connect();
}

function onAverageWindowClick(event) {
  const button = event.currentTarget;
  averageWindow = Number(button.dataset.window);

  for (const candidate of windowButtons) {
    candidate.setAttribute("aria-pressed", candidate === button ? "true" : "false");
  }
}

function onMagneticRangeClick(event) {
  setMagneticRange(event.currentTarget.dataset.range);
}

function onBikeModelChange() {
  const next = bikeModelSelect.value;
  bikeModelKey = window.BIKE_KINEMATICS[next] ? next : "WILD_S";
  saveBikeSettings();
  resetPeak();
}

function onSensorDirectionChange() {
  sensorDirection = Number(sensorDirSelect.value) === -1 ? -1 : 1;
  saveBikeSettings();
  resetPeak();
}

async function onZeroClick() {
  if (!commandCharacteristic || zeroBtn.disabled) {
    return;
  }

  zeroBtn.disabled = true;
  try {
    await sendCommand("R");
    resetPeak();
    console.log("[eSAG] Cero SAG enviado y pico reiniciado.");
  } catch (error) {
    console.error("[eSAG] Error al fijar el cero SAG:", error);
  } finally {
    zeroBtn.disabled = !commandCharacteristic;
  }
}

async function onCalibrateClick() {
  if (!commandCharacteristic || calibrateBtn.disabled) {
    return;
  }

  calibrateBtn.disabled = true;
  try {
    await sendCommand("C");
  } catch (error) {
    console.error("[eSAG] Error al calibrar el IMU:", error);
    calibrateBtn.disabled = !commandCharacteristic;
    return;
  }

  calNoticeEl.hidden = false;
  window.clearTimeout(calibrateTimer);
  calibrateTimer = window.setTimeout(() => {
    calibrateTimer = 0;
    calNoticeEl.hidden = true;
    calibrateBtn.disabled = !commandCharacteristic;
  }, CALIBRATION_HOLD_MS);
}

function onResetPeakClick() {
  resetPeak();
  console.log("[eSAG] Pico máximo reiniciado en la web.");
}

function onVisibilityChange() {
  if (document.visibilityState === "visible" && wantWakeLock) {
    acquireWakeLock();
  }
}

async function connect() {
  if (!navigator.bluetooth) {
    console.error("[eSAG] Web Bluetooth no está disponible en este navegador o contexto.");
    setStatus("Bluetooth no disponible", "idle");
    return;
  }

  connectInFlight = true;
  setStatus("Conectando...", "busy");
  connectBtn.disabled = true;

  // Se pide en el mismo gesto que requestDevice. Si el usuario cancela, se libera.
  wantWakeLock = true;
  const wakeLockPromise = acquireWakeLock();

  try {
    console.log("[eSAG] Solicitando dispositivo.", {
      name: DEVICE_NAME,
      service: SERVICE_UUID,
    });

    const device = await navigator.bluetooth.requestDevice({
      filters: [
        {
          name: DEVICE_NAME,
          services: [SERVICE_UUID],
        },
      ],
      optionalServices: [OTA_SERVICE_UUID],
    });

    bluetoothDevice = device;
    device.addEventListener("gattserverdisconnected", onGattServerDisconnected);

    console.log("[eSAG] Dispositivo seleccionado:", device.name || DEVICE_NAME);
    console.log("[eSAG] Conectando al servidor GATT...");

    gattServer = await device.gatt.connect();
    console.log("[eSAG] Servidor GATT conectado.");

    console.log("[eSAG] Obteniendo servicio principal:", SERVICE_UUID);
    const service = await gattServer.getPrimaryService(SERVICE_UUID);
    console.log("[eSAG] Servicio principal obtenido:", service.uuid);

    resetTelemetry();
    await subscribeToTelemetry(service);
    await bindOtaService();
    await wakeLockPromise;

    if (rangeNoticeTimer) {
      statusEl.classList.add("is-connected");
      statusEl.classList.remove("is-busy");
    } else {
      setStatus("Conectado al ESP32-C3", "connected");
    }
    connectBtn.textContent = "Desconectar";
    connectBtn.classList.add("is-connected");
    connectBtn.setAttribute("aria-pressed", "true");
    setControlsEnabled(true);
  } catch (error) {
    if (error && error.name === "NotFoundError") {
      console.log("[eSAG] Selección de dispositivo cancelada.");
    } else {
      console.error("[eSAG] Error durante la conexión:", error);
    }

    wantWakeLock = false;
    await wakeLockPromise;

    if (bluetoothDevice?.gatt?.connected) {
      bluetoothDevice.gatt.disconnect();
    }

    cleanupDevice();
    setStatus("Desconectado", "idle");
    connectBtn.textContent = "Conectar Sensor";
    connectBtn.classList.remove("is-connected");
    connectBtn.setAttribute("aria-pressed", "false");
  } finally {
    connectInFlight = false;
    connectBtn.disabled = false;
  }
}

async function subscribeToTelemetry(service) {
  await subscribeAngle(service, "avg10", CHARACTERISTICS.avg10);
  await subscribeAngle(service, "avg50", CHARACTERISTICS.avg50);
  await subscribeAngle(service, "avg100", CHARACTERISTICS.avg100);
  await subscribeImu(service, CHARACTERISTICS.imu);
  await bindCommand(service);
}

async function subscribeAngle(service, key, uuid) {
  console.log("[eSAG] Obteniendo característica:", key, uuid);
  const characteristic = await service.getCharacteristic(uuid);
  const handler = (event) => onAngleNotification(key, event);
  characteristic.addEventListener("characteristicvaluechanged", handler);
  activeSubscriptions.push({ characteristic, handler });

  console.log("[eSAG] Activando notificaciones:", key);
  await characteristic.startNotifications();
  console.log("[eSAG] Notificaciones activas:", key, characteristic.uuid);
}

async function subscribeImu(service, uuid) {
  console.log("[eSAG] Obteniendo característica:", "imu", uuid);
  const characteristic = await service.getCharacteristic(uuid);
  characteristic.addEventListener("characteristicvaluechanged", onImuNotification);
  activeSubscriptions.push({ characteristic, handler: onImuNotification });

  console.log("[eSAG] Activando notificaciones:", "imu");
  await characteristic.startNotifications();
  console.log("[eSAG] Notificaciones activas:", "imu", characteristic.uuid);
}

async function bindCommand(service) {
  console.log("[eSAG] Obteniendo característica de comando:", CHARACTERISTICS.command);
  commandCharacteristic = await service.getCharacteristic(CHARACTERISTICS.command);
  console.log("[eSAG] Característica de comando lista:", commandCharacteristic.uuid);
  await setMagneticRange(loadMagneticRange());
}

async function bindOtaService() {
  otaSupported = false;
  otaControlCharacteristic = null;
  otaDataCharacteristic = null;
  let control = null;

  try {
    console.log("[eSAG] Obteniendo servicio OTA:", OTA_SERVICE_UUID);
    const service = await gattServer.getPrimaryService(OTA_SERVICE_UUID);
    control = await service.getCharacteristic(OTA_CONTROL_UUID);
    const data = await service.getCharacteristic(OTA_DATA_UUID);
    control.addEventListener("characteristicvaluechanged", onOtaControlNotification);
    activeSubscriptions.push({ characteristic: control, handler: onOtaControlNotification });
    console.log("[eSAG] Activando notificaciones OTA:", OTA_CONTROL_UUID);
    await control.startNotifications();
    otaControlCharacteristic = control;
    otaDataCharacteristic = data;
    otaSupported = true;
    otaStatusLocked = false;
    setOtaStatus("Esperando archivo...");
    console.log("[eSAG] Servicio OTA listo.");
  } catch (error) {
    console.warn("[eSAG] OTA no habilitado en este firmware:", error);
    if (control) {
      control.removeEventListener("characteristicvaluechanged", onOtaControlNotification);
    }
    activeSubscriptions = activeSubscriptions.filter((entry) => entry.handler !== onOtaControlNotification);
    otaControlCharacteristic = null;
    otaDataCharacteristic = null;
    otaSupported = false;
    setOtaStatus("OTA no habilitado en este firmware");
  }

  refreshOtaControls();
}

function onOtaFileChange() {
  otaStatusLocked = false;
  setOtaProgress(0, otaFileInput.files?.length ? "Esperando archivo..." : "Esperando archivo...");
  refreshOtaControls();
}

function onOtaUploadClick() {
  const file = otaFileInput.files?.[0];
  if (!file || otaUploadBtn.disabled) {
    return;
  }

  uploadFirmwareOTA(file);
}

function onOtaControlNotification(event) {
  if (!event.target?.value) {
    return;
  }

  const text = textDecoder.decode(event.target.value).replace(/\0/g, "").trim().toUpperCase();
  console.log("[eSAG] Notificación OTA:", text);
  if (!otaWaiter) {
    return;
  }

  if (text.startsWith("ERR") || text === otaWaiter.expect) {
    const waiter = otaWaiter;
    otaWaiter = null;
    if (text.startsWith("ERR")) {
      waiter.reject(new Error(text));
    } else {
      waiter.resolve(text);
    }
  }
}

async function uploadFirmwareOTA(file) {
  if (otaUploading || !otaSupported || !otaControlCharacteristic || !otaDataCharacteristic) {
    return;
  }

  if (!file.name.toLowerCase().endsWith(".bin")) {
    setOtaStatus("Selecciona un archivo .bin");
    return;
  }

  otaUploading = true;
  otaStatusLocked = false;
  const run = ++otaRun;
  refreshOtaControls();
  setOtaProgress(0, "Subiendo: 0% (0 / 0 KB)");

  try {
    const buffer = await file.arrayBuffer();
    if (run !== otaRun) {
      return;
    }
    if (buffer.byteLength === 0) {
      throw new Error("El archivo está vacío");
    }

    const totalKb = formatKb(buffer.byteLength);
    setOtaProgress(0, `Subiendo: 0% (0 / ${totalKb} KB)`);
    console.log("[eSAG] OTA BEGIN", buffer.byteLength);

    const readyPromise = waitForOtaSignal("READY", OTA_READY_TIMEOUT_MS);
    await writeCharacteristic(
      otaControlCharacteristic,
      textEncoder.encode(`BEGIN:${buffer.byteLength}`),
      true
    );
    await readyPromise;
    if (run !== otaRun) {
      return;
    }

    let offset = 0;
    let index = 0;
    while (offset < buffer.byteLength) {
      if (run !== otaRun || !bluetoothDevice?.gatt?.connected) {
        throw new Error("Sensor desconectado");
      }

      const length = Math.min(OTA_CHUNK_SIZE, buffer.byteLength - offset);
      const chunk = new Uint8Array(buffer, offset, length);
      const isLast = offset + length >= buffer.byteLength;
      await writeCharacteristic(otaDataCharacteristic, chunk, otaChunkNeedsResponse(index, isLast));
      offset += length;
      index += 1;

      const percent = Math.floor((offset / buffer.byteLength) * 100);
      setOtaProgress(percent, `Subiendo: ${percent}% (${formatKb(offset)} / ${totalKb} KB)`);
    }

    console.log("[eSAG] OTA END");
    const okPromise = waitForOtaSignal("OK", OTA_OK_TIMEOUT_MS);
    await writeCharacteristic(otaControlCharacteristic, textEncoder.encode("END"), true);
    await okPromise;

    otaStatusLocked = true;
    setOtaProgress(100, "¡Actualización completada! Reiniciando sensor...");
    console.log("[eSAG] OTA OK. El sensor va a reiniciarse.");
  } catch (error) {
    if (run !== otaRun && otaStatusLocked) {
      return;
    }
    console.error("[eSAG] Error OTA:", error);
    otaStatusLocked = false;
    const message = error && error.message ? error.message : "Fallo de escritura";
    setOtaStatus(`Error OTA: ${message}`);
  } finally {
    if (run === otaRun) {
      otaUploading = false;
      refreshOtaControls();
    }
  }
}

function otaChunkNeedsResponse(index, isLast) {
  // Grupos de 8 escrituras sin respuesta y la 9.ª con respuesta.
  // El último bloque también espera respuesta para vaciar la cola antes de END.
  return isLast || (index + 1) % 9 === 0;
}

function waitForOtaSignal(expect, timeoutMs) {
  if (otaWaiter) {
    otaWaiter.reject(new Error("Señal OTA sustituida"));
    otaWaiter = null;
  }

  return new Promise((resolve, reject) => {
    const timer = window.setTimeout(() => {
      if (otaWaiter && otaWaiter.expect === expect) {
        otaWaiter = null;
        reject(new Error(`Timeout esperando ${expect}`));
      }
    }, timeoutMs);

    otaWaiter = {
      expect,
      resolve: (value) => {
        window.clearTimeout(timer);
        resolve(value);
      },
      reject: (error) => {
        window.clearTimeout(timer);
        reject(error);
      },
    };
  });
}

async function writeCharacteristic(characteristic, value, withResponse) {
  if (withResponse || typeof characteristic.writeValueWithoutResponse !== "function") {
    if (typeof characteristic.writeValueWithResponse === "function") {
      await characteristic.writeValueWithResponse(value);
    } else {
      await characteristic.writeValue(value);
    }
    return;
  }

  try {
    await characteristic.writeValueWithoutResponse(value);
  } catch (error) {
    console.warn("[eSAG] Cola BLE llena, reintentando el bloque con respuesta:", error);
    if (typeof characteristic.writeValueWithResponse === "function") {
      await characteristic.writeValueWithResponse(value);
    } else {
      await characteristic.writeValue(value);
    }
  }
}

function setOtaProgress(percent, text) {
  const clamped = Math.min(100, Math.max(0, percent));
  otaProgressFillEl.style.width = `${clamped}%`;
  otaProgressEl.setAttribute("aria-valuenow", String(clamped));
  setOtaStatus(text);
}

function setOtaStatus(text) {
  otaStatusEl.textContent = text;
}

function formatKb(bytes) {
  return String(Math.round(bytes / 1024));
}

function refreshOtaControls() {
  const connected = Boolean(bluetoothDevice?.gatt?.connected);
  const hasFile = Boolean(otaFileInput.files?.length);
  otaFileInput.disabled = otaUploading || (connected && !otaSupported);
  otaUploadBtn.disabled = !(otaSupported && connected && hasFile && !otaUploading);
}

async function setMagneticRange(rangeValue) {
  const range = normalizeMagneticRange(rangeValue);
  applyMagneticRangeButtons(range);

  try {
    localStorage.setItem(MAG_RANGE_KEY, range);
  } catch (error) {
    console.warn("[eSAG] No se pudo guardar el rango magnético:", error);
  }

  if (!commandCharacteristic) {
    return;
  }

  const command = `RANGE:${range}`;
  try {
    await commandCharacteristic.writeValue(textEncoder.encode(command));
    console.log("[eSAG] Comando BLE enviado:", command);
    showRangeNotice(range);
  } catch (error) {
    console.error("[eSAG] Error al ajustar el rango magnético:", error);
  }
}

function showRangeNotice(range) {
  setStatus(`Rango ajustado a ±${range} mT`, "connected");
  window.clearTimeout(rangeNoticeTimer);
  rangeNoticeTimer = window.setTimeout(() => {
    rangeNoticeTimer = 0;
    if (bluetoothDevice?.gatt?.connected) {
      setStatus("Conectado al ESP32-C3", "connected");
    }
  }, RANGE_NOTICE_MS);
}

function applyMagneticRangeButtons(range) {
  const selected = normalizeMagneticRange(range);
  for (const button of rangeButtons) {
    button.setAttribute("aria-pressed", button.dataset.range === selected ? "true" : "false");
  }
}

function loadMagneticRange() {
  try {
    return normalizeMagneticRange(localStorage.getItem(MAG_RANGE_KEY));
  } catch (error) {
    console.warn("[eSAG] No se pudo leer el rango magnético:", error);
    return "50";
  }
}

function normalizeMagneticRange(rangeValue) {
  const range = String(rangeValue);
  if (range === "25" || range === "50" || range === "100") {
    return range;
  }
  return "50";
}

function onAngleNotification(key, event) {
  if (!event.target?.value) {
    return;
  }

  const text = textDecoder.decode(event.target.value);
  const values = parseCsv(text, 2);
  if (!values) {
    warnBadFrame(key, text);
    return;
  }

  const bucket = window.telemetryState[key];
  bucket.abs = values[0];
  bucket.rel = values[1];
}

function onImuNotification(event) {
  if (!event.target?.value) {
    return;
  }

  const text = textDecoder.decode(event.target.value);
  const values = parseCsv(text, 4);
  if (!values) {
    warnBadFrame("imu", text);
    return;
  }

  // El IMU va rotado 90° en la PCB: el primer campo es la tumbada lateral
  // y el segundo es la pendiente longitudinal (+ subida / − bajada).
  const imu = window.telemetryState.imu;
  imu.roll = values[0];
  imu.pitch = values[1];
  imu.gyroZ = values[2];
  imu.accMag = values[3];
  markImuPacket();
}

function markImuPacket() {
  imuPacketCount += 1;
  lastImuAt = performance.now();
}

async function sendCommand(letter) {
  if (!commandCharacteristic) {
    throw new Error("Característica de comando no disponible.");
  }

  const payload = textEncoder.encode(letter);
  if (typeof commandCharacteristic.writeValueWithResponse === "function") {
    await commandCharacteristic.writeValueWithResponse(payload);
  } else {
    await commandCharacteristic.writeValue(payload);
  }

  console.log("[eSAG] Comando BLE enviado:", letter);
}

function disconnect() {
  if (!bluetoothDevice?.gatt?.connected) {
    cleanupDevice();
    setStatus("Desconectado", "idle");
    return;
  }

  console.log("[eSAG] Cerrando conexión GATT...");
  bluetoothDevice.gatt.disconnect();
}

function onGattServerDisconnected() {
  console.log("[eSAG] Evento gattserverdisconnected: el sensor se ha desconectado.");
  cleanupDevice();
  setStatus("Desconectado", "idle");
  connectBtn.disabled = false;
  connectBtn.textContent = "Conectar Sensor";
  connectBtn.classList.remove("is-connected");
  connectBtn.setAttribute("aria-pressed", "false");
}

function cleanupDevice() {
  window.clearTimeout(calibrateTimer);
  calibrateTimer = 0;
  window.clearTimeout(rangeNoticeTimer);
  rangeNoticeTimer = 0;
  calNoticeEl.hidden = true;

  otaRun += 1;
  otaUploading = false;
  otaSupported = false;
  otaControlCharacteristic = null;
  otaDataCharacteristic = null;
  if (otaWaiter) {
    const waiter = otaWaiter;
    otaWaiter = null;
    waiter.reject(new Error("Sensor desconectado"));
  }

  for (const { characteristic, handler } of activeSubscriptions) {
    characteristic.removeEventListener("characteristicvaluechanged", handler);
  }

  activeSubscriptions = [];
  commandCharacteristic = null;
  setControlsEnabled(false);
  if (!otaStatusLocked) {
    setOtaProgress(0, "Esperando archivo...");
  }
  refreshOtaControls();
  releaseWakeLock();

  if (bluetoothDevice) {
    bluetoothDevice.removeEventListener("gattserverdisconnected", onGattServerDisconnected);
  }

  bluetoothDevice = null;
  gattServer = null;
  resetTelemetry();
}

function resetTelemetry() {
  const state = window.telemetryState;
  state.avg10.abs = null;
  state.avg10.rel = null;
  state.avg50.abs = null;
  state.avg50.rel = null;
  state.avg100.abs = null;
  state.avg100.rel = null;
  state.imu.pitch = null;
  state.imu.roll = null;
  state.imu.gyroZ = null;
  state.imu.accMag = null;

  imuPacketCount = 0;
  displayHz = 0;
  lastImuAt = 0;
  rateWindowStart = performance.now();
  warnedFrames.clear();
}

function resetPeak() {
  peakTravelMm = 0;
  peakPercent = 0;
}

function rememberPeak(result) {
  if (result.travelMm > peakTravelMm) {
    peakTravelMm = result.travelMm;
    peakPercent = result.percent;
  }
}

function setControlsEnabled(enabled) {
  zeroBtn.disabled = !enabled;
  if (!calibrateTimer) {
    calibrateBtn.disabled = !enabled;
  }
}

async function acquireWakeLock() {
  if (!("wakeLock" in navigator)) {
    console.warn("[eSAG] Wake Lock no disponible en este navegador.");
    return;
  }

  const epoch = ++wakeLockEpoch;
  try {
    const lock = await navigator.wakeLock.request("screen");
    if (epoch !== wakeLockEpoch || !wantWakeLock) {
      await lock.release().catch(() => {});
      return;
    }

    screenWakeLock = lock;
    console.log("[eSAG] Wake Lock de pantalla activo.");
  } catch (error) {
    if (epoch === wakeLockEpoch) {
      console.warn("[eSAG] No se pudo activar Wake Lock:", error);
    }
  }
}

function releaseWakeLock() {
  wantWakeLock = false;
  wakeLockEpoch += 1;
  const current = screenWakeLock;
  screenWakeLock = null;
  if (current) {
    current.release().catch(() => {});
  }
}

function renderFrame(now) {
  const elapsed = now - rateWindowStart;
  if (elapsed >= RATE_WINDOW_MS) {
    displayHz = (imuPacketCount * 1000) / elapsed;
    imuPacketCount = 0;
    rateWindowStart = now;
  }

  const sample = window.telemetryState[`avg${averageWindow}`] || { abs: null, rel: null };
  const imu = window.telemetryState.imu;
  const hasAngle = sample.rel != null && Number.isFinite(sample.rel);
  const kinematics = hasAngle
    ? window.interpolateRearShock(sample.rel, bikeModelKey, sensorDirection)
    : extendedShock();

  if (hasAngle) {
    rememberPeak(kinematics);
  }

  const live = lastImuAt !== 0 && now - lastImuAt < LIVE_WINDOW_MS;
  const freefall = imu.accMag != null && imu.accMag < FREEFALL_G;
  const side = formatRollSide(imu.roll);
  const model = window.BIKE_KINEMATICS[bikeModelKey] || window.BIKE_KINEMATICS.WILD_S;

  paintText(sagPercentEl, "sagPercent", formatTenths(kinematics.percent));
  paintText(sagMmEl, "sagMm", formatTenths(kinematics.travelMm));
  paintText(peakValueEl, "peak", `${formatTenths(peakPercent)} % / ${formatTenths(peakTravelMm)} mm`);
  paintText(
    shockLengthEl,
    "shock",
    `${formatTenths(model.eyeToEyeMaxMm)} → ${formatTenths(kinematics.shockLengthMm)} mm`
  );
  paintText(deltaThetaEl, "delta", formatDegrees(kinematics.deltaTheta));
  paintText(relAngleEl, "rel", formatDegrees(sample.rel));
  paintText(absAngleEl, "abs", formatDegrees(sample.abs));
  paintText(pitchEl, "pitch", formatDegrees(imu.pitch));
  paintText(pitchGradeEl, "grade", formatGrade(imu.pitch));
  paintText(rollEl, "roll", formatDegrees(imu.roll));
  paintText(rollSideEl, "side", side);
  paintText(gyroZEl, "gyroZ", formatGyro(imu.gyroZ));
  paintText(accMagEl, "accMag", formatG(imu.accMag));
  paintText(packetHzEl, "hz", `${Math.round(displayHz)} Hz`);
  paintTravel(kinematics.percent);
  updateBikeVisualization(
    kinematics,
    Number.isFinite(imu.pitch) ? imu.pitch : 0,
    Number.isFinite(imu.roll) ? imu.roll : 0
  );

  const sideMuted = side === "—";
  if (sideMuted !== painted.sideMuted) {
    painted.sideMuted = sideMuted;
    rollSideEl.classList.toggle("is-muted", sideMuted);
  }

  if (live !== painted.live) {
    painted.live = live;
    liveDotEl.classList.toggle("is-live", live);
  }

  if (freefall !== painted.freefall) {
    painted.freefall = freefall;
    accMetricEl.classList.toggle("is-alert", freefall);
    accFlagEl.hidden = !freefall;
  }

  requestAnimationFrame(renderFrame);
}

function paintTravel(percent) {
  const clamped = Math.min(Math.max(percent, 0), 100);
  const width = `${clamped.toFixed(1)}%`;
  if (painted.travel !== width) {
    painted.travel = width;
    travelFillEl.style.width = width;
    travelBarEl.setAttribute("aria-valuenow", clamped.toFixed(1));
  }

  const band = travelBand(clamped);
  if (painted.band !== band) {
    painted.band = band;
    travelFillEl.dataset.band = band;
  }
}

function travelBand(percent) {
  if (percent < 25) {
    return "low";
  }
  if (percent <= 33) {
    return "optimal";
  }
  if (percent <= 85) {
    return "warn";
  }
  return "bottom";
}

function updateBikeVisualization(kinematicsData, pitchDeg, rollDeg) {
  const delta = Number.isFinite(kinematicsData.deltaTheta) ? kinematicsData.deltaTheta : 0;
  const model = window.BIKE_KINEMATICS[bikeModelKey] || window.BIKE_KINEMATICS.WILD_S;
  const maxDelta = model.deltaTheta[model.deltaTheta.length - 1];
  const deltaDraw = Math.min(Math.max(delta, 0), maxDelta);
  const percent = Math.min(Math.max(Number.isFinite(kinematicsData.percent) ? kinematicsData.percent : 0, 0), 100);
  const travelMm = Number.isFinite(kinematicsData.travelMm) ? kinematicsData.travelMm : 0;
  const pitchDraw = Math.min(Math.max(pitchDeg, -30), 30);
  const rollDraw = Math.min(Math.max(rollDeg, -45), 45);

  const pitchTransform = `rotate(${(-pitchDraw).toFixed(2)} 132 248)`;
  if (visualPose.pitch !== pitchTransform) {
    visualPose.pitch = pitchTransform;
    svgBikePitchGroup.setAttribute("transform", pitchTransform);
  }

  const rearTransform = `rotate(${deltaDraw.toFixed(3)} 228 156)`;
  if (visualPose.rear !== rearTransform) {
    visualPose.rear = rearTransform;
    svgRearTriangle.setAttribute("transform", rearTransform);
  }

  const tip = SHOCK_TIP_EXTENDED - ((SHOCK_TIP_EXTENDED - SHOCK_TIP_COMPRESSED) * percent) / 100;
  const tipText = tip.toFixed(2);
  if (visualPose.shock !== tipText) {
    visualPose.shock = tipText;
    svgShockPiston.setAttribute("x2", tipText);
    svgShockEye.setAttribute("cx", tipText);
  }

  const shockColor = SHOCK_COLORS[travelBand(percent)];
  if (visualPose.shockColor !== shockColor) {
    visualPose.shockColor = shockColor;
    svgShockPiston.setAttribute("stroke", shockColor);
    svgShockEye.setAttribute("fill", shockColor);
  }

  const rollTransform = `rotate(${rollDraw.toFixed(2)} 110 258)`;
  if (visualPose.roll !== rollTransform) {
    visualPose.roll = rollTransform;
    svgBikeRearView.setAttribute("transform", rollTransform);
  }

  paintText(svgHudSag, "hudSag", `${formatTenths(percent)} %`);
  paintText(svgHudMm, "hudMm", `${formatTenths(travelMm)} mm`);
  paintText(svgHudPitch, "hudPitch", `Pitch ${formatDegrees(pitchDeg)}°`);
  paintText(svgHudRoll, "hudRoll", `${formatDegrees(rollDeg)}°`);
  paintText(svgHudRollSide, "hudSide", formatRollSide(rollDeg));
}

function extendedShock() {
  const model = window.BIKE_KINEMATICS[bikeModelKey] || window.BIKE_KINEMATICS.WILD_S;
  return {
    travelMm: 0,
    percent: 0,
    shockLengthMm: model.eyeToEyeMaxMm,
    deltaTheta: 0,
  };
}

function paintText(node, key, text) {
  if (painted[key] === text) {
    return;
  }

  painted[key] = text;
  node.textContent = text;
}

function parseCsv(text, count) {
  const parts = text.trim().split(",");
  if (parts.length < count) {
    return null;
  }

  const values = [];
  for (let index = 0; index < count; index += 1) {
    const value = parseFloat(parts[index]);
    if (!Number.isFinite(value)) {
      return null;
    }
    values.push(value);
  }

  return values;
}

function warnBadFrame(key, text) {
  if (warnedFrames.has(key)) {
    return;
  }

  warnedFrames.add(key);
  console.warn("[eSAG] Trama no válida en", key, text);
}

function formatDegrees(value) {
  if (value == null || !Number.isFinite(value)) {
    return "0.00";
  }

  return value.toFixed(2);
}

function formatTenths(value) {
  if (value == null || !Number.isFinite(value)) {
    return "0.0";
  }

  return value.toFixed(1);
}

function formatGyro(value) {
  if (value == null || !Number.isFinite(value)) {
    return "0";
  }

  return String(Math.round(value));
}

function formatG(value) {
  if (value == null || !Number.isFinite(value)) {
    return "1.00";
  }

  return value.toFixed(2);
}

function formatGrade(pitchDeg) {
  if (pitchDeg == null || !Number.isFinite(pitchDeg)) {
    return "0.0 %";
  }

  const grade = Math.tan((pitchDeg * Math.PI) / 180) * 100;
  const rounded = Math.round(grade * 10) / 10;
  if (rounded === 0) {
    return "0.0 %";
  }

  const body = Math.abs(rounded).toFixed(1);
  return `${rounded > 0 ? "+" : "-"}${body} %`;
}

function formatRollSide(rollDeg) {
  if (rollDeg == null || !Number.isFinite(rollDeg) || rollDeg === 0) {
    return "—";
  }

  return rollDeg < 0 ? "IZQ" : "DER";
}

function loadBikeModel() {
  try {
    const stored = localStorage.getItem(BIKE_MODEL_KEY);
    if (stored && window.BIKE_KINEMATICS[stored]) {
      return stored;
    }
  } catch (error) {
    console.warn("[eSAG] No se pudo leer el cuadro:", error);
  }
  return "WILD_S";
}

function loadSensorDirection() {
  try {
    const stored = localStorage.getItem(SENSOR_DIR_KEY);
    if (stored === "-1") {
      return -1;
    }
  } catch (error) {
    console.warn("[eSAG] No se pudo leer el sentido del sensor:", error);
  }
  return 1;
}

function saveBikeSettings() {
  try {
    localStorage.setItem(BIKE_MODEL_KEY, bikeModelKey);
    localStorage.setItem(SENSOR_DIR_KEY, String(sensorDirection));
  } catch (error) {
    console.warn("[eSAG] No se pudieron guardar los parámetros de suspensión:", error);
  }
}

function applyBikeSettings() {
  bikeModelSelect.replaceChildren();
  for (const [key, model] of Object.entries(window.BIKE_KINEMATICS)) {
    const option = document.createElement("option");
    option.value = key;
    option.textContent = model.name;
    bikeModelSelect.append(option);
  }
  bikeModelSelect.value = bikeModelKey;
  sensorDirSelect.value = String(sensorDirection);
}

/**
 * @param {string} text
 * @param {"idle" | "busy" | "connected"} mode
 */
function setStatus(text, mode) {
  statusEl.textContent = text;
  statusEl.classList.toggle("is-busy", mode === "busy");
  statusEl.classList.toggle("is-connected", mode === "connected");
}
