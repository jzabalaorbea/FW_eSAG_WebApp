const DEVICE_NAME = "eSag_Sensor";
const SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const FREEFALL_G = 0.25;
const LIVE_WINDOW_MS = 400;
const RATE_WINDOW_MS = 1000;
const CALIBRATION_HOLD_MS = 2200;
const SETTINGS_KEY = "esag-suspension-settings";

const CHARACTERISTICS = {
  avg10: "beb5483e-36e1-4688-b7f5-ea07361b26a8",
  avg50: "beb5483e-36e1-4688-b7f5-ea07361b26a9",
  avg100: "beb5483e-36e1-4688-b7f5-ea07361b26aa",
  imu: "beb5483e-36e1-4688-b7f5-ea07361b26ab",
  command: "beb5483e-36e1-4688-b7f5-ea07361b26ac",
};

const DEFAULT_SETTINGS = {
  shockTravelMm: 60,
  forkTravelMm: 160,
  maxAngleDeg: 45,
  element: "shock",
};

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
const shockTravelInput = document.getElementById("shock-travel");
const forkTravelInput = document.getElementById("fork-travel");
const maxAngleInput = document.getElementById("max-angle");
const windowButtons = Array.from(document.querySelectorAll("[data-window]"));
const elementButtons = Array.from(document.querySelectorAll("[data-element]"));
const settingInputs = [shockTravelInput, forkTravelInput, maxAngleInput];

/** @type {BluetoothDevice | null} */
let bluetoothDevice = null;

/** @type {BluetoothRemoteGATTServer | null} */
let gattServer = null;

/** @type {BluetoothRemoteGATTCharacteristic | null} */
let commandCharacteristic = null;

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
let peakRelAngle = 0;
let suspensionSettings = loadSuspensionSettings();

const painted = {
  sagPercent: "",
  sagMm: "",
  peak: "",
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
};

const warnedFrames = new Set();

window.telemetryState = {
  avg10: { abs: null, rel: null },
  avg50: { abs: null, rel: null },
  avg100: { abs: null, rel: null },
  imu: { pitch: null, roll: null, gyroZ: null, accMag: null },
};

applySettingsToForm();

connectBtn.addEventListener("click", onConnectClick);
zeroBtn.addEventListener("click", onZeroClick);
calibrateBtn.addEventListener("click", onCalibrateClick);
resetPeakBtn.addEventListener("click", onResetPeakClick);
document.addEventListener("visibilitychange", onVisibilityChange);

for (const button of windowButtons) {
  button.addEventListener("click", onAverageWindowClick);
}

for (const button of elementButtons) {
  button.addEventListener("click", onElementClick);
}

for (const input of settingInputs) {
  input.addEventListener("input", onSettingInput);
  input.addEventListener("change", onSettingInput);
  input.addEventListener("blur", onSettingBlur);
}

requestAnimationFrame(renderFrame);

/**
 * Convierte el ángulo relativo del sensor en recorrido de suspensión.
 * No lee estado global: la ventana BLE y el elemento medido se pasan ya resueltos.
 *
 * @param {number} relAngleDeg Ángulo relativo respecto al cero de SAG, en grados.
 * @param {number} maxTravelMm Recorrido total del amortiguador o la horquilla, en mm.
 * @param {number} maxAngleDeg Ángulo del sensor con la suspensión a tope de carrera.
 * @returns {{ percent: number, travelMm: number }}
 */
function computeSuspensionKinematics(relAngleDeg, maxTravelMm, maxAngleDeg) {
  if (!Number.isFinite(relAngleDeg) || !(maxTravelMm >= 0) || !(maxAngleDeg > 0)) {
    return { percent: 0, travelMm: 0 };
  }

  // Punto de sustitución: reemplazar este `ratio` lineal por las ecuaciones
  // cinemáticas reales de la bieleta (relación instantánea ángulo → recorrido).
  const ratio = Math.min(Math.max(relAngleDeg / maxAngleDeg, 0), 1);
  return {
    percent: ratio * 100,
    travelMm: ratio * maxTravelMm,
  };
}

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

function onElementClick(event) {
  const button = event.currentTarget;
  suspensionSettings.element = button.dataset.element === "fork" ? "fork" : "shock";
  saveSuspensionSettings();

  for (const candidate of elementButtons) {
    candidate.setAttribute("aria-pressed", candidate === button ? "true" : "false");
  }
}

function onSettingInput(event) {
  const input = event.currentTarget;
  const value = Number(input.value);
  if (!Number.isFinite(value) || value <= 0) {
    return;
  }

  suspensionSettings[input.dataset.setting] = value;
  saveSuspensionSettings();
}

function onSettingBlur(event) {
  const input = event.currentTarget;
  const key = input.dataset.setting;
  const value = Number(input.value);
  if (!Number.isFinite(value) || value <= 0) {
    input.value = String(suspensionSettings[key]);
  }
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
    await wakeLockPromise;

    setStatus("Conectado al ESP32-C3", "connected");
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
  calNoticeEl.hidden = true;

  for (const { characteristic, handler } of activeSubscriptions) {
    characteristic.removeEventListener("characteristicvaluechanged", handler);
  }

  activeSubscriptions = [];
  commandCharacteristic = null;
  setControlsEnabled(false);
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
  peakRelAngle = 0;
}

function rememberPeak(relAngleDeg) {
  if (relAngleDeg > peakRelAngle) {
    peakRelAngle = relAngleDeg;
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
  const travelMm = activeTravelMm();
  const hasAngle = sample.rel != null && Number.isFinite(sample.rel);
  const kinematics = hasAngle
    ? computeSuspensionKinematics(sample.rel, travelMm, suspensionSettings.maxAngleDeg)
    : { percent: 0, travelMm: 0 };

  if (hasAngle) {
    rememberPeak(sample.rel);
  }

  const peak = computeSuspensionKinematics(
    peakRelAngle,
    travelMm,
    suspensionSettings.maxAngleDeg
  );

  const live = lastImuAt !== 0 && now - lastImuAt < LIVE_WINDOW_MS;
  const freefall = imu.accMag != null && imu.accMag < FREEFALL_G;
  const side = formatRollSide(imu.roll);

  paintText(sagPercentEl, "sagPercent", formatTenths(kinematics.percent));
  paintText(sagMmEl, "sagMm", formatTenths(kinematics.travelMm));
  paintText(peakValueEl, "peak", `${formatTenths(peak.percent)} % / ${formatTenths(peak.travelMm)} mm`);
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

function activeTravelMm() {
  return suspensionSettings.element === "fork"
    ? suspensionSettings.forkTravelMm
    : suspensionSettings.shockTravelMm;
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

function loadSuspensionSettings() {
  try {
    const raw = localStorage.getItem(SETTINGS_KEY);
    if (!raw) {
      return { ...DEFAULT_SETTINGS };
    }

    const parsed = JSON.parse(raw);
    return {
      shockTravelMm: clampNumber(parsed.shockTravelMm, DEFAULT_SETTINGS.shockTravelMm, 1, 400),
      forkTravelMm: clampNumber(parsed.forkTravelMm, DEFAULT_SETTINGS.forkTravelMm, 1, 400),
      maxAngleDeg: clampNumber(parsed.maxAngleDeg, DEFAULT_SETTINGS.maxAngleDeg, 0.1, 180),
      element: parsed.element === "fork" ? "fork" : "shock",
    };
  } catch (error) {
    console.warn("[eSAG] No se pudieron leer los parámetros de suspensión:", error);
    return { ...DEFAULT_SETTINGS };
  }
}

function saveSuspensionSettings() {
  try {
    localStorage.setItem(SETTINGS_KEY, JSON.stringify(suspensionSettings));
  } catch (error) {
    console.warn("[eSAG] No se pudieron guardar los parámetros de suspensión:", error);
  }
}

function applySettingsToForm() {
  shockTravelInput.value = String(suspensionSettings.shockTravelMm);
  forkTravelInput.value = String(suspensionSettings.forkTravelMm);
  maxAngleInput.value = String(suspensionSettings.maxAngleDeg);

  for (const button of elementButtons) {
    button.setAttribute("aria-pressed", button.dataset.element === suspensionSettings.element ? "true" : "false");
  }
}

function clampNumber(value, fallback, min, max) {
  const number = Number(value);
  if (!Number.isFinite(number)) {
    return fallback;
  }

  return Math.min(Math.max(number, min), max);
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
