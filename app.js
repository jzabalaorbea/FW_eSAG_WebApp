const DEVICE_NAME = "eSag_Sensor";
const SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";

const connectBtn = document.getElementById("connect-btn");
const statusEl = document.getElementById("status");

/** @type {BluetoothDevice | null} */
let bluetoothDevice = null;

/** @type {BluetoothRemoteGATTServer | null} */
let gattServer = null;

let connectInFlight = false;

connectBtn.addEventListener("click", onConnectClick);

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

async function connect() {
  if (!navigator.bluetooth) {
    console.error("[eSAG] Web Bluetooth no está disponible en este navegador o contexto.");
    setStatus("Bluetooth no disponible", "idle");
    return;
  }

  connectInFlight = true;
  setStatus("Conectando...", "busy");
  connectBtn.disabled = true;

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

    setStatus("Conectado al ESP32-C3", "connected");
    connectBtn.textContent = "Desconectar";
    connectBtn.classList.add("is-connected");
    connectBtn.setAttribute("aria-pressed", "true");
  } catch (error) {
    if (error && error.name === "NotFoundError") {
      console.log("[eSAG] Selección de dispositivo cancelada.");
    } else {
      console.error("[eSAG] Error durante la conexión:", error);
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
  if (bluetoothDevice) {
    bluetoothDevice.removeEventListener("gattserverdisconnected", onGattServerDisconnected);
  }

  bluetoothDevice = null;
  gattServer = null;
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
