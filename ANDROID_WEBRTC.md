# Documentație Aplicație Android — WebRTC + MQTT

## 1. Prezentare Generală

Aplicația Android este o reimplementare nativă a receiver-ului PC Windows, cu funcționalități extinse de control prin giroscop. Este dezvoltată în **Kotlin** cu **Jetpack Compose** și urmează arhitectura **MVVM**.

**Testată pe:** Samsung Galaxy S9+ (landscape forțat)

**Funcționalități principale:**
- Recepție stream video WebRTC de la Raspberry Pi 5
- Afișare telemetrie în timp real (unghi, viteză, CPU, RAM, baterie)
- Control manual prin joystick virtual
- Control prin giroscop (accelerometru + magnetometru)
- Comutare mod autonom/manual prin MQTT
- Grafic PID rolling (robot/pid_telemetry, 20 Hz)

---

## 2. Arhitectura MVVM

```
┌─────────────────────────────────────────────────────┐
│                     VIEW (Compose)                  │
│  StreamScreen │ SettingsScreen │ LogsScreen         │
└───────────────────────┬─────────────────────────────┘
                        │ StateFlow / Events
┌───────────────────────▼─────────────────────────────┐
│               MainViewModel (MVVM)                  │
│  - orchestrare servicii                             │
│  - state management (ConnectionState, SensorData)  │
│  - publishCommand() la 20 Hz                       │
└──────┬───────────┬───────────┬────────────┬─────────┘
       │           │           │            │
  WebRtcSvc  MqttService  Signaling  Gyroscope
  .kt         .kt          Service    Controller
                            .kt        .kt
```

### State expus prin StateFlow

| StateFlow | Tip | Descriere |
|-----------|-----|-----------|
| `connectionState` | `ConnectionState` | DISCONNECTED / CONNECTING / CONNECTED |
| `mqttState` | `MqttState` | stare broker MQTT |
| `sensorData` | `SensorData?` | date senzori sincronizate cu frame |
| `systemData` | `SystemData?` | CPU, RAM, temperatură, baterie |
| `controlMode` | `ControlMode` | AUTO / MANUAL |
| `logs` | `List<LogEntry>` | log conexiuni și erori |

---

## 3. Dependențe principale (build.gradle)

```kotlin
dependencies {
    // WebRTC
    implementation("io.getstream:stream-webrtc-android:1.3.8")

    // MQTT
    implementation("com.hivemq:hivemq-mqtt-client:1.3.3")

    // HTTP Signaling
    implementation("com.squareup.okhttp3:okhttp:4.12.0")

    // JSON
    implementation("com.google.code.gson:gson:2.10.1")

    // Jetpack Compose BOM
    implementation(platform("androidx.compose:compose-bom:2024.09.00"))
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.material3:material3")
    implementation("androidx.compose.ui:ui-tooling-preview")

    // DataStore (setări persistente)
    implementation("androidx.datastore:datastore-preferences:1.1.1")

    // ViewModel + Lifecycle
    implementation("androidx.lifecycle:lifecycle-viewmodel-compose:2.8.0")
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.8.0")
}
```

---

## 4. Serviciul WebRTC (WebRtcService.kt)

### 4.1 Inițializare PeerConnection

```kotlin
class WebRtcService(private val context: Context) {

    private val eglBase = EglBase.create()
    private lateinit var peerConnectionFactory: PeerConnectionFactory
    private var peerConnection: PeerConnection? = null

    fun initialize() {
        PeerConnectionFactory.initialize(
            PeerConnectionFactory.InitializationOptions.builder(context)
                .setEnableInternalTracer(true)
                .createInitializationOptions()
        )
        peerConnectionFactory = PeerConnectionFactory.builder()
            .setVideoDecoderFactory(DefaultVideoDecoderFactory(eglBase.eglBaseContext))
            .setVideoEncoderFactory(DefaultVideoEncoderFactory(eglBase.eglBaseContext, true, true))
            .createPeerConnectionFactory()
    }

    fun createPeerConnection(observer: PeerConnection.Observer): PeerConnection? {
        val iceServers = listOf(
            PeerConnection.IceServer.builder("stun:stun.l.google.com:19302").createIceServer()
        )
        val config = PeerConnection.RTCConfiguration(iceServers).apply {
            sdpSemantics = PeerConnection.SdpSemantics.UNIFIED_PLAN
        }
        return peerConnectionFactory.createPeerConnection(config, observer)
            .also { peerConnection = it }
    }
}
```

### 4.2 Flux de semnalizare (Offer/Answer)

```
Android (Receiver)              Signaling Server (PC:8080)    Pi (Sender)
      │                                   │                       │
      │  GET /offer (polling)             │                       │
      ├──────────────────────────────────►│                       │
      │                                   │  ◄── POST /offer ─────┤
      │  ◄── SDP Offer ───────────────────┤                       │
      │                                   │                       │
      │  setRemoteDescription(offer)      │                       │
      │  createAnswer()                   │                       │
      │  setLocalDescription(answer)      │                       │
      │                                   │                       │
      │  POST /answer ────────────────────►                       │
      │                                   │  ── GET /answer ─────►│
      │                                   │                       │
      │  ◄══ WebRTC P2P (DTLS/SRTP) ═════════════════════════════►│
```

### 4.3 Primirea video și rendering

```kotlin
// Atașare VideoTrack la SurfaceViewRenderer
peerConnection?.addTrack(videoTrack)

// În Composable:
@Composable
fun VideoView(webRtcService: WebRtcService) {
    AndroidView(factory = { ctx ->
        SurfaceViewRenderer(ctx).also { renderer ->
            renderer.init(eglBase.eglBaseContext, null)
            renderer.setScalingType(RendererCommon.ScalingType.SCALE_ASPECT_FIT)
            webRtcService.attachRenderer(renderer)
        }
    })
}
```

> **Avantaj față de Tkinter (PC):** SurfaceViewRenderer randarează frame-urile WebRTC direct pe suprafața hardware — fără conversie BGR→RGB→PIL→ImageTk, latența de rendering este eliminată.

---

## 5. Serviciul MQTT (MqttService.kt)

### 5.1 Conexiunea la broker

```kotlin
class MqttService {
    private lateinit var client: Mqtt5AsyncClient

    fun connect(brokerIp: String, port: Int = 1883) {
        client = MqttClient.builder()
            .useMqttVersion5()
            .serverHost(brokerIp)
            .serverPort(port)
            .automaticReconnect().applyAutomaticReconnect()
            .buildAsync()

        client.connectWith()
            .cleanStart(true)
            .send()
            .thenAccept { onConnected() }
    }
}
```

### 5.2 Topicuri MQTT

| Topic | Direcție | Frecvență | Payload JSON |
|-------|----------|-----------|--------------|
| `robot/senzori` | Pi → Android | 10 Hz | `{"unghi": float, "viteza": float, "timestamp": ms}` |
| `robot/sistem` | Pi → Android | 0.5 Hz | `{"cpu_usage", "ram_usage", "temperature", "battery", "bat_voltage", "bat_current", "charging"}` |
| `robot/pid_telemetry` | Pi → Android | 20 Hz | `{"reference": 0.0, "response": float, "steering_angle": float, "timestamp": ms}` |
| `robot/control/mod` | Android → Pi | La comandă | `{"mod_de_functionare": 0\|1}` |
| `robot/control/unghi_manual` | Android → Pi | 20 Hz | `{"unghi_manual": float}` |
| `robot/control/viteza_manual` | Android → Pi | 20 Hz | `{"viteza_manual": float}` |

### 5.3 Subscripție și sincronizare timestamp

```kotlin
// Buffer circular pentru sincronizare cu frame-uri video
private val sensorBuffer = ArrayDeque<Pair<Long, SensorData>>(100)

fun subscribeAll() {
    client.subscribeWith()
        .topicFilter("robot/senzori")
        .callback { msg ->
            val data = gson.fromJson(msg.payloadAsString, SensorData::class.java)
            // Menține buffer circular de 100 intrări
            if (sensorBuffer.size >= 100) sensorBuffer.removeFirst()
            sensorBuffer.addLast(Pair(data.timestamp, data))
        }.send()
}

// Căutare date cu toleranță ±150 ms (identic cu implementarea Python)
fun getSensorDataForTimestamp(videoTimestamp: Long): SensorData? {
    return sensorBuffer
        .minByOrNull { abs(it.first - videoTimestamp) }
        ?.takeIf { abs(it.first - videoTimestamp) <= 150L }
        ?.second
}
```

---

## 6. Controlul prin Joystick Virtual (JoystickComponent.kt)

```kotlin
@Composable
fun JoystickControl(
    onSteeringChange: (Float) -> Unit,  // -50f .. +50f grade
    onThrottleChange: (Float) -> Unit   // 0f .. 100f %
) {
    val thumbOffset = remember { mutableStateOf(Offset.Zero) }
    val radius = 80.dp

    Box(
        modifier = Modifier
            .size(radius * 2)
            .clip(CircleShape)
            .background(Color.White.copy(alpha = 0.15f))
            .pointerInput(Unit) {
                detectDragGestures(
                    onDragEnd = {
                        thumbOffset.value = Offset.Zero
                        onSteeringChange(0f)
                        onThrottleChange(0f)
                    }
                ) { change, _ ->
                    val radiusPx = radius.toPx()
                    val clamped = change.position
                        .let { Offset(it.x - radiusPx, it.y - radiusPx) }
                        .let { if (it.getDistance() > radiusPx) it / it.getDistance() * radiusPx else it }
                    thumbOffset.value = clamped
                    onSteeringChange(clamped.x / radiusPx * 50f)
                    onThrottleChange(-clamped.y / radiusPx * 100f)
                }
            }
    ) {
        // Thumbstick
        Box(
            modifier = Modifier
                .size(40.dp)
                .offset { IntOffset(thumbOffset.value.x.roundToInt(), thumbOffset.value.y.roundToInt()) }
                .clip(CircleShape)
                .background(Color.White.copy(alpha = 0.6f))
                .align(Alignment.Center)
        )
    }
}
```

**Publicare comenzi la 20 Hz din ViewModel:**

```kotlin
fun startCommandPublisher() {
    commandJob = viewModelScope.launch {
        while (isActive) {
            if (controlMode.value == ControlMode.MANUAL) {
                mqttService.publish("robot/control/unghi_manual",
                    """{"unghi_manual": ${steeringAngle.value}}""")
                mqttService.publish("robot/control/viteza_manual",
                    """{"viteza_manual": ${throttle.value}}""")
            }
            delay(50) // 20 Hz
        }
    }
}
```

---

## 7. Controlul prin Giroscop (GyroscopeController.kt)

```kotlin
class GyroscopeController(private val context: Context) : SensorEventListener {

    private val sensorManager = context.getSystemService(SensorManager::class.java)
    private val rotationMatrix = FloatArray(9)
    private val orientation = FloatArray(3)
    private var calibrationOffset = FloatArray(3) { 0f }

    // Deadzone ±5° pe ambele axe
    private val DEADZONE_RAD = Math.toRadians(5.0).toFloat()

    fun calibrate() {
        // Setează poziția curentă ca referință neutră
        calibrationOffset = orientation.copyOf()
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_ROTATION_VECTOR -> {
                SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
                SensorManager.getOrientation(rotationMatrix, orientation)

                val roll  = orientation[2] - calibrationOffset[2]  // stânga/dreapta
                val pitch = orientation[1] - calibrationOffset[1]  // față/spate

                val steering = if (abs(roll) > DEADZONE_RAD)
                    (roll * (180f / Math.PI.toFloat())).coerceIn(-50f, 50f) else 0f
                val throttle  = if (abs(pitch) > DEADZONE_RAD)
                    (-pitch * (180f / Math.PI.toFloat())).coerceIn(0f, 100f) else 0f

                onGyroscopeUpdate(steering, throttle)
            }
        }
    }

    fun start() {
        sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
    }

    fun stop() = sensorManager.unregisterListener(this)
}
```

---

## 8. Interfața Utilizator (StreamScreen.kt)

### 8.1 Layout principal (landscape)

```
┌─────────────────────────────────────────────────────────────────┐
│  [WebRTC ●]  [MQTT ●]  │  FPS: 28  │  Latență: ~230ms         │ ← TopStatusBar
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│                   VIDEO STREAM (full-screen)                    │
│              SurfaceViewRenderer — VP8 decoded                  │
│                                                                 │
│  ┌─────────────────────┐              ┌─────────────────────┐   │
│  │ Unghi:   +12.3°     │              │      JOYSTICK       │   │
│  │ Viteză:   45%       │              │      VIRTUAL        │   │
│  │ CPU:      65%       │              │                     │   │
│  │ RAM:      42%       │              └─────────────────────┘   │
│  │ Temp:     71°C      │                                        │
│  │ Baterie:  78% 7.9V  │   [AUTO/MANUAL]  [CALIBRARE]  [⚙]   │
│  └─────────────────────┘                                        │
└─────────────────────────────────────────────────────────────────┘
```

### 8.2 Structura Composables

```kotlin
@Composable
fun StreamScreen(viewModel: MainViewModel) {
    val uiState by viewModel.uiState.collectAsState()

    Box(modifier = Modifier.fillMaxSize().background(Color.Black)) {
        // 1. Video full-screen
        VideoView(webRtcService = viewModel.webRtcService)

        // 2. Status bar (top)
        TopStatusBar(
            webRtcConnected = uiState.webRtcConnected,
            mqttConnected   = uiState.mqttConnected,
            fps             = uiState.fps
        )

        // 3. Overlay telemetrie (stânga)
        SensorOverlay(
            sensorData = uiState.sensorData,
            systemData = uiState.systemData,
            modifier   = Modifier.align(Alignment.CenterStart).padding(16.dp)
        )

        // 4. Control (dreapta jos)
        ControlPanel(
            controlMode      = uiState.controlMode,
            onModeToggle     = { viewModel.toggleMode() },
            onCalibrate      = { viewModel.calibrateGyroscope() },
            onSteeringChange = { viewModel.setSteering(it) },
            onThrottleChange = { viewModel.setThrottle(it) },
            modifier         = Modifier.align(Alignment.BottomEnd).padding(16.dp)
        )
    }
}
```

---

## 9. Setări persistente (DataStore)

```kotlin
data class AppSettings(
    val signalingServerIp: String = "192.168.1.100",
    val signalingServerPort: Int  = 8080,
    val mqttBrokerIp: String      = "192.168.1.100",
    val mqttBrokerPort: Int       = 1883,
    val gyroscopeEnabled: Boolean = false,
    val gyroscopeDeadzone: Float  = 5f,          // grade
    val joystickMaxSteering: Float = 50f,        // grade
    val commandFrequencyHz: Int   = 20
)

// Salvare în DataStore (Proto sau Preferences)
val Context.dataStore by preferencesDataStore("app_settings")
```

---

## 10. AndroidManifest.xml — Permisiuni necesare

```xml
<uses-permission android:name="android.permission.INTERNET" />
<uses-permission android:name="android.permission.ACCESS_NETWORK_STATE" />
<uses-permission android:name="android.permission.ACCESS_WIFI_STATE" />

<!-- Giroscop (nu necesită permisiune runtime) -->
<uses-feature android:name="android.hardware.sensor.gyroscope" android:required="false" />
<uses-feature android:name="android.hardware.sensor.accelerometer" android:required="false" />

<application
    android:hardwareAccelerated="true"  <!-- obligatoriu pentru SurfaceViewRenderer -->
    ...>
    <activity
        android:screenOrientation="landscape"
        android:configChanges="orientation|screenSize|keyboardHidden"
        ...>
    </activity>
</application>
```

---

## 11. Flux complet de conectare

```
1. User deschide aplicația
        │
2. MainActivity → MainViewModel.init()
        │
3. Signaling: GET http://{ip}:8080/offer  (polling la 2s)
        │
4. Pi trimite POST /offer cu SDP Offer
        │
5. Android: setRemoteDescription(offer)
           createAnswer()
           setLocalDescription(answer)
        │
6. Android: POST http://{ip}:8080/answer
        │
7. ICE negotiation (STUN: stun.l.google.com)
        │
8. WebRTC P2P CONNECTED → video stream începe
        │
9. MQTT: connect({mqttIp}:1883)
         subscribe("robot/senzori")
         subscribe("robot/sistem")
         subscribe("robot/pid_telemetry")
        │
10. UI actualizat în timp real prin StateFlow
```

---

## 12. Performanță și optimizări

| Metric | Target | Implementare |
|--------|--------|-------------|
| Latență video | < 300 ms | SurfaceViewRenderer hardware |
| FPS primit | ~25–30 FPS | Depinde de rețea Wi-Fi |
| Comenzi joystick | 20 Hz | coroutine cu `delay(50)` |
| Citire giroscop | ~100 Hz | `SENSOR_DELAY_GAME` |
| Buffer senzori | 100 intrări | ArrayDeque cu evicție FIFO |
| Toleranță sync | ±150 ms | Identic cu implementarea Python |

### Optimizări cheie vs. receiver PC:
- **Rendering hardware direct** — SurfaceViewRenderer vs. Tkinter Label (elimină 1–2 conversii de format)
- **MQTT async (HiveMQ v5)** — non-blocking vs. `paho-mqtt` callbacks Python
- **Giroscop nativ** — SensorManager Android, nu emulat

---

## 13. Troubleshooting

| Problemă | Cauză probabilă | Soluție |
|----------|----------------|---------|
| Video negru / no stream | Signaling server offline | Verifică `signaling_server.py` pe PC |
| MQTT nu se conectează | IP greșit sau Mosquitto oprit | Verifică `mosquitto.conf` și portul 1883 |
| Giroscop drift | Calibrare lipsă | Apasă "Calibrare" în poziție neutră |
| Latență mare video | Wi-Fi 2.4 GHz congestionat | Folosește banda 5 GHz |
| Comenzi întârziate | Frecvență MQTT prea mică | Crește `commandFrequencyHz` la 20 Hz |
| ICE failed | NAT traversal blocat | Verifică firewall; adaugă TURN server |

---

## 14. Integrare cu sistemul existent — fără modificări pe Pi

Aplicația Android comunică prin **aceleași protocoale și topicuri** ca receiver-ul PC:
- Același server de semnalizare HTTP pe portul 8080
- Același broker Mosquitto pe portul 1883
- Aceleași topicuri MQTT

**Nu este necesară nicio modificare în `sender_mqtt.py` sau configurația Raspberry Pi** pentru a trece de la controlul PC la cel Android.
