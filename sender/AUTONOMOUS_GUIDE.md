# Autonomous Lane Following - User Guide

## Overview

Sistem complet de urmarire automata a benzii folosind:
- **Sliding Window Search** - detectie precisa a liniilor
- **Polynomial Curve Fitting** - curbe smooth, nu linii drepte
- **PID Controller** - control smooth fara zigzag
- **Adaptive Speed** - incetineste in curbe, accelereaza pe dreapta

---

## Quick Start

### 1. Test pe Video Inregistrat (RECOMANDAT)

```bash
cd ~/sender

# Test simplu (fara hardware)
python3 test_lane_video.py --video track_video.mp4

# Cu salvare output
python3 test_lane_video.py --video track_video.mp4 --output result.mp4

# Playback rapid (2x speed)
python3 test_lane_video.py --video track_video.mp4 --speed 2.0

# Proces doar 100 frames
python3 test_lane_video.py --video track_video.mp4 --max-frames 100
```

**Comenzi in timpul rularii:**
- `q` - Quit (opreste)
- `p` - Pause (pauza)
- `s` - Save frame (salveaza frame-ul curent)

---

### 2. Test cu Hardware (ATENTIE!)

```bash
# ASIGURA-TE ca robotul e pe stand/ridicat de jos!
python3 test_lane_video.py --video track_video.mp4 --hardware
```

---

## Componente Sistem

### 1. Lane Detection (`lane_detection.py`)

**Ce face:**
- Preprocess: HLS color space + binary threshold
- Perspective transform: Bird's eye view
- Sliding window: gaseste pixelii liniilor
- Polynomial fit: curbe de gradul 2 pentru smooth lines

**Parametri ajustabili:**

```python
detector = LaneDetector(
    img_width=1920,
    img_height=1080,
    lane_width_meters=0.6,  # Latimea fizica intre linii
    nwindows=9,             # Numar ferestre sliding
    margin=100,             # Latime fereastra
    minpix=50               # Pixeli minim pentru recenter
)
```

---

### 2. PID Controller (`pid_controller.py`)

**Ce face:**
- Calculeaza unghi steering smooth
- Previne zigzag prin exponential smoothing
- Rate limiting pentru miscari bruște

**Tuning parametri:**

```python
pid = PIDController(
    kp=0.5,    # Proportional: raspuns la eroare curenta
               # Mai mare = mai agresiv
               # Mai mic = mai blând
    
    ki=0.0,    # Integral: corecteaza offset constant
               # Creste daca ai drift constant
    
    kd=0.3,    # Derivative: previne overshoot
               # Mai mare = mai damped
    
    alpha=0.3  # Smoothing: 0=foarte smooth, 1=instant
               # Incepe cu 0.3, creste pentru raspuns rapid
)
```

**Cum sa tunezi:**

1. **Prea mult zigzag?**
   - Micsoreaza `kp` (ex: 0.3)
   - Creste `alpha` pentru mai mult smoothing (ex: 0.2)

2. **Raspuns prea lent?**
   - Creste `kp` (ex: 0.7)
   - Micsoreaza `alpha` (ex: 0.4)

3. **Overshoot in curbe?**
   - Creste `kd` (ex: 0.5)

4. **Drift constant (merge la o parte)?**
   - Creste `ki` (ex: 0.05)

---

### 3. Adaptive Speed (`pid_controller.py`)

**Ce face:**
- Incetineste automat in curbe
- Accelereaza pe portiuni drepte

```python
speed_ctrl = AdaptiveSpeedController(
    base_speed=40.0,   # Viteza normala (%)
    min_speed=20.0,    # Viteza minima in curbe
    max_speed=60.0     # Viteza maxima pe dreapta
)
```

---

## Calibrare pentru Traseu

### Pas 1: Inregistreaza Video Test

Filmeaza traseu cu camera robotului:
- Asigura-te ca liniile sunt vizibile
- Lumina uniforma (evita umbre puternice)
- Frame rate constant (30 FPS)

### Pas 2: Ajusteaza ROI (Region of Interest)

In `lane_detection.py`, modifica:

```python
def _calculate_roi(self):
    # Ajusteaza aceste valori!
    bottom_left = [int(self.img_width * 0.1), self.img_height]
    bottom_right = [int(self.img_width * 0.9), self.img_height]
    top_left = [int(self.img_width * 0.4), int(self.img_height * 0.6)]
    top_right = [int(self.img_width * 0.6), int(self.img_height * 0.6)]
```

**Verificare:** Ruleaza test si uita-te la masca ROI (verde) - trebuie sa acopere doar drumul.

### Pas 3: Ajusteaza Color Thresholds

Daca liniile nu sunt detectate bine:

```python
# In preprocess_frame():

# Pentru linii albe:
l_thresh_min = 200  # Creste daca prea multe false positives
l_thresh_max = 255

# Pentru linii galbene/colorate:
s_thresh_min = 100  # Ajusteaza pentru culoarea ta
s_thresh_max = 255
```

### Pas 4: Tuneaza PID

Urmareste algoritmul de tuning de mai sus!

---

## Debugging

### Problema: Nu Detecteaza Linii

**Verificari:**

```python
# In test_lane_video.py, adauga:
cv2.imshow('Binary Threshold', detection['binary'] * 255)
cv2.imshow('Bird Eye View', detection['warped'] * 255)
```

- Daca `binary` e negru → ajusteaza color thresholds
- Daca `warped` arata gresit → ajusteaza perspective transform

### Problema: Zigzag

1. Micsoreaza `kp` in PID
2. Creste smoothing (`alpha` mai mic)
3. Verifica ca `max_angle_change_per_second` nu e prea mare

### Problema: Reactie Lenta

1. Creste `kp` in PID
2. Micsoreaza smoothing (`alpha` mai mare)
3. Creste `max_angle_change_per_second`

---

## Performance

Pe Raspberry Pi 5:
- **Detectie + Control:** ~30-40 FPS (720p)
- **Detectie + Control:** ~20-30 FPS (1080p)
- **Latenta:** <50ms

---

## Safety

**INAINTE de a activa hardware:**

1. Testeaza pe video inregistrat
2. Verifica ca detectia e stabila
3. Pune robotul pe stand (roti in aer)
4. Testeaza cu `--hardware` pe stand
5. Doar apoi testeaza pe podea, cu spatiu liber

**Emergency stop:** Apasa `q` sau Ctrl+C

---

## Troubleshooting

| Simptom | Cauza | Solutie |
|---------|-------|---------|
| Nu vad linii verzi | Detectie failed | Verifica thresholds |
| Zigzag puternic | PID prea agresiv | Micsoreaza Kp, alpha |
| Iese din traseu | Offset gresit | Verifica calibrare |
| FPS scazut | Rezolutie prea mare | Rescaleaza la 720p |
| Crash in curbe | Viteza prea mare | Micsoreaza max_speed |

---

## Next Steps

Dupa ce functioneaza pe video:
1. Integrare cu camera live (picamera2)
2. Integrare cu WebRTC streaming
3. Mode switch AUTOMAT/MANUAL prin MQTT
4. Telemetry logging pentru analiza

---

**Succes!** 🚗💨
