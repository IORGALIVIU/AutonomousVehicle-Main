#!/usr/bin/env python3
"""
Central configuration loader.

Încarcă config.yaml o singură dată la importul modulului.
Toate celelalte module importă variabilele direct din acest modul:

    from config import CFG, PROCESS_WIDTH, PROCESS_HEIGHT, CONTROL_DT

Dacă config.yaml lipsește, se folosesc valorile implicite (aceleași ca în YAML)
astfel încât codul funcționează și fără fișierul de configurare.
"""

import os
import logging

logger = logging.getLogger(__name__)

# ── Locația fișierului de configurare ─────────────────────────────────────────
_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")


def _load_yaml(path: str) -> dict:
    """Încarcă YAML fără dependință obligatorie la PyYAML."""
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        logger.info(f"Config loaded from {path}")
        return data or {}
    except ImportError:
        logger.warning("PyYAML not installed — using built-in defaults. Run: pip install pyyaml")
        return {}
    except FileNotFoundError:
        logger.warning(f"config.yaml not found at {path} — using built-in defaults")
        return {}
    except Exception as e:
        logger.error(f"Error loading config.yaml: {e} — using built-in defaults")
        return {}


# ── Valori implicite (oglindă a config.yaml) ──────────────────────────────────
_DEFAULTS: dict = {
    "camera": {
        "width": 1280, "height": 720, "fps": 30,
        "process_width": 640, "process_height": 480,
    },
    "pid": {
        "kp": 1.5, "ki": 0.012, "kd": 0.35,
        "max_angle": 50.0, "alpha": 0.3,
    },
    "speed": {"base": 30.0, "min": 25.0, "max": 35.0},
    "hardware": {
        "i2c_address": 0x7F, "pwm_frequency": 50, "servo_channel": 0,
        "trim_offset": 10,
        "motor_a_in1": 1, "motor_a_in2": 2, "motor_a_ena": 3,
        "motor_b_in3": 4, "motor_b_in4": 5, "motor_b_enb": 6,
        "pwm_min_percent": 25, "pwm_max_percent": 100,
    },
    "lane": {
        "width_meters": 0.3, "lookahead_frac": 0.50,
        "roi_top_frac": 0.60, "roi_left_frac": 0.05, "roi_right_frac": 0.95,
    },
    "lane_detection": {
        "nwindows": 9, "max_lost_frames": 5,
        "peak_threshold_frac": 0.3,
        "max_window_jump": 80, "min_separation": 150, "max_empty_windows": 2,
        "s_thresh_min": 35, "s_thresh_max": 255,
        "l_thresh_min": 100, "l_thresh_max": 255,
        "morph_kernel_size": 5,
    },
    "telemetry": {"mqtt_hz": 10},
    "control": {"loop_hz": 30},
}


def _deep_merge(defaults: dict, overrides: dict) -> dict:
    """Merge recursiv: override-urile suprascriu defaulturile, secțiunile lipsă rămân default."""
    result = defaults.copy()
    for key, val in overrides.items():
        if key in result and isinstance(result[key], dict) and isinstance(val, dict):
            result[key] = _deep_merge(result[key], val)
        else:
            result[key] = val
    return result


# ── Configurația finală (singleton) ───────────────────────────────────────────
_raw = _load_yaml(_CONFIG_PATH)
CFG: dict = _deep_merge(_DEFAULTS, _raw)

# ── Shortcut-uri convenabile (importate direct în alte module) ─────────────────

# Camera
CAMERA_WIDTH    = CFG["camera"]["width"]
CAMERA_HEIGHT   = CFG["camera"]["height"]
CAMERA_FPS      = CFG["camera"]["fps"]
PROCESS_WIDTH   = CFG["camera"]["process_width"]
PROCESS_HEIGHT  = CFG["camera"]["process_height"]

# Control loop
CONTROL_LOOP_HZ = CFG["control"]["loop_hz"]
CONTROL_DT      = 1.0 / CONTROL_LOOP_HZ

# Telemetrie
TELEMETRY_HZ       = CFG["telemetry"]["mqtt_hz"]
TELEMETRY_INTERVAL = 1.0 / TELEMETRY_HZ

# Hardware
TRIM_OFFSET = CFG["hardware"]["trim_offset"]

# Lane
LANE_WIDTH_METERS = CFG["lane"]["width_meters"]
LOOKAHEAD_FRAC    = CFG["lane"]["lookahead_frac"]


if __name__ == "__main__":
    import json
    print("=== Loaded Configuration ===")
    print(json.dumps(CFG, indent=2, default=str))
    print(f"\nShortcuts:")
    print(f"  PROCESS:      {PROCESS_WIDTH}x{PROCESS_HEIGHT}")
    print(f"  CONTROL_DT:   {CONTROL_DT*1000:.1f}ms ({CONTROL_LOOP_HZ}Hz)")
    print(f"  TELEMETRY:    {TELEMETRY_HZ}Hz ({TELEMETRY_INTERVAL*1000:.0f}ms)")
    print(f"  TRIM_OFFSET:  {TRIM_OFFSET}°")
    print(f"  LANE_WIDTH:   {LANE_WIDTH_METERS}m")