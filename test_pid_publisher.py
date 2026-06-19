#!/usr/bin/env python3
"""
Test script: Simuleaza un sender care publica date PID pe robot/pid_telemetry.
Ruleaza pe Windows pentru a testa graficul PID fara Raspberry Pi.

Usage:
    python test_pid_publisher.py
    python test_pid_publisher.py --broker 192.168.x.x
"""

import argparse
import json
import math
import time
import paho.mqtt.client as mqtt

TOPIC = "robot/pid_telemetry"


def main():
    parser = argparse.ArgumentParser(description="PID telemetry test publisher")
    parser.add_argument("--broker", default="localhost", help="MQTT broker IP")
    parser.add_argument("--port",   type=int, default=1883)
    parser.add_argument("--hz",     type=int, default=20, help="Publish rate Hz")
    args = parser.parse_args()

    client = mqtt.Client(
        client_id="test_pid_publisher",
        callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
    )

    def on_connect(c, userdata, flags, rc):
        if rc == 0:
            print(f"[OK] Conectat la MQTT broker {args.broker}:{args.port}")
            print(f"[OK] Publicare pe topic: {TOPIC}  @ {args.hz} Hz")
            print("     Apasa Ctrl+C pentru a opri.\n")
        else:
            print(f"[ERR] Conectare esuata rc={rc}")

    client.on_connect = on_connect
    client.connect(args.broker, args.port, keepalive=60)
    client.loop_start()

    interval = 1.0 / args.hz
    start = time.time()
    count = 0

    try:
        while True:
            t = time.time()
            ts_ms = int(t * 1000)

            # Semnal sinusoidal — simuleaza oscilatie PID
            response = math.sin(2 * math.pi * 0.2 * (t - start))   # 0.2 Hz
            steering = response * 45.0                               # [-45°, 45°]

            payload = {
                "timestamp":      ts_ms,
                "reference":      0.0,
                "response":       round(response, 4),
                "steering_angle": round(steering, 2),
            }

            result = client.publish(TOPIC, json.dumps(payload), qos=0)
            count += 1

            if count % args.hz == 0:   # print o data pe secunda
                print(f"  [{count:>5} pkt] response={response:+.3f}  "
                      f"steering={steering:+.1f}°   ts={ts_ms}")

            # Mentine ritmul
            next_t = start + count * interval
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print(f"\n[STOP] Trimise {count} pachete PID.")

    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
