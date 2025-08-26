#!/usr/bin/env python3
"""
Script de teste para verificar conexão MQTT com HiveMQ Cloud
"""

import paho.mqtt.client as mqtt
import ssl
import time
import json

# Configurações HiveMQ Cloud
BROKER = "972e24210b544ba49bfb9c1d3164d02b.s1.eu.hivemq.cloud"
PORT = 8883
USERNAME = "jetracer"
PASSWORD = "Ft_seame5"
TOPICS = [
    "jetracer/passadeira",
    "jetracer/stop_sign",
    "jetracer/speed_50",
    "jetracer/speed_80",
    "jetracer/jetracer",
    "jetracer/gate",
    "jetracer/lane_touch"
]

def on_connect(client, userdata, flags, rc):
    """Callback quando conecta ao broker"""
    if rc == 0:
        print(f"✅ Conectado ao HiveMQ Cloud com sucesso! (RC: {rc})")

        # Inscrever em todos os tópicos
        for topic in TOPICS:
            client.subscribe(topic)
            print(f"📡 Inscrito no tópico: {topic}")
    else:
        print(f"❌ Falha na conexão! RC: {rc}")

def on_message(client, userdata, msg):
    """Callback quando recebe uma mensagem"""
    timestamp = time.strftime("%H:%M:%S")
    print(f"[{timestamp}] 📨 {msg.topic}: {msg.payload.decode()}")

def on_disconnect(client, userdata, rc):
    """Callback quando desconecta"""
    print(f"🔌 Desconectado do broker (RC: {rc})")

def main():
    print("🚀 Iniciando teste de conexão MQTT com HiveMQ Cloud...")
    print(f"📍 Broker: {BROKER}:{PORT}")
    print(f"👤 Usuário: {USERNAME}")
    print("=" * 50)

    # Criar cliente MQTT
    client = mqtt.Client()
    client.username_pw_set(USERNAME, PASSWORD)

    # Configurar callbacks
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    # Configurar TLS
    client.tls_set(ca_certs="/etc/ssl/certs/ca-certificates.crt",
                   certfile=None,
                   keyfile=None,
                   cert_reqs=ssl.CERT_REQUIRED,
                   tls_version=ssl.PROTOCOL_TLSv1_2,
                   ciphers=None)

    try:
        # Conectar ao broker
        print("🔗 Conectando ao broker...")
        client.connect(BROKER, PORT, 60)

        # Loop principal
        print("⏳ Aguardando mensagens... (Ctrl+C para sair)")
        client.loop_forever()

    except KeyboardInterrupt:
        print("\n🛑 Interrompido pelo usuário")
    except Exception as e:
        print(f"❌ Erro: {e}")
    finally:
        client.disconnect()
        print("🔌 Conexão encerrada")

if __name__ == "__main__":
    main()
