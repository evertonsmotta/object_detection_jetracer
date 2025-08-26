#!/usr/bin/env python3
"""
Script de Teste para Tópicos MQTT do Sistema YOLOv8
Simula e envia mensagens para todos os tópicos específicos
"""

import paho.mqtt.client as mqtt
import time
import signal
import sys
import json
import random
from datetime import datetime

# Configurações MQTT
BROKER = "972e24210b544ba49bfb9c1d3164d02b.s1.eu.hivemq.cloud"
PORT = 8883
USERNAME = "jetracer"
PASSWORD = "Ft_seame5"

# Tópicos MQTT do sistema
TOPICS = {
    "lane_touch": "jetracer/lane_touch",
    "passadeira": "jetracer/passadeira",
    "stop_sign": "jetracer/stop_sign",
    "speed_50": "jetracer/speed_50",
    "speed_80": "jetracer/speed_80",
    "jetracer": "jetracer/jetracer",
    "gate": "jetracer/gate"
}

# Variáveis globais
client = None
connected = False
message_count = 0
simulation_running = False

def signal_handler(sig, frame):
    """Tratamento de sinal para encerramento limpo"""
    print("\n[INFO] Sinal recebido - Encerrando simulação...")
    stop_simulation()
    if client:
        client.disconnect()
    sys.exit(0)

def on_connect(client, userdata, flags, rc):
    """Callback de conexão"""
    global connected
    if rc == 0:
        print("[MQTT] Conectado com sucesso ao broker!")
        connected = True
    else:
        print(f"[MQTT] Falha na conexão, código: {rc}")
        error_messages = {
            1: "Protocolo incorreto",
            2: "Identificador de cliente inválido",
            3: "Servidor indisponível",
            4: "Credenciais inválidas",
            5: "Não autorizado"
        }
        error_msg = error_messages.get(rc, "Erro desconhecido")
        print(f"[MQTT] Descrição: {error_msg}")
        connected = False

def on_disconnect(client, userdata, rc):
    """Callback de desconexão"""
    global connected
    print(f"[MQTT] Desconectado do broker, código: {rc}")
    connected = False

def on_publish(client, userdata, mid):
    """Callback de publicação"""
    print(f"[MQTT] Mensagem publicada com sucesso, ID: {mid}")

def on_log(client, userdata, level, buf):
    """Callback de log para debug"""
    if level <= mqtt.MQTT_LOG_WARNING:  # Apenas logs importantes
        print(f"[MQTT LOG] {buf}")

def create_test_message(topic_name, value=None):
    """Cria mensagem de teste baseada no tópico"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    if topic_name == "lane_touch":
        # Simula toque na faixa (0 ou 1)
        value = random.choice([0, 1])
        return {
            "value": value,
            "timestamp": timestamp,
            "description": "Lane touch detected" if value else "No lane touch",
            "confidence": random.uniform(0.7, 0.95)
        }

    elif topic_name in ["passadeira", "stop_sign", "gate"]:
        # Objetos binários (detectado ou não)
        value = random.choice([0, 1])
        return {
            "detected": bool(value),
            "timestamp": timestamp,
            "confidence": random.uniform(0.8, 0.98),
            "position": {
                "x": random.uniform(100, 500),
                "y": random.uniform(100, 400)
            }
        }

    elif topic_name in ["speed_50", "speed_80"]:
        # Sinais de velocidade
        speed_value = int(topic_name.split("_")[1])
        value = random.choice([0, 1])
        return {
            "speed_limit": speed_value,
            "detected": bool(value),
            "timestamp": timestamp,
            "confidence": random.uniform(0.85, 0.99),
            "distance": random.uniform(5.0, 50.0)
        }

    elif topic_name == "jetracer":
        # Status do JetRacer
        value = random.choice([0, 1])
        return {
            "detected": bool(value),
            "timestamp": timestamp,
            "confidence": random.uniform(0.9, 0.99),
            "status": "active" if value else "inactive",
            "battery": random.uniform(20.0, 100.0),
            "speed": random.uniform(0.0, 30.0)
        }

    else:
        # Tópico desconhecido
        return {
            "value": value or random.randint(0, 100),
            "timestamp": timestamp,
            "topic": topic_name
        }

def publish_test_message(topic_name, topic_path):
    """Publica mensagem de teste para um tópico específico"""
    global message_count

    if not connected:
        print(f"[WARNING] MQTT não conectado, pulando {topic_name}")
        return False

    # Criar mensagem de teste
    message_data = create_test_message(topic_name)

    # Converter para JSON
    message_json = json.dumps(message_data, indent=2)

    # Publicar mensagem
    result = client.publish(topic_path, message_json, qos=0)

    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        message_count += 1
        print(f"[PUBLISH] {topic_name}: {message_json[:100]}...")
        return True
    else:
        print(f"[ERROR] Falha ao publicar {topic_name}: {result.rc}")
        return False

def publish_all_topics():
    """Publica mensagens para todos os tópicos"""
    print("\n[INFO] Publicando mensagens para todos os tópicos...")

    success_count = 0
    for topic_name, topic_path in TOPICS.items():
        if publish_test_message(topic_name, topic_path):
            success_count += 1
        time.sleep(0.5)  # Pequena pausa entre publicações

    print(f"[INFO] Publicação concluída: {success_count}/{len(TOPICS)} tópicos enviados com sucesso")
    return success_count

def start_simulation():
    """Inicia simulação contínua"""
    global simulation_running
    simulation_running = True
    print("[INFO] Iniciando simulação contínua...")
    print("[INFO] Pressione Ctrl+C para parar")

    cycle = 0
    while simulation_running and connected:
        cycle += 1
        print(f"\n[INFO] === CICLO DE SIMULAÇÃO {cycle} ===")

        # Publicar para todos os tópicos
        publish_all_topics()

        # Aguardar próximo ciclo
        print(f"[INFO] Aguardando 10 segundos para próximo ciclo...")
        for i in range(10, 0, -1):
            if not simulation_running:
                break
            print(f"[INFO] Próximo ciclo em {i}s...", end="\r")
            time.sleep(1)
        print()

def stop_simulation():
    """Para a simulação"""
    global simulation_running
    simulation_running = False
    print("[INFO] Simulação parada")

def show_menu():
    """Mostra menu de opções"""
    print("\n" + "="*60)
    print("           SCRIPT DE TESTE MQTT - YOLOv8")
    print("="*60)
    print("1. Testar conexão MQTT")
    print("2. Publicar mensagem única para todos os tópicos")
    print("3. Iniciar simulação contínua")
    print("4. Parar simulação")
    print("5. Mostrar estatísticas")
    print("6. Sair")
    print("="*60)

def show_statistics():
    """Mostra estatísticas da sessão"""
    print(f"\n[STATS] Estatísticas da sessão:")
    print(f"  - Mensagens enviadas: {message_count}")
    print(f"  - Tópicos configurados: {len(TOPICS)}")
    print(f"  - Status MQTT: {'CONECTADO' if connected else 'DESCONECTADO'}")
    print(f"  - Simulação ativa: {'SIM' if simulation_running else 'NÃO'}")

    print(f"\n[STATS] Tópicos configurados:")
    for topic_name, topic_path in TOPICS.items():
        print(f"  - {topic_name}: {topic_path}")

def main():
    """Função principal"""
    global client, connected

    print("[INFO] Script de Teste MQTT para YOLOv8")
    print("[INFO] Use Ctrl+C para sair")

    # Configurar tratamento de sinal
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Criar cliente MQTT
    client = mqtt.Client(client_id="test_topics_client", clean_session=True)

    # Configurar callbacks
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_publish = on_publish
    client.on_log = on_log

    # Configurar autenticação
    client.username_pw_set(USERNAME, PASSWORD)

    # Configurar TLS para HiveMQ Cloud
    client.tls_set()
    client.tls_insecure_set(True)

    # Conectar ao broker
    print(f"[INFO] Conectando ao broker MQTT...")
    print(f"[INFO] Broker: {BROKER}:{PORT}")
    print(f"[INFO] Usuário: {USERNAME}")

    try:
        client.connect(BROKER, PORT, 60)
    except Exception as e:
        print(f"[ERROR] Falha ao conectar: {e}")
        return 1

    # Iniciar loop em thread separada
    client.loop_start()

    # Aguardar conexão
    timeout = 10
    start_time = time.time()
    while not connected and (time.time() - start_time) < timeout:
        time.sleep(0.1)

    if not connected:
        print("[ERROR] Timeout na conexão MQTT")
        client.loop_stop()
        client.disconnect()
        return 1

    print("[INFO] Conexão estabelecida! Iniciando interface...")

    # Loop principal do menu
    try:
        while True:
            show_menu()
            choice = input("\nEscolha uma opção (1-6): ").strip()

            if choice == "1":
                print(f"[INFO] Status da conexão: {'CONECTADO' if connected else 'DESCONECTADO'}")

            elif choice == "2":
                publish_all_topics()

            elif choice == "3":
                if not simulation_running:
                    start_simulation()
                else:
                    print("[INFO] Simulação já está rodando")

            elif choice == "4":
                if simulation_running:
                    stop_simulation()
                else:
                    print("[INFO] Nenhuma simulação ativa")

            elif choice == "5":
                show_statistics()

            elif choice == "6":
                print("[INFO] Encerrando...")
                break

            else:
                print("[ERROR] Opção inválida")

            if choice in ["2", "3", "4", "5"]:
                input("\nPressione Enter para continuar...")

    except KeyboardInterrupt:
        print("\n[INFO] Interrupção do usuário")
    except Exception as e:
        print(f"[ERROR] Erro inesperado: {e}")
    finally:
        # Limpeza
        print("[INFO] Encerrando conexão MQTT...")
        stop_simulation()
        client.loop_stop()
        client.disconnect()
        print("[INFO] Teste concluído")

    return 0

if __name__ == "__main__":
    sys.exit(main())
