#!/bin/bash

echo "=== Instalando dependências Python para teste MQTT ==="

# Verificar se Python3 está instalado
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] Python3 não está instalado. Instalando..."
    sudo apt-get update
    sudo apt-get install -y python3 python3-pip
else
    echo "[INFO] Python3 já está instalado"
fi

# Verificar se pip3 está instalado
if ! command -v pip3 &> /dev/null; then
    echo "[ERROR] pip3 não está instalado. Instalando..."
    sudo apt-get update
    sudo apt-get install -y python3-pip
else
    echo "[INFO] pip3 já está instalado"
fi

# Instalar biblioteca MQTT Python
echo "[INFO] Instalando biblioteca MQTT Python..."
pip3 install paho-mqtt

# Verificar instalação
if python3 -c "import paho.mqtt.client" 2>/dev/null; then
    echo "[SUCCESS] Biblioteca MQTT Python instalada com sucesso!"
else
    echo "[ERROR] Falha ao instalar biblioteca MQTT Python"
    exit 1
fi

echo "[INFO] Todas as dependências foram instaladas!"
echo "[INFO] Para testar, execute: python3 test_mqtt_simple.py"
