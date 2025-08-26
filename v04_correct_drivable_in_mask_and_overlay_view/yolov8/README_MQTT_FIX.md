# Correção do Sistema MQTT - YOLOv8 Segmentation

## Problema Identificado

O sistema estava enfrentando erros "Broken pipe" na comunicação MQTT, indicando que:
- A conexão MQTT era perdida durante a execução
- Não havia sistema de reconexão automática
- As funções de publicação não verificavam o estado da conexão
- O loop MQTT não estava sendo executado adequadamente

## Soluções Implementadas

### 1. Sistema de Reconexão Automática
- **Verificação de estado**: Todas as funções de publicação agora verificam se a conexão está ativa
- **Reconexão automática**: Sistema tenta reconectar automaticamente a cada 30 frames
- **Thread dedicada**: Loop MQTT executado em thread separada para não bloquear o processamento principal

### 2. Callbacks MQTT Robustos
- **on_connect**: Gerencia o estado de conexão
- **on_disconnect**: Detecta desconexões automaticamente
- **Tratamento de erros**: Códigos de erro MQTT são interpretados e logados

### 3. Tratamento de Sinal para Encerramento Limpo
- **SIGINT/SIGTERM**: Captura Ctrl+C e sinais de encerramento
- **Limpeza automática**: Encerra threads MQTT e desconecta do broker
- **Saída limpa**: Evita vazamentos de memória e conexões órfãs

### 4. Configuração TLS para HiveMQ Cloud
- **Porta 8883**: Configuração correta para conexões seguras
- **TLS inseguro**: Configuração necessária para testes (pode ser ajustada para produção)

## Arquivos Modificados

### `yolov8_seg.cpp`
- Adicionadas variáveis globais para controle de estado MQTT
- Implementadas funções de callback MQTT
- Adicionado sistema de reconexão automática
- Implementado tratamento de sinal para encerramento limpo
- Melhorado tratamento de erros em todas as funções de publicação

## Arquivos de Teste Criados

### `test_mqtt_connection.cpp`
- Programa C++ para testar conectividade MQTT
- Compilação: `make -f Makefile_test`
- Execução: `./test_mqtt_connection`

### `test_mqtt_simple.py`
- Script Python para teste MQTT
- Dependências: `./install_python_deps.sh`
- Execução: `python3 test_mqtt_simple.py`

### `Makefile_test`
- Makefile para compilar o teste C++
- Comandos: `make`, `make clean`, `make run`

## Como Usar

### 1. Testar Conectividade MQTT
```bash
# Opção 1: Teste Python (recomendado para debug)
./install_python_deps.sh
python3 test_mqtt_simple.py

# Opção 2: Teste C++ (requer libmosquitto-dev)
make -f Makefile_test
./test_mqtt_connection
```

### 2. Executar Sistema Principal
```bash
# Compilar o sistema principal
make

# Executar com câmera
./yolov8_seg -d best_202507181755.engine cam c my_classes.txt
```

## Monitoramento

O sistema agora fornece logs detalhados sobre o estado MQTT:
- `[MQTT] Status da conexão: CONECTADO/DESCONECTADO` (a cada 60 frames)
- `[MQTT] Status: Desconectado - tentando reconectar...` (quando necessário)
- `[MQTT] Conectado com sucesso ao broker!` (após reconexão)

## Configurações MQTT

### HiveMQ Cloud (Ativo)
- **Broker**: `972e24210b544ba49bfb9c1d3164d02b.s1.eu.hivemq.cloud`
- **Porta**: `8883` (TLS)
- **Usuário**: `jetracer`
- **Senha**: `Ft_seame5`

### Broker Local (Comentado)
- **Broker**: `10.21.221.67`
- **Porta**: `1883` (não-TLS)

## Troubleshooting

### Erro "Broken pipe"
- O sistema agora detecta automaticamente e tenta reconectar
- Verifique logs para status da conexão
- Teste conectividade com os scripts de teste

### Falha na conexão
- Verifique credenciais MQTT
- Teste conectividade de rede
- Use scripts de teste para isolar problemas

### Performance
- O sistema MQTT roda em thread separada
- Não impacta o processamento de vídeo
- Reconexão automática sem interrupção do streaming

## Próximos Passos

1. **Testar conectividade** com os scripts fornecidos
2. **Verificar logs** do sistema principal para status MQTT
3. **Monitorar reconexões** automáticas durante execução
4. **Ajustar configurações** conforme necessário para seu ambiente

## Suporte

Para problemas específicos:
1. Execute os scripts de teste para verificar conectividade
2. Verifique logs do sistema principal
3. Confirme configurações de rede e firewall
4. Teste com broker MQTT local se necessário
