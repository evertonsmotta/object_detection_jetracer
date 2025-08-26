# Script de Teste MQTT para YOLOv8

## Visão Geral

Mantive apenas o script `test_mqtt_topics.py` que é o mais completo e funcional para testar e simular mensagens MQTT para todos os tópicos do sistema YOLOv8.

## Tópicos MQTT Testados

O script testa os seguintes tópicos configurados no sistema:

| Tópico | Descrição | Formato |
|--------|-----------|---------|
| `jetracer/lane_touch` | Toque na faixa | JSON estruturado |
| `jetracer/passadeira` | Detecção de passadeira | JSON estruturado |
| `jetracer/stop_sign` | Detecção de sinal de parada | JSON estruturado |
| `jetracer/speed_50` | Detecção de limite 50 km/h | JSON estruturado |
| `jetracer/speed_80` | Detecção de limite 80 km/h | JSON estruturado |
| `jetracer/jetracer` | Detecção do JetRacer | JSON estruturado |
| `jetracer/gate` | Detecção de portão | JSON estruturado |

## Script Disponível

### `test_mqtt_topics.py` - **SCRIPT PRINCIPAL E COMPLETO**

**Funcionalidade**: Script completo com simulação avançada
**Uso**: Simulações realistas com dados estruturados

**Características**:
- ✅ Mensagens JSON estruturadas
- ✅ Dados simulados realistas
- ✅ Simulação contínua automática
- ✅ Estatísticas detalhadas
- ✅ Interface completa com menu
- ✅ Tratamento robusto de erros
- ✅ Reconexão automática

**Funcionalidades avançadas**:
- **Mensagens JSON**: Dados estruturados com timestamp, confiança, etc.
- **Simulação realista**: Valores de confiança, posições, distâncias
- **Ciclos automáticos**: Simulação contínua a cada 10 segundos
- **Logs detalhados**: Informações completas de cada mensagem
- **Menu interativo**: Controle total sobre os testes

## Como Usar

### Execução do Script
```bash
python3 test_mqtt_topics.py
```

### Opções do Menu
1. **Testar conexão MQTT**: Verifica status da conexão
2. **Publicar mensagem única para todos os tópicos**: Envia uma mensagem para cada tópico
3. **Iniciar simulação contínua**: Loop automático de simulação
4. **Parar simulação**: Interrompe simulação em andamento
5. **Mostrar estatísticas**: Exibe estatísticas da sessão
6. **Sair**: Encerra o script

## Exemplo de Mensagem JSON Enviada

```json
{
  "speed_limit": 80,
  "detected": true,
  "timestamp": "2025-01-26 15:30:45",
  "confidence": 0.95,
  "distance": 25.3
}
```

## Instalação e Dependências

### 1. Verificar Python3
```bash
python3 --version
```

### 2. Instalar biblioteca MQTT
```bash
pip3 install paho-mqtt
```

### 3. Tornar script executável
```bash
chmod +x test_mqtt_topics.py
```

## Verificação no HiveMQ Cloud

Após executar o script, você pode verificar as mensagens no HiveMQ Cloud:

1. **Acesse**: [HiveMQ Cloud](https://www.hivemq.com/cloud/)
2. **Faça login** com suas credenciais
3. **Vá para**: Web Client
4. **Inscreva-se** no tópico: `jetracer/#` (wildcard para todos os tópicos)
5. **Observe** as mensagens chegando em tempo real

## Exemplos de Uso

### Teste Básico
```bash
# Executar script
python3 test_mqtt_topics.py

# Escolher opção 2 para teste único
# Escolher opção 3 para simulação contínua
```

### Simulação Contínua
```bash
python3 test_mqtt_topics.py

# Escolher opção 3 para simulação contínua
# Observar mensagens JSON no HiveMQ Cloud
# Pressionar Ctrl+C para parar
```

## Troubleshooting

### Erro de Conexão
```
[ERROR] Falha ao conectar: [Errno 110] Connection timed out
```
**Solução**: Verificar conectividade de rede e firewall

### Erro de Autenticação
```
[MQTT] Falha na conexão, código: 4
[MQTT] Descrição: Credenciais inválidas
```
**Solução**: Verificar usuário e senha no script

### Erro de TLS
```
[MQTT] Falha na conexão, código: 1
[MQTT] Descrição: Protocolo incorreto
```
**Solução**: Verificar se a porta 8883 está acessível

## Status Confirmado

✅ **Script funcionando perfeitamente**
✅ **Mensagens chegando no HiveMQ Cloud**
✅ **Formato JSON estruturado**
✅ **Dados simulados realistas**
✅ **Interface completa e funcional**

## Próximos Passos

1. **Execute o script** para verificar conectividade MQTT
2. **Monitore no HiveMQ Cloud** para ver as mensagens
3. **Use a simulação contínua** para testes prolongados
4. **Integre com seu sistema** principal quando necessário

## Suporte

Para problemas ou dúvidas:
1. Verifique se as dependências estão instaladas
2. Confirme conectividade de rede
3. Teste com o script principal
4. Verifique logs de erro detalhados
5. Confirme credenciais MQTT corretas
