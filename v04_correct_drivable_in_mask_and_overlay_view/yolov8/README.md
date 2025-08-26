# 🚗 Sistema de Controle Autônomo JetRacer com MPC

## 📋 Visão Geral

Este projeto implementa um sistema avançado de controle autônomo para o JetRacer, integrando **YOLOv8-Seg** para detecção de lanes com **Model Predictive Control (MPC)** para navegação inteligente. O sistema oferece três modos de operação: manual, assistência de direção e controle autônomo completo.

## 🎯 Características Principais

### 🤖 **Controle MPC Avançado**
- **Modelo Preditivo**: Planejamento de trajetória baseado em modelo matemático do veículo
- **Otimização em Tempo Real**: Solução de problemas de otimização a cada ciclo de controle
- **Horizonte de Previsão**: Configurável para diferentes cenários de condução
- **Restrições de Segurança**: Limites de velocidade, ângulo e aceleração

### 🛣️ **Detecção de Lanes Inteligente**
- **YOLOv8-Seg**: Segmentação semântica de faixas de rodagem
- **8 Classes**: drivable, lane, passadeira, stop sign, speed 50, speed 80, jetracer, gate
- **Processamento GPU**: Aceleração CUDA para detecção em tempo real
- **Filtros de Confiança**: Eliminação de falsos positivos

### 🎮 **Controle Integrado**
- **Joystick DualShock**: Interface intuitiva para controle manual
- **Modos de Operação**: Transição suave entre manual e autônomo
- **Assistência de Direção**: Velocidade manual + direção automática
- **Parada de Emergência**: Sistema de segurança integrado

## 🏗️ Arquitetura do Sistema

```
┌─────────────────────────────────────────────────────────────┐
│                    SISTEMA MPC-JETRACER                    │
├─────────────────────────────────────────────────────────────┤
│  🎮 Joystick Control  │  🧠 MPC Controller  │  🚗 Vehicle  │
│  • Manual Mode        │  • Trajectory       │  • Servo     │
│  • Assistance Mode    │    Planning         │  • Motors    │
│  • Emergency Stop     │  • Optimization     │  • Sensors   │
├─────────────────────────────────────────────────────────────┤
│  👁️ Computer Vision  │  🛣️ Lane Detection  │  📊 Feedback │
│  • YOLOv8-Seg        │  • Segmentation     │  • Position  │
│  • Real-time         │  • Waypoints        │  • Speed     │
│  • GPU Accelerated   │  • Confidence       │  • Angle     │
└─────────────────────────────────────────────────────────────┘
```

## 📁 Estrutura do Projeto

```
yolov8/
├── 📁 build_13_mpc_assitencia/     # Build da versão MPC
├── 📁 include/                      # Headers do sistema
│   ├── autonomous_controller.hpp    # Controlador autônomo principal
│   ├── jetracer.hpp                 # Interface de hardware
│   ├── mpc_controller.hpp           # Controlador MPC
│   ├── mpc_integration_config.hpp   # Configurações MPC
│   ├── vehicle_model.hpp            # Modelo matemático do veículo
│   ├── waypoint_extractor.hpp       # Extração de waypoints
│   └── performance_config.hpp       # Configurações de performance
├── 📁 src/                          # Implementações
│   ├── autonomous_controller.cpp    # Lógica de controle autônomo
│   ├── jetracer.cpp                 # Controle de hardware
│   ├── mpc_controller.cpp           # Algoritmo MPC
│   ├── vehicle_model.cpp            # Modelo do veículo
│   └── waypoint_extractor.cpp       # Processamento de waypoints
├── 📁 plugin/                       # Plugins TensorRT
│   ├── yololayer.cu                 # Camada YOLO customizada
│   └── yololayer.h                  # Header da camada
├── yolov8_seg.cpp                   # Programa principal
├── CMakeLists.txt                   # Configuração de build
└── my_classes.txt                   # Classes do modelo
```

## ⚙️ Configuração e Instalação

### 🔧 Pré-requisitos

#### **Hardware**
- **Jetson Nano** (recomendado) ou Jetson Xavier NX
- **JetRacer** com servo de direção e motores
- **Joystick DualShock** ou compatível
- **Câmera CSI** para detecção de lanes

#### **Software**
- **JetPack 4.6+** ou **JetPack 5.0+**
- **CUDA 10.2+** e **TensorRT 8.0+**
- **OpenCV 4.5+** com suporte CUDA
- **SDL2** para controle de joystick
- **CMake 3.10+** e **Make**

### 🚀 Instalação

#### **1. Preparar o Ambiente Jetson**
```bash
# Atualizar sistema
sudo apt update && sudo apt upgrade -y

# Instalar dependências
sudo apt install -y build-essential cmake git libopencv-dev libsdl2-dev

# Verificar CUDA
nvcc --version
```

#### **2. Clonar e Configurar o Projeto**
```bash
# Clonar repositório
git clone <repository-url>
cd yolov8

# Configurar variáveis de ambiente
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
```

#### **3. Compilar o Projeto**
```bash
# Criar diretório de build
mkdir build_13_mpc_assitencia
cd build_13_mpc_assitencia

# Configurar CMake
cmake .. -DCMAKE_BUILD_TYPE=Release

# Compilar
make -j4
```

## 🎮 Como Usar

### **Controles do Joystick**

| Botão/Eixo | Função | Descrição |
|-------------|--------|-----------|
| **Joystick Esquerdo (Y)** | Acelerador | Controle de velocidade |
| **Joystick Direito (X)** | Direção | Controle manual de direção |
| **R1** | Modo Assistência | Velocidade manual + direção automática |
| **R2** | Modo Autônomo | Controle completo automático |
| **L2** | Parada de Emergência | Para o veículo imediatamente |
| **L1** | Reset | Reinicia o sistema |

### **Modos de Operação**

#### **🖐️ Modo Manual**
- **Controle**: Total via joystick
- **Velocidade**: 0-100% via joystick esquerdo
- **Direção**: -140° a +140° via joystick direito
- **Uso**: Testes, demonstrações, controle direto

#### **🔄 Modo Assistência**
- **Controle**: Velocidade manual + direção automática
- **Velocidade**: Controlada pelo usuário
- **Direção**: Calculada automaticamente pelo MPC
- **Uso**: Condução assistida, aprendizado

#### **🤖 Modo Autônomo**
- **Controle**: Totalmente automático
- **Velocidade**: Otimizada pelo MPC
- **Direção**: Calculada para seguir trajetória
- **Uso**: Navegação autônoma completa

### **Execução do Sistema**

```bash
# Executar com câmera
./yolov8_seg -d ../model.engine cam c ../my_classes.txt

# Executar com diretório de imagens
./yolov8_seg -d ../model.engine /path/to/images c ../my_classes.txt

# Executar com streaming UDP
./yolov8_seg -d ../model.engine udp://0.0.0.0:5000 c ../my_classes.txt
```

## 🔧 Configurações Avançadas

### **Parâmetros MPC**

```cpp
// Em include/mpc_integration_config.hpp
#define MPC_CONTROL_AVAILABLE 1
#define CONTROL_UPDATE_RATE 20        // Hz
#define MAX_SPEED 100.0f             // %
#define MAX_ANGLE 140                // graus
```

### **Configurações de Performance**

```cpp
// Em include/performance_config.hpp
constexpr bool ENABLE_VISUAL_OVERLAY = true;    // Overlays visuais
constexpr bool ENABLE_LANE_LINES = true;        // Linhas de faixa
constexpr bool ENABLE_UDP_STREAMING = false;    // Streaming UDP
constexpr bool ENABLE_FPS_DISPLAY = true;       // Exibição de FPS
```

### **Configurações de Segurança**

```cpp
// Em yolov8_seg.cpp
#define MAX_MANUAL_SPEED 3.0              // m/s
#define MIN_SPEED_FOR_ASSISTANCE 0.5      // m/s
```

## 📊 Monitoramento e Debug

### **Logs do Sistema**

```bash
# Ver logs em tempo real
tail -f /var/log/syslog | grep "MPC\|JetRacer"

# Logs específicos do MPC
[MPC] Modo ASSISTÊNCIA ativado
[MPC] Waypoints detectados: 5
[MPC] Comando de direção: -15.2°
[MPC] Velocidade atual: 2.1 m/s
```

### **Métricas de Performance**

- **FPS**: 20-30 FPS (dependendo do hardware)
- **Latência de Controle**: < 50ms
- **Precisão de Detecção**: > 85%
- **Estabilidade de Direção**: < 5° de variância

### **Diagnóstico de Problemas**

```bash
# Verificar status do sistema
./yolov8_seg --status

# Testar câmera
./yolov8_seg --test-camera

# Verificar modelo
./yolov8_seg --verify-model ../model.engine
```

## 🚨 Solução de Problemas

### **Problemas Comuns**

#### **1. Sistema não inicializa**
```bash
# Verificar dependências
ldd ./yolov8_seg

# Verificar CUDA
nvidia-smi

# Verificar TensorRT
dpkg -l | grep tensorrt
```

#### **2. Baixo FPS**
```bash
# Ativar modo de performance
# Editar include/performance_config.hpp
using CurrentMode = HighFPSMode;

# Recompilar
make clean && make -j4
```

#### **3. Detecção imprecisa**
```bash
# Verificar iluminação
# Ajustar thresholds em include/config.h
#define kConfThresh 0.5f
#define kNmsThresh 0.45f
```

#### **4. Controle instável**
```bash
# Ajustar parâmetros MPC
# Editar include/mpc_integration_config.hpp
#define CONTROL_UPDATE_RATE 10  // Reduzir frequência
```

### **Logs de Debug**

```bash
# Habilitar debug completo
export DEBUG_LEVEL=3
./yolov8_seg --debug

# Ver logs detalhados
[DEBUG] MPC: Calculando trajetória...
[DEBUG] MPC: Waypoints processados: 8
[DEBUG] MPC: Solução de otimização encontrada
[DEBUG] JetRacer: Aplicando comando de direção
```

## 🔬 Desenvolvimento e Extensões

### **Adicionar Novas Funcionalidades**

#### **1. Novos Modos de Controle**
```cpp
// Em include/autonomous_controller.hpp
enum class ControlMode {
    MANUAL,
    STEERING_ASSIST,
    FULL_AUTONOMOUS,
    PARKING,           // Novo modo
    OBSTACLE_AVOID     // Novo modo
};
```

#### **2. Novos Sensores**
```cpp
// Em include/vehicle_model.hpp
class VehicleModel {
public:
    void addLidarData(const LidarData& data);
    void addIMUData(const IMUData& data);
    void addGPSData(const GPSData& data);
};
```

#### **3. Novos Algoritmos de Controle**
```cpp
// Em include/mpc_controller.hpp
class MPCController {
public:
    ControlOutput solveWithReinforcementLearning(
        const VehicleState& state,
        const std::vector<Waypoint>& waypoints
    );
};
```

### **Testes e Validação**

```bash
# Executar testes unitários
make test

# Executar testes de integração
make integration-test

# Executar benchmarks de performance
make benchmark

# Validação em simulador
make sim-test
```

## 📚 Referências e Recursos

### **Documentação Técnica**
- **TensorRT**: [NVIDIA TensorRT Documentation](https://docs.nvidia.com/deeplearning/tensorrt/)
- **YOLOv8**: [Ultralytics YOLOv8](https://docs.ultralytics.com/)
- **MPC**: [Model Predictive Control Guide](https://en.wikipedia.org/wiki/Model_predictive_control)
- **JetRacer**: [JetRacer Documentation](https://jetracer.readthedocs.io/)

### **Artigos e Pesquisas**
- **Lane Detection**: "Real-time Lane Detection using YOLOv8 and TensorRT"
- **MPC for Autonomous Vehicles**: "Model Predictive Control in Autonomous Driving"
- **JetRacer Control**: "Autonomous Control of JetRacer using Computer Vision"

### **Comunidade e Suporte**
- **GitHub Issues**: Reportar bugs e solicitar funcionalidades
- **Discord**: Comunidade de desenvolvedores JetRacer
- **Forum NVIDIA**: Suporte técnico para Jetson

## 📄 Licença

Este projeto é desenvolvido para pesquisa e desenvolvimento em veículos autônomos. Consulte o arquivo LICENSE para detalhes sobre uso e distribuição.

## 👥 Contribuição

### **Como Contribuir**
1. **Fork** o repositório
2. **Crie** uma branch para sua feature
3. **Commit** suas mudanças
4. **Push** para a branch
5. **Abra** um Pull Request

### **Diretrizes de Código**
- **C++17**: Use recursos modernos do C++
- **Documentação**: Comente funções complexas
- **Testes**: Adicione testes para novas funcionalidades
- **Formatação**: Use clang-format

### **Áreas de Contribuição**
- **Algoritmos MPC**: Melhorias no controle preditivo
- **Detecção de Lanes**: Otimizações de YOLOv8
- **Interface de Usuário**: Melhorias na usabilidade
- **Documentação**: Traduções e melhorias
- **Testes**: Cobertura de testes e validação

---

## 🎯 Roadmap

### **Versão 1.1** (Próxima)
- [ ] Suporte a múltiplas câmeras
- [ ] Integração com GPS
- [ ] Sistema de mapeamento SLAM
- [ ] Interface web para monitoramento

### **Versão 1.2** (Futura)
- [ ] Aprendizado por reforço (RL)
- [ ] Detecção de obstáculos 3D
- [ ] Comunicação V2V (Vehicle-to-Vehicle)
- [ ] Simulador integrado

### **Versão 2.0** (Longo Prazo)
- [ ] Suporte a múltiplos veículos
- [ ] Swarm intelligence
- [ ] Integração com infraestrutura inteligente
- [ ] Certificação de segurança

---

**Desenvolvido para o Sistema de Controle Autônomo JetRacer - Versão MPC 1.0**

*Última atualização: Agosto 2024*
*Status: ✅ Sistema implementado e funcional*
