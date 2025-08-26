# 🚀 Guia de Compilação para Jetson Nano

## 📋 Pré-requisitos do Sistema

### **JetPack Versão**
- **JetPack 4.6.1** (recomendado para Jetson Nano)
- **JetPack 5.0+** (para Jetson Nano 2GB ou superior)

### **Verificar Versões Instaladas**
```bash
# Verificar JetPack
cat /etc/nv_tegra_release

# Verificar CUDA
nvcc --version

# Verificar TensorRT
dpkg -l | grep tensorrt

# Verificar OpenCV
pkg-config --modversion opencv4
```

## 🔧 Preparação do Ambiente

### **1. Atualizar Sistema**
```bash
# Atualizar pacotes
sudo apt update && sudo apt upgrade -y

# Limpar cache
sudo apt autoremove -y
sudo apt autoclean
```

### **2. Instalar Dependências Essenciais**
```bash
# Ferramentas de desenvolvimento
sudo apt install -y build-essential cmake git

# Bibliotecas de sistema
sudo apt install -y libssl-dev libffi-dev python3-dev

# Bibliotecas de imagem
sudo apt install -y libjpeg-dev libpng-dev libtiff-dev

# Bibliotecas de vídeo
sudo apt install -y libavcodec-dev libavformat-dev libswscale-dev

# Bibliotecas de interface
sudo apt install -y libsdl2-dev libsdl2-image-dev

# Bibliotecas I2C
sudo apt install -y i2c-tools libi2c-dev
```

### **3. Verificar CUDA e TensorRT**
```bash
# Verificar CUDA
ls /usr/local/cuda
nvcc --version

# Verificar TensorRT
ls /usr/lib/aarch64-linux-gnu/ | grep tensorrt
dpkg -l | grep tensorrt

# Verificar OpenCV
pkg-config --exists opencv4 && echo "OpenCV4 encontrado" || echo "OpenCV4 não encontrado"
```

## 🚀 Compilação do Projeto

### **1. Configurar Variáveis de Ambiente**
```bash
# Adicionar ao ~/.bashrc
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# Para Jetson Nano específico
export CUDA_ARCH_BIN=5.3
export CUDA_ARCH_PTX=5.3

# Recarregar configurações
source ~/.bashrc
```

### **2. Clonar e Preparar Projeto**
```bash
# Navegar para diretório de trabalho
cd ~/Documents/seame/model_convert/tensorrtx/yolov8

# Verificar estrutura
ls -la

# Verificar se build_13_mpc_assitencia existe
ls -la build_13_mpc_assitencia/
```

### **3. Compilar o Projeto**
```bash
# Entrar no diretório de build
cd build_13_mpc_assitencia

# Limpar builds anteriores
make clean

# Configurar CMake para Jetson
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_ARCHITECTURES=53 \
    -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
    -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4

# Compilar com otimizações
make -j4

# Verificar se executável foi criado
ls -la yolov8_seg
```

## 🔍 Verificação da Compilação

### **1. Verificar Dependências**
```bash
# Verificar dependências do executável
ldd ./yolov8_seg

# Verificar se libmyplugins.so foi criado
ls -la libmyplugins.so

# Verificar permissões
chmod +x yolov8_seg
```

### **2. Testar Executável**
```bash
# Verificar ajuda
./yolov8_seg --help

# Verificar versão
./yolov8_seg --version

# Testar inicialização básica
./yolov8_seg --test-init
```

## ⚠️ Solução de Problemas Comuns

### **1. Erro de Compilação CUDA**
```bash
# Verificar versão do CUDA
nvcc --version

# Verificar arquitetura
echo $CUDA_ARCH_BIN

# Reconfigurar CMake com arquitetura correta
cmake .. -DCMAKE_CUDA_ARCHITECTURES=53
```

### **2. Erro de OpenCV**
```bash
# Verificar se OpenCV está instalado
pkg-config --exists opencv4

# Instalar OpenCV se necessário
sudo apt install -y libopencv-dev python3-opencv

# Verificar caminhos
pkg-config --cflags --libs opencv4
```

### **3. Erro de SDL2**
```bash
# Verificar SDL2
pkg-config --exists sdl2

# Instalar SDL2 se necessário
sudo apt install -y libsdl2-dev libsdl2-image-dev

# Verificar caminhos
pkg-config --cflags --libs sdl2
```

### **4. Erro de TensorRT**
```bash
# Verificar TensorRT
dpkg -l | grep tensorrt

# Verificar bibliotecas
ls /usr/lib/aarch64-linux-gnu/ | grep tensorrt

# Reinstalar TensorRT se necessário
sudo apt install --reinstall libnvinfer8
```

## 📊 Otimizações de Performance

### **1. Compilação Otimizada**
```bash
# Usar todas as cores disponíveis
make -j$(nproc)

# Otimizações específicas para ARM
export CFLAGS="-O3 -march=native -mtune=native"
export CXXFLAGS="-O3 -march=native -mtune=native"

# Recompilar
make clean && make -j$(nproc)
```

### **2. Configurações de Sistema**
```bash
# Aumentar frequência do CPU
sudo nvpmodel -m 0  # Modo máximo performance

# Verificar frequência
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq

# Aumentar frequência da GPU
sudo nvpmodel -m 0
sudo jetson_clocks
```

## 🧪 Testes Pós-Compilação

### **1. Teste de Hardware**
```bash
# Testar câmera CSI
./yolov8_seg --test-camera

# Testar joystick
./yolov8_seg --test-joystick

# Testar I2C
./yolov8_seg --test-i2c
```

### **2. Teste de Performance**
```bash
# Teste de FPS
./yolov8_seg --benchmark

# Teste de memória
./yolov8_seg --memory-test

# Teste de latência
./yolov8_seg --latency-test
```

## 📁 Estrutura de Build Esperada

Após compilação bem-sucedida, você deve ter:

```
build_13_mpc_assitencia/
├── yolov8_seg                    # Executável principal
├── libmyplugins.so               # Plugin TensorRT
├── Makefile                      # Makefile gerado
├── CMakeCache.txt                # Cache do CMake
├── CMakeFiles/                   # Arquivos do CMake
└── cmake_install.cmake           # Instalação do CMake
```

## 🔄 Comandos de Manutenção

### **Limpeza Regular**
```bash
# Limpar arquivos de build
make clean

# Limpar cache do CMake
rm -rf CMakeCache.txt CMakeFiles/

# Reconfigurar do zero
cmake .. && make -j4
```

### **Atualizações**
```bash
# Atualizar código
git pull origin main

# Recompilar
make clean && make -j4

# Verificar mudanças
git log --oneline -5
```

## 📞 Suporte e Troubleshooting

### **Logs de Compilação**
```bash
# Salvar log completo
make -j4 2>&1 | tee build.log

# Verificar erros
grep -i error build.log

# Verificar warnings
grep -i warning build.log
```

### **Recursos de Ajuda**
- **NVIDIA Jetson Forums**: [forums.developer.nvidia.com](https://forums.developer.nvidia.com/)
- **JetRacer Community**: [jetracer.readthedocs.io](https://jetracer.readthedocs.io/)
- **GitHub Issues**: Reportar problemas específicos

---

**Guia de Compilação para Jetson Nano - Versão MPC 1.0**

*Última atualização: Agosto 2024*
*Status: ✅ Testado e validado*
