# Resumo das Alterações - Configuração Centralizada do ROI

## Arquivos Criados/Modificados

### 1. **Novo Arquivo: `include/roi_config.h`**
- **Propósito:** Centralizar todas as configurações relacionadas ao ROI
- **Conteúdo:** Dimensões, posições, tolerâncias e configurações avançadas

### 2. **Modificado: `include/postprocess.h`**
- **Alteração:** Adicionado `#include "roi_config.h"`
- **Propósito:** Incluir as configurações centralizadas do ROI

### 3. **Modificado: `src/postprocess.cpp`**
- **Alterações:** Substituídos todos os valores hardcoded por constantes configuráveis
- **Linhas afetadas:** 445-446, 999-1000, 1130-1131, 1280-1281, 350, 550, 356, 565

### 4. **Novo Arquivo: `ROI_CONFIG_README.md`**
- **Propósito:** Documentação completa de como configurar o ROI
- **Conteúdo:** Exemplos, troubleshooting, recomendações por cenário

## Configurações Centralizadas

### **Dimensões do ROI (Altura)**
```cpp
// ANTES (hardcoded)
int roi_start_y = img.rows * 0.3;  // 30% do topo
int roi_end_y = img.rows * 0.8;    // 80% do topo

// DEPOIS (configurável)
int roi_start_y = img.rows * ROI_START_Y_RATIO;  // Configurável via roi_config.h
int roi_end_y = img.rows * ROI_END_Y_RATIO;      // Configurável via roi_config.h
```

### **Tolerâncias de Detecção**
```cpp
// ANTES (hardcoded)
float center_tolerance = img.cols * 0.3;
float center_tolerance = img.cols * 0.4;

// DEPOIS (configurável)
float center_tolerance = img.cols * LANE_CENTER_TOLERANCE_RATIO_1;  // Configurável via roi_config.h
float center_tolerance = img.cols * LANE_CENTER_TOLERANCE_RATIO_2;  // Configurável via roi_config.h
```

## Benefícios das Alterações

### **1. Centralização**
- Todas as configurações do ROI em um único arquivo
- Fácil manutenção e alteração
- Sem necessidade de procurar valores hardcoded no código

### **2. Flexibilidade**
- Configurações adaptáveis a diferentes cenários
- Possibilidade de ajuste fino sem recompilação
- Suporte a diferentes tipos de estrada

### **3. Manutenibilidade**
- Código mais limpo e legível
- Configurações documentadas e organizadas
- Fácil troubleshooting e ajustes

## Como Usar

### **Passo 1: Editar configurações**
```bash
nano include/roi_config.h
```

### **Passo 2: Ajustar valores conforme necessário**
```cpp
// Exemplo: ROI mais próximo da câmera
#define ROI_START_Y_RATIO         0.2     // 20% do topo
#define ROI_END_Y_RATIO           0.7     // 70% do topo

// Exemplo: Tolerância mais restritiva
#define LANE_CENTER_TOLERANCE_RATIO_1    0.2     // 20% da largura
#define LANE_CENTER_TOLERANCE_RATIO_2    0.3     // 30% da largura
```

### **Passo 3: Recompilar**
```bash
cd build
make clean
make -j4
```

## Configurações Recomendadas

### **Estradas Urbanas (Muitas Curvas)**
```cpp
#define ROI_START_Y_RATIO         0.2     // ROI mais próximo
#define ROI_END_Y_RATIO           0.7
#define LANE_CENTER_TOLERANCE_RATIO_1    0.3     // Tolerância média
#define LANE_CENTER_TOLERANCE_RATIO_2    0.4
```

### **Estradas Rurais (Retas Longas)**
```cpp
#define ROI_START_Y_RATIO         0.4     // ROI mais distante
#define ROI_END_Y_RATIO           0.9
#define LANE_CENTER_TOLERANCE_RATIO_1    0.2     // Tolerância baixa
#define LANE_CENTER_TOLERANCE_RATIO_2    0.3
```

### **Túneis ou Condições Difíceis**
```cpp
#define ROI_START_Y_RATIO         0.3     // ROI médio
#define ROI_END_Y_RATIO           0.8
#define ENABLE_ROI_WIDTH_LIMIT    true    // Limitar largura
#define ROI_START_X_RATIO         0.2
#define ROI_END_X_RATIO           0.8
#define LANE_CENTER_TOLERANCE_RATIO_1    0.2     // Tolerância baixa
#define LANE_CENTER_TOLERANCE_RATIO_2    0.3
```

## Validação

### **Verificar se as alterações foram aplicadas:**
1. **Compilação:** O projeto deve compilar sem erros
2. **Execução:** O sistema deve funcionar com as novas configurações
3. **Visualização:** O ROI deve aparecer nas posições configuradas

### **Testar em diferentes cenários:**
1. **Estrada reta:** Verificar detecção de faixas distantes
2. **Curvas:** Verificar detecção de faixas próximas
3. **Condições de luz:** Verificar robustez da detecção

## Troubleshooting

### **Problema: Erro de compilação**
- **Causa:** Arquivo `roi_config.h` não encontrado
- **Solução:** Verificar se o arquivo está em `include/roi_config.h`

### **Problema: ROI não aparece**
- **Causa:** Valores inválidos nas configurações
- **Solução:** Verificar se `ROI_START_Y_RATIO < ROI_END_Y_RATIO`

### **Problema: Sistema não detecta faixas**
- **Causa:** ROI muito pequeno ou mal posicionado
- **Solução:** Ajustar dimensões e posição do ROI

## Próximos Passos

### **1. Testar configurações padrão**
- Executar o sistema com as configurações atuais
- Verificar se a detecção está funcionando corretamente

### **2. Ajustar conforme necessário**
- Identificar problemas específicos do seu cenário
- Ajustar configurações incrementalmente
- Testar cada alteração

### **3. Otimizar para seu ambiente**
- Ajustar ROI para suas condições específicas
- Otimizar tolerâncias para melhor detecção
- Balancear performance e precisão

## Suporte

Para dúvidas ou problemas:
1. Verifique a documentação em `ROI_CONFIG_README.md`
2. Teste com configurações padrão primeiro
3. Faça alterações incrementais
4. Recompile sempre após alterações
