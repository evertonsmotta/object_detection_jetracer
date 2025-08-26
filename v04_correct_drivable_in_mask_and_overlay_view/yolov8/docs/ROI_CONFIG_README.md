# Configuração das Dimensões do ROI (Region of Interest)

## Visão Geral

O ROI (Region of Interest) é a área da imagem onde o sistema detecta e analisa as faixas de trânsito. Este arquivo explica como alterar as dimensões e posição do ROI para otimizar a detecção de faixas.

## Arquivo de Configuração

**Localização:** `include/roi_config.h`

## Configurações Principais

### 1. **Posição Vertical (ALTURA)**

```cpp
// Posição de início do ROI (parte superior)
#define ROI_START_Y_RATIO         0.3     // 30% do topo da imagem

// Posição de fim do ROI (parte inferior)
#define ROI_END_Y_RATIO           0.8     // 80% do topo da imagem
```

**Como funciona:**
- `ROI_START_Y_RATIO`: Define onde o ROI começa (0.0 = topo, 1.0 = base)
- `ROI_END_Y_RATIO`: Define onde o ROI termina
- Os valores são porcentagens da altura total da imagem

### 2. **Posição Horizontal (LARGURA)**

```cpp
// Ativar limitação de largura do ROI
#define ENABLE_ROI_WIDTH_LIMIT    false   // true = limitar largura, false = largura total

// Limites horizontais (quando ENABLE_ROI_WIDTH_LIMIT = true)
#define ROI_START_X_RATIO         0.1     // 10% da esquerda da imagem
#define ROI_END_X_RATIO           0.9     // 90% da esquerda da imagem
```

**Por padrão:** O ROI se estende por toda a largura da imagem (`ENABLE_ROI_WIDTH_LIMIT = false`)

### 3. **Configurações de Tolerância**

```cpp
// Tolerância para posição lateral das faixas (em porcentagem da largura da imagem)
#define LANE_CENTER_TOLERANCE_RATIO_1    0.3     // 30% da largura (filtro inicial)
#define LANE_CENTER_TOLERANCE_RATIO_2    0.4     // 40% da largura (filtro de penalização)
```

**Como funciona:**
- `LANE_CENTER_TOLERANCE_RATIO_1`: Tolerância para o filtro inicial de faixas laterais
- `LANE_CENTER_TOLERANCE_RATIO_2`: Tolerância para o filtro de penalização (mais restritivo)
- Valores maiores = mais tolerante com faixas laterais
- Valores menores = mais restritivo, foca apenas no centro

### 4. **Configurações de Cores e Tamanhos**

```cpp
// Cor dos círculos no topo das lanes
#define LANE_TOP_POINT_COLOR     cv::Scalar(0, 255, 255)    // Ciano

// Tamanho dos círculos no topo das lanes
#define LANE_TOP_POINT_RADIUS    6                           // Raio em pixels
```

**Como funciona:**
- `LANE_TOP_POINT_COLOR`: Cor dos círculos no topo de cada lane (formato BGR)
- `LANE_TOP_POINT_RADIUS`: Raio dos círculos em pixels
- Os círculos sempre têm contorno branco para melhor visibilidade
- Cada lane individual tem seu próprio círculo no topo

## Exemplos de Configuração

### **Configuração Padrão (Atual)**
```cpp
#define ROI_START_Y_RATIO         0.3     // 30% do topo
#define ROI_END_Y_RATIO           0.8     // 80% do topo
```
- **Uso:** Detecção em estradas com visão média
- **Vantagem:** Balanceia proximidade e distância da câmera

### **ROI Superior (Mais Próximo da Câmera)**
```cpp
#define ROI_START_Y_RATIO         0.1     // 10% do topo
#define ROI_END_Y_RATIO           0.6     // 60% do topo
```
- **Uso:** Detecção em curvas fechadas ou estradas com muitas mudanças
- **Vantagem:** Melhor detecção de faixas próximas
- **Desvantagem:** Pode perder faixas distantes

### **ROI Inferior (Mais Distante da Câmera)**
```cpp
#define ROI_START_Y_RATIO         0.5     // 50% do topo
#define ROI_END_Y_RATIO           0.9     // 90% do topo
```
- **Uso:** Detecção em estradas retas ou com visão longa
- **Vantagem:** Melhor detecção de faixas distantes
- **Desvantagem:** Pode perder faixas próximas

### **ROI Estreito (Apenas Centro)**
```cpp
#define ENABLE_ROI_WIDTH_LIMIT    true
#define ROI_START_X_RATIO         0.3     // 30% da esquerda
#define ROI_END_X_RATIO           0.7     // 70% da esquerda
```
- **Uso:** Evitar detecções laterais indesejadas
- **Vantagem:** Foco apenas na área central da estrada
- **Desvantagem:** Pode perder faixas em curvas

### **Configurações de Tolerância**

#### **Tolerância Alta (Mais Permissivo)**
```cpp
#define LANE_CENTER_TOLERANCE_RATIO_1    0.4     // 40% da largura
#define LANE_CENTER_TOLERANCE_RATIO_2    0.5     // 50% da largura
```
- **Uso:** Estradas largas ou com múltiplas faixas
- **Vantagem:** Detecta faixas laterais
- **Desvantagem:** Pode incluir objetos indesejados

#### **Tolerância Baixa (Mais Restritivo)**
```cpp
#define LANE_CENTER_TOLERANCE_RATIO_1    0.2     // 20% da largura
#define LANE_CENTER_TOLERANCE_RATIO_2    0.3     // 30% da largura
```
- **Uso:** Estradas estreitas ou túneis
- **Vantagem:** Foco apenas no centro
- **Desvantagem:** Pode perder faixas em curvas

### **Configurações de Cores das Lanes**

#### **Cores Padrão**
```cpp
#define LANE_TOP_POINT_COLOR     cv::Scalar(0, 255, 255)    // Ciano
#define LANE_TOP_POINT_RADIUS    6                           // Raio de 6 pixels
```

#### **Cores Personalizadas**
```cpp
// Círculos azuis
#define LANE_TOP_POINT_COLOR     cv::Scalar(255, 0, 0)      // Azul

// Círculos verdes
#define LANE_TOP_POINT_COLOR     cv::Scalar(0, 255, 0)      // Verde

// Círculos grandes
#define LANE_TOP_POINT_RADIUS    10                          // Raio de 10 pixels
```

## Como Alterar

### **Passo 1: Editar o arquivo de configuração**
```bash
nano include/roi_config.h
```

### **Passo 2: Modificar os valores desejados**
```cpp
// Exemplo: ROI mais próximo da câmera
#define ROI_START_Y_RATIO         0.2     // 20% do topo
#define ROI_END_Y_RATIO           0.7     // 70% do topo
```

### **Passo 3: Recompilar o projeto**
```bash
cd build
make clean
make -j4
```

## Recomendações por Cenário

### **Estradas Urbanas (Muitas Curvas)**
```cpp
#define ROI_START_Y_RATIO         0.2     // ROI mais próximo
#define ROI_END_Y_RATIO           0.7
```

### **Estradas Rurais (Retas Longas)**
```cpp
#define ROI_START_Y_RATIO         0.4     // ROI mais distante
#define ROI_END_Y_RATIO           0.9
```

### **Túneis ou Condições de Baixa Visibilidade**
```cpp
#define ROI_START_Y_RATIO         0.3     // ROI médio
#define ROI_END_Y_RATIO           0.8
#define ENABLE_ROI_WIDTH_LIMIT    true    // Limitar largura
#define ROI_START_X_RATIO         0.2
#define ROI_END_X_RATIO           0.8
```

## Validação das Configurações

### **Verificar se os valores são válidos:**
- `ROI_START_Y_RATIO` deve ser **MENOR** que `ROI_END_Y_RATIO`
- Todos os valores devem estar entre **0.0** e **1.0**
- Se `ENABLE_ROI_WIDTH_LIMIT = true`, `ROI_START_X_RATIO` deve ser **MENOR** que `ROI_END_X_RATIO`

### **Testar em diferentes condições:**
1. **Estrada reta:** Verificar se detecta faixas distantes
2. **Curvas:** Verificar se detecta faixas próximas
3. **Condições de luz:** Verificar se o ROI não está muito próximo ou distante

## Troubleshooting

### **Problema: ROI muito pequeno**
- **Sintoma:** Detecção de faixas inconsistente
- **Solução:** Aumentar a diferença entre `ROI_START_Y_RATIO` e `ROI_END_Y_RATIO`

### **Problema: ROI muito próximo da câmera**
- **Sintoma:** Perde faixas distantes
- **Solução:** Aumentar `ROI_END_Y_RATIO` (ex: de 0.8 para 0.9)

### **Problema: ROI muito distante da câmera**
- **Sintoma:** Perde faixas próximas
- **Solução:** Diminuir `ROI_START_Y_RATIO` (ex: de 0.3 para 0.2)

### **Problema: ROI muito largo**
- **Sintoma:** Detecta objetos laterais indesejados
- **Solução:** Ativar `ENABLE_ROI_WIDTH_LIMIT = true` e ajustar `ROI_START_X_RATIO` e `ROI_END_X_RATIO`

### **Problema: Detecta muitas faixas laterais indesejadas**
- **Sintoma:** Sistema detecta objetos que não são faixas principais
- **Solução:** Diminuir `LANE_CENTER_TOLERANCE_RATIO_1` e `LANE_CENTER_TOLERANCE_RATIO_2`

### **Problema: Perde faixas em curvas**
- **Sintoma:** Sistema não detecta faixas que estão lateralmente
- **Solução:** Aumentar `LANE_CENTER_TOLERANCE_RATIO_1` e `LANE_CENTER_TOLERANCE_RATIO_2`

### **Problema: Círculos no topo das lanes não aparecem**
- **Sintoma:** Não vê círculos cianos no topo das lanes
- **Solução:** Verificar se `SHOW_LANE_TOP_POINTS` está `true` em `visualization_config.h`

### **Problema: Cores dos círculos incorretas**
- **Sintoma:** Círculos no topo das lanes têm cor errada
- **Solução:** Verificar `LANE_TOP_POINT_COLOR` em `roi_config.h`

## Performance

### **Impacto no FPS:**
- **ROI pequeno:** Menor impacto (~1-2 FPS)
- **ROI grande:** Maior impacto (~3-5 FPS)
- **ROI com limitação de largura:** Impacto mínimo adicional

### **Recomendações de Performance:**
- Use ROI menor para máxima performance
- Use ROI maior para máxima precisão
- Balanceie conforme suas necessidades

## Arquivos Afetados

As configurações do ROI afetam os seguintes arquivos:
- `src/postprocess.cpp` - Lógica principal de detecção
- `include/postprocess.h` - Headers incluídos

## Suporte

Para dúvidas ou problemas com a configuração do ROI:
1. Verifique se os valores estão dentro dos limites válidos
2. Teste com configurações padrão primeiro
3. Faça alterações incrementais e teste cada uma
4. Recompile sempre após alterações
