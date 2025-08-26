# Configuração de Visualização - Sistema de Detecção de Faixas

## Visão Geral

Este sistema permite controlar individualmente cada elemento visual da detecção de faixas, permitindo personalizar a interface conforme suas necessidades de performance ou clareza visual.

## Arquivo de Configuração

As configurações estão localizadas em: `include/visualization_config.h`

## Como Configurar

### 1. **Configurações Básicas**

Para ativar/desativar elementos, altere os valores de `true` para `false`:

```cpp
// Exemplo: Ocultar o retângulo do ROI
#define SHOW_ROI_RECTANGLE        false   // Retângulo do ROI

// Exemplo: Mostrar informações de debug
#define SHOW_DEBUG_INFO          true    // Informações de debug
```

### 2. **Elementos Controláveis**

#### **ROI (Region of Interest)**
- `SHOW_ROI_RECTANGLE` - Retângulo que delimita a região de interesse
- `SHOW_ROI_TEXT` - Textos como "ROI", "ROI (Two Lanes)", etc.
- `SHOW_ROI_BASE_CENTER` - Círculo e texto "ROI Base Center"
- `SHOW_ROI_TOP_CENTER` - **NOVO!** Círculo e texto "ROI Top Center"

#### **Centro da Pista**
- `SHOW_CENTER_POINT` - Círculo verde do centro calculado
- `SHOW_CENTER_TEXT` - Texto "Center"
- `SHOW_CENTER_REFERENCE_LINE` - Linha de referência vertical

#### **Centro Estimado (uma faixa)**
- `SHOW_ESTIMATED_CENTER` - Círculo ciano do centro estimado
- `SHOW_ESTIMATED_TEXT` - Texto "Est. Center"
- `SHOW_ESTIMATED_REFERENCE_LINE` - Linha de referência vertical

#### **Conexões e Linhas**
- `SHOW_CENTER_TO_LANES_CONNECTION` - **NOVO!** Linhas conectando centro às faixas
- `SHOW_LANE_BASE_POINTS` - Círculos vermelhos na base das faixas
- `SHOW_LANE_LABELS` - Textos "Left Lane Used", "Right Lane Used"

#### **Debug**
- `SHOW_DEBUG_INFO` - Coordenadas, medidas físicas, etc.
- `SHOW_CONFLICT_RESOLUTION` - Informações de resolução de conflitos

### 3. **Personalização de Cores**

```cpp
// Exemplo: Alterar cor do ROI para azul
#define ROI_RECTANGLE_COLOR      cv::Scalar(255, 0, 0)    // Azul

// Exemplo: Alterar cor do centro para amarelo
#define CENTER_POINT_COLOR       cv::Scalar(0, 255, 255)  // Amarelo
```

### 4. **Personalização de Tamanhos**

```cpp
// Exemplo: Aumentar tamanho do círculo do centro
#define CENTER_POINT_RADIUS         12

// Exemplo: Aumentar espessura das linhas de conexão
#define CONNECTION_LINE_THICKNESS   2
```

## Exemplos de Configuração

### **Configuração Mínima (Máximo FPS)**
```cpp
#define SHOW_ROI_RECTANGLE        false
#define SHOW_ROI_TEXT            false
#define SHOW_ROI_BASE_CENTER     false
#define SHOW_LANE_TOP_POINTS     false
#define SHOW_CENTER_POINT        false
#define SHOW_CENTER_TEXT         false
#define SHOW_CENTER_REFERENCE_LINE false
#define SHOW_ESTIMATED_CENTER    false
#define SHOW_ESTIMATED_TEXT      false
#define SHOW_ESTIMATED_REFERENCE_LINE false
#define SHOW_CENTER_TO_LANES_CONNECTION false
#define SHOW_LANE_BASE_POINTS    false
#define SHOW_LANE_LABELS         false
#define SHOW_DEBUG_INFO          false
```

### **Configuração Completa (Máxima Informação)**
```cpp
#define SHOW_ROI_RECTANGLE        true
#define SHOW_ROI_TEXT            true
#define SHOW_ROI_BASE_CENTER     true
#define SHOW_LANE_TOP_POINTS     true
#define SHOW_CENTER_POINT        true
#define SHOW_CENTER_TEXT         true
#define SHOW_CENTER_REFERENCE_LINE true
#define SHOW_ESTIMATED_CENTER    true
#define SHOW_ESTIMATED_TEXT      true
#define SHOW_ESTIMATED_REFERENCE_LINE true
#define SHOW_CENTER_TO_LANES_CONNECTION true
#define SHOW_LANE_BASE_POINTS    true
#define SHOW_LANE_LABELS         true
#define SHOW_DEBUG_INFO          true
```

### **Configuração de Debug (Desenvolvimento)**
```cpp
#define SHOW_ROI_RECTANGLE        true
#define SHOW_ROI_TEXT            true
#define SHOW_ROI_BASE_CENTER     true
#define SHOW_LANE_TOP_POINTS     true
#define SHOW_CENTER_POINT        true
#define SHOW_CENTER_TEXT         true
#define SHOW_CENTER_REFERENCE_LINE true
#define SHOW_ESTIMATED_CENTER    true
#define SHOW_ESTIMATED_TEXT      true
#define SHOW_ESTIMATED_REFERENCE_LINE true
#define SHOW_CENTER_TO_LANES_CONNECTION true
#define SHOW_LANE_BASE_POINTS    true
#define SHOW_LANE_LABELS         true
#define SHOW_DEBUG_INFO          true
#define SHOW_CONFLICT_RESOLUTION true
```

## Novos Recursos

### **Círculos no Topo das Lanes**
- **Configuração:** `SHOW_LANE_TOP_POINTS`
- **Descrição:** Desenha círculos no topo de cada lane individual dentro do ROI
- **Uso:** Mostra onde cada lane entra na região de interesse
- **Cor:** Ciano (configurável via `roi_config.h`)
- **Tamanho:** Configurável via `LANE_TOP_POINT_RADIUS`

### **Linhas de Conexão Centro-Faixas**
- **Configuração:** `SHOW_CENTER_TO_LANES_CONNECTION`
- **Descrição:** Desenha linhas conectando o centro calculado/estimado às faixas detectadas
- **Uso:** Útil para visualizar a relação espacial entre o centro e as faixas
- **Cor:** Verde (configurável)
- **Espessura:** 1 pixel (configurável)

### **Informações de Debug Expandidas**
- **Configuração:** `SHOW_DEBUG_INFO`
- **Descrição:** Mostra coordenadas, medidas físicas e informações técnicas
- **Uso:** Ideal para desenvolvimento e calibração

## Performance

### **Impacto no FPS**
- **Elementos de ROI:** Baixo impacto (~1-2 FPS)
- **Círculos e linhas:** Impacto médio (~2-3 FPS)
- **Texto:** Impacto médio (~2-4 FPS)
- **Linhas de conexão:** Baixo impacto (~1-2 FPS)
- **Círculos no topo das lanes:** Baixo impacto (~0.5-1 FPS)
- **Debug completo:** Alto impacto (~5-8 FPS)

### **Recomendações**
- **Produção:** Use configuração mínima para máximo FPS
- **Desenvolvimento:** Use configuração completa para debug
- **Testes:** Use configuração intermediária para balancear FPS e informação

## Compilação

Após alterar as configurações, recompile o projeto:

```bash
cd build
make clean
make -j4
```

## Troubleshooting

### **Elementos não aparecem**
1. Verifique se a constante está definida como `true`
2. Verifique se o arquivo `visualization_config.h` está sendo incluído
3. Recompile o projeto

### **Performance baixa**
1. Desative elementos desnecessários (`false`)
2. Use configuração mínima para produção
3. Verifique se `ENABLE_ANTIALIASING` está `false`

### **Cores incorretas**
1. Verifique o formato BGR (Blue, Green, Red) do OpenCV
2. Use valores entre 0-255 para cada canal
3. Exemplo: `cv::Scalar(255, 0, 0)` = Azul puro
