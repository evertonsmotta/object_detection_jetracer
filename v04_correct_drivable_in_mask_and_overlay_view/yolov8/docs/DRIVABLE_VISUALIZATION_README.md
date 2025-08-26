# Visualização das Áreas Drivable - YOLOv8

## Visão Geral

Este sistema agora inclui funcionalidades avançadas para visualizar e destacar as áreas "drivable" (dirigíveis) detectadas pelo modelo YOLOv8, especialmente as áreas que estão entre as faixas de tráfego ativas. **Os bboxs não são mais exibidos por padrão** para uma visualização mais limpa.

## Funcionalidades Implementadas

### 1. Visualização das Áreas Drivable
- **Detecção automática**: Identifica automaticamente todas as áreas classificadas como "drivable"
- **Máscaras de segmentação**: Aplica as máscaras de segmentação para mostrar exatamente onde estão as áreas dirigíveis
- **Transparência configurável**: Permite ajustar a opacidade para não sobrepor outras informações importantes
- **Controle de exibição**: Três modos diferentes para controlar o que é exibido

### 2. Destaque das Áreas Entre Lanes
- **Identificação inteligente**: Detecta automaticamente quais áreas drivable estão entre as faixas de tráfego ativas
- **Cores diferenciadas**: Usa cores diferentes para áreas drivable entre lanes vs. outras áreas
- **Bordas destacadas**: Desenha bordas claras ao redor das áreas importantes
- **Labels informativos**: Adiciona texto explicativo nas áreas destacadas

### 3. Controle de Bboxs
- **Bboxs desabilitados por padrão**: Para uma visualização mais limpa
- **Configurável**: Pode ser habilitado via configuração se necessário
- **Foco nas máscaras**: Prioriza a visualização das máscaras de segmentação

## Configurações

### Arquivo: `include/drivable_visualization_config.h`

```cpp
// ====== MODO DE EXIBIÇÃO DAS ÁREAS DRIVABLE ======
// DRIVABLE_DISPLAY_MODE:
// 0 = Não exibir áreas drivable
// 1 = Exibir apenas áreas drivable entre as lanes do ROI (entre base e topo)
// 2 = Exibir todas as áreas drivable detectadas
#define DRIVABLE_DISPLAY_MODE 1

// ====== CONFIGURAÇÕES DO ROI ======
// Habilitar/desabilitar exibição de bboxs
#define SHOW_BBOX false

// Configurações de área do ROI
#define ROI_BASE_Y_RATIO 0.8        // Posição Y da base do ROI (80% da altura da imagem)
#define ROI_TOP_Y_RATIO 0.3         // Posição Y do topo do ROI (30% da altura da imagem)
#define ROI_LANE_TOLERANCE 0.15     // Tolerância para considerar lanes dentro do ROI (15% da largura)

// Cores para diferentes tipos de áreas drivable
#define DRIVABLE_BETWEEN_LANES_COLOR cv::Scalar(0, 255, 255)    // Ciano brilhante
#define DRIVABLE_OTHER_AREAS_COLOR cv::Scalar(0, 128, 0)        // Verde escuro
#define DRIVABLE_ROI_COLOR cv::Scalar(255, 255, 0)              // Amarelo para áreas no ROI
#define DRIVABLE_BORDER_COLOR cv::Scalar(0, 255, 255)           // Ciano para bordas

// Configurações de transparência
#define DRIVABLE_OPACITY 0.6              // 60% de opacidade para áreas entre lanes
#define DRIVABLE_OTHER_OPACITY 0.3        // 30% de opacidade para outras áreas
#define DRIVABLE_ROI_OPACITY 0.7          // 70% de opacidade para áreas no ROI
```

## Modos de Exibição

### Modo 0: Sem Visualização Drivable
```cpp
#define DRIVABLE_DISPLAY_MODE 0
```
- Não exibe nenhuma área drivable
- Apenas lanes são visualizadas
- Performance máxima

### Modo 1: Apenas ROI (Recomendado)
```cpp
#define DRIVABLE_DISPLAY_MODE 1
```
- Exibe apenas áreas drivable que estão entre as lanes do ROI
- Área definida entre 30% e 80% da altura da imagem
- Foco nas áreas mais relevantes para navegação
- Performance otimizada

### Modo 2: Todas as Áreas
```cpp
#define DRIVABLE_DISPLAY_MODE 2
```
- Exibe todas as áreas drivable detectadas
- Áreas no ROI são destacadas em amarelo
- Outras áreas são exibidas em verde escuro
- Visualização completa, mas pode ser mais lenta

## Como Usar

### 1. Compilação
```bash
cd yolov8
mkdir build && cd build
cmake ..
make -j4
```

### 2. Execução
```bash
./yolov8_seg
```

### 3. Visualização
- **Left View**: Mostra máscaras das lanes em verde
- **Right View**: Mostra overlay completo com:
  - Lanes detectadas (ciano/laranja)
  - Áreas drivable no ROI (amarelo brilhante)
  - ROI visual (borda magenta)
  - Labels explicativos

## Exemplos de Uso

### Exemplo 1: Modo ROI (Recomendado para Navegação)
```cpp
#define DRIVABLE_DISPLAY_MODE 1
#define SHOW_BBOX false
#define ROI_BASE_Y_RATIO 0.8
#define ROI_TOP_Y_RATIO 0.3
```

### Exemplo 2: Visualização Completa
```cpp
#define DRIVABLE_DISPLAY_MODE 2
#define SHOW_BBOX false
#define DRIVABLE_ROI_OPACITY 0.8
#define DRIVABLE_OTHER_OPACITY 0.5
```

### Exemplo 3: Habilitar Bboxs (Para Debug)
```cpp
#define DRIVABLE_DISPLAY_MODE 1
#define SHOW_BBOX true
#define SHOW_DRIVABLE_LABELS true
```

### Exemplo 4: Personalizar Cores
```cpp
#define DRIVABLE_ROI_COLOR cv::Scalar(255, 0, 255)    // Magenta
#define DRIVABLE_OTHER_AREAS_COLOR cv::Scalar(128, 128, 0)      // Verde-oliva
#define DRIVABLE_BORDER_COLOR cv::Scalar(255, 255, 255)         // Branco
```

## Estrutura do Código

### Funções Principais

1. **`draw_mask_only()`**: Visualização das lanes e áreas drivable baseada no modo configurado
2. **`draw_mask_bbox()`**: Visualização com bboxs (controlada por SHOW_BBOX)
3. **`draw_lane_lines()`**: Linhas das faixas de tráfego

### Fluxo de Processamento

1. **Detecção**: Modelo YOLOv8 detecta lanes e áreas drivable
2. **Filtragem**: Identifica lanes ativas (mais próximas do carro)
3. **Análise do ROI**: Calcula área de interesse entre base e topo
4. **Filtragem Drivable**: Aplica modo de exibição configurado
5. **Visualização**: Aplica cores e transparência apropriadas

## Configurações do ROI

### Posicionamento
- **Base**: 80% da altura da imagem (próximo ao carro)
- **Topo**: 30% da altura da imagem (distante)
- **Centro**: Centro horizontal da imagem
- **Largura**: 30% da largura da imagem (15% para cada lado)

### Personalização
```cpp
// Ajustar altura do ROI
#define ROI_BASE_Y_RATIO 0.9        // ROI mais próximo
#define ROI_TOP_Y_RATIO 0.2         // ROI mais distante

// Ajustar largura do ROI
#define ROI_LANE_TOLERANCE 0.2      // ROI mais largo
```

## Troubleshooting

### Problema: Áreas drivable não aparecem
- Verifique se `DRIVABLE_DISPLAY_MODE` não está 0
- Confirme se o modelo está detectando a classe "drivable"
- Verifique se as máscaras estão sendo processadas corretamente

### Problema: ROI muito pequeno/grande
- Ajuste `ROI_BASE_Y_RATIO` e `ROI_TOP_Y_RATIO`
- Modifique `ROI_LANE_TOLERANCE` para ajustar largura
- Use valores entre 0.1 e 0.9 para as proporções

### Problema: Performance baixa
- Use `DRIVABLE_DISPLAY_MODE 1` para melhor performance
- Reduza `DRIVABLE_UPDATE_INTERVAL` se necessário
- Ajuste `MIN_INTERSECTION_AREA` para filtrar áreas menores

### Problema: Bboxs aparecem quando não deveriam
- Verifique se `SHOW_BBOX` está `false`
- Confirme se a função `draw_mask_bbox` não está sendo chamada
- Verifique se as configurações foram aplicadas corretamente

## Contribuições

Para contribuir com melhorias na visualização das áreas drivable:

1. Modifique as configurações no arquivo `drivable_visualization_config.h`
2. Teste as mudanças com diferentes cenários de tráfego
3. Documente novas funcionalidades neste README
4. Mantenha a compatibilidade com o sistema existente

## Suporte

Para dúvidas ou problemas:
- Verifique os logs de console para mensagens de erro
- Teste com configurações básicas primeiro
- Consulte a documentação do OpenCV para operações de imagem
- Use o modo ROI (1) para melhor performance e foco nas áreas relevantes
