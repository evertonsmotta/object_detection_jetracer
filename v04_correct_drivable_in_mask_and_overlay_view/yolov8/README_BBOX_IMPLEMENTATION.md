# Implementação de Bounding Boxes para Classes Específicas

## Visão Geral

Implementei com sucesso a funcionalidade para exibir bounding boxes (bbox) das classes específicas solicitadas:
- **passadeira**
- **stop sign**
- **speed 50**
- **speed 80**
- **jetracer**
- **gate**

## Funcionalidades Implementadas

### 1. Função `drawSpecificClassBBoxes`

**Localização**: `yolov8_seg.cpp` (linha ~280)

**Funcionalidades**:
- **Cores personalizadas** para cada classe
- **Labels legíveis** com fundo preto para melhor visibilidade
- **Bounding boxes destacados** com espessura 3
- **Percentual de confiança** exibido abaixo de cada bbox
- **Posicionamento inteligente** do texto (acima ou abaixo do bbox conforme necessário)

### 2. Esquema de Cores

| Classe | Cor | Código BGR |
|--------|-----|------------|
| passadeira | Ciano | (0, 255, 255) |
| stop sign | Vermelho | (0, 0, 255) |
| speed 50 | Magenta | (255, 0, 255) |
| speed 80 | Verde | (0, 255, 0) |
| jetracer | Amarelo | (255, 255, 0) |
| gate | Laranja | (255, 165, 0) |

### 3. Formatação dos Labels

- **passadeira** → "passadeira"
- **stop sign** → "STOP"
- **speed 50** → "50"
- **speed 80** → "80"
- **jetracer** → "jetracer"
- **gate** → "gate"

### 4. Integração no Sistema

**Modo Câmera (Tempo Real)**:
- Bounding boxes são desenhados na `right_view` (Overlay View)
- Aplicados após `draw_mask_only` e `draw_drivable_highlight`
- Atualizados a cada frame processado

**Modo Diretório (Processamento em Lote)**:
- Bounding boxes são desenhados em cada imagem processada
- Aplicados antes de salvar as imagens
- Incluídos em todas as imagens de saída

## Como Funciona

### 1. Detecção de Classes
```cpp
// Verificar se é uma das classes que queremos destacar
if (class_colors.find(class_name) != class_colors.end()) {
    // Processar e desenhar bbox
}
```

### 2. Desenho do Bounding Box
```cpp
// Obter retângulo do bbox
cv::Rect r = get_rect(img, det.bbox);

// Desenhar retângulo colorido
cv::rectangle(img, r, color, 3);
```

### 3. Label com Fundo
```cpp
// Calcular tamanho do texto
cv::Size text_size = cv::getTextSize(label, font_face, font_scale, thickness, nullptr);

// Desenhar fundo preto
cv::rectangle(img, bg_pt1, bg_pt2, cv::Scalar(0, 0, 0), -1);

// Desenhar texto branco
cv::putText(img, label, text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
```

### 4. Percentual de Confiança
```cpp
// Calcular e exibir confiança
std::string conf_text = std::to_string(static_cast<int>(det.conf * 100)) + "%";
cv::putText(img, conf_text, conf_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 1);
```

## Exemplo de Saída Visual

```
┌─────────────────┐
│     STOP        │ ← Label com fundo preto
└─────────────────┘ ← Bbox vermelho (espessura 3)
        85%        ← Confiança em vermelho
```

## Configurações Personalizáveis

### 1. Cores
As cores podem ser facilmente alteradas no mapa `class_colors`:
```cpp
std::unordered_map<std::string, cv::Scalar> class_colors = {
    {"passadeira", cv::Scalar(0, 255, 255)},      // Ciano
    {"stop sign", cv::Scalar(0, 0, 255)},         // Vermelho
    // ... outras cores
};
```

### 2. Espessura dos Bounding Boxes
```cpp
cv::rectangle(img, r, color, 3);  // Espessura 3 (pode ser alterada)
```

### 3. Tamanho da Fonte
```cpp
double font_scale = 0.8;  // Escala da fonte (pode ser ajustada)
int thickness = 2;         // Espessura do texto
```

## Performance

- **Impacto mínimo**: A função só processa as classes específicas
- **Otimizada**: Verificação rápida com `unordered_map`
- **Integrada**: Executa junto com outras funções de visualização
- **Eficiente**: Não aloca memória desnecessária

## Teste da Implementação

### 1. Compilação
```bash
cd build
make
```

### 2. Execução com Câmera
```bash
./yolov8_seg -d best_202507181755.engine cam c my_classes.txt
```

### 3. Execução com Diretório
```bash
./yolov8_seg -d best_202507181755.engine /caminho/para/imagens c my_classes.txt
```

## Resultado Esperado

Quando objetos das classes especificadas forem detectados, você verá:

1. **Bounding boxes coloridos** ao redor dos objetos
2. **Labels legíveis** com fundo preto
3. **Percentual de confiança** abaixo de cada bbox
4. **Cores consistentes** para cada classe
5. **Integração perfeita** com a visualização existente

## Próximos Passos

1. **Testar em tempo real** com a câmera
2. **Verificar visibilidade** dos bounding boxes
3. **Ajustar cores** se necessário para melhor contraste
4. **Otimizar posicionamento** dos labels se necessário

## Suporte

Para ajustes ou problemas:
1. Verifique se a compilação foi bem-sucedida
2. Confirme que as classes estão sendo detectadas
3. Ajuste cores ou espessuras conforme necessário
4. Teste com diferentes condições de iluminação
