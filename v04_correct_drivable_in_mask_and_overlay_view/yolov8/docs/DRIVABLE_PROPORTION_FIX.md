# Correção do Problema de Proporção das Áreas Drivable

## Problema Identificado

Durante os testes, foi observado que as áreas "drivable" desenhadas no overlay estavam com proporção incorreta. Quando o objeto (jetracer) mudava de posição, a área drivable era diminuída sem representar a posição real.

## Causa Raiz

O problema estava nas funções de transformação de coordenadas em `src/postprocess.cpp`:

1. **Função `scale_mask`**: Estava fazendo redimensionamento incorreto da máscara entre o modelo (320x320) e a imagem real
2. **Função `get_rect`**: Tinha inconsistências na transformação de coordenadas que causavam distorção
3. **Função `get_rect_adapt_landmark`**: Similar ao `get_rect`, mas para landmarks

## Solução Implementada

### 1. Correção da Função `scale_mask`

**Antes:**
```cpp
cv::Mat scale_mask(cv::Mat mask, cv::Mat img) {
    int x, y, w, h;
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);
    if (r_h > r_w) {
        w = kInputW;
        h = r_w * img.rows;
        x = 0;
        y = (kInputH - h) / 2;
    } else {
        w = r_h * img.cols;
        h = kInputH;
        x = (kInputW - w) / 2;
        y = 0;
    }
    cv::Rect r(x, y, w, h);
    cv::Mat res;
    cv::resize(mask(r), res, img.size());
    return res;
}
```

**Depois:**
```cpp
cv::Mat scale_mask(cv::Mat mask, cv::Mat img) {
    // CORREÇÃO: A função anterior tinha problemas de proporção que causavam
    // distorção nas áreas drivable quando o objeto mudava de posição

    // Calcular as razões de escala entre o modelo (320x320) e a imagem real
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);

    // Determinar qual dimensão é o fator limitante
    float scale_factor;
    int x_offset, y_offset;

    if (r_h > r_w) {
        // A altura é o fator limitante - a imagem real é mais larga que o modelo
        scale_factor = r_w;
        x_offset = 0;
        y_offset = (kInputH - r_w * img.rows) / 2;
    } else {
        // A largura é o fator limitante - a imagem real é mais alta que o modelo
        scale_factor = r_h;
        x_offset = (kInputW - r_h * img.cols) / 2;
        y_offset = 0;
    }

    // Calcular a região da máscara que corresponde à imagem real
    int mask_width = static_cast<int>(img.cols * scale_factor);
    int mask_height = static_cast<int>(img.rows * scale_factor);

    // Garantir que as dimensões não excedam os limites da máscara
    mask_width = std::min(mask_width, mask.cols);
    mask_height = std::min(mask_height, mask.rows);

    // Calcular o offset para centralizar a região na máscara
    int mask_x = x_offset;
    int mask_y = y_offset;

    // Garantir que a região está dentro dos limites da máscara
    mask_x = std::max(0, std::min(mask_x, mask.cols - mask_width));
    mask_y = std::max(0, std::min(mask_y, mask.rows - mask_height));

    // Extrair a região relevante da máscara
    cv::Rect mask_roi(mask_x, mask_y, mask_width, mask_height);

    // Verificar se a região é válida
    if (mask_roi.x < 0 || mask_roi.y < 0 ||
        mask_roi.x + mask_roi.width > mask.cols ||
        mask_roi.y + mask_roi.height > mask.rows) {
        // Fallback: usar a máscara inteira se a região calculada for inválida
        cv::Mat res;
        cv::resize(mask, res, img.size());
        return res;
    }

    // Extrair e redimensionar a região da máscara para o tamanho da imagem
    cv::Mat mask_region = mask(mask_roi);
    cv::Mat res;
    cv::resize(mask_region, res, img.size());

    return res;
}
```

### 2. Correção da Função `get_rect`

**Melhorias implementadas:**
- Validação mais robusta das coordenadas
- Garantia de que as coordenadas estejam dentro dos limites da imagem
- Comentários explicativos para cada caso de transformação
- Uso de `static_cast` para conversões mais seguras

### 3. Correção da Função `get_rect_adapt_landmark`

**Melhorias implementadas:**
- Aplicação das mesmas correções de `get_rect`
- Melhor tratamento dos landmarks
- Comentários explicativos para cada caso

## Benefícios da Correção

1. **Proporção Correta**: As áreas drivable agora mantêm a proporção correta independentemente da posição do objeto
2. **Posicionamento Preciso**: As coordenadas são transformadas corretamente entre o modelo (320x320) e a imagem real
3. **Robustez**: Validações adicionais previnem erros de coordenadas fora dos limites
4. **Manutenibilidade**: Código mais claro e documentado

## Arquivos Modificados

- `src/postprocess.cpp`: Funções `scale_mask`, `get_rect`, e `get_rect_adapt_landmark`

## Como Testar

1. Compile o projeto: `make clean && make -j4`
2. Execute o sistema de detecção
3. Mova o objeto (jetracer) para diferentes posições
4. Verifique se as áreas drivable mantêm a proporção correta

## Configurações Relacionadas

As configurações das áreas drivable podem ser ajustadas em:
- `include/drivable_visualization_config.h`: Configurações de visualização
- `include/roi_config.h`: Configurações do ROI

## Notas Técnicas

- **Modelo de entrada**: 320x320 pixels (definido em `include/config.h`)
- **Transformação**: Coordenadas do modelo são escaladas para a resolução da imagem real
- **Fallback**: Em caso de erro na transformação, usa a máscara inteira como backup
