function hsv = rgb2hsv(rgb)
%RGB2HSV Convierte imagen RGB a HSV.
%   hsv = RGB2HSV(rgb)
%   rgb: Imagen RGB con valores en [0, 1]
%   hsv: Imagen HSV con H en [0, 1], S en [0, 1], V en [0, 1]

    % Normalizar si está en [0, 255]
    if max(rgb(:)) > 1
        rgb = double(rgb) / 255;
    end

    r = rgb(:,:,1);
    g = rgb(:,:,2);
    b = rgb(:,:,3);

    % Calcular V (Value)
    v = max(max(r, g), b);

    % Calcular S (Saturation)
    delta = v - min(min(r, g), b);
    s = zeros(size(v));
    s(v ~= 0) = delta(v ~= 0) ./ v(v ~= 0);

    % Calcular H (Hue)
    h = zeros(size(v));

    % Red es el máximo
    idx = (v == r) & (delta ~= 0);
    h(idx) = 60 * mod((g(idx) - b(idx)) ./ delta(idx), 6);

    % Green es el máximo
    idx = (v == g) & (delta ~= 0);
    h(idx) = 60 * ((b(idx) - r(idx)) ./ delta(idx) + 2);

    % Blue es el máximo
    idx = (v == b) & (delta ~= 0);
    h(idx) = 60 * ((r(idx) - g(idx)) ./ delta(idx) + 4);

    % Normalizar H a [0, 1]
    h = h / 360;

    % Construir imagen HSV
    hsv = zeros(size(rgb));
    hsv(:,:,1) = h;
    hsv(:,:,2) = s;
    hsv(:,:,3) = v;
end
