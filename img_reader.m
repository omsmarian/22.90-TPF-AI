%% Limpiar todo
clear; close all; clc;
%% 1. Leer imagen (Peter Corke)
img = iread("img/new2.jpeg");
figure(1);
idisp(img);
title('Imagen original');

%% 2. DETECCIÓN DE VERDE (sobre imagen sin rojo)

img = igamm(img, 0.75);

I = img;

I(:,:,2) = niblack(I(:,:,2), -0.05, 5);

HSV = rgb2hsv(I);

H = HSV(:,:,1);
S = HSV(:,:,2);
V = HSV(:,:,3);


%% 3. Realce de verde
green_enhanced = (H > 0.25 & H < 0.65) ... % se ajusta por abajo el H para verde
                & (S > 0.15 & S < 0.75);    % se ajusta por abajo el H para eliminar grises

kgaus = kgauss(5);

green_enhanced = iconv(green_enhanced,kgaus);
T = otsu(green_enhanced);
green_enhanced = green_enhanced > T;
green_enhanced = iclose(green_enhanced, kcircle(2));
green_enhanced = iopen(green_enhanced, kcircle(4));

figure(2)
idisp(green_enhanced)
title('Mascar Verde')

%% 4. Detección de Blobs y Esquinas Extremas
% Detectamos todos los candidatos iniciales
candidatos = iblobs(green_enhanced, 'class', 1, 'boundary');

% --- FILTRADO DE BLOBS POR ASPECT RATIO Y FORMA ---
rectangulo = []; % Limpiamos para guardar solo los válidos
for i = 1:length(candidatos)
    b = candidatos(i);
    
    % Calculamos el Aspect Ratio (eje menor / eje mayor)
    aspect_ratio = b.b / b.a;
    
    % Aplicamos los criterios: Area, Circularidad y tu Aspect Ratio (0.4 - 0.6)
    if b.area > 300 && b.circularity < 0.65 && (aspect_ratio >= 0.35 && aspect_ratio <= 0.65)
        rectangulo = [rectangulo; b];
    end
end

% Verificación de seguridad
if isempty(rectangulo)
    error('No se detectó ninguna esquina con los filtros actuales.');
end

figure(3);
idisp(green_enhanced);
title('Detección de Esquinas Extremas (Puntos Violetas)');
hold on;

% 1. Calcular el CENTRO GLOBAL aproximado del área de trabajo
% (Ahora usando solo los blobs que pasaron el filtro)
center_u = mean([rectangulo.uc]);
center_v = mean([rectangulo.vc]);

% Dibujamos el centro global como referencia (cruz azul)
plot(center_u, center_v, 'bx', 'MarkerSize', 20, 'LineWidth', 2);

corners = [];

% Iteramos solo sobre los blobs filtrados
for i = 1:length(rectangulo)
    b = rectangulo(i);
    
    % Obtener los puntos del borde [x, y]
    edge = b.edge'; 
    
    % ============================================================
    % SOLUCIÓN: DISTANCIA AL CENTRO GLOBAL
    % ============================================================
    
    % Calculamos la distancia al cuadrado al centro global
    dist_sq = (edge(:,1) - center_u).^2 + (edge(:,2) - center_v).^2;
    
    % Encontramos el índice del punto con la distancia MÁXIMA
    [~, idx_max] = max(dist_sq);
    
    % Este es tu "Punto Violeta"
    corner_point = edge(idx_max, :);
    corners = [corners; corner_point];

    % --- Visualización ---
    
    % Dibujar el borde completo en amarillo suave
    plot(edge(:,1), edge(:,2), 'y.', 'MarkerSize', 2);
    
    % Dibujar la ESQUINA EXTREMA (Punto Violeta)
    plot(corner_point(1), corner_point(2), 'o', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', [0.5 0 0.5], ... % Color violeta
        'Color', 'white'); 
    
    % Etiqueta
    text(corner_point(1)+15, corner_point(2)+15, ...
        sprintf('C%d', i), 'Color','cyan', 'FontWeight','bold');
end

hold off;

% Mostrar coordenadas finales en consola
disp('Coordenadas de los extremos detectados (u, v):');
disp(corners);

%% 5. WARP DE ALTA RESOLUCIÓN (Sin pérdida de calidad)

% 1. Ordenar esquinas originales (posi)
corners_sorted = sortrows(corners, 2); 
top = sortrows(corners_sorted(1:2, :), 1);
bot = sortrows(corners_sorted(3:4, :), 1);
posi = [top(1,:)', top(2,:)', bot(2,:)', bot(1,:)'];

% 2. CALCULAR TAMAÑO DE SALIDA ÓPTIMO (en píxeles)
% Medimos cuántos píxeles mide el borde superior en la imagen original
ancho_px = norm(top(1,:) - top(2,:)); 
% Mantenemos la relación de aspecto 200:150 (4:3) para que no se deforme
alto_px = ancho_px * (150/200);

% 3. Definir destino (posf) con la resolución real de la cámara
posf = [1, ancho_px, ancho_px, 1; 
        1, 1,        alto_px,  alto_px];

% 4. Homografía y Warp
matH = homography(posi, posf);
warped_raw = homwarp(matH, img, 'full');

% --- RECORTE DE PRECISIÓN ---
% Proyectamos las esquinas de la imagen original para saber el desplazamiento (offset)
[h_orig, w_orig, ~] = size(img);
esquinas_img_orig = [1, w_orig, w_orig, 1; 
                     1, 1, h_orig, h_orig];
esquinas_dest = homtrans(matH, esquinas_img_orig);

% El punto (1,1) de nuestra área de interés está en estas coordenadas 
% dentro de la matriz warped_raw:
offset_u = min(esquinas_dest(1,:));
offset_v = min(esquinas_dest(2,:));

start_u = round(1 - offset_u) + 1;
start_v = round(1 - offset_v) + 1;

% Recortamos usando el ancho y alto que calculamos para máxima calidad
img_warped = warped_raw(start_v : start_v + round(alto_px) - 1, ...
                        start_u : start_u + round(ancho_px) - 1, :);

% 5. Guardar factor de escala para el robot
% Este valor te dirá cuántos mm vale cada píxel ahora (ej: 0.25 mm/px)
mm_per_pixel = 200 / ancho_px;

figure(20);
idisp(img_warped);
title(sprintf('Warp Alta Resolución (%.1f x %.1f px)', ancho_px, alto_px));

%% 6. DETECCIÓN DE LÍNEA ROJA (Extremos)

% Asegurar que la imagen sea uint8 para procesar

img_warped_u8 = img_warped;


% 1. Convertir a HSV para segmentar color
hsv = rgb2hsv(img_warped_u8);
H = hsv(:,:,1);
S = hsv(:,:,2);
V = hsv(:,:,3);

% 2. Máscara de Rojo
% El rojo está en dos partes del Hue: cerca de 0 y cerca de 1
mask_red = (H < 0.1 | H > 0.85) & (S > 0.1) & (V > 0.2);

% Limpiar ruido (cerrar huecos y quitar puntitos sueltos)
mask_red = iclose(mask_red, kcircle(4));
mask_red = iopen(mask_red, kcircle(2));

figure(5);
idisp(mask_red);
title('Máscara Roja');

% 4. Análisis de Blobs (Corke)
% Filtramos por area mínima para ignorar ruido pequeño
blobs = iblobs(mask_red, 'class', 1);

if isempty(blobs)
    error('No se detectó ninguna línea roja. Revisa el umbral HSV.');
end

% Asumimos que la línea es el blob más grande encontrado
[~, idx_biggest] = max([blobs.area]);
linea_blob = blobs(idx_biggest);

% 5. Encontrar extremos matemáticamente (Sin bwmorph)
% Obtenemos TODOS los pixeles que pertenecen a la máscara roja
[v_inds, u_inds] = find(mask_red); 
% Nota: find devuelve (fila, columna) -> (y, x) -> (v, u)

pixels = [u_inds, v_inds]; % Matriz Nx2 de coordenadas [u, v]

if isempty(pixels)
    error('La máscara está vacía.');
end

% 5. Algoritmo de Distancia Máxima para hallar los extremos
% A. Punto P1: El más alejado del promedio (centroide) de la línea
centroide = mean(pixels);
dist_al_centro = sum((pixels - centroide).^2, 2);
[~, idx1] = max(dist_al_centro);
p1 = pixels(idx1, :);

% B. Punto P2: El más alejado de P1 (el otro extremo)
dist_a_p1 = sum((pixels - p1).^2, 2);
[~, idx2] = max(dist_a_p1);
p2 = pixels(idx2, :);

%% 7. VISUALIZACIÓN DE PUNTOS Y COORDENADAS MM

% Dimensiones reales del plano (Página 4)
ANCHO_REAL_MM = 200; 
ALTO_REAL_MM = 150;

% Factores de conversión
W_pix = size(img_warped, 2);
H_pix = size(img_warped, 1);
scale_x = ANCHO_REAL_MM / W_pix;
scale_y = ALTO_REAL_MM / H_pix;

% Puntos en milímetros
P1_mm = [p1(1) * scale_x, p1(2) * scale_y];
P2_mm = [p2(1) * scale_x, p2(2) * scale_y];

% --- Mostrar Imagen con Puntos ---
figure(6);
idisp(img_warped); 
title('Extremos de la línea detectados (en mm)');
hold on;

% Dibujar la línea (verde para que resalte)
plot([p1(1) p2(1)], [p1(2) p2(2)], 'g-', 'LineWidth', 2);

% Dibujar los dos puntos extremos (Círculos azules)
plot(p1(1), p1(2), 'bo', 'MarkerSize', 12, 'LineWidth', 3);
plot(p2(1), p2(2), 'bo', 'MarkerSize', 12, 'LineWidth', 3);

% Etiquetas de texto sobre la imagen
text(p1(1)+10, p1(2), sprintf('P1: [%.1f, %.1f]', P1_mm), 'Color', 'yellow', 'FontWeight', 'bold');
text(p2(1)+10, p2(2), sprintf('P2: [%.1f, %.1f]', P2_mm), 'Color', 'yellow', 'FontWeight', 'bold');

hold off;

% Imprimir resultados para el robot
fprintf('\n--- COORDENADAS PARA TRAYECTORIA ROBOT ---\n');
fprintf('Punto Inicio (mm): X=%.2f, Y=%.2f\n', P1_mm(1), P1_mm(2));
fprintf('Punto Fin    (mm): X=%.2f, Y=%.2f\n', P2_mm(1), P2_mm(2));