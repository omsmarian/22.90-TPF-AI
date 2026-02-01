%% Limpiar todo
clear; close all; clc;
%% 1. Leer imagen (Peter Corke)
img = iread("img/test1.png");
figure(1);
idisp(img);
title('Imagen original');

%% 2. DETECCIÓN DE VERDE (sobre imagen sin rojo)

img = igamm(img, 0.75);

I = img;

I(:,:,2) = niblack(I(:,:,2), -0.05, 5);

figure(10);
idisp(I);

HSV = rgb2hsv(I);

H = HSV(:,:,1);
S = HSV(:,:,2);
V = HSV(:,:,3);


%% 3. Realce de verde
green_enhanced = (H > 0.166 & H < 0.65) ... 
                & (S > 0.1 & S < 0.5);

kgaus = kgauss(5);

green_enhanced = iconv(green_enhanced,kgaus);
T = otsu(green_enhanced);
green_enhanced = green_enhanced > T;
green_enhanced = iclose(green_enhanced, strel('disk',2));
green_enhanced = iopen(green_enhanced, strel('disk',4));

figure(5)
idisp(green_enhanced)
title('Mascar Verde')

%% 4. Detección de Blobs y Esquinas Extremas
rectangulo = iblobs(green_enhanced, 'class', 1, 'boundary');

figure(6);
idisp(green_enhanced);
title('Detección de Esquinas Extremas (Puntos Violetas)');
hold on;

% 1. Calcular el CENTRO GLOBAL aproximado del área de trabajo
% Promediamos los centros de todos los blobs encontrados
center_u = mean([rectangulo.uc]);
center_v = mean([rectangulo.vc]);

% Dibujamos el centro global como referencia (cruz azul)
plot(center_u, center_v, 'bx', 'MarkerSize', 20, 'LineWidth', 2);

corners = [];

for i = 1:length(rectangulo)
    b = rectangulo(i);
    
    % Obtener los puntos del borde [x, y]
    edge = b.edge'; 
    
    % ============================================================
    % SOLUCIÓN: DISTANCIA AL CENTRO GLOBAL
    % El vértice exterior es el punto más lejano al centro de la mesa
    % ============================================================
    
    % Calculamos la distancia al cuadrado de cada punto del borde al centro global
    % (Usamos cuadrado para ahorrar la raíz cuadrada, el resultado es el mismo)
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

%% 5. WARP

% --- Paso A: Ordenar las esquinas ---
% Necesitamos el orden: [Arriba-Izq, Arriba-Der, Abajo-Der, Abajo-Izq]
% corners tiene formato [x, y] o [u, v]

if size(corners, 1) ~= 4
    error('Se necesitan exactamente 4 esquinas para el warp. Revisa la detección.');
end

% Ordenamos primero por la coordenada Y (v) para separar los de "arriba" y "abajo"
corners_sorted = sortrows(corners, 2); 

% Tomamos los 2 de arriba (menor Y) y los ordenamos por X para saber cual es Izq y Der
top_pts = sortrows(corners_sorted(1:2, :), 1);
tl = top_pts(1, :); % Top-Left
tr = top_pts(2, :); % Top-Right

% Tomamos los 2 de abajo (mayor Y) y ordenamos por X
bot_pts = sortrows(corners_sorted(3:4, :), 1);
bl = bot_pts(1, :); % Bottom-Left
br = bot_pts(2, :); % Bottom-Right

% Juntamos todo ordenado
movingPoints = [tl; tr; br; bl];

% --- Paso B: Definir el tamaño de la imagen de salida ---
% Podemos calcular el ancho/alto promedio basado en las distancias reales
% o fijar un tamaño arbitrario (ej. 600x400).

% Calculamos ancho superior e inferior y promediamos (Euclídea)
w1 = norm(tl - tr);
w2 = norm(bl - br);
W_real = round(mean([w1 w2]));

% Calculamos alto izquierdo y derecho
h1 = norm(tl - bl);
h2 = norm(tr - br);
H_real = round(mean([h1 h2]));

% Puntos destino en la nueva imagen [x, y]
% El orden debe coincidir: TL, TR, BR, BL
fixedPoints = [
    0,       0;          % TL
    W_real,  0;          % TR
    W_real,  H_real;     % BR
    0,       H_real      % BL
];

% --- Paso C: Calcular la transformación y hacer el Warp ---

% Calculamos la homografía (Matriz de transformación proyectiva)
tform = fitgeotrans(movingPoints, fixedPoints, 'projective');

% Aplicamos la transformación a la imagen ORIGINAL (img)
% 'OutputView' asegura que la imagen resultante tenga el tamaño exacto que definimos
ref = imref2d([H_real W_real]);
img_warped = imwarp(img, tform, 'OutputView', ref);

% --- Visualización Final ---
figure(20);
idisp(img_warped);
title('Área de Trabajo Rectificada (Warp)');

% Opcional: Mostrar los ejes para verificar coordenadas
axis on;


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
mask_red = (H < 0.1 | H > 0.9) & (S > 0.25) & (V > 0.2);

idisp(mask_red, 'signed');

% Limpiar ruido (cerrar huecos y quitar puntitos sueltos)
mask_red = iclose(mask_red, strel('disk', 3));
mask_red = iopen(mask_red, strel('disk', 2));

figure(21);
idisp(mask_red);
title('Máscara Roja');

% 4. Análisis de Blobs (Corke)
% Filtramos por area mínima para ignorar ruido pequeño
blobs = iblobs(mask_red, 'class', 1)

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
figure(22);
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