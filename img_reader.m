%% Limpiar todo
clear; close all; clc;
%% 1. Leer imagen (Peter Corke)
img = iread("img/test1.png");
figure(1);
idisp(img);
title('Imagen original');

%% 7. DETECCIÓN DE VERDE (sobre imagen sin rojo)

img = igamm(img, 0.75);

I = img;

I(:,:,2) = niblack(I(:,:,2), -0.05, 5);

figure(10);
idisp(I);

HSV = rgb2hsv(I);

H = HSV(:,:,1);
S = HSV(:,:,2);
V = HSV(:,:,3);


%% 8. Realce de verde (clásico Corke)
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

%% Detección de Blobs y Esquinas Extremas
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

%% 9. WARP (Corrección de Perspectiva)

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

%% 9. Detección de bordes (Corke)
%edges = icanny(green_enhanced, 1);


%figure(7);
%idisp(edges);
%title('Bordes verdes');


%% 6. Transformada de Hough (Corke)
%Hh = Hough(edges);


%Hh.plot();

error('miau');

%% 2. Extraer canales RGB
R = img(:,:,1);
G = img(:,:,2);
B = img(:,:,3);

%% 3. Realce de rojo mejorado
red_enhanced = R - max(G, B);

%% 4. Umbral adaptativo para rojo
vals = red_enhanced(:);
vals = vals(vals > 0);
th = prctile(vals, 98);
fprintf('Umbral calculado: %.4f\n', th);

%% 5. Máscara de rojo
red_mask = red_enhanced > th;
se = kcircle(9);
red_mask = iclose(red_mask, se);
red_mask = iopen(red_mask, se);

%% 6. Eliminar línea roja
img_masked = img;
img_masked(:,:,1) = img_masked(:,:,1) .* ~red_mask;
img_masked(:,:,2) = img_masked(:,:,2) .* ~red_mask;
img_masked(:,:,3) = img_masked(:,:,3) .* ~red_mask;
figure(2);
idisp(img_masked);
title('Imagen sin línea roja');