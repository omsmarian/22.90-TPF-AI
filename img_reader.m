%% Limpiar todo
clear; close all; clc;
%% 1. Leer imagen (Peter Corke)
img = iread("img/test1.png", 'double');
figure(1);
idisp(img);
title('Imagen original');

%% 2. Extraer canales RGB
R = img(:,:,1);
G = img(:,:,2);
B = img(:,:,3);

%% 3. Realce de rojo mejorado
red_enhanced = R - max(G, B);
%figure(2);
%idisp(red_enhanced, 'signed');
%title('Realce de rojo');

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
%figure(4);
%idisp(red_mask);
%title('Máscara de línea roja (limpia)');

%% 6. Eliminar línea roja
img_masked = img;
img_masked(:,:,1) = img_masked(:,:,1) .* ~red_mask;
img_masked(:,:,2) = img_masked(:,:,2) .* ~red_mask;
img_masked(:,:,3) = img_masked(:,:,3) .* ~red_mask;
figure(5);
idisp(img_masked);
title('Imagen sin línea roja');