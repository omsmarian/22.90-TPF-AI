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

%% 10. Hacemos por blobs

rectangulo = iblobs(green_enhanced, 'class', 1)

%% 9. Detección de bordes (Corke)
edges = icanny(green_enhanced, 1);


figure(7);
idisp(edges);
title('Bordes verdes');


%% 6. Transformada de Hough (Corke)
Hh = Hough(edges);


Hh.show();

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

