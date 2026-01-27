%% Detección de línea roja usando Machine Vision Toolbox (versión clásica)
clear; clc;

%% 1. Leer imagen
I = iread("img/test1.png");    % RGB
I = igamm(I, 0.75);
idisp(I);
I(:,:,2) = niblack(I(:,:,2), -0.05, 5);
idisp(I);

%% 2. DETECCIÓN DE ESQUINAS VERDES

HSV = rgb2hsv(I);

H = HSV(:,:,1);
S = HSV(:,:,2);
V = HSV(:,:,3);

mask_green = (H > 0.166 & H < 0.65);
idisp(mask_green);

%mask_green = iopen(mask_green, kcircle(2));
mask_green = iclose(mask_green, ones(3));
mask_green = iopen(mask_green, ones(3));
idisp(mask_green);

%% 3. BLOBS VERDES
blobs_green = iblobs(mask_green, 'area', [50, 2500]);
length(blobs_green)
blobs_green