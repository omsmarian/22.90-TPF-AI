%% Limpiar todo
clear; close all; clc;

%% Archivo de prueba del modelo de visión
imgPath = "img/new9.jpeg";

[P1_mm, P2_mm, debug] = vision_model(imgPath, struct('showFigures', true));

if isequal(P1_mm, 0) && isequal(P2_mm, 0)
    fprintf('No se detectaron líneas. Motivo: %s\n', debug.reason);
else
    fprintf('\n--- COORDENADAS PARA TRAYECTORIA ROBOT ---\n');
    fprintf('Punto Inicio (mm): X=%.2f, Y=%.2f\n', P1_mm(1), P1_mm(2));
    fprintf('Punto Fin    (mm): X=%.2f, Y=%.2f\n', P2_mm(1), P2_mm(2));
end