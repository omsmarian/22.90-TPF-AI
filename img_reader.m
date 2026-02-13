%% Limpiar todo
clear; close all; clc;

% Agregar carpeta de funciones al path
addpath('functions');

%% Archivo de prueba del modelo de visión
imgPath = "img/new1.jpeg";

[P1_mm, P2_mm, debug] = vision_model(imgPath, struct('showFigures', true));

if (debug.ok ~= true)
    fprintf('No se detectaron líneas. Motivo: %s\n', debug.reason);
else
    fprintf('\n--- COORDENADAS PARA TRAYECTORIA ROBOT ---\n');
    fprintf('Punto Inicio (mm): X=%.2f, Y=%.2f\n', P1_mm(1), P1_mm(2));
    fprintf('Punto Fin    (mm): X=%.2f, Y=%.2f\n', P2_mm(1), P2_mm(2));

    % 2. Convertir a coordenadas del Robot
    [x1, y1] = vision_to_robot( P1_mm(1), P1_mm(2));
    [x2, y2] = vision_to_robot( P2_mm(1), P2_mm(2));
    
    % 3. Ejecutar Simulación
    fprintf('Dibujando línea de (%.3f, %.3f) a (%.3f, %.3f)...\n', x1, y1, x2, y2);
    Robot_Sim(x1, y1, x2, y2);
end