%% Script Principal
clear; clc; close all;

% 1. Datos que vienen de la cámara (ejemplo: una línea diagonal)
u_inicio = 20;  v_inicio = 50;   % Cerca de la esquina superior izquierda
u_fin    = 130; v_fin    = 50;  % Cerca de la esquina inferior derecha

% 2. Convertir a coordenadas del Robot
[x1, y1] = vision_to_robot(u_inicio, v_inicio);
[x2, y2] = vision_to_robot(u_fin, v_fin);

% 3. Ejecutar Simulación
fprintf('Dibujando línea de (%.3f, %.3f) a (%.3f, %.3f)...\n', x1, y1, x2, y2);
Robot_Sim(x1, y1, x2, y2);