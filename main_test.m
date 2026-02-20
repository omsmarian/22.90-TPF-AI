%% Script Principal
clear; clc; close all;
addpath('functions');

% 1. Datos que vienen de la cámara (ejemplo: una línea diagonal)
u_inicio = 0;  v_inicio = 150;   % U es eje Y & V es eje X
u_fin    = 200; v_fin    = 0;  % (0,0) es la esquina superior izquierda de la hoja, (200, 150) es la esquina derecha inferior

% 2. Convertir a coordenadas del Robot
[x1, y1] = vision_to_robot(u_inicio, v_inicio);
[x2, y2] = vision_to_robot(u_fin, v_fin);

% 3. Ejecutar Simulación
fprintf('Dibujando línea de (%.3f, %.3f) a (%.3f, %.3f)...\n', x1, y1, x2, y2);
%Robot_Sim(x1, y1, x2, y2);
Robot_Sim_err(x1, y1, x2, y2);