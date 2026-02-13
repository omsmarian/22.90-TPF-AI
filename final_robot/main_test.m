%% Script Principal de Prueba
clear; clc; close all;

fprintf('Iniciando batería de pruebas del Robot...\n');

% CASO 1: Línea vertical corta (Prueba estándar)
fprintf('>>> Prueba 1: Línea Vertical\n');
Robot_Sim(0.15, -0.05, 0.15, 0.05);

fprintf('Presiona una tecla para la siguiente prueba...\n');
pause; 

% CASO 2: Línea diagonal (Cruzando la hoja)
fprintf('>>> Prueba 2: Línea Diagonal\n');
Robot_Sim(0.10, -0.08, 0.18, 0.08);

fprintf('Presiona una tecla para la siguiente prueba...\n');
pause;

% CASO 3: Línea horizontal (Más difícil para el codo)
fprintf('>>> Prueba 3: Línea Horizontal\n');
Robot_Sim(0.12, 0.05, 0.18, 0.05);

fprintf('¡Pruebas finalizadas!\n');