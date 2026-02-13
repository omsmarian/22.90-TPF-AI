%% 1. Definición del Robot 
clear; clc; close all;

L1=130; L2=144; L3=50; L4=144; L5=144; % mm
L = [L1 L2 L3 L4 L5];
L_m  = L/1000; % Pasar a metros
d_eq = sqrt(L(2)^2 + L(3)^2)/1000;

% Definición de Links (DH Modificado)
R1 = Link('revolute', 'alpha',     0, 'a',  0,    'd', L_m(1), 'offset',   0,   'modified');
R2 = Link('revolute', 'alpha',  pi/2, 'a',  0,    'd', 0,     'offset',  pi/2, 'modified');
R3 = Link('revolute', 'alpha',     0, 'a', d_eq,  'd', 0,     'offset',   0,   'modified');
R4 = Link('revolute', 'alpha',     0, 'a', L_m(4),'d', 0,     'offset',  pi/2, 'modified');
R5 = Link('revolute', 'alpha',  pi/2, 'a',  0,    'd', 0,     'offset',   0,   'modified');

% Herramienta y Robot
EE = transl(0, 0, L_m(5)); 
Robot = SerialLink([R1 R2 R3 R4 R5], 'tool', EE, 'name', 'Robotito');

% --- POSE TIPO "GRÚA" (ELBOW UP) ---
Qreposo = [0, -0.6, -1, -0.9, 0]; %Esta pose ayuda a que no haya problemas de choque en el movimiento
%% 2. Definición de la Misión (Puntos Clave)
% -- CONFIGURACIÓN DEL USUARIO --
% Ajusté X a 0.15 para que caiga DENTRO de la hoja (que va de 0.06 a 0.21)
x_hoja_inicial = 0.15; y_hoja_inicial = -0.08; % Y ajustado un poco para margen
x_hoja_final   = 0.15; y_hoja_final   =  0.08; 

% Alturas
z_segura = 0.05; 
z_papel  = 0.00; 
% -------------------------------

T_home = Robot.fkine(Qreposo);
P_home = T_home.t';

% Waypoints
W1_Aprox     = [x_hoja_inicial, y_hoja_inicial, z_segura];
W2_Contacto  = [x_hoja_inicial, y_hoja_inicial, z_papel];
W3_DibujoFin = [x_hoja_final,   y_hoja_final,   z_papel];
W4_Retiro    = [x_hoja_final,   y_hoja_final,   z_segura];

%% 3. Generación y Concatenación (mtraj)
[Traj1, ~, ~] = mtraj(@tpoly, P_home,      W1_Aprox,     40); 
[Traj2, ~, ~] = mtraj(@tpoly, W1_Aprox,    W2_Contacto,  20); 
[Traj3, ~, ~] = mtraj(@tpoly, W2_Contacto, W3_DibujoFin, 60); 
[Traj4, ~, ~] = mtraj(@tpoly, W3_DibujoFin, W4_Retiro,   20); 
[Traj5, ~, ~] = mtraj(@tpoly, W4_Retiro,    P_home,      40); 

P_deseada = [Traj1; Traj2(2:end,:); Traj3(2:end,:); Traj4(2:end,:); Traj5(2:end,:)];
pasos = size(P_deseada, 1);

%% 4. Bucle de Control (Jacobiana Inversa SIMPLE)
q_solucion = zeros(pasos, 5); 
q_actual = Qreposo; 
fprintf('Iniciando Misión Completa (%d pasos)...\n', pasos);

for i = 1:pasos
    P_objetivo = P_deseada(i, :); 
    T_actual = Robot.fkine(q_actual);
    % T_actual es una matriz de transformación homogénea 4x4 (pose del efector).
    % .t devuelve el vector traslacional [x y z]' (propiedad de SerialLink.fkine).
    % El transpose (') lo convierte en fila [x y z] para que coincida con P_objetivo.
    P_actual = T_actual.t';
    error = P_objetivo - P_actual;
    
    J = Robot.jacob0(q_actual);
    J_xyz = J(1:3, :); 
    
    dq = (pinv(J_xyz) * error')'; % pinv calcula la pseudo-inversa del jacobiano y se utiliza para la ley de control
    
    q_actual = q_actual + dq;
    q_solucion(i, :) = q_actual;
end

%% 5. Recalcular Trayectoria REAL
P_real = zeros(pasos, 3);
for i = 1:pasos
    T_temp = Robot.fkine(q_solucion(i,:));
    P_real(i,:) = T_temp.t';
end

%% 6. Cálculo de Errores
Error_mm = (P_deseada - P_real) * 1000;

%% 7. Gráficos Completos
% --- FIGURA 1: Robot y Escenario ---
figure(1); clf; hold on;

% A) DIBUJAR LA HOJA (Rectángulo Negro)
x_start = 0.06; 
w_hoja  = 0.15; % Ancho X
l_hoja  = 0.20; % Largo Y

vX = [x_start, x_start + w_hoja, x_start + w_hoja, x_start];
vY = [-l_hoja/2, -l_hoja/2, l_hoja/2, l_hoja/2];
vZ = [0, 0, 0, 0];

patch(vX, vY, vZ, [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 2);

% B) DIBUJAR OBJETIVO EN LA HOJA
plot3([x_hoja_inicial, x_hoja_final], [y_hoja_inicial, y_hoja_final], [0, 0], 'k-', 'LineWidth', 2);
plot3(x_hoja_inicial, y_hoja_inicial, 0, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'g');
plot3(x_hoja_final, y_hoja_final, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 2);

% C) DIBUJAR TRAYECTORIAS
plot3(P_real(:,1), P_real(:,2), P_real(:,3), 'm-', 'LineWidth', 2);

% D) ANIMAR ROBOT (pero sin que la animación cambie el zoom)
% Guardamos límites actuales antes de plot del robot
xlims = [-0.05, 0.5];
ylims = [-0.15, 0.3];
zlims = [-0.2, 0.4];

% Ajuste de la cámara para "alejar" la vista: ampliar límites y ajustar view/zoom
axis([xlims ylims zlims]);
view(45,25); % ángulo de vista (azimut, elevación) 
camproj('perspective');
camva(10); % ángulo de visión de la cámara (valores menores = más zoom), aumentar para alejar
% También se puede desplazar la cámara más atrás en el eje de visión:
camdolly(-0.2, 0, 0, 'headline'); % desplaza la cámara hacia atrás (ajustar magnitud)

% Finalmente dibujamos el robot (no dejar que Robot.plot cambie los límites)
hold on;
Robot.plot(q_solucion, 'noshadow', 'nobase', 'notiles', 'fps', 60);
% Restaurar los límites para asegurarnos que no cambien durante la animación
axis([xlims ylims zlims]);

legend('Hoja', 'Objetivo', 'Inicio', 'Fin', 'Deseada', 'Real', 'Robot');
title('Simulación con Entorno de Trabajo');
grid on; view(3);

% --- FIGURA 2: Desglose por Ejes ---
figure(2); clf;
subplot(2,1,1); hold on;
plot(P_deseada(:,1), 'b--', 'LineWidth', 1.5); plot(P_real(:,1), 'b-', 'LineWidth', 1.5); 
plot(P_deseada(:,2), 'r--', 'LineWidth', 1.5); plot(P_real(:,2), 'r-', 'LineWidth', 1.5); 
plot(P_deseada(:,3), 'g--', 'LineWidth', 1.5); plot(P_real(:,3), 'g-', 'LineWidth', 1.5); 
ylabel('Posición [m]'); title('Comparación XYZ'); grid on;
legend('X Ref','X Real','Y Ref','Y Real','Z Ref','Z Real', 'Location', 'BestOutside');

subplot(2,1,2); hold on;
plot(Error_mm(:,1), 'b'); plot(Error_mm(:,2), 'r'); plot(Error_mm(:,3), 'g');
ylabel('Error [mm]'); xlabel('Pasos'); title('Error de seguimiento'); grid on;
legend('Err X', 'Err Y', 'Err Z', 'Location', 'BestOutside');

% --- FIGURA 3: Motores ---
figure(3); clf;
plot(q_solucion * (180/pi), 'LineWidth', 1.5);
ylabel('Ángulo [deg]'); xlabel('Pasos'); title('Comandos a Motores'); grid on;
legend('q1','q2','q3','q4','q5');