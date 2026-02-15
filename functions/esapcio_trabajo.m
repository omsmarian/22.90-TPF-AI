%% 1. Definición del Robot 
clear; clc; close all;

L1=130; L2=144; L3=50; L4=144; L5=144; % mm
L = [L1 L2 L3 L4 L5];
L_m  = L/1000; % Pasar a metros
d_eq = sqrt(L(2)^2 + L(3)^2)/1000;

% Definición de Links (DH Modificado)
R1 = Link('revolute', 'alpha',     0, 'a',  0,    'd', L_m(1), 'offset',  0,   'modified');
R2 = Link('revolute', 'alpha',  pi/2, 'a',  0,    'd', 0,     'offset',  pi/2, 'modified');
R3 = Link('revolute', 'alpha',     0, 'a', d_eq,  'd', 0,     'offset',   0,   'modified');
R4 = Link('revolute', 'alpha',     0, 'a', L_m(4),'d', 0,     'offset',  pi/2, 'modified');
R5 = Link('revolute', 'alpha',  pi/2, 'a',  0,    'd', 0,     'offset',   0,   'modified');

% Herramienta y Robot
EE = transl(0, 0, L_m(5)); 
Robot = SerialLink([R1 R2 R3 R4 R5], 'tool', EE, 'name', 'Robotito');

% --- POSE TIPO "GRÚA" (ELBOW UP) ---
Qreposo = [0, -0.6, -1, -0.9, 0]; %Esta pose ayuda a que no haya problemas de choque en el movimiento



%% === SIMULACIÓN DEL ESPACIO DE TRABAJO (WORKSPACE) ===
% Colocar este bloque al final de "%% 1. Definición del Robot",
% después de crear Robot y Qreposo, y antes de "%% 2..."

% --- 1) Límites articulares (ajustalos a los de tu informe/robot real) ---
% Si ya tenés Robot.qlim definido, úsalo. Si no, define algo razonable:
    % [min max] por junta (rad)  --> AJUSTAR segun tus "Ángulos Límite"
        qlim = [ deg2rad([-60 60]);   % q1
             deg2rad([ -90 0]);   % q2
             deg2rad([-90 0]);   % q3
             deg2rad([-90 0]);   % q4
             deg2rad([-180 180]) ]; % q5


% --- 2) Muestreo aleatorio de configuraciones ---
N = 5000;  % subí/bajá según performance
q_rand = zeros(N,5);
low = qlim(:,1)';  % Límites inferiores (fila)
high = qlim(:,2)'; % Límites superiores (fila)

q_rand = low + (high - low) .* rand(N, 5);

% (Opcional) sesgo leve alrededor de Qreposo para densificar zona útil:
% mix = 0.3;
% q_rand = (1-mix)*q_rand + mix*(Qreposo + 0.3*randn(N,5));

% --- 3) FK para obtener puntos alcanzables del efector final ---
Pws = zeros(N,3);
for k = 1:N
    T = Robot.fkine(q_rand(k,:));
    Pws(k,:) = T.t';
end

% --- 4) Plot del workspace (nube de puntos) ---
figure; clf; hold on; grid on;

 % A) DIBUJAR LA HOJA (Rectángulo Negro)
    x_start = 0.2; 
    w_hoja  = 0.15; % Ancho X
    l_hoja  = 0.20; % Largo Y
    
    vX = [x_start, x_start + w_hoja, x_start + w_hoja, x_start];
    vY = [-l_hoja/2, -l_hoja/2, l_hoja/2, l_hoja/2];
    vZ = [0, 0, 0, 0];
    
    patch(vX, vY, vZ, [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 2);


plot3(Pws(:,1), Pws(:,2), Pws(:,3), '.', 'MarkerSize', 3, 'Color','r');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('Espacio de trabajo (muestreo aleatorio, N=%d)', N));
axis equal; view(45,25);

% --- 5) (Opcional) Dibujar también el robot en reposo para referencia ---
try
    Robot.plot(Qreposo, 'noshadow', 'nobase', 'notiles');
catch
    % si no querés animación/plot del robot, ignorá
end
