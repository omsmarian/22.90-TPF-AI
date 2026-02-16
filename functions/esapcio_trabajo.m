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
% --- 1) Límites articulares  ---

qlim = [ deg2rad([-60 60]);   % q1
     deg2rad([ -90 0]);   % q2
     deg2rad([-90 0]);   % q3
     deg2rad([-90 0]);   % q4
     deg2rad([-180 180]) ]; % q5


% --- 2) Muestreo aleatorio de configuraciones ---
N = 10000;  % subir/bajar para precisión
q_rand = zeros(N,5);
low = qlim(:,1)';  % Límites inferiores (fila)
high = qlim(:,2)'; % Límites superiores (fila)

q_rand = low + (high - low) .* rand(N, 5);

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

xlims = [-0.6, 0.6];
ylims = [-0.5, 0.5];
zlims = [-0.6, 0.6];

% Ajuste de la cámara para "alejar" la vista: ampliar límites y ajustar view/zoom
axis([xlims ylims zlims]);
plot3(Pws(:,1), Pws(:,2), Pws(:,3), '.', 'MarkerSize', 3.6, 'Color','r');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('Espacio de trabajo (muestreo aleatorio, N=%d)', N));
view(45,25);

% --- 5) (Opcional) Dibujar también el robot en reposo para referencia ---
try
    Robot.plot(Qreposo, 'noshadow', 'nobase', 'notiles');
catch
    % si no querés animación/plot del robot, ignorá
end
