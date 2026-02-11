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

% POSE INICIAL (Evitamos singularidad con ángulos pequeños)
Qreposo = [0, -pi/10, pi/10, 0, 0];

%% 2. Definir Puntos de Inicio y Fin (Cartesianos)
T_inicio = Robot.fkine(Qreposo); 
P_inicio = T_inicio.t'; 
% Definimos destino (Mover 10cm en X, 50cm en Y, bajar 50cm en Z)
desplazamiento = [0.20, 0.1, -0.1]; 
P_final = P_inicio + desplazamiento;

%% 3. Generar la trayectoria DESEADA (mtraj)
pasos = 50;
[P_deseada, Pd, Pdd] = mtraj(@tpoly, P_inicio, P_final, pasos);
% P_deseada es nuestra referencia ideal (Línea punteada)

%% 4. Bucle de Control (Jacobiana Inversa)
q_solucion = zeros(pasos, 5); 
q_actual = Qreposo; 
fprintf('Calculando cinemática inversa diferencial...\n');

for i = 1:pasos
    % A. Referencia del momento
    P_objetivo = P_deseada(i, :); 
    
    % B. Medición actual
    T_actual = Robot.fkine(q_actual);
    P_actual = T_actual.t'; 
    
    % C. Error
    error = P_objetivo - P_actual;
    
    % D. Jacobiana (Solo XYZ para robot de 5 ejes)
    J = Robot.jacob0(q_actual);
    J_xyz = J(1:3, :); 
    
    % E. Ley de Control (dq = J# * error)
    % Agregamos una ganancia K < 1 para estabilidad, o 1 para velocidad
    K = 1.0; 
    dq = pinv(J_xyz) * error' * K;
 
    % F. Integración
    q_actual = q_actual + dq';
    q_solucion(i, :) = q_actual;
end

%% 5. Recalcular la Trayectoria REAL 
% Ahora que tenemos los Q que el robot "ejecutó", vemos por dónde pasó realmente.
P_real = zeros(pasos, 3);

for i = 1:pasos
    T_temp = Robot.fkine(q_solucion(i,:));
    P_real(i,:) = T_temp.t';
end

%% 6. Cálculo de Errores
% Calculamos la diferencia y pasamos a milímetros
Error_m = P_deseada - P_real;       % Error en metros
Error_mm = Error_m * 1000;          % Error en milímetros

%% 7. Gráficos
% --- FIGURA 1: Robot y Trayectoria 3D ---
figure(1); clf;
Robot.plot(q_solucion, 'trail', 'k:', 'noshadow', 'nobase', 'notiles'); hold on;
% Dibujamos la Deseada (Línea gruesa punteada cyan para contraste)
plot3(P_deseada(:,1), P_deseada(:,2), P_deseada(:,3), 'c--', 'LineWidth', 2);
% Dibujamos la Real (Línea sólida magenta)
plot3(P_real(:,1), P_real(:,2), P_real(:,3), 'm-', 'LineWidth', 1.5);
legend('Robot', 'Trayectoria Deseada', 'Trayectoria Real');
title('Movimiento en el Espacio 3D');
grid on; view(3);

% --- FIGURA 2: Desglose por Ejes y Errores ---
figure(2); clf;

% Subplot Superior: Comparación de Posiciones (mismo color por eje)
subplot(2,1,1); hold on;
% EJE X (Azul)
plot(P_deseada(:,1), 'b--', 'LineWidth', 1.5); 
plot(P_real(:,1),    'b-',  'LineWidth', 1.5); 
% EJE Y (Rojo)
plot(P_deseada(:,2), 'r--', 'LineWidth', 1.5); 
plot(P_real(:,2),    'r-',  'LineWidth', 1.5); 
% EJE Z (Verde)
plot(P_deseada(:,3), 'g--', 'LineWidth', 1.5); 
plot(P_real(:,3),    'g-',  'LineWidth', 1.5); 

grid on;
ylabel('Posición [m]');
title('Seguimiento de Trayectoria por Eje (Deseado vs Real)');
legend('X Ref','X Real','Y Ref','Y Real','Z Ref','Z Real', 'Location', 'BestOutside');

% Subplot Inferior: Error en mm (mismos colores)
subplot(2,1,2); hold on;
plot(Error_mm(:,1), 'b', 'LineWidth', 1.5); % Error X
plot(Error_mm(:,2), 'r', 'LineWidth', 1.5); % Error Y
plot(Error_mm(:,3), 'g', 'LineWidth', 1.5); % Error Z

grid on;
ylabel('Error [mm]'); 
xlabel('Pasos de tiempo');
title('Error de Posición en Milímetros');
legend('Error X', 'Error Y', 'Error Z', 'Location', 'BestOutside');

% --- FIGURA 3: Comandos (Motores) ---
figure(3); clf;
plot(q_solucion * (180/pi), 'LineWidth', 1.5); % A grados
grid on;
xlabel('Pasos'); ylabel('Ángulo [grados]');
title('Evolución de los Ángulos Articulares');
legend('q1','q2','q3','q4','q5', 'Location', 'BestOutside');