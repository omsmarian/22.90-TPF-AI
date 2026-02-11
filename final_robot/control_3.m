%% CONTROL_TRAYECTORIA_CORREGIDO.m
clc; clear; close all;

%% ================== 1) DEFINICION DEL BRAZO ================================

L1 = 0.13;      % altura base
L2 = 0.144;     % link 2
L3 = 0.05;      % link 3 (si lo estás usando como "codo corto")
L4 = 0.144;     % link 4
L_marker = 0.144; % longitud marcador (tool)

links(1) = Link('revolute', 'alpha', 0,    'a', 0,               'd', L1, 'offset', 0,     'modified');
links(2) = Link('revolute', 'alpha', pi/2, 'a', 0,               'd', 0,  'offset', pi/2,  'modified');
links(3) = Link('revolute', 'alpha', 0,    'a', sqrt(L2^2+L3^2), 'd', 0,  'offset', 0,     'modified');
links(4) = Link('revolute', 'alpha', 0,    'a', L3,              'd', 0,  'offset', 0,     'modified');
links(5) = Link('revolute', 'alpha', pi/2, 'a', L4,              'd', 0,  'offset', -pi/2, 'modified');

Robot = SerialLink(links, 'name', 'Robotito');

% Tool: marcador en +Y del efector final
Robot.tool = transl(0, L_marker, 0);

%% ================== 2) LIMITES ARTICULARES =================================
Robot.qlim = deg2rad([ ...
   -150  150;
    -90   90;
    -90   90;
   -100  100;
   -150  150]);

%% ================== 3) POSICION DE REPOSO ==================================
q_rest_deg = [90  -45  90  -45  0];
q_rest = deg2rad(q_rest_deg);

%% ================== 4) DEFINICION DE LA HOJA ===============================
rect_Lx = 0.200;
rect_Ly = 0.150;

dist_base_hoja = 0.210;
x_c = dist_base_hoja;  y_c = 0.0;  z_hoja = 0.0;

%% ================== 5) RECTA A DIBUJAR =====================================
u1=0.20; v1=0.30; u2=0.80; v2=0.60;

P_ini = [x_c - rect_Lx/2 + u1*rect_Lx, ...
         y_c - rect_Ly/2 + v1*rect_Ly, z_hoja];

P_fin = [x_c - rect_Lx/2 + u2*rect_Lx, ...
         y_c - rect_Ly/2 + v2*rect_Ly, z_hoja];

%% ================== 6) DISCRETIZACION ======================================
n_segmentos = 6;
npts_por_segmento = 15;
npts_total = n_segmentos*npts_por_segmento;

s = linspace(0,1,npts_total)';
P_line = P_ini + (P_fin - P_ini).*s;

%% ================== 7) IK NUMERICA CON FMINCON =============================
fprintf('\nCalculando cinemática inversa...\n');

q_traj = zeros(npts_total,5);
q_prev = q_rest;

ANGULO_BASE_DESEADO = 180;
TOLERANCIA_BASE = 15;

options = optimoptions('fmincon', ...
    'Display','off', ...
    'Algorithm','sqp', ...
    'MaxIterations',200, ...
    'OptimalityTolerance',1e-6, ...
    'StepTolerance',1e-6);

lb0 = Robot.qlim(:,1)';   % límites base
ub0 = Robot.qlim(:,2)';

for k = 1:npts_total
    pd = P_line(k,:)';

    cost_fun = @(q) costo_posicion_orientacion(q, pd, Robot);

    lb = lb0; ub = ub0;

    % forzar q1 cerca del ángulo deseado
    lb(1) = deg2rad(ANGULO_BASE_DESEADO - TOLERANCIA_BASE);
    ub(1) = deg2rad(ANGULO_BASE_DESEADO + TOLERANCIA_BASE);

    [q_sol, fval, exitflag] = fmincon(cost_fun, q_prev, ...
        [],[],[],[], lb, ub, [], options);

    if exitflag < 1
        warning('No convergió en punto %d (costo %.4g)', k, fval);
    end

    q_traj(k,:) = q_sol;
    q_prev = q_sol;
end

%% ================== 8) VERIFICACION ========================================
P_real = zeros(npts_total,3);
for k = 1:npts_total
    T = Robot.fkine(q_traj(k,:));
    P_real(k,:) = transl(T)';
end

err = vecnorm(P_line - P_real, 2, 2);
fprintf('\nError máximo: %.2f mm\n', 1000*max(err));
fprintf('Error medio : %.2f mm\n', 1000*mean(err));




%% ================== 11) GRAFICO DE ANGULOS VS TIEMPO =======================

figure('Name', 'Evolución de Ángulos', 'NumberTitle', 'off');
set(gcf, 'Position', [100 100 1000 700]);

% Opción A: tiempo normalizado (0 a 1)
t = linspace(0, 1, npts_total);

% % Opción B: tiempo real aproximado (si usás delay=0.03 en la animación)
% dt = 0.03;                 % [s] igual al delay que usaste en Robot.plot
% t = (0:npts_total-1)*dt;   % tiempo real [s]

nombres_joints = {'Base (q1)', 'Hombro (q2)', 'Codo (q3)', ...
                  'Muñeca 1 (q4)', 'Muñeca 2 (q5)'};

for i = 1:5
    subplot(5,1,i);
    plot(t, rad2deg(q_traj(:,i)), 'b-', 'LineWidth', 1.5); hold on;

    % Límites articulares del robot
    qmin = rad2deg(Robot.qlim(i,1));
    qmax = rad2deg(Robot.qlim(i,2));

    plot([t(1) t(end)], [qmin qmin], 'r--', 'LineWidth', 1);
    plot([t(1) t(end)], [qmax qmax], 'r--', 'LineWidth', 1);

    ylabel(sprintf('%s [°]', nombres_joints{i}), 'FontSize', 10);
    grid on;

    % margen visual
    ylim([qmin-5, qmax+5]);

    if i == 1
        title('Evolución de los Ángulos Articulares', 'FontSize', 12);
    end
    if i == 5
        xlabel('Tiempo', 'FontSize', 10);
    end
end
%% ================== 9) VISUALIZACION =======================================
figure('Name','Trayectoria','NumberTitle','off');

xv = x_c + rect_Lx/2 * [-1  1  1 -1 -1];
yv = y_c + rect_Ly/2 * [-1 -1  1  1 -1];
zv = z_hoja * ones(1,5);

plot3(xv,yv,zv,'g-','LineWidth',3); hold on;
patch(xv(1:4), yv(1:4), zv(1:4), [0.8 1.0 0.8], ...
      'FaceAlpha', 0.2, 'EdgeColor','g', 'LineWidth',2);

plot3(P_line(:,1), P_line(:,2), P_line(:,3), 'r-', 'LineWidth',3);
plot3(P_real(:,1), P_real(:,2), P_real(:,3), 'b--','LineWidth',2);

grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Trayectoria deseada (rojo) vs real (azul)');
view(45,30);

%% ================== 10) ANIMACION ==========================================
figure('Name','Animación','NumberTitle','off');
Robot.plot(q_rest, 'workspace', [-0.05 0.45 -0.25 0.25 -0.05 0.35], 'scale',0.4);
hold on;
patch(xv(1:4), yv(1:4), zv(1:4), [0.8 1.0 0.8], ...
      'FaceAlpha', 0.3, 'EdgeColor','g', 'LineWidth',3);
plot3(P_line(:,1), P_line(:,2), P_line(:,3), 'r-', 'LineWidth', 3);

Robot.plot(q_traj, 'trail', 'b-', 'delay', 0.03);

%% ================== FUNCION LOCAL ==========================================
function costo = costo_posicion_orientacion(q, pd, robot)
    T = robot.fkine(q);

    % Error de posición
    p = transl(T);
    epos = norm(p - pd);

    % Orientación: queremos que el eje Y del EE apunte a -Z global
    R = t2r(T);        % rotación
    y_ee = R(:,2);
    zdes = [0;0;-1];
    eori = norm(y_ee - zdes);

    % costo ponderado
    costo = epos + 0.1*eori;
end