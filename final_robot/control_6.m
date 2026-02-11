%% robot_hoja_recta_mover.m
% Robot + hoja + ingreso de 2 puntos (HOJA) + mover el robot siguiendo la recta
% Trayectoria cartesiana: mtraj(@lspb, P1, P2, m)
% IK numérica por fminsearch (solo posición) con semilla q_prev

clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

%% 1) Robot 
L1=130; L2=144; L3=50; L4=144; L5=144; % mm
L = [L1 L2 L3 L4 L5];

L_m  = L/1000;
d_eq = sqrt(L(2)^2 + L(3)^2)/1000;

R1 = Link('revolute', 'alpha',     0, 'a',  0,    'd', L_m(1), 'offset',   0,   'modified');
R2 = Link('revolute', 'alpha',  pi/2, 'a',  0,    'd', 0,     'offset',  pi/2, 'modified');
R3 = Link('revolute', 'alpha',     0, 'a', d_eq,  'd', 0,     'offset',   0,   'modified');
R4 = Link('revolute', 'alpha',     0, 'a', L_m(4),'d', 0,     'offset',  pi/2, 'modified');
R5 = Link('revolute', 'alpha',  pi/2, 'a',  0,    'd', 0,     'offset',   0,   'modified');

EE = transl(0, 0, L_m(5)); % tool 
Robot = SerialLink([R1 R2 R3 R4 R5], 'tool', EE, 'name', 'Robotito');

Qreposo = [0 0 0 0 0];

%% 2) Hoja (mm)
Lhojax    = 150;   % X hoja
Lhojay    = 200;   % Y hoja
DistHojax = 60;    % hoja arranca en X=60mm
z_hoja_mm = 0;

% Trayectoria
VelocidadTraj = 2;                 % 1..10 (1 = lento = más puntos)
AlturaSeguridad_mm = 40;           % subir antes/Después (mm)

%% 3) Figura: robot + hoja
figure('Name','Robot + Hoja + Trazo');
Robot.plot(Qreposo, 'workspace', [-0.5 0.5 -0.5 0.5 -0.05 0.8], 'tilesize', 0.2);
view(135,25); hold on; grid on; axis equal;

% hoja en BASE (mm)
sheet_b_mm = [DistHojax,        -Lhojay/2, z_hoja_mm;
              DistHojax+Lhojax, -Lhojay/2, z_hoja_mm;
              DistHojax+Lhojax,  Lhojay/2, z_hoja_mm;
              DistHojax,         Lhojay/2, z_hoja_mm;
              DistHojax,        -Lhojay/2, z_hoja_mm];
sheet_b_m = sheet_b_mm/1000;
plot3(sheet_b_m(:,1), sheet_b_m(:,2), sheet_b_m(:,3), 'k-', 'LineWidth', 2);
text(sheet_b_m(1,1), sheet_b_m(1,2), sheet_b_m(1,3), '  (0,0) HOJA', 'FontSize', 10);

%% 4) Pedir 2 puntos en HOJA (mm)
disp('=== INGRESO DE 2 PUNTOS EN HOJA (mm) ===');
fprintf('Hoja: %.0f x %.0f mm | Origen: esquina superior izquierda\n', Lhojax, Lhojay);
fprintf('Rangos: x∈[0,%.0f], y∈[0,%.0f]\n', Lhojax, Lhojay);

P1_h = pedir_punto_hoja('P1 [xh yh] = ', Lhojax, Lhojay);
P2_h = pedir_punto_hoja('P2 [xh yh] = ', Lhojax, Lhojay);

P1_b_mm = hoja2base_mm(P1_h, Lhojay, DistHojax, z_hoja_mm);
P2_b_mm = hoja2base_mm(P2_h, Lhojay, DistHojax, z_hoja_mm);

P1 = P1_b_mm/1000;   % m
P2 = P2_b_mm/1000;   % m

% dibujar puntos y línea
plot3(P1(1), P1(2), P1(3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot3(P2(1), P2(2), P2(3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
plot3([P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)], 'b--', 'LineWidth', 1.5);

%% 5) Trayectoria cartesiana (mtraj) + IK (fminsearch) + animación

% cantidad de puntos en la trayectoria según "velocidad"
m = max(40, round(250 / max(1, VelocidadTraj)));

% trayectoria suavizada P1->P2
[p_traj,~,~] = mtraj(@lspb, P1, P2, m);

% IK settings
opts = optimset('Display','off', 'TolX',1e-7, 'TolFun',1e-7, ...
                'MaxIter',3000, 'MaxFunEvals',20000);

lambda = 0.08;  % suavidad joints (subí si hace saltos)

% trayectoria completa: reposo -> arriba(P1) -> bajar -> recta -> subir
q_prev = Qreposo;

% subir a altura de seguridad sobre P1
P1_up = [P1(1) P1(2) (z_hoja_mm + AlturaSeguridad_mm)/1000];

q_prev = mover_a_punto(Robot, q_prev, P1_up, opts, lambda);
% bajar a P1

q_prev = mover_a_punto(Robot, q_prev, P1, opts, lambda);
% seguir recta
for k = 1:size(p_traj,1)
    q_prev = IK_una_pose(Robot, q_prev, p_traj(k,:), opts, lambda);
    Robot.animate(q_prev);
    drawnow;
end

% subir en P2
P2_up = [P2(1) P2(2) (z_hoja_mm + AlturaSeguridad_mm)/1000];

q_prev = mover_a_punto(Robot, q_prev, P2_up, opts, lambda);
% volver a reposo (trayectoria articular)
q_back = jtraj(q_prev, Qreposo, 60);
for k=1:size(q_back,1)
    Robot.animate(q_back(k,:));
    drawnow;
end

disp('✅ Movimiento completado.');

%% ================= FUNCIONES LOCALES =================

function p = pedir_punto_hoja(prompt, Lx, Ly)
    while true
        s = input(prompt, 's');
        v = str2num(s); %#ok<ST2NM>
        if numel(v) ~= 2
            fprintf('Error: ingresá [xh yh]. Ej: [10 20]\n');
            continue;
        end
        xh=v(1); yh=v(2);
        if xh<0 || xh>Lx || yh<0 || yh>Ly
            fprintf('Fuera de hoja. x∈[0,%.0f], y∈[0,%.0f]\n', Lx, Ly);
            continue;
        end
        p = [xh yh];
        return;
    end
end

function P_b_mm = hoja2base_mm(P_h_xy_mm, Ly, DistX, z_mm)
    xh = P_h_xy_mm(1);
    yh = P_h_xy_mm(2);
    xb = DistX + xh;
    yb = -Ly/2 + yh;
    zb = z_mm;
    P_b_mm = [xb yb zb];
end

function q_out = IK_una_pose(robot, q_prev, pd, opts, lambda)
    q_out = fminsearch(@(q) IK_cost_pos(robot, q, pd, q_prev, lambda), q_prev, opts);
end

function J = IK_cost_pos(robot, q, pd, q_prev, lambda)
    q = q(:)';
    p = transl(robot.fkine(q));
    epos = norm(p - pd);
    esuave = lambda * norm(q - q_prev);
    J = epos + esuave;
end

function q_fin = mover_a_punto(robot, q_ini, pd, opts, lambda)
    p0 = transl(robot.fkine(q_ini));
    n = 25;
    [ptr,~,~] = mtraj(@lspb, p0, pd, n);

    q = q_ini;
    for i = 1:n
        q = fminsearch(@(qq) IK_cost_pos(robot, qq, ptr(i,:), q, lambda), q, opts);
        robot.animate(q); drawnow;
    end
    q_fin = q;
end

%% ================== GRAFICO JOINTS (TRAMO DIBUJO) ==========================
if exist('q_Dibujo','var') && ~isempty(q_Dibujo)
    N = size(q_Dibujo,1);
    s_plot = linspace(0,1,N);

    figure('Name','Joints vs progreso (tramo dibujo)','NumberTitle','off');
    set(gcf,'Position',[100 80 900 700]);

    nombres = {'q1 (base)','q2 (hombro)','q3 (codo)','q4 (muñeca1)','q5 (muñeca2)'};

    for i = 1:5
        subplot(5,1,i);
        plot(s_plot, rad2deg(q_Dibujo(:,i)), 'LineWidth', 1.6); hold on;

        if ~isempty(Robot.qlim)
            yline(rad2deg(Robot.qlim(i,1)), '--', 'LineWidth', 1);
            yline(rad2deg(Robot.qlim(i,2)), '--', 'LineWidth', 1);
        end

        grid on;
        ylabel([nombres{i} ' [deg]']);
        if i == 1, title('Evolución de articulaciones (solo tramo de dibujo)'); end
        if i == 5, xlabel('Progreso sobre la recta (0 \rightarrow 1)'); end
    end
else
    disp('q_Dibujo no existe o está vacío: no se registró el tramo de dibujo.');
end

%% ================== GRAFICO JOINTS (TRAYECTORIA COMPLETA) ==================
% Mostrar gráficas de ángulos de juntas y la trayectoria seguida por el EE
if exist('Q_Trayec','var') && ~isempty(Q_Trayec)
    M = size(Q_Trayec,1);
    k = 1:M;

    % Figuras: joints vs muestras
    figure('Name','Joints vs muestras (trayectoria completa)','NumberTitle','off');
    set(gcf,'Position',[120 60 900 700]);

    nombres = {'q1 (base)','q2 (hombro)','q3 (codo)','q4 (muñeca1)','q5 (muñeca2)'};

    for i = 1:5
        subplot(5,1,i);
        plot(k, rad2deg(Q_Trayec(:,i)), 'LineWidth', 1.4); hold on;

        if isprop(Robot,'qlim') && ~isempty(Robot.qlim)
            yline(rad2deg(Robot.qlim(i,1)), '--', 'LineWidth', 1);
            yline(rad2deg(Robot.qlim(i,2)), '--', 'LineWidth', 1);
        end

        grid on;
        ylabel([nombres{i} ' [deg]']);
        if i == 1, title('Evolución de articulaciones (trayectoria completa)'); end
        if i == 5, xlabel('Muestra k'); end
    end

    % Calcular y plotear trayectoria del extremo (XYZ)
    % Cada fila de Q_Trayec -> fkine -> transl
    P_tray = zeros(M,3);
    for idx = 1:M
        T = Robot.fkine(Q_Trayec(idx,:));
        P_tray(idx,:) = transl(T);
    end

    figure('Name','Trayectoria del extremo (XYZ)','NumberTitle','off');
    set(gcf,'Position',[220 80 800 600]);
    plot3(P_tray(:,1), P_tray(:,2), P_tray(:,3), '-b', 'LineWidth', 1.6); hold on;
    plot3(P_tray(1,1), P_tray(1,2), P_tray(1,3), 'go', 'MarkerSize',8, 'LineWidth',1.5);
    plot3(P_tray(end,1), P_tray(end,2), P_tray(end,3), 'ro', 'MarkerSize',8, 'LineWidth',1.5);
    grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Trayectoria del extremo del manipulador');
    legend('Trayectoria','Inicio','Fin');
else
    disp('Q_Trayec no existe o está vacío: no se registró la trayectoria completa.');
end
