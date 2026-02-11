%% BRAZO_DH_CORREGIDO.m
% WidowX MK-II (según tabla DH del PDF)
clc; clear; close all;

%% 1) PARAMETROS GEOMETRICOS [m]
L1 = 0.13;      % altura base
L2 = 0.144;      % link 2
L3 = 0.05;      % link 3
L4 = 0.144;      % link 4
L_marker = 0.144; % marcador

%% 2) DEFINICION DH (Modified DH)
% Link | theta_i | d_i | a_i | alpha_i | offset
%  1   |  q1     | L1  | 0   |   0     |   0
%  2   |  q2     | 0   | 0   |  pi/2   |  pi/2
%  3   |  q3     | 0   | L2  |   0     |   0
%  4   |  q4     | 0   | L3  |   0     |   0
%  5   |  q5     | 0   | L4  |  pi/2   | -pi/2

links(1) = Link('revolute', 'alpha', 0,    'a', 0,  'd', L1, 'offset', 0,     'modified');
links(2) = Link('revolute', 'alpha', pi/2, 'a', 0,  'd', 0,  'offset', pi/2,  'modified');
links(3) = Link('revolute', 'alpha', 0,    'a', sqrt(L2^2+L3^2), 'd', 0,  'offset', 0,     'modified');
links(4) = Link('revolute', 'alpha', 0,    'a', L3, 'd', 0,  'offset', 0,     'modified');
links(5) = Link('revolute', 'alpha', pi/2, 'a', L4, 'd', 0,  'offset', -pi/2, 'modified');
EE = transl([0, 0, L_marker]);

brazo = SerialLink(links, 'name', 'WidowX_MKII');

%% 3) HERRAMIENTA (MARCADOR EN Y DEL EE)
brazo.tool = transl(0, L_marker, 0);

%% 4) LIMITES ARTICULARES
brazo.qlim = deg2rad([ ...
   -150  150;     % q1
    -90   90;     % q2
    -90   90;     % q3
   -100  100;     % q4
   -150  150]);   % q5

%% 5) POSICION DE REPOSO
q_rest = deg2rad([0 -45 90 -45 0]);

%% 8) NUBE DE PUNTOS DEL ESPACIO DE TRABAJO
fprintf('\nGenerando nube de puntos del espacio de trabajo...\n');

nq = 5;   % 5^5 = 3125

q1_vec = linspace(brazo.qlim(1,1), brazo.qlim(1,2), nq);
q2_vec = linspace(brazo.qlim(2,1), brazo.qlim(2,2), nq);
q3_vec = linspace(brazo.qlim(3,1), brazo.qlim(3,2), nq);
q4_vec = linspace(brazo.qlim(4,1), brazo.qlim(4,2), nq);
q5_vec = linspace(brazo.qlim(5,1), brazo.qlim(5,2), nq);

Ntot = nq^5;
P = zeros(Ntot, 3);

idx = 0;
for q1 = q1_vec
  for q2 = q2_vec
    for q3 = q3_vec
      for q4 = q4_vec
        for q5 = q5_vec
          idx = idx + 1;
          q = [q1 q2 q3 q4 q5];
          T = brazo.fkine(q);
          P(idx,:) = transl(T)';
        end
      end
    end
  end
end

fprintf('Total de puntos generados: %d\n', Ntot);

figure('Name','Espacio de Trabajo','NumberTitle','off');
scatter3(P(:,1), P(:,2), P(:,3), 5, P(:,3), 'filled');
colormap(jet); colorbar;
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Nube de Puntos del Espacio de Trabajo');
view(45, 30);

%% ================= Nube de Trabajo Restringida =================


qlim_reducido = deg2rad([ ...
   -60   60;     % q1
     0   90;     % q2
     0  180/2;     % q3
   -45   45;     % q4
   -90   90]);   % q5


nq= 5;
q1_vec= linspace(qlim_reducido(1,1), qlim_reducido(1,2), nq);
q2_vec= linspace(qlim_reducido(2,1), qlim_reducido(2,2), nq);
q3_vec= linspace(qlim_reducido(3,1), qlim_reducido(3,2), nq);
q4_vec= linspace(qlim_reducido(4,1), qlim_reducido(4,2), nq);
q5_vec= linspace(qlim_reducido(5,1), qlim_reducido(5,2), nq);

Ntot = nq^5;
P_red = zeros(Ntot,3);

idx = 0;
for q1 = q1_vec
  for q2 = q2_vec
    for q3 = q3_vec
      for q4 = q4_vec
        for q5 = q5_vec
          idx = idx + 1;
          q = [q1 q2 q3 q4 q5];
          T = brazo.fkine(q);
          P_red(idx,:) = transl(T)';
        end
      end
    end
  end
end

figure;
scatter3(P_red(:,1), P_red(:,2), P_red(:,3), 8, 'r', 'filled');
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Espacio de trabajo con qlim restringidos');
