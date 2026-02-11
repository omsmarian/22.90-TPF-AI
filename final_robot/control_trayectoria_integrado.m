%% control_trayectoria_integrado.m
% Integración: reposo -> arriba -> bajar -> dibujar recta -> subir -> reposo
% IK numérica con fminsearch (compatible, sin ikine/SE3)

clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

%% 1) Parametros (mm) y opciones
L1  = 130;   % d1 (altura)
L2  = 144;   % para d_eq
L3  = 50;    % para d_eq
L4  = 144;   % a4
L5  = 144;   % tool (longitud marcador)

Error = 0;   % 0: nominal; 1: aplicar error porcentual
Cerrorp = 0;
if Error ~= 0
    Cerrorp = 1; % [%] (ej -5 a 5)
end
Cerror = Cerrorp/100 + 1;
L = [L1 L2 L3 L4 L5]*Cerror;   % [mm]
Qreposo = [0 0 0 0 0];      % [rad]

%% 2) Trajectory specification (cartesian points) - example points
% Define a set of desired Cartesian waypoints for the end-effector (in mm)
% The user can replace these with any Nx3 array of [x y z] in mm.
waypoints_mm = [ ...
    150  0  200;   % punto 1
    150 50  200;   % punto 2
    100 50  150;   % punto 3
    100 -50 150;   % punto 4
    150 -50 200;   % punto 5
    150  0  200];  % volver al punto 1
% Parameters for trajectory generation
samples_per_segment = 50; % points between waypoints

% Interpolate straight-line trajectory in Cartesian space (mm)
traj_mm = [];
for k = 1:size(waypoints_mm,1)-1
    p1 = waypoints_mm(k,:);
    p2 = waypoints_mm(k+1,:);
    t = linspace(0,1,samples_per_segment);
    seg = (1-t')*p1 + t'*p2; % samples_per_segment x 3
    traj_mm = [traj_mm; seg]; %#ok<AGROW>
end

% Convert trajectory to meters and build homogeneous transforms for tool tip
traj_m = traj_mm/1000; % [m]
nPoints = size(traj_m,1);
T_traj = repmat(eye(4),1,1,nPoints);
for i = 1:nPoints
    T_traj(:,:,i) = transl(traj_m(i,1), traj_m(i,2), traj_m(i,3));
end

%% 3) Crear robot (DH modificado) en metros
L_m = L/1000;  % [m]
d_eq = sqrt(L(2)^2 + L(3)^2)/1000;  % [m]

R1 = Link('revolute', 'alpha',     0, 'a',  0     , 'd', L_m(1), 'offset',   0   , 'modified');
R2 = Link('revolute', 'alpha', pi/2,  'a',  0     , 'd',     0, 'offset', pi/2  , 'modified');
R3 = Link('revolute', 'alpha',     0, 'a', d_eq  , 'd',     0, 'offset',   0   , 'modified');
R4 = Link('revolute', 'alpha',     0, 'a', L_m(4)  , 'd',     0, 'offset',  pi/2 , 'modified');
R5 = Link('revolute', 'alpha',  pi/2, 'a',  0     , 'd',  0, 'offset',   0   , 'modified');

EE = transl([0, 0, L_m(5)]); % Tool d5 (en metros)
Robot = SerialLink([R1 R2 R3 R4 R5], 'tool', EE,'name','Robotito');

%% 4) Inverse kinematics for whole Cartesian trajectory (numerical fminsearch per point)
% Define simple IK cost function that minimizes pose error (position + orientation)
ik_options = optimset('Display','off','TolX',1e-6,'TolFun',1e-6,'MaxIter',500,'MaxFunEvals',2000);

q_traj = zeros(nPoints, 5);
q0 = Qreposo; % initial guess
for i = 1:nPoints
    Tdes = T_traj(:,:,i);
    cost = @(q) poseErrorCost(q, Robot, Tdes);
    q_sol = fminsearch(cost, q0, ik_options);
    q_traj(i,:) = q_sol;
    q0 = q_sol; % warm start for next point
end

%% 5) Simple visualization: plot robot following trajectory
figure(1); clf;
Robot.plot(q_traj, 'trail', 'r-', 'noraise');
hold on;
plot3(traj_m(:,1), traj_m(:,2), traj_m(:,3), 'b.-','MarkerSize',8);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Robot following Cartesian trajectory');
view(3);
axis equal;
grid on;

%% --- Helper function (local) ------------------------------------------------
function e = poseErrorCost(q, Robot, Tdes)
    % Compute forward kinematics for given q and return scalar error to minimize.
    T = Robot.fkine(q);
    % Position error (Euclidean)
    dp = transl(T) - transl(Tdes);
    pos_err = norm(dp);
    % Orientation error: use rotation matrix difference (Frobenius)
    R1 = t2r(T);
    R2 = t2r(Tdes);
    rot_err = norm(R1 - R2,'fro')/sqrt(2); % normalized
    % Weighted sum (position prioritized)
    e = pos_err + 0.1*rot_err;
end
