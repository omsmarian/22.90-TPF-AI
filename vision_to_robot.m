function [x_m, y_m] = vision_to_robot(u_mv, v_mv)
% VISION_TO_ROBOT Convierte coordenadas de Machine Vision a Terna Base del Robot
%
%   Entrada:
%       u_mv: Coordenada horizontal imagen (0 a 200) [mm]
%       v_mv: Coordenada vertical imagen (0 a 150) [mm]
%
%   Salida:
%       x_m: Coordenada X Robot [metros]
%       y_m: Coordenada Y Robot [metros]

    %% 1. Cálculo del Eje X (Profundidad)
    % La imagen va de 0 (lejos) a 150 (cerca).
    % El robot va de 210 (lejos) a 60 (cerca).
    x_mm = 210 - v_mv;

    %% 2. Cálculo del Eje Y (Lateral)
    % La imagen va de 0 (izquierda) a 200 (derecha).
    % El robot va de 100 (izquierda) a -100 (derecha).
    y_mm = 100 - u_mv;

    %% 3. Conversión a Metros
    % La simulación requiere metros
    x_m = x_mm / 1000;
    y_m = y_mm / 1000;
    
end