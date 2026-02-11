
%% ---- Metodo 1: Transformadas a mano ----
% Cadena es R–P–P - multiplicacion de transformadas paso a paso
clear; clc;

%------Parametros geometricos-------
syms th1 th2 th3 th4 th5 L1 L2 L3 L4 L5 real
P = sym(pi);

%----- Helpers: matrices homogeneas basicas -----
Rztita = @(t) [  cos(t)  -sin(t)   0   0;     % rotacion alrededor de z
                 sin(t)  cos(t)    0   0;
                  0       0        1   0;
                  0       0        0   1 ];

Rxalfa = @(t) [ 1  0        0       0;         % Rotacion alrededor de x
                0  cos(t)  -sin(t)  0;
                0  sin(t)  cos(t)   0;
                0   0       0       1 ];

Dxa = @(a) [ 1 0 0 a;                          % traslacion a lo largo de x
            0 1 0 0;
            0 0 1 0;
            0 0 0 1 ];

Dza = @(a) [ 1 0 0 0;                          % traslacion a lo largo de z
            0 1 0 0;
            0 0 1 a;
            0 0 0 1 ];


% ----- Transformadas de cada eslabon -----
% ^0T1: gira la base z por th1 y luego avanza L1 sobre x del eslabon 1
T01 = Rxalfa(0) *Dxa(0) * Rztita(th1) * Dza(L1);

% ^1T2: (plano) giro relativo th2 y traslacion L2 sobre x del eslabon 2
T12 = Rxalfa(P/2) *Dxa(0) * Rztita(th2 +P/2) * Dza(0);

% ^2T3: (muñeca/herramienta) giro relativo th3 y offset L3
T23 = Rxalfa(0) *Dxa((L2^2+L3^2)^(1/2)) * Rztita(th3) * Dza(0);

% ^3T4: (muñeca/herramienta) giro relativo th3 y offset L3
T34 = Rxalfa(0) *Dxa(L4) * Rztita(th4+P/2) * Dza(0);


% ^4T5: (muñeca/herramienta) giro relativo th3 y offset L3
T45 = Rxalfa(P/2) *Dxa(0) * Rztita(th5) * Dza(0);


% ^5Tee: (muñeca/herramienta) giro relativo th3 y offset L3
T5ee = Rxalfa(0) *Dxa(0) * Rztita(0) * Dza(L5);


%----- Transformada total hasta el extremo -----
T0ee = T01 * T12 * T23 * T34* T45* T5ee;        % ^0T3 = ^0T1 * ^1T2 * ^2T3

T0ee = simplify(T01 * T12 * T23 * T34* T45* T5ee);
R03 = T0ee(1:3,1:3);  % Toma las filas 1 a 3 y las columnas 1 a 3
p03 = T0ee(1:3,4);    % Aca solo toma las filas 1 a 3 y la columna 4 completa. 


% ----- Mostrar en consola -----
%Ejemplo de la funcion "disp":
%disp('^0T1 =');   % Muestra el texto literal
%disp(T01);        % Muestra el contenido numérico de la variable T01

disp('^0T1 ='); disp(T01);  % disp --> es Display
disp('^1T2 ='); disp(T12);
disp('^2T3 ='); disp(T23);
disp('^3T4 ='); disp(T34);
disp('^4T5 ='); disp(T45);
disp('^5Tee ='); disp(T5ee);
disp('T0ee ='); disp(T0ee);
disp('p03 ='); disp(p03);

%% ================ Velocidades y Fuerzas ======================

% Rotaciones de cada articulacion
% Necesito las traspuestas --> ' 
% --- Transformadas por eslabón (de i-1 a i) ---
T = {T01, T12, T23, T34, T45, T5ee};   % 6 transformadas, 5 juntas + ee

n = numel(T);      % n = 6 frames (1..6), juntas = 5
nj = 5;            % cantidad de juntas (revolutas): 1..5

% Rotaciones y traslaciones p_{i-1,i} expresadas en frame (i-1)
R = cell(1,n);
p = cell(1,n);
for i = 1:n
    R{i} = T{i}(1:3,1:3);   % ^{i-1}R_i
    p{i} = T{i}(1:3,4);     % ^{i-1}p_i  (vector O_{i-1}->O_i en frame i-1)
end


syms th1 th2 th3 th4 th5 real
syms th1d th2d th3d th4d th5d real     % qdot
syms th1dd th2dd th3dd th4dd th5dd real % qddot

q   = [th1;  th2;  th3;  th4;  th5];
qd  = [th1d; th2d; th3d; th4d; th5d];
qdd = [th1dd;th2dd;th3dd;th4dd;th5dd];

z = sym([0;0;1]);   % z unitario de cada frame (en su propio frame)



% Estados base (frame 0 expresado en 0)
w0  = sym([0;0;0]);
wd0 = sym([0;0;0]);
v0  = sym([0;0;0]);

%  dinámica con gravedad:
syms g real
vd0 = sym([0;0;0]);        % o vd0 = [0;0;g] dependiendo convención

% Almacenamiento
w  = cell(1,n);   wd = cell(1,n);
v  = cell(1,n);   vd = cell(1,n);

w_prev  = w0;  wd_prev = wd0;
v_prev  = v0;  vd_prev = vd0;

for i = 1:n
    % Rotación para pasar de frame (i-1) a frame i:
    % ^iR_{i-1} = (^{i-1}R_i)'
    Ri = R{i}.';  

    % Junta i: solo existe para i=1..5 (las 5 revolutas); en ee no hay junta
    if i <= nj
        qi_d  = qd(i);
        qi_dd = qdd(i);

        % --- Revoluta ---
        w{i}  = Ri*w_prev + qi_d*z;
        wd{i} = Ri*wd_prev + cross(Ri*w_prev, qi_d*z) + qi_dd*z;

        v{i}  = Ri*( v_prev + cross(w_prev, p{i}) );
        vd{i} = Ri*( vd_prev + cross(wd_prev, p{i}) + cross(w_prev, cross(w_prev, p{i})) );

        % (Si fuera PRISMÁTICA: w{i}=Ri*w_prev; wd{i}=Ri*wd_prev; y sumás +qd*z, +qdd*z en v/vd)
    else
        % --- frame EE sin junta ---
        w{i}  = Ri*w_prev;
        wd{i} = Ri*wd_prev;
        v{i}  = Ri*( v_prev + cross(w_prev, p{i}) );
        vd{i} = Ri*( vd_prev + cross(wd_prev, p{i}) + cross(w_prev, cross(w_prev, p{i})) );
    end

    % avanzar
    w_prev  = w{i};  wd_prev = wd{i};
    v_prev  = v{i};  vd_prev = vd{i};
end

% (Opcional) simplificar
for i=1:n
    w{i}  = simplify(w{i},  'Steps', 50);
    wd{i} = simplify(wd{i}, 'Steps', 50);
    v{i}  = simplify(v{i},  'Steps', 50);
    vd{i} = simplify(vd{i}, 'Steps', 50);
end


% Parámetros dinámicos (solo para links 1..5; el ee opcional)
syms m1 m2 m3 m4 m5 real
m = {m1,m2,m3,m4,m5};

% Centros de masa (placeholders): rCi en frame i
syms rc1x rc1y rc1z rc2x rc2y rc2z rc3x rc3y rc3z rc4x rc4y rc4z rc5x rc5y rc5z real
rC = { [rc1x;rc1y;rc1z], [rc2x;rc2y;rc2z], [rc3x;rc3y;rc3z], [rc4x;rc4y;rc4z], [rc5x;rc5y;rc5z] };

% Inercias (matrices 3x3 en el CM, expresadas en frame i)
syms I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz I4xx I4yy I4zz I5xx I5yy I5zz real
I = { diag([I1xx I1yy I1zz]), diag([I2xx I2yy I2zz]), diag([I3xx I3yy I3zz]), diag([I4xx I4yy I4zz]), diag([I5xx I5yy I5zz]) };

% Velocidad/aceleración en CM + fuerzas y momentos
vC  = cell(1,nj);  aC = cell(1,nj);
F   = cell(1,nj);  N = cell(1,nj);

for i=1:nj
    vC{i} = v{i} + cross(w{i}, rC{i});
    aC{i} = vd{i} + cross(wd{i}, rC{i}) + cross(w{i}, cross(w{i}, rC{i}));

    F{i}  = m{i} * aC{i};
    N{i}  = I{i}*wd{i} + cross(w{i}, I{i}*w{i});

    vC{i} = simplify(vC{i}); aC{i} = simplify(aC{i});
    F{i}  = simplify(F{i});  N{i}  = simplify(N{i});
end



f = cell(1,n);   nn = cell(1,n);   % fuerza y momento transmitidos
tau = sym(zeros(nj,1));

% condiciones al final (sin carga externa):
f_next  = sym([0;0;0]);
n_next  = sym([0;0;0]);

for i = nj:-1:1
    % Rotación de frame i a frame i+1 expresada en frame i:
    % ^iR_{i+1} = R{i+1}  (porque R{i+1} es ^iR_{i+1})
    Rip1 = R{i+1};  

    % vector desde origen i a origen i+1 en frame i:
    pip1 = p{i+1};

    % fuerza y momento recursivos
    f{i}  = Rip1*f_next + F{i};
    nn{i} = N{i} + Rip1*n_next + cross(rC{i}, F{i}) + cross(pip1, Rip1*f_next);

    % torque de junta revoluta i
    tau(i) = simplify( nn{i}.' * z );

    % avanzar hacia atrás
    f_next = f{i};
    n_next = nn{i};
end

disp('Torques articulares tau ='); disp(tau);
