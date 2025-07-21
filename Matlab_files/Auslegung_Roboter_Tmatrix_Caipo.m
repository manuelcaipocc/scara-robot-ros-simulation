clc; clear;

% ----------- Symbolic parameters -----------
syms Hs H0 L0 L1 D1 L2 DA2 L3 H3 L5 L4 
syms phi0 phi1 phi2 phi4  % all in radians now
syms nx ny nz ux uy uz ax ay az px py pz real
syms dl1 dl2 dl3 dl4 dh3
pi = sym(pi);

% Joint parameters (angles now in radians!)
alpha = [  0,  pi/2,  -pi/2,   0,    pi,     0,    0,    0];
L     = [  0,    0,     L1,    0,     L2,     0,    0,  0];
D     = [dl1, dl2,   0,   D1,     0,  dl3, dh3,dl4];
phi   = [0, phi0, phi1, 0, phi2, 0, 0,phi4];  % all in radians

% Number of joints
n = length(alpha);

% ----------- Homogeneous transformation matrix function (radians) -----------
getA = @(alpha, L, D, phi) ...
    [ cos(phi), -cos(alpha)*sin(phi),  sin(alpha)*sin(phi), L*cos(phi);
      sin(phi),  cos(alpha)*cos(phi), -sin(alpha)*cos(phi), L*sin(phi);
           0,         sin(alpha),           cos(alpha),         D;
           0,              0,                  0,              1];

% ----------- Step-by-step transformation matrices -----------
A = sym(zeros(4,4,n)); % Symbolic 3D array
for i = 1:n
    fprintf('\n------ Joint %d ------\n', i);
    % Show parameter values
    fprintf('alpha_%d = %.4f rad\n', i, double(alpha(i)));
    fprintf('L_%d = %s\n', i, string(L(i)));
    fprintf('D_%d = %s\n', i, string(D(i)));
    fprintf('phi_%d = %s rad\n', i, string(phi(i)));
    
    % Build and simplify matrix
    A_i = getA(alpha(i), L(i), D(i), phi(i));
    A_i_simple = simplify(A_i, 'Steps', 100);
    A_i_clean = vpa(A_i_simple, 6);  % Round to 6 decimal places
    
    A(:,:,i) = A_i_clean;
    % Show matrix
    disp('Simplified & rounded matrix:');
    disp(A_i_clean);
end

% ----------- Final total transformation -----------
T_total = eye(4);
for i = 1:n
    T_total = T_total * A(:,:,i);
end
disp('====== Final Total Transformation T_base_TCP ======');
disp(vpa(simplify(T_total, 'Steps', 100), 6));  % Also rounded

% ----------- Inverse transformations -----------
disp('====== Inverse Transformation ======');
T = [nx ux ax px;
     ny uy ay py;
     nz uz az pz;
     0   0   0  1];

% Inverse transformation matrix
% n is already defined as length(alpha)
INV= sym(zeros(4,4,n)); % Symbolic 3D array
for i =1:n
    fprintf("Inverse joint : %d \n",i)
    INV(:,:,i)= simplify(inv(A(:,:,i)));
    disp(INV(:,:,i));
    
end