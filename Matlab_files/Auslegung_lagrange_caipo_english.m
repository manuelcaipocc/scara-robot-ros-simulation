
% Clear workspace and command window, disable all warnings
clc;
clear;
% warning('off','all');

% Display a header for the script
fprintf('                               ╔══════════════════════════════════════════════════╗\n');
fprintf('                               ║          Robot Lagrangian Dynamics             ║\n');
fprintf('                               ╠══════════════════════════════════════════════════╣\n');
fprintf('                               ║   (Combining Jacobian and Inertia Matrices)  ║\n');
fprintf('                               ╚════════════════════════════════════════════════╝\n');

%% 1. Symbolic Parameters and Jacobian Calculation (Original Code)

% Define symbolic variables for geometric parameters
syms Hs H0 L0 L1 D1 L2 DA2 L3 H3 L5 L4 KH3 real;
% Define symbolic variables for joint angles/displacements (all in radians for angles)
syms phi0 phi1 phi2 phi3 phi4 phi5 phi6 dh3 real;
% Define symbolic variables for joint velocities
syms dphi0 dphi1 dphi2 ddh3 dphi4 real;
% Define symbolic variables for joint accelerations
syms ddphi0 ddphi1 ddphi2 dddh3 ddphi4 real;
% Define symbolic variables for specific link lengths/displacements
syms dl1 dl2 dl3 dl4 real;
% Unused symbolic variables (can be removed if not needed elsewhere)
syms d2 dh d6 real;
% Unused symbolic variables for pose (can be removed if not needed elsewhere)
syms nx ny nz ux uy uz ax ay az px py pz real;
% Unused symbolic variables (can be removed if not needed elsewhere)
syms a1 a2 a3 real;

% Define pi as a symbolic constant
pi = sym(pi);

% Define target variables (active joints) for dynamics calculation
target_vars = [phi0, phi1, phi2, dh3, phi4];
target_var_names = {'phi0', 'phi1', 'phi2', 'dh3', 'phi4'};

% Denavit-Hartenberg (DH) parameters: alpha, L, D, phi
% alpha: Twist angle
% L: Link length
% D: Link offset
% phi: Joint angle (theta in standard DH)
alpha = [  0,  pi/2,  -pi/2,   0,    pi,     0,    0,    0];
L     = [  0,    0,     L1,    0,     L2,     0,    0,  0];
D     = [dl1, dl2,   0,   D1,     0,  dl3, dh3,dl4];
phi   = [0, phi0, phi1, 0, phi2, 0, 0,phi4];  % All angles in radians

% Number of joints
n = length(alpha);

% Anonymous function for homogeneous transformation matrix (standard DH parameters)
getA = @(alpha, L, D, phi) ...
    [ cos(phi), -cos(alpha)*sin(phi),  sin(alpha)*sin(phi), L*cos(phi);
      sin(phi),  cos(alpha)*cos(phi), -sin(alpha)*cos(phi), L*sin(phi);
           0,         sin(alpha),           cos(alpha),         D;
           0,              0,                  0,              1];

% Symbolic transformation matrices for each link
A = sym(zeros(4,4,n)); % 3D symbolic array
for i = 1:n
    A_i = getA(alpha(i), L(i), D(i), phi(i));
    A_i_simple = simplify(A_i, 'Steps', 100); % Simplify expression
    A_i_clean = vpa(A_i_simple, 6);  % Round to 6 decimal places for cleaner output
    A(:,:,i) = A_i_clean;
end

% A_eval will store transformation matrices with numerical geometric constants
A_eval = sym(zeros(4, 4, n));

% === Replace Geometric Constants with Numerical Values ===
L1_val  = 0.4;
L2_val  = 0.5;
D1_val  = 0.11;
dl1_val = 0.1;
dl2_val = 0.9;
dl3_val = 0.76;
dl4_val = 0.35;

% Create a struct for substitutions
subs_vals = struct( ...
    'L1',  L1_val, ...
    'L2',  L2_val, ...
    'D1',  D1_val, ...
    'dl1', dl1_val, ...
    'dl2', dl2_val, ...
    'dl3', dl3_val, ...
    'dl4', dl4_val ...
);

% Populate A_eval by applying numerical geometric values
for i = 1:n
    A_i_eval = getA(alpha(i), L(i), D(i), phi(i));
    A_i_eval = subs(A_i_eval, subs_vals);      % Apply geometric values
    A_i_eval = simplify(A_i_eval, 'Steps', 100); % Simplify again
    A_i_eval = vpa(A_i_eval, 6);               % Round visually
    A_eval(:,:,i) = A_i_eval;
end

% Total transformation matrix from base to end-effector
T_total = eye(4);
for i = 1:n
    T_total = T_total * A(:,:,i);
end

% Calculate accumulated z-axes and origin positions for each joint frame
z_list = sym(zeros(3, n)); % List of z-axes of each frame
o_list = sym(zeros(3, n)); % List of origins of each frame

T = eye(4); % Initialize cumulative transformation
for i = 1:n
    T = T * A(:,:,i);
    z_list(:,i) = T(1:3, 3);        % Third column of rotation matrix is z-axis
    o_list(:,i) = T(1:3, 4);        % Fourth column (position vector) is origin
end

% Final end-effector position
o_end = T_total(1:3, 4);

% Initialize Geometric Jacobian (6 rows: 3 linear, 3 angular)
Jg = sym(zeros(6, length(target_vars)));

% Add base frame z and p (needed for first joint calculation)
z_prev = [0; 0; 1]; % Z-axis of the base frame
o_prev = [0; 0; 0]; % Origin of the base frame

% Populate the Geometric Jacobian columns
for idx = 1:length(target_vars)
    var_name = target_var_names{idx}; % Get the name of the current target variable

    if strcmp(var_name, 'dh3') % Special case for prismatic joint dh3 (linear displacement)
        i = 7; % Corresponds to the 7th frame in the DH table for dh3
        z = z_list(:, i-1);  % Z-axis of the frame BEFORE the prismatic joint (z6)
        Jg(1:3, idx) = z;       % Linear velocity component is simply the z-axis direction
        Jg(4:6, idx) = [0; 0; 0]; % Angular velocity component is zero for a prismatic joint
    else % Revolute joints (phi0, phi1, phi2, phi4)
        i = find(phi == target_vars(idx)); % Find which DH frame corresponds to this joint variable
        if isempty(i)
            continue; % Skip if the variable is not found in the phi list
        end

        if i == 1 % For the first joint (phi0)
            z = z_prev; % Use base frame z-axis
            o = o_prev; % Use base frame origin
        else % For subsequent joints
            z = z_list(:, i-1); % Z-axis of the frame BEFORE the current joint
            o = o_list(:, i-1); % Origin of the frame BEFORE the current joint
        end

        Jg(1:3, idx) = cross(z, (o_end - o)); % Linear velocity component (cross product)
        Jg(4:6, idx) = z; % Angular velocity component (z-axis of the joint)
    end
end

% Simplify the Geometric Jacobian
Jg = simplify(Jg);

fprintf('\n=== Final Geometric Jacobian ===\n');
disp(Jg);

% Apply numerical geometric constant substitutions to the Jacobian
Jg_num = subs(Jg, subs_vals);
% Round visually for cleaner numerical output
Jg_num = vpa(Jg_num, 4);  % 4 decimal places

fprintf('\n=== Geometric Jacobian with evaluated geometric constants ===\n');
disp(Jg_num);

% Calculation of Jacobians for Centers of Mass

% 1. First, we need the positions of the centers of mass for each link.
% We assume the center of mass is at the midpoint of each link,
% unless specified otherwise for CAD-derived bodies.

% Transformation matrices to each center of mass
T_cm = sym(zeros(4, 4, n));
J_cm = cell(1, n); % Cell array to store Jacobians for each center of mass

for i = 1:n
    % Accumulator for transformation up to the previous link (i-1)
    T_prev = eye(4);
    for j = 1:i-1
        T_j = getA(alpha(j), L(j), D(j), phi(j));
        T_j = subs(T_j, subs_vals);  % Substitute geometric constants
        T_prev = T_prev * T_j;
    end

    % Lengths to midpoint for CM (these values are already defined in subs_vals)
    % For links with length, this will be L(i)/2, for prismatic, D(i)/2 etc.
    % This part assumes L(i) and D(i) are symbolic, so we need to substitute.
    L_half = subs(L(i), subs_vals) / 2;
    D_half = subs(D(i), subs_vals) / 2;

    % Advance to the center of mass of link i
    A_half = getA(alpha(i), L_half, D_half, phi(i));
    A_half = subs(A_half, subs_vals);  % Substitution applied here as well

    % Total transformation to the center of mass of link i
    T_cm(:,:,i) = simplify(T_prev * A_half, 'Steps', 50);
end

% 2. Now, we calculate the Jacobian for each center of mass.
for cm_idx = 1:n
    % Initialize Jacobian for the current center of mass
    J_current = sym(zeros(6, length(target_vars)));
    o_cm = T_cm(1:3, 4, cm_idx);  % Position of the current center of mass

    for var_idx = 1:length(target_vars)
        var_name = target_var_names{var_idx};

        if strcmp(var_name, 'dh3') % Prismatic joint case
            i = 7; % Corresponds to the 7th DH frame for dh3
            if cm_idx >= i  % dh3 affects subsequent bodies
                z = z_list(:, i-1);  % z-axis of the frame BEFORE the prismatic joint (z6)
                J_current(1:3, var_idx) = z; % Linear velocity component
                J_current(4:6, var_idx) = [0; 0; 0]; % Angular velocity component is zero
            end
        else % Revolute joint case
            i = find(phi == target_vars(var_idx));
            if isempty(i)
                continue;
            end

            if i == 1 % First joint
                z = [0; 0; 1]; % Z-axis of the base frame
                o = [0; 0; 0]; % Origin of the base frame
            else
                z = z_list(:, i-1); % Z-axis of the frame BEFORE the current joint
                o = o_list(:, i-1); % Origin of the frame BEFORE the current joint
            end

            if i <= cm_idx  % Only previous joints influence the current link's CM velocity
                J_current(1:3, var_idx) = cross(z, (o_cm - o)); % Linear velocity component
                J_current(4:6, var_idx) = z; % Angular velocity component
            end
        end
    end
    J_cm{cm_idx} = simplify(J_current); % Simplify the Jacobian for the current CM
    fprintf('\n=== Jacobian for link %d center of mass ===\n', cm_idx);
    disp(J_cm{cm_idx});
end

% 3. Apply numerical substitutions to the CM Jacobians.
J_cm_num = cell(1, n);
for cm_idx = 1:n
    if ~isempty(J_cm{cm_idx})
        J_cm_num{cm_idx} = subs(J_cm{cm_idx}, subs_vals);
        J_cm_num{cm_idx} = vpa(J_cm_num{cm_idx}, 4); % Round to 4 decimal places
        fprintf('\n=== Numerical Jacobian for link %d center of mass ===\n', cm_idx);
        disp(J_cm_num{cm_idx});
    end
end

%2. Calculation of Inertia Matrices for Each Link

% Density of aluminum in kg/m^3
density = 7750;

% Define geometries for each link:
% [Length, Type, Outer Radius/Side (r_ext), Inner Radius/Side (r_int),
%  Outer Side (a_ext), Inner Side (a_int), Principal Axis Direction (1=X, 2=Y, 3=Z)]
% Type: 1 = Hollow Cylinder, 2 = Hollow Square Prism, 3 = CAD-derived (Cylinder + Sphere), 4 = CAD-derived (Square + Cylinder)
geometrias = {
    [0.200, 1, 0.300/2, 0.200/2, 0,     0, 3]; % Link 1: Hollow Cylinder
    [0.800, 3, 0.200/2, 0.150/2, 0,     0, 3]; % Link 2: CAD-derived (Cylinder + Sphere) - Note: L is probably unused for type 3/4
    [0.400, 2, 0,     0,     0.100, 0.080, 1]; % Link 3: Hollow Square Prism
    [0.110, 4, 0,     0,     0.100, 0.080, 3]; % Link 4: CAD-derived (Square + Cylinder)
    [0.500, 1, 0.100/2, 0.080/2, 0,     0, 1]; % Link 5: Hollow Cylinder
    [0.050+0.630+0.08, 1, 0.100/2, 0.080/2, 0, 0, 3]; % Link 6: Hollow Cylinder
    [0.360, 1, 0.140/2, 0.080/2, 0,     0, 3]; % Link 7: Hollow Cylinder
    [0.300 + 0.050, 1, 0.140/2, 0.080/2, 0, 0, 3]; % Link 8: Hollow Cylinder
};

% Calculate inertia matrices for each link
I_bodies = cell(1, n); % Cell array to store inertia matrices (in local frame)
masses = sym(zeros(1, n)); % Symbolic array to store mass of each link

fprintf('\n=== Link Inertia Matrices ===\n');
for i = 1:n
    fprintf('\n =======================Link=================\n');
    fprintf(' ======================= %d ===================\n',i);
    params = geometrias{i};
    L_     = params(1);
    tipo   = params(2);
    r_ext  = params(3);
    r_int  = params(4);
    a_ext  = params(5);
    a_int  = params(6);

    % Calculate inertia matrix for the current link
    I_bodies{i} = computeInertiaMatrix(tipo, L_, r_ext, r_int, a_ext, a_int, density);

    % Calculate mass based on geometry type
    V = 0; % Initialize volume
    if tipo == 1 % Hollow cylinder
        V = pi * (r_ext^2 - r_int^2) * L_;
    elseif tipo == 2 % Hollow square prism
        V = (a_ext^2 - a_int^2) * L_;
    elseif tipo == 3 % CAD-derived (Cylinder + Sphere) - Volume derived from CAD
        V = 0.012967447276;
    elseif tipo == 4 % CAD-derived (Square + Cylinder) - Volume derived from CAD
        V = 0.000740789794;
    end
    masses(i) = density * V; % Calculate mass

    fprintf('Link %d - Mass: %.3f kg\n', i, masses(i));
    disp(I_bodies{i});
end

%3. Calculation of the Mass Matrix using the Jacobian

% Initialize the inertial mass matrix
M = sym(zeros(length(target_vars)));

% Iterate through each movable link (assuming all links contribute)
for k = 1:n
    % Get the linear and angular Jacobian for the center of mass of link k
    % (We use the pre-calculated Jacobians for the center of mass)
    if isempty(J_cm_num{k})
        continue;  % Skip if there's no Jacobian for this center of mass
    end
    Jk_lin = J_cm_num{k}(1:3,:);  % Linear Jacobian for CM
    Jk_ang = J_cm_num{k}(4:6,:);  % Angular Jacobian for CM

    % Get the accumulated transformation T_k for body k
    % T_k = eye(4);
    % for m = 1:k
    %     T_k = T_k * A_eval(:,:,m); % Use A_eval which has numerical geometric constants
    % end

    % Get the rotation matrix R_k for body k
    %R_k = T_k(1:3, 1:3);
    
    % Get the transformation matrix to the center of mass of body k
    T_k = T_cm(:,:,k); % More accurate for rotating the inertia matrix
    R_k = T_k(1:3, 1:3);

    % Transform the inertia matrix from the local frame to the base frame
    I_global_k = vpa(simplify(R_k * I_bodies{k} * R_k.'), 4);

    fprintf('\n--- Inertia Transformation for Link %d ---\n', k);
    fprintf('R_k:\n');
    disp(R_k);
    fprintf('I_body (local):\n');
    disp(I_bodies{k});
    fprintf('I_global = R_k * I_local * R_k^T:\n');
    disp(I_global_k);

    % Calculate contribution to the mass matrix (using the generalized inertia equation)
    M = M + masses(k)*(Jk_lin')*Jk_lin + (Jk_ang')*I_global_k*Jk_ang;
end

M = simplify(M); % Simplify the final mass matrix
fprintf('\n=== Inertial Mass Matrix M(q) ===\n');
vpa(M, 3); % Display with 3 significant figures
disp(M);

% 4. Calculation of Coriolis and Centrifugal Terms

% Initialize the Christoffel symbol matrix (which forms C(q, dq)*dq)
C = sym(zeros(length(target_vars)));

% Define symbolic time for differentiation
syms t real;

% Define symbolic joint position (q) and velocity (dq) vectors
q = target_vars;
dq = sym(zeros(size(q)));
for i = 1:length(q)
    dq(i) = sym(['d' char(q(i))], 'real'); % e.g., dphi0, dphi1
end

% Calculate Christoffel symbols (and thus the Coriolis/Centrifugal matrix C)
% C_ijk = 0.5 * (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i)
% C_matrix(i,j) = sum(C_ijk * dq_k)
for i = 1:length(q)
    for j = 1:length(q)
        for k = 1:length(q)
            % Christoffel symbol component
            c = 0.5*(diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(j,k), q(i)));
            C(i,j) = C(i,j) + c*dq(k); % Accumulate into C matrix
        end
    end
end

C = simplify(C); % Simplify the C matrix
fprintf('\n=== Coriolis and Centrifugal Terms C(q,dq) ===\n');
vpa(C, 3); % Display with 3 significant figures
disp(C);

%5. Calculation of Gravity Terms

% Assume gravity acts in the -Z direction
g = [0; 0; -9.81];

% Initialize gravity vector
G = sym(zeros(length(q),1));

% Calculate total potential energy V
V = 0;
for k = 1:n
    % Get the transformation matrix T_k for link k (to its end, or to its CM if more precise)
    T_k = eye(4);
    for m = 1:k
        T_k = T_k * A_eval(:,:,m); % Use A_eval for numerical geometric constants
    end

    % Determine the local center of mass (COM) position based on the geometry type and principal axis.
    params = geometrias{k};
    L_k = params(1);          % Length of the body
    tipo_k = params(2);       % Type of body
    direccion = 3;            % Default principal axis is Z

    if size(params, 2) >= 7
        direccion = params(7);  % If specified, use the given principal axis
    end

    % === Local Center of Mass based on principal direction ===
    com_local = zeros(3,1); % Initialize local COM vector
    switch direccion
        case 1  % X-axis
            com_local(1) = L_k/2;
        case 2  % Y-axis
            com_local(2) = L_k/2;
        case 3  % Z-axis
            com_local(3) = L_k/2;
        otherwise
            error('Unrecognized direction for body %d', k);
    end

    % === If the body is CAD-derived, override com_local with CAD values ===
    if tipo_k == 3 % CAD type 3 (Cylinder + Sphere)
        com_local = zeros(3,1); % Reset to zero
        com_local(direccion) = 0.470732; % Specific CAD-derived COM distance for this type
    elseif tipo_k == 4 % CAD type 4 (Square + Cylinder)
        com_local = zeros(3,1); % Reset to zero
        com_local(direccion) = 0.111934; % Specific CAD-derived COM distance for this type
    end

    % === Transform the center of mass to the base frame ===
    R_k = T_k(1:3, 1:3); % Rotation matrix from T_k
    com_k = R_k * com_local + T_k(1:3, 4); % Transform local COM to base frame

    % Accumulate potential energy: V = sum(mass_k * g' * com_k)
    V = V + masses(k)*g'*com_k;
end

% Calculate partial derivatives of V with respect to each joint variable (q_i)
% G(i) = dV/dq_i
for i = 1:length(q)
    G(i) = diff(V, q(i));
end

G = simplify(G); % Simplify the gravity vector
fprintf('\n=== Gravity Terms G(q) ===\n');
vpa(G, 3); % Display with 3 significant figures
disp(G);

% 6. Final Equations of Motion

fprintf('\n=== Complete Equations of Motion ===\n');
fprintf('M(q)*ddq + C(q,dq)*dq + G(q) = tau\n');

tol = 1e-6;  % Threshold to consider a number as zero

% === Cleaning and Rounding M Matrix ===
M_clean = vpa(M, 3);  % Step 1: Round to 3 significant figures
M_clean = mapSymType(M_clean, 'constant', ...
    @(x) piecewise(abs(x) < tol, 0, x));  % Step 2: Set small terms to zero
M_clean = simplify(M_clean, 'Steps', 100);  % Step 3: Further simplification


% === Parámetros ===
tol = 1e-4;  % Umbral de tolerancia para considerar como cero
digits(5);   % Precisión de vpa

% === Paso 1: Redondeo global ===
M_clean = simplify(M, 'Steps', 50);  % Opcional: simplifica primero

% === Paso 2: Redondear coeficientes numéricos dentro de cada entrada ===
M_clean = arrayfun(@(x) vpa(x, 3), M_clean);
 

% === Paso 3: Eliminar coeficientes menores que tolerancia ===
M_clean = mapSymType(M_clean, 'constant', ...
    @(x) piecewise(abs(x) < tol, 0, x));

% === Paso 4: Simplificar final ===
M_clean = simplify(M_clean, 'Steps', 100);

% === Mostrar resultado ===
fprintf('\n=== Cleaned and Rounded M Matrix test 2===\n');
disp(M_clean)




% === Cleaning and Rounding C Matrix ===
C_clean = vpa(C, 3);  % Step 1: Round to 3 significant figures
C_clean = mapSymType(C_clean, 'constant', ...
    @(x) piecewise(abs(x) < tol, 0, x));  % Step 2: Set small terms to zero
C_clean = simplify(C_clean, 'Steps', 100);  % Step 3: Further simplification

% === Parámetros ===
tol = 1e-4;  % Umbral de tolerancia para considerar como cero
digits(5);   % Precisión de vpa

% === Paso 1: Redondeo global ===
C_clean = simplify(C, 'Steps', 50);  % Opcional: simplifica primero

% === Paso 2: Redondear coeficientes numéricos dentro de cada entrada ===
C_clean = arrayfun(@(x) vpa(x, 3), C_clean);

% === Paso 3: Eliminar coeficientes menores que tolerancia ===
C_clean = mapSymType(C_clean, 'constant', ...
    @(x) piecewise(abs(x) < tol, 0, x));

% === Paso 4: Simplificar final ===
C_clean = simplify(C_clean, 'Steps', 100);

% === Mostrar resultado ===
fprintf('\n=== Cleaned and Rounded C Matrix test 2===\n');
disp(C_clean)


% === Cleaning and Rounding G Vector ===
G_clean = vpa(G, 3);  % Step 1: Round
G_clean = mapSymType(G_clean, 'constant', ...
    @(x) piecewise(abs(x) < tol, 0, x));  % Step 2: Set small terms to zero
G_clean = simplify(G_clean, 'Steps', 100);  % Step 3: Further simplification


% === Parámetros ===
tol = 1e-4;  % Umbral de tolerancia para considerar como cero
digits(5);   % Precisión de vpa

% === Paso 1: Redondeo global ===
G_clean = simplify(G, 'Steps', 50);  % Opcional: simplifica primero

% === Paso 2: Redondear coeficientes numéricos dentro de cada entrada ===
G_clean = arrayfun(@(x) vpa(x, 3), G_clean);

% === Paso 3: Eliminar coeficientes menores que tolerancia ===
G_clean = mapSymType(G_clean, 'constant', ...
    @(x) piecewise(abs(x) < tol, 0, x));

% === Paso 4: Simplificar final ===
G_clean = simplify(G_clean, 'Steps', 100);

% === Mostrar resultado ===
fprintf('\n=== Cleaned and Rounded G Matrix test 2===\n');
disp(G_clean)



% Save the cleaned matrices to a .mat file
save('lagrange_matrices.mat', 'M_clean', 'C_clean', 'G_clean', 'target_vars');

% Helper function to compute inertia matrix for various geometries
function I_body = computeInertiaMatrix(type, L, r_ext, r_int, a_ext, a_int, density)
    fprintf('\n========== Inertia Calculation==========\n');
    fprintf('Length (L): %.3f m\n', L);
    fprintf('Density (ρ): %.1f kg/m³\n', density);

    I_xx = 0; I_yy = 0; I_zz = 0; % Initialize inertia components

    if type == 1  % Hollow cylinder
        fprintf('Geometry: Hollow Cylinder\n');
        fprintf('Outer radius (r_ext): %.3f m\n', r_ext);
        fprintf('Inner radius (r_int): %.3f m\n', r_int);
        % Formulas for hollow cylinder inertia about its principal axes (assuming Z-axis along length)
        I_xx = (1/12) * density * pi * (r_ext^2 - r_int^2) * L * (3*(r_ext^2 + r_int^2) + L^2);
        I_yy = I_xx; % Symmetric for cylinders
        I_zz = (1/2) * density * pi * (r_ext^2 - r_int^2) * L * (r_ext^2 + r_int^2);
        fprintf('\nFormula: I_xx = (1/12) * ρ * π * (r_e² - r_i²) * L * [3(r_e² + r_i²) + L²]\n');
        fprintf('I_xx = I_yy = %.6f kg·m²\n', I_xx);
        fprintf('Formula: I_zz = (1/2) * ρ * π * (r_e² - r_i²) * L * (r_e² + r_i²)\n');
        fprintf('I_zz = %.6f kg·m²\n', I_zz);
    elseif type == 2  % Hollow square prism
        fprintf('Geometry: Hollow Square Prism\n');
        fprintf('Outer side length (a_ext): %.3f m\n', a_ext);
        fprintf('Inner side length (a_int): %.3f m\n', a_int);
        % Formulas for hollow square prism inertia about its principal axes (assuming Z-axis along length)
        I_xx = (1/12) * density * (a_ext^2 - a_int^2) * L * (L^2 + (1/3)*(a_ext^2 + a_int^2));
        I_yy = I_xx; % Symmetric for square prisms
        I_zz = (1/6) * density * (a_ext^2 - a_int^2) * L * (a_ext^2 + a_int^2);
        fprintf('\nFormula: I_xx = (1/12) * ρ * (a_e² - a_i²) * L * [L² + (1/3)(a_e² + a_i²)]\n');
        fprintf('I_xx = I_yy = %.6f kg·m²\n', I_xx);
        fprintf('Formula: I_zz = (1/6) * ρ * (a_e² - a_i²) * L * (a_e² + a_i²)\n');
        fprintf('I_zz = %.6f kg·m²\n', I_zz);
    elseif type == 3 % CAD-derived values for a specific part (Cylinder + Sphere)
        fprintf('Geometry: Hollow Cylinder + Sphere (from CAD)\n');
        fprintf('Using CAD-derived inertia values (in kg·m²):\n');
        I_xx = 7749265.619e-6;  % From CAD
        I_yy = 776332.607e-6;   % From CAD
        I_zz = 7749265.619e-6;  % From CAD
        % Note: Center of gravity (Schwerpunkt) at 563.834 mm along the principal direction
    elseif type == 4 % CAD-derived values for another specific part (Square + Top Cylinder)
        fprintf('Geometry: Hollow Square + Top Cylinder (from CAD)\n');
        fprintf('Using CAD-derived inertia values (in kg·m²):\n');
        I_xx = 36063.992e-6;  % From CAD
        I_yy = 18013.291e-6;  % From CAD
        I_zz = 36063.991e-6;  % From CAD
        % Note: Center of gravity (Schwerpunkt) at 129.031 mm along the principal axis
    else
        error('Invalid geometry type. Must be 1 (cylinder), 2 (square), 3 (CAD type 1), or 4 (CAD type 2).');
    end

    % Round inertia components and form the diagonal inertia matrix
    I_xx = round(I_xx, 4);
    I_yy = round(I_yy, 4);
    I_zz = round(I_zz, 4);
    I_body = diag([I_xx, I_yy, I_zz]);
    I_body = vpa(I_body, 4); % Display with 4 significant figures
end


%7. Numerical Evaluation of Torques/Forces using Velocity Profiles

% Load pre-generated trajectories (ensure 'ruckig_trajectories.mat' exists)
load('ruckig_trajectories.mat'); % Loads 'trajectories' struct

% Extract time vectors (assuming phi0 has the longest/most complete time vector)
time_phi0 = trajectories.phi0.time;
time_phi1 = trajectories.phi1.time;
time_phi2 = trajectories.phi2.time;
time_dh3  = trajectories.dh3.time;
time_phi4 = trajectories.phi4.time;

% Extract position, velocity, and acceleration data for each joint
phi0_values = trajectories.phi0.position;
phi1_values = trajectories.phi1.position;
phi2_values = trajectories.phi2.position;
dh3_values = trajectories.dh3.position;
phi4_values = trajectories.phi4.position;

dphi0_values = trajectories.phi0.velocity;
dphi1_values = trajectories.phi1.velocity;
dphi2_values = trajectories.phi2.velocity;
ddh3_values = trajectories.dh3.velocity;
dphi4_values = trajectories.phi4.velocity;

ddphi0_values = trajectories.phi0.acceleration;
ddphi1_values = trajectories.phi1.acceleration;
ddphi2_values = trajectories.phi2.acceleration;
dddh3_values = trajectories.dh3.acceleration;
ddphi4_values = trajectories.phi4.acceleration;

% Number of time points (before interpolation/padding)
n_points_orig = length(phi0_values);

% Preallocate space for calculated torques/forces
tau_eval = zeros(5, n_points_orig); % 5 joints, n_points_orig time steps

% Force all trajectory data to be row vectors for consistent indexing
phi0_values = phi0_values(:)';
phi1_values = phi1_values(:)';
phi2_values = phi2_values(:)';
dh3_values  = dh3_values(:)';
phi4_values = phi4_values(:)';

dphi0_values = dphi0_values(:)';
dphi1_values = dphi1_values(:)';
dphi2_values = dphi2_values(:)';
ddh3_values  = ddh3_values(:)';
dphi4_values = dphi4_values(:)';

ddphi0_values = ddphi0_values(:)';
ddphi1_values = ddphi1_values(:)';
ddphi2_values = ddphi2_values(:)';
dddh3_values  = dddh3_values(:)';
ddphi4_values = ddphi4_values(:)';

% Display original trajectory statistics
fprintf('\n=========== ORIGINAL TRAJECTORY STATISTICS ===========\n');
% Helper function to print stats (defined at the end of the script)
print_stats(trajectories.phi0.position,      'phi0.position');
print_stats(trajectories.phi0.velocity,      'phi0.velocity');
print_stats(trajectories.phi0.acceleration,  'phi0.acceleration');
print_stats(trajectories.phi0.jerk,          'phi0.jerk');

print_stats(trajectories.phi1.position,      'phi1.position');
print_stats(trajectories.phi1.velocity,      'phi1.velocity');
print_stats(trajectories.phi1.acceleration,  'phi1.acceleration');
print_stats(trajectories.phi1.jerk,          'phi1.jerk');

print_stats(trajectories.phi2.position,      'phi2.position');
print_stats(trajectories.phi2.velocity,      'phi2.velocity');
print_stats(trajectories.phi2.acceleration,  'phi2.acceleration');
print_stats(trajectories.phi2.jerk,          'phi2.jerk');

print_stats(trajectories.dh3.position,       'dh3.position');
print_stats(trajectories.dh3.velocity,       'dh3.velocity');
print_stats(trajectories.dh3.acceleration,   'dh3.acceleration');
print_stats(trajectories.dh3.jerk,           'dh3.jerk');

print_stats(trajectories.phi4.position,      'phi4.position');
print_stats(trajectories.phi4.velocity,      'phi4.velocity');
print_stats(trajectories.phi4.acceleration,  'phi4.acceleration');
print_stats(trajectories.phi4.jerk,          'phi4.jerk');

fprintf('\n=== Shape of position variables (before common time axis) ===\n');
disp(['phi0: ', mat2str(size(phi0_values))]);
disp(['phi1: ', mat2str(size(phi1_values))]);
disp(['phi2: ', mat2str(size(phi2_values))]);
disp(['dh3:  ', mat2str(size(dh3_values))]);
disp(['phi4: ', mat2str(size(phi4_values))]);

% Create a common time axis and interpolate/pad trajectory data.
% 1. Find the maximum final time across all trajectories
final_time = max([
    trajectories.phi0.time(end);
    trajectories.phi1.time(end);
    trajectories.phi2.time(end);
    trajectories.dh3.time(end);
    trajectories.phi4.time(end)
]);

% 2. Create a new common time vector with the maximum number of points
n_points = max([length(trajectories.phi0.time); length(trajectories.phi1.time);
                length(trajectories.phi2.time); length(trajectories.dh3.time);
                length(trajectories.phi4.time)]);
common_time = linspace(0, final_time, n_points);

% Helper function to pad data with zeros if its length is less than target_len
pad_with_zeros = @(data, target_len) [data, zeros(1, target_len - length(data))];

% Pad all trajectory data to the common length (n_points)
phi0_values    = pad_with_zeros(trajectories.phi0.position, n_points);
dphi0_values   = pad_with_zeros(trajectories.phi0.velocity, n_points);
ddphi0_values  = pad_with_zeros(trajectories.phi0.acceleration, n_points);

phi1_values    = pad_with_zeros(trajectories.phi1.position, n_points);
dphi1_values   = pad_with_zeros(trajectories.phi1.velocity, n_points);
ddphi1_values  = pad_with_zeros(trajectories.phi1.acceleration, n_points);

phi2_values    = pad_with_zeros(trajectories.phi2.position, n_points);
dphi2_values   = pad_with_zeros(trajectories.phi2.velocity, n_points);
ddphi2_values  = pad_with_zeros(trajectories.phi2.acceleration, n_points);

dh3_values     = pad_with_zeros(trajectories.dh3.position, n_points);
ddh3_values    = pad_with_zeros(trajectories.dh3.velocity, n_points);
dddh3_values   = pad_with_zeros(trajectories.dh3.acceleration, n_points); % Note: ddh3_values is velocity here, dddh3_values is acceleration

phi4_values    = pad_with_zeros(trajectories.phi4.position, n_points);
dphi4_values   = pad_with_zeros(trajectories.phi4.velocity, n_points);
ddphi4_values  = pad_with_zeros(trajectories.phi4.acceleration, n_points);

% Display trajectory statistics after padding/interpolation
fprintf('\n=========== STATISTICS AFTER INTERPOLATION/PADDING ===========\n');
print_stats(phi0_values,      'phi0.position (interp)');
print_stats(dphi0_values,     'phi0.velocity (interp)');
print_stats(ddphi0_values,    'phi0.acceleration (interp)');

print_stats(phi1_values,      'phi1.position (interp)');
print_stats(dphi1_values,     'phi1.velocity (interp)');
print_stats(ddphi1_values,    'phi1.acceleration (interp)');

print_stats(phi2_values,      'phi2.position (interp)');
print_stats(dphi2_values,     'phi2.velocity (interp)');
print_stats(ddphi2_values,    'phi2.acceleration (interp)');

print_stats(dh3_values,       'dh3.position (interp)');
print_stats(ddh3_values,      'dh3.velocity (interp)');
print_stats(dddh3_values,     'dh3.acceleration (interp)');

print_stats(phi4_values,      'phi4.position (interp)');
print_stats(dphi4_values,     'phi4.velocity (interp)');
print_stats(ddphi4_values,    'phi4.acceleration (interp)');

time = common_time; % Use the common time vector
n_points = length(time); % Update n_points to the new length

fprintf('\n=== Shape of position variables (after common time axis) ===\n');
disp(['phi0: ', mat2str(size(phi0_values))]);
disp(['phi1: ', mat2str(size(phi1_values))]);
disp(['phi2: ', mat2str(size(phi2_values))]);
disp(['dh3:  ', mat2str(size(dh3_values))]);
disp(['phi4: ', mat2str(size(phi4_values))]);

% Group symbolic variables
q_syms   = [phi0, phi1, phi2, dh3, phi4];
dq_syms  = [dphi0, dphi1, dphi2, ddh3, dphi4];
ddq_syms = [ddphi0, ddphi1, ddphi2, dddh3, ddphi4];

% Preallocate torque array again with updated n_points
tau_eval = zeros(5, n_points);

% === Evaluate torques/forces using the interpolated trajectory data ===
for i = 1:n_points
    % Numerical values for position, velocity, acceleration at current time step
    q_num   = [phi0_values(i), phi1_values(i), phi2_values(i), dh3_values(i), phi4_values(i)];
    dq_num  = [dphi0_values(i), dphi1_values(i), dphi2_values(i), ddh3_values(i), dphi4_values(i)];
    ddq_num = [ddphi0_values(i), ddphi1_values(i), ddphi2_values(i), dddh3_values(i), ddphi4_values(i)];

    try
        % Substitute numerical values into the symbolic M, C, G matrices
        M_i = double(subs(M_clean, [q_syms, dq_syms, ddq_syms], [q_num, dq_num, ddq_num]));
        C_i = double(subs(C_clean, [q_syms, dq_syms], [q_num, dq_num]));
        G_i = double(subs(G_clean, q_syms, q_num));

        % Calculate torque/force using the dynamic equation: tau = M*ddq + C*dq + G
        tau_eval(:, i) = M_i * ddq_num' + C_i * dq_num' + G_i;

        % Display progress and calculated values every 100 steps or at the end
        if mod(i, 100) == 1 || i == n_points
            fprintf("\ntau_eval(:, i) = M_i * ddq_num' + C_i * dq_num' + G_i'\n");
            fprintf('\n=== Step %d / %d ===\n', i, n_points);
            fprintf('Time = %.3f s\n', time(i));
            fprintf('q     = [%8.4f, %8.4f, %8.4f, %8.4f, %8.4f]\n', q_num);
            fprintf('dq    = [%8.4f, %8.4f, %8.4f, %8.4f, %8.4f]\n', dq_num);
            fprintf('ddq   = [%8.4f, %8.4f, %8.4f, %8.4f, %8.4f]\n', ddq_num);
            fprintf('tau   = [%8.4f, %8.4f, %8.4f, %8.4f, %8.4f]\n', tau_eval(:,i));

            fprintf("Tau_i:\n");
            disp(tau_eval(:, i))
            fprintf("M_i * ddq_num':\n");
            disp(M_i * ddq_num')
            fprintf("M_i:\n");
            disp(M_i)
            fprintf("C_i * dq_num'\n");
            disp(C_i * dq_num')
            fprintf("C_i:\n");
            disp(C_i)
            fprintf("G_i:\n");
            disp(G_i)
            fprintf('\n=== Numerical torque/force values per time instant ===\n');
            fprintf('%8s | %10s | %10s | %10s | %10s | %10s\n', 'Time', 'Tau_0', 'Tau_1', 'Tau_2', 'Tau_dh3', 'Tau_4');
            fprintf('%8.3f | %10.3f | %10.3f | %10.3f | %10.3f | %10.3f\n', ...
                time(i), tau_eval(1,i), tau_eval(2,i), tau_eval(3,i), tau_eval(4,i), tau_eval(5,i));
        end

    catch ME
        fprintf("Error at step %d: %s\n", i, ME.message);
        disp("Values used:");
        disp([q_num, dq_num, ddq_num]);
        break; % Exit loop on error
    end
end

fprintf('\nGenerating Torque vs Time and Position vs Time plots (approx.)...\n');

% === Symbolic Labels and Plot Configuration ===
joint_labels   = {'$\tau_0$ [Nm]', '$\tau_1$ [Nm]', '$\tau_2$ [Nm]', '$\tau_{d_{h3}}$ [N]', '$\tau_4$ [Nm]'};
symbol_names   = {'$\varphi_0$', '$\varphi_1$', '$\varphi_2$', '$d_{h3}$', '$\varphi_4$'};
units_tau      = {'Nm', 'Nm', 'Nm', 'N', 'Nm'};
positions      = {phi0_values, phi1_values, phi2_values, dh3_values, phi4_values};
file_names     = {'Aprox_tau_phi0.png', 'Aprox_tau_phi1.png', 'Aprox_tau_phi2.png', 'Aprox_tau_dh3.png', 'Aprox_tau_phi4.png'};
is_angle       = [true, true, true, false, true];  % Identify which joints are angular (for degrees conversion)

% === Matching Colors for Y-axes ===
color_torque   = [0 0 0];       % Black
color_position = [0 0 0.6];     % Dark Blue

% Generate plots for each joint
for i = 1:5
    figure('Name', ['Torque and Position - Joint ', symbol_names{i}], ...
           'Color', 'w', 'Position', [100 100 900 600]);

    % === Left Y-axis: Torque or Force ===
    yyaxis left;
    ax = gca;
    ax.YColor = color_torque; % Set color for left Y-axis
    plot(time, tau_eval(i,:), '-', 'LineWidth', 2, 'Color', color_torque);
    ylabel(joint_labels{i}, 'FontSize', 12, 'Interpreter', 'latex');
    ylim padded; % Adjust y-axis limits automatically
    grid on;

    % === Right Y-axis: Angular or Linear Position ===
    yyaxis right;
    ax.YColor = color_position; % Set color for right Y-axis
    if is_angle(i)
        pos = rad2deg(positions{i}); % Convert radians to degrees for angular joints
        plot(time, pos, '--', 'LineWidth', 1.5, 'Color', color_position);
        ylabel(sprintf('%s [degree]', symbol_names{i}), 'Interpreter', 'latex', 'FontSize', 12);
    else
        plot(time, positions{i}, '--', 'LineWidth', 1.5, 'Color', color_position);
        ylabel(sprintf('%s [m]', symbol_names{i}), 'Interpreter', 'latex', 'FontSize', 12);
    end
    ylim padded; % Adjust y-axis limits automatically

    % === X-axis: Time ===
    xlabel('Time [s]', 'FontSize', 12, 'Interpreter', 'latex');

    % === Title and Legend ===
    title(sprintf('Approx. Torque/Force and Position for %s', symbol_names{i}), ...
          'Interpreter', 'latex', 'FontSize', 14);
    if is_angle(i)
        legend({'Torque [Nm]', 'Position [°]'}, 'Location', 'best', 'Interpreter', 'latex');
    else
        legend({'Force [N]', 'Position [m]'}, 'Location', 'best', 'Interpreter', 'latex');
    end
    set(gca, 'FontSize', 12); % Set font size for current axes
    box on; % Display box around the plot

    % Save plot as JPG with high resolution

    jpg_name = strrep(file_names{i}, '.png', '.jpg');  
    exportgraphics(gcf, jpg_name, 'Resolution', 300); 
end

% Helper function to print signal statistics
function print_stats(signal, name)
    fprintf('\n--- Statistics for %s ---\n', name);
    fprintf('  Minimum:    %.6f\n', min(signal));
    fprintf('  Maximum:    %.6f\n', max(signal));
    fprintf('  Average:  %.6f\n', mean(signal));
    fprintf('  Std. Dev: %.6f\n', std(signal));
end

% Calculation and Visualization of Total Robot Mass

% Sum all individual link masses
total_mass = sum(masses);

fprintf('\n=== Total Robot Mass ===\n');
fprintf('Total mass: %.3f kg\n', total_mass);
fprintf('Breakdown by links:\n');
for i = 1:n
    fprintf('  Link %d: %.3f kg\n', i, masses(i));
end
