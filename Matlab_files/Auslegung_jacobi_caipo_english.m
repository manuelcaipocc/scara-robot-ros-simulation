clc; clear;
warning('off','all');
fprintf('                               ╔══════════════════════════════════════════════════╗\n');
fprintf('                               ║          Jacobi                                  ║\n');
fprintf('                               ╠══════════════════════════════════════════════════╣\n');
fprintf('                               ╚══════════════════════════════════════════════════╝\n');
% ----------- Symbolic parameters -----------
syms Hs H0 L0 L1 D1 L2 DA2 L3 H3 L5 L4 KH3
syms phi0 phi1 phi2 phi3 phi4 phi5 phi6 % all in radians now
syms dl1 dl2 dl3 dl4 dh3 
syms d2 dh d6 real
syms nx ny nz ux uy uz ax ay az px py pz real
syms a1 a2 a3 real
pi = sym(pi);
% Define target variables
target_vars = [phi0, phi1, phi2, dh3, phi4];
target_var_names = {'phi0', 'phi1', 'phi2', 'dh3', 'phi4'};
alpha = [  0,  pi/2,  -pi/2,   0,    pi,     0,    0,    0];
L     = [  0,    0,     L1,    0,     L2,     0,    0,  0];
D     = [dl1, dl2,   0,   D1,     0,  dl3, dh3,dl4];
phi   = [0, phi0, phi1, 0, phi2, 0, 0,phi4];  % in radians
% Number of joints
n = length(alpha);
% ----------- Homogeneous transformation matrix function -----------
getA = @(alpha, L, D, phi) ...
    [ cos(phi), -cos(alpha)*sin(phi),  sin(alpha)*sin(phi), L*cos(phi);
      sin(phi),  cos(alpha)*cos(phi), -sin(alpha)*cos(phi), L*sin(phi);
           0,         sin(alpha),           cos(alpha),         D;
           0,              0,                  0,              1];
% ----------- Step-by-step transformation matrices -----------
A = sym(zeros(4,4,n)); % Symbolic 3D array
for i = 1:n
    A_i = getA(alpha(i), L(i), D(i), phi(i));
    A_i_simple = simplify(A_i, 'Steps', 100);
    A_i_clean = vpa(A_i_simple, 6);  % Round to 6 decimal places
    A(:,:,i) = A_i_clean;
end
% ----------- Final total transformation -----------
T_total = eye(4);
for i = 1:n
    T_total = T_total * A(:,:,i);
end
% ----------- Accumulated z_i and o_i calculation -----------
z_list = sym(zeros(3, n));
o_list = sym(zeros(3, n));
T = eye(4);
for i = 1:n
    T = T * A(:,:,i);
    fprintf('\n>>> Accumulated Transformation T_%d (...*A%d):\n', i, i);
    disp(T);  % <<--- here you see the accumulated transformed matrix
    z_list(:,i) = T(1:3, 3);        % third rotation column (z-axis)
    o_list(:,i) = T(1:3, 4);        % position
    
    % Display z_i and o_i
    fprintf('\n=== Transformation Matrix A%d ===\n', i);
    disp(A(:,:,i));
    fprintf('z%d (z-axis after ...*A%d):\n', i, i);
    disp(z_list(:,i));
    fprintf('O%d (position after ...*A%d):\n', i, i);
    disp(o_list(:,i));
end
% End-effector final position
o_end = T_total(1:3, 4);
fprintf('\n=== Final End-effector Position: O_end ===\n');
disp(o_end);
% Indices of joints with mobile variables
indices_vars = [2, 3, 4, 7, 8];
fprintf('\n=== Variables considered for the Jacobian ===\n');
disp(target_var_names);
% Initialize Jacobian
Jg = sym(zeros(6, length(target_vars)));  % 6 rows (3 linear + 3 angular)
% Add base frame z and p (needed for first joint calculation)
z_prev = [0; 0; 1];
o_prev = [0; 0; 0];
fprintf("\n Z_0:");
disp(z_prev);
fprintf("\n O_0:");
disp(o_prev)
for idx = 1:length(target_vars)
    var_name = target_var_names{idx};
    
    if strcmp(var_name, 'dh3')
        i = 7;
        z = z_list(:, i-1);  % z6
        Jg(1:3, idx) = z;
        Jg(4:6, idx) = [0; 0; 0];
        fprintf('\n=== Calculation for variable %s (prismatic joint %d) ===\n', var_name, i);
        fprintf('z%d:\n', i-1);
        disp(z);
        fprintf('Column %d of the Jacobian (prismatic):\n', idx);
        disp(Jg(:, idx));
    else
        i = find(phi == target_vars(idx));
        if isempty(i)
            continue; 
        end
        
        if i == 1
            z = z_prev;
            o = o_prev;
        else
            z = z_list(:, i-1);
            o = o_list(:, i-1);
        end
        
        fprintf('\n=== Calculation for variable %s (rotational joint %d) ===\n', var_name, i);
        fprintf('z%d:\n', i-1);
        disp(z);
        
        fprintf('O%d:\n', i-1);
        disp(o);
        
        fprintf('O_end - O%d:\n', i-1);
        disp(o_end - o);
        
        Jg(1:3, idx) = cross(z, (o_end - o));
        Jg(4:6, idx) = z;
        
        fprintf('cross(z%d, (O_end - O%d)):\n', i-1, i-1);
        disp(cross(z, (o_end - o)));
        
        fprintf('Column %d of the Jacobian (rotational):\n', idx);
        disp(Jg(:, idx));
    end
end
% Simplify the Jacobian
Jg = simplify(Jg);
fprintf('\n=== Final Geometric Jacobian ===\n');
disp(Jg);

% =============================================
% NUMERICAL EVALUATION
% =============================================
% Numerical values in radians and meters
geo_subs = {
    Hs, 200/1000;
    H0, 100/1000;
    L0, 800/1000;
    L1, 400/1000;
    L2, 500/1000;
    L3, 630/1000;
    L4, 600/1000;
    L5, 50/1000;
    D1, 110/1000;
    DA2, 100/1000;
    KH3, 80/1000;
    dl1, 100/1000;    % Hs_val - H0_val = 200 - 100 = 100
    dl2, 900/1000;    % L0_val + H0_val = 800 + 100 = 900
    dl3, 760/1000;    % (DA2_val/2) + L3_val + KH3_val = 50 + 630 + 80 = 760
    dl4, 350/1000     % (L4_val/2) + L5_val = 300 + 50 = 350
};
% Joint configurations to evaluate (angles in degrees)
configurations = [
    90,0,0,-50/1000,0;
    90,90,0,-50/1000,0;
    90, 0.0000, -126.87, -50/1000, -66.87
];
% Convert angle columns (1, 2, 3, 5) from degrees to radians
configurations_rad = configurations;
configurations_rad(:, [1, 2, 3, 5]) = deg2rad(configurations(:, [1, 2, 3, 5]));
% Extract configuration values for each variable
phi0_vals = configurations_rad(:, 1);
phi1_vals = configurations_rad(:, 2);
phi2_vals = configurations_rad(:, 3);
dh3_vals = configurations(:, 4);  % dh3 is in meters, no conversion needed
phi4_vals = configurations_rad(:, 5);
% Create an array to store numerical Jacobians
num_Jg = zeros(6, 5, size(configurations, 1));
for k = 1:size(configurations, 1)
    % Substitute geometric values (geo_subs)
    Jg_subs = subs(Jg, geo_subs(:, 1), geo_subs(:, 2));
    
    % Substitute joint values for configuration k
    Jg_num = subs(Jg_subs, ...
        [phi0, phi1, phi2, dh3, phi4], ...
        [phi0_vals(k), phi1_vals(k), phi2_vals(k), dh3_vals(k), phi4_vals(k)]);
    
    % Convert to numeric
    num_Jg(:, :, k) = double(Jg_num);
end

% =============================================
% ENHANCED SINGULARITY ANALYSIS
% =============================================
for k = 1:size(configurations, 1)
    fprintf('\n================ Configuration %d ============================\n', k);
    fprintf('Angles: phi0=%.2f°, phi1=%.2f°, phi2=%.2f°, dh3=%.4f m, phi4=%.2f°\n', ...
            configurations(k,1), configurations(k,2), configurations(k,3), ...
            configurations(k,4), configurations(k,5));
    J = num_Jg(:,:,k);
    fprintf("The Jacobian is:\n");
    disp(J);
    if k==3
        % My Jacobian has linear velocity first, then angular
        F_TCP = [30; 0; 100; -50; 25; 75];  % Original (Mx, My, Mz, Fx, Fy, Fz)
        F_TCP_ordered = [F_TCP(4:6); F_TCP(1:3)];  % Now: Fx, Fy, Fz, Mx, My, Mz
        fprintf('tau is:\n');
        tau = J' * F_TCP_ordered;
        disp(tau);
        % The first, second, third, and fifth values will be moments [Nm]
        % The fourth value is the required linear force on the prismatic axis 3 [N]
        fprintf('\nTau (joint efforts):\n');
        fprintf('Tau(1): %.2f Nm  (Joint 1 - Moment)\n', tau(1));
        fprintf('Tau(2): %.2f Nm  (Joint 2 - Moment)\n', tau(2));
        fprintf('Tau(3): %.2f Nm  (Joint 3 - Moment)\n', tau(3));
        fprintf('Tau(4): %.2f  N   (Joint 4 - Linear force prismatic axis)\n', tau(4));
        fprintf('Tau(5): %.2f Nm  (Joint 5 - Moment)\n', tau(5));
    end
    
    % -----------------------------------------------------------
    % 1. ENHANCED SVD ANALYSIS (FOR RECTANGULAR MATRICES)
    % -----------------------------------------------------------
    [U,S,V] = svd(J);
    sv = diag(S);
    cond_number = cond(J);
    
    fprintf('\n1. SVD Analysis (Rectangular Matrix):\n');
    fprintf('   - Singular values: '); fprintf('%.4f  ', sv); fprintf('\n');
    fprintf('   - Condition number: %.2e\n', cond_number);
    
    % Enhanced singularity criteria
    sv_threshold = 1e-6; % Threshold for "zero" singular values
    if any(sv < sv_threshold)
        fprintf('   → SINGULARITY DETECTED: ');
        num_zero_sv = sum(sv < sv_threshold);
        fprintf('%d singular value(s) below threshold (%.1e)\n', num_zero_sv, sv_threshold);
        
        % Show problematic rows/columns
        [min_sv, idx] = min(sv);
        problematic_dir = U(:,idx);
        fprintf('   - Most constrained direction: [');
        fprintf('%.2f ', problematic_dir);
        fprintf(']\n');
    else
        fprintf('   → No singularity detected via SVD\n');
    end
    
    % -----------------------------------------------------------
    % 2. ZERO-ROW HANDLING & SQUARE SUBMATRICES
    % -----------------------------------------------------------
    fprintf('\n2. Zero-Row Handling & Square Submatrices:\n');
    
    % Automatically remove zero rows
    zero_rows = all(abs(J) < 1e-10, 2); % Numerical tolerance
    J_clean = J(~zero_rows, :);
    
    if any(zero_rows)
        fprintf('   - Removed %d zero row(s): ', sum(zero_rows));
        fprintf('rows %s\n', mat2str(find(zero_rows)'));
    else
        fprintf('   - No zero rows found\n');
    end
    
    % Clean matrix analysis
    if size(J_clean, 1) == size(J_clean, 2)
        % Square matrix after cleaning
        det_clean = det(J_clean);
        fprintf('   - Clean matrix is square (size %dx%d)\n', size(J_clean,1), size(J_clean,2));
        fprintf('   - Determinant: %.2e\n', det_clean);
        
        if abs(det_clean) < sv_threshold
            fprintf('   → SINGULAR SUBMATRIX: Near-zero determinant\n');
        else
            fprintf('   → Full-rank square submatrix\n');
        end
    else
        fprintf('   - Clean matrix is still rectangular (%dx%d)\n', size(J_clean,1), size(J_clean,2));
        
        % Analysis of all possible square submatrices
        possible_rows = nchoosek(1:size(J,1), size(J,2));
        subdets = zeros(size(possible_rows,1),1);
        
        for i = 1:size(possible_rows,1)
            subJ = J(possible_rows(i,:),:);
            subdets(i) = det(subJ);
        end
        
        fprintf('   - Found %d square submatrices\n', length(subdets));
        fprintf('   - Min determinant: %.2e, Max: %.2e\n', min(abs(subdets)), max(abs(subdets)));
        
        if any(abs(subdets) < sv_threshold)
            fprintf('   → SINGULAR SUBMATRICES: %d/%d have near-zero det\n',...
                   sum(abs(subdets) < sv_threshold), length(subdets));
        end
    end
    
    % -----------------------------------------------------------
    % 3. UNIFIED CONCLUSION
    % -----------------------------------------------------------
    fprintf('\n3. Final Singularity Conclusion:\n');
    
    % Combined criteria
    is_singular = any(sv < sv_threshold) || ...           % SVD
                 (exist('subdets','var') && any(abs(subdets) < sv_threshold)) || ... % Submatrices
                 cond_number > 1e3;                      
    
    if is_singular
        fprintf('   → ROBOT IS IN SINGULAR CONFIGURATION\n');
        fprintf('   Reasons:\n');
        
        if any(sv < sv_threshold)
            fprintf('     - %d singular value(s) below threshold\n', sum(sv < sv_threshold));
        end
        
        if exist('subdets','var') && any(abs(subdets) < sv_threshold)
            fprintf('     - %d singular submatrix(ces) found\n', sum(abs(subdets) < sv_threshold));
        end
        
        if cond_number > 1e3
            fprintf('     - High condition number (%.2e > 1e3)\n', cond_number);
        end
    else
        fprintf('   → Configuration is NOT singular\n');
    end
    
    % -----------------------------------------------------------
    % 4. ADDITIONAL VISUALIZATION (OPTIONAL)
    % -----------------------------------------------------------
    fprintf('\n4. Additional Info:\n');
    fprintf('   - Matrix rank: %d (out of %d)\n', rank(J), min(size(J)));
    fprintf('   - Clean matrix rank: %d\n', rank(J_clean));
end
% ----------- Substitute all geometric parameters with their numerical values -----------
% (This removes D1, L1, L2, dl3, dl4, etc., leaving only phi0, phi1, phi2, dh3, phi4)
Jg_sym = subs(Jg, geo_subs(:, 1), geo_subs(:, 2));
% Simplify the Jacobian (optional, but recommended for symbolic analysis)
Jg_sym = simplify(Jg_sym);
% Display the simplified Jacobian
fprintf('\n=== Geometric Jacobian (joint variables only) ===\n');
disp(Jg_sym);
% ----------- Symbolic Singularity Analysis -----------
% Option 1: Calculate the determinant of J^T * J (for rectangular matrices)
JtJ = Jg_sym.' * Jg_sym;
det_JtJ = det(JtJ);
det_JtJ = simplify(det_JtJ);
fprintf('\n=== Determinant of J^T * J ===\n');
disp(det_JtJ);
% Option 2: Analysis of square submatrices (if J is 6x5, we take combinations of 5 rows)
fprintf('\n=== Analysis of 5x5 Square Submatrices ===\n');
rows = 1:6;
combinations = nchoosek(rows, 5);
for i = 1:size(combinations, 1)
    J_sub = Jg_sym(combinations(i, :), :);
    det_sub = det(J_sub.' * J_sub);  % J^T * J for submatrix
    det_sub = simplify(det_sub);
    
    fprintf('\nSubmatrix with rows %s:\n', mat2str(combinations(i, :)));
    fprintf('det(J^T * J) = \n');
    disp(det_sub);
    
end
% ----------- Check ranks -----------
fprintf('\n=== Symbolic Rank of the Jacobian ===\n');
rank_J = rank(Jg_sym);
fprintf('Symbolic rank: %d\n', rank_J);
if rank_J < min(size(Jg_sym))
    fprintf('→ Jacobian is singular (rank < %d)\n', min(size(Jg_sym)));
else
    fprintf('→ Jacobian has full rank (rank = %d)\n', min(size(Jg_sym)));
end