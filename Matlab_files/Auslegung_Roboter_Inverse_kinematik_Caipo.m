clc; clear;
warning('off','all');
selected_solutions_file = 'selected_solutions.mat';
if isfile(selected_solutions_file)
    load(selected_solutions_file, 'selected_solutions');
    skip_solution_selection = true;
    fprintf('Previous solutions loaded from %s\n', selected_solutions_file);
else
    selected_solutions = struct();
    skip_solution_selection = false;
end

fprintf('                               ╔══════════════════════════════════════════════════╗\n');
fprintf('                               ║           INVERSE KINEMATICS TRANSFORMATION      ║\n');
fprintf('                               ╠══════════════════════════════════════════════════╣\n');
fprintf('                               ╚══════════════════════════════════════════════════╝\n');

global eqs_one_var eqs_two_vars eqs_three_vars;
eqs_one_var = {};   % Cell array for 1-variable equations
eqs_two_vars = {};  % Cell array for 2-variable equations
eqs_three_vars = {};
% ----------- Symbolic parameters -----------
syms Hs H0 L0 L1 D1 L2 DA2 L3 H3 L5 L4
syms phi0 phi1 phi2 phi4  % all in radians now
syms dl1 dl2 dl3 dl4 dh3
syms nx ny nz ux uy uz ax ay az px py pz real
pi = sym(pi);
% Define target variables
% target_vars = [phi0, phi1, phi2, phi4, H3];
% target_var_names = {'phi0', 'phi1', 'phi2', 'phi4', 'H3'};

% Define target variables
target_vars = [phi0, phi1, phi2, phi4, dh3];
target_var_names = {'phi0', 'phi1', 'phi2', 'phi4', 'dh3'};

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
% ----------- Inverse transformations -----------
T = [nx ux ax px;
     ny uy ay py;
     nz uz az pz;
     0   0   0  1];
INV = sym(zeros(4,4,n)); % Symbolic 3D array
for i =1:n
    INV(:,:,i)= simplify(inv(A(:,:,i)));
end


atemp_pairs = {
    {T, T_total},  % Comparación completa

    % Paso 1: inv(A1) * T ≈ A2*A3*...*A8
    {inv(A(:,:,1)) * T, A(:,:,2)*A(:,:,3)*A(:,:,4)*A(:,:,5)*A(:,:,6)*A(:,:,7)*A(:,:,8)},
    % Paso 2: inv(A2)*inv(A1)*T ≈ A3*...*A8
    {inv(A(:,:,2)) * inv(A(:,:,1)) * T, A(:,:,3)*A(:,:,4)*A(:,:,5)*A(:,:,6)*A(:,:,7)*A(:,:,8)},
    % Paso 3: inv(A3)*...*A1*T ≈ A4*...*A8
    {inv(A(:,:,3)) * inv(A(:,:,2)) * inv(A(:,:,1)) * T, A(:,:,4)*A(:,:,5)*A(:,:,6)*A(:,:,7)*A(:,:,8)},
    % Paso 4:
    {inv(A(:,:,4)) * inv(A(:,:,3)) * inv(A(:,:,2)) * inv(A(:,:,1)) * T, A(:,:,5)*A(:,:,6)*A(:,:,7)*A(:,:,8)},
    % Paso 5:
    {inv(A(:,:,5)) * inv(A(:,:,4)) * inv(A(:,:,3)) * inv(A(:,:,2)) * inv(A(:,:,1)) * T, A(:,:,6)*A(:,:,7)*A(:,:,8)},
    % Paso 6:
    {inv(A(:,:,6)) * inv(A(:,:,5)) * inv(A(:,:,4)) * inv(A(:,:,3)) * inv(A(:,:,2)) * inv(A(:,:,1)) * T, A(:,:,7)*A(:,:,8)},
    % Paso 7:
    {inv(A(:,:,7)) * inv(A(:,:,6)) * inv(A(:,:,5)) * inv(A(:,:,4)) * inv(A(:,:,3)) * inv(A(:,:,2)) * inv(A(:,:,1)) * T, A(:,:,8)}
};



% ----------- Improved analysis function -----------
function analyze_equation_pair(atemp_left, atemp_right, target_vars, target_var_names)
    global eqs_one_var eqs_two_vars eqs_three_vars;
    
    for i = 1:3
        for j = 1:4
            eq = simplify(atemp_left(i,j) - atemp_right(i,j));
            eq = vpa(eq, 6);
            
            % Identify target variables in equation
            all_vars = symvar(eq);
            target_vars_present = intersect(all_vars, target_vars);
            
            % Create equation info structure
            eq_info = struct();
            eq_info.equation = eq == 0;
            eq_info.position = [i, j];
            eq_info.target_vars = target_vars_present;
            
            % Classify and store equations
            if length(target_vars_present) == 1
                var_name = target_var_names(target_vars == target_vars_present(1));
                eq_info.var_name = var_name{1};
                eqs_one_var{end+1} = eq_info;
                
            elseif length(target_vars_present) == 2
                var1_name = target_var_names(target_vars == target_vars_present(1));
                var2_name = target_var_names(target_vars == target_vars_present(2));
                eq_info.var_names = {var1_name{1}, var2_name{1}};
                eqs_two_vars{end+1} = eq_info;
            elseif length(target_vars_present) == 3
                var_names = arrayfun(@(v) target_var_names{target_vars == v}, target_vars_present, 'UniformOutput', false);
                eq_info.var_names = var_names;
                eqs_three_vars{end+1} = eq_info;
            end
        end
    end
end
% ----------- Execute analysis for all pairs -----------
for pair_idx = 1:length(atemp_pairs)
    current_pair = atemp_pairs{pair_idx};
    analyze_equation_pair(current_pair{1}, current_pair{2}, target_vars, target_var_names);
end
% ----------- Display equations in organized matrices -----------
fprintf('\n\n=== EQUATIONS WITH ONE TARGET VARIABLE ===\n');
for k = 1:length(eqs_one_var)
    eq_info = eqs_one_var{k};
    fprintf('\nEquation %d (position %d,%d) for %s:\n', k, eq_info.position(1), eq_info.position(2), eq_info.var_name);
    disp(eq_info.equation);
end
fprintf('\n\n=== EQUATIONS WITH TWO TARGET VARIABLES ===\n');
for k = 1:length(eqs_two_vars)
    eq_info = eqs_two_vars{k};
    fprintf('\nEquation %d (position %d,%d) variables: %s , %s:\n', k, eq_info.position(1), eq_info.position(2), ...
           eq_info.var_names{1}, eq_info.var_names{2});
    disp(eq_info.equation);
end
fprintf('\n\n=== EQUATIONS WITH THREE TARGET VARIABLES ===\n');
for k = 1:length(eqs_three_vars)
    eq_info = eqs_three_vars{k};
    fprintf('\nEquation %d (position %d,%d) variables:  %s, %s ,%s :\n', k, eq_info.position(1), eq_info.position(2), ...
           eq_info.var_names{1}, eq_info.var_names{2}, eq_info.var_names{3});
    disp(eq_info.equation);
end

% ----------- Solve equations systematically -----------
fprintf('\n\n........................................................ SOLVING EQUATIONS ........................................................\n');
solutions = struct();
solutions_tan = struct(); % New struct for tangent-based solutions
fprintf('============================ Equations with 1 Target Variable=================================================\n');
fprintf('========================================================================================\n');
% First solve single-variable equations
selected_solutions = struct(); % Will store the selected solutions
for k = 1:length(eqs_one_var)
    fprintf('===== [Equation %d/%d with 1 variable] =====\n', k, length(eqs_one_var));
    fprintf('If no real solutions are found, please enter 0 and proceed to the next equation\n');
    eq_info = eqs_one_var{k};
    eq = eq_info.equation;
    var = eq_info.target_vars;
    var_name = eq_info.var_name;
    
    fprintf('\nSolving equation for %s:\n', var_name);
    disp(eq);
    
    % Standard solution
    sol = [];
    sol_struct = [];
    try
        sol_struct = solve(eq, var, 'ReturnConditions', true, 'PrincipalValue', true);
        if ~isempty(sol_struct)
            sol = sol_struct.(char(var));
        end
    catch ME
        fprintf('Error in standard solution for %s: %s\n', var_name, ME.message);
    end
    
    % Tangent substitution solution
    sol_tan = [];
    if has(eq, 'sin') || has(eq, 'cos')
        try
            syms u real;
            eq_tan = eq;
            if has(eq, 'sin')
                sin_expr = sin(var);
                eq_tan = subs(eq_tan, sin_expr, (2*u)/(1+u^2));
            end
            if has(eq, 'cos')
                cos_expr = cos(var);
                eq_tan = subs(eq_tan, cos_expr, (1-u^2)/(1+u^2));
            end
            sol_u = solve(eq_tan, u);
            if ~isempty(sol_u)
                sol_tan = atan(sol_u);
                sol_tan = simplify(sol_tan);
            end
        catch ME
            fprintf('Error in tangent substitution solution for %s: %s\n', var_name, ME.message);
        end
    end
    
    % Display all found solutions
    fprintf('=== FOUND SOLUTIONS FOR %s ===\n', var_name);
    
    if ~isempty(sol)
        fprintf('Standard solutions:\n');
        for i = 1:length(sol)
            fprintf('[%d] ', i);
            disp(vpa(sol(i), 6));
        end
    end
    
    if ~isempty(sol_tan)
        fprintf('Solutions by tangent substitution:\n');
        for i = 1:length(sol_tan)
            fprintf('[%d] ', length(sol)+i);
            disp(vpa(sol_tan(i), 6));
        end
    end
    
    if isempty(sol) && isempty(sol_tan)
        fprintf('No solutions found for %s\n', var_name);
        continue;
    end
    
    % Allow user to select solutions
    if isfield(selected_solutions, var_name) && ~isempty(selected_solutions.(var_name))
        fprintf('Previously selected solutions for %s:\n', var_name);
        disp(vpa(selected_solutions.(var_name), 6));
        continue;  % Do nothing further if a solution is already saved
    else
        % Allow user to select solutions
        fprintf('Select the solutions you consider reasonable, preferably choose only one, check is not imaginary  (e.g., [1 3] or 0 for none):\n');
        selected_indices = input('Enter solution indices: ');
        
        all_solutions = [sol; sol_tan];
        if ~isequal(selected_indices, 0)
            selected_solutions.(var_name) = all_solutions(selected_indices);
            fprintf('Selected solutions for %s:\n', var_name);
            disp(vpa(selected_solutions.(var_name), 6));
        else
            fprintf('No solutions selected for %s\n', var_name);
        end
    end
    % Store the selected solutions
    all_solutions = [sol; sol_tan];
    
    if ~isequal(selected_indices, 0)
        for idx = selected_indices
            if idx <= length(all_solutions)
                if ~isfield(selected_solutions, var_name) || isempty(selected_solutions.(var_name))
                    selected_solutions.(var_name) = all_solutions(idx);
                else
                    selected_solutions.(var_name) = [selected_solutions.(var_name); all_solutions(idx)];
                end
            end
        end
    end
    
    fprintf('Selected solutions for %s:\n', var_name);
    if isfield(selected_solutions, var_name) && ~isempty(selected_solutions.(var_name))
        disp(vpa(selected_solutions.(var_name), 6));
    else
        fprintf('No solutions selected for %s\n', var_name);
    end

end
if ~skip_solution_selection
    save(selected_solutions_file, 'selected_solutions');
    % ===== Symbolic replacement k=0, l=0, m=0 in the selected solutions =====
    % syms k l m n real;  % Ensure these symbols exist
    %
    % fprintf('\nEvaluating k=0, l=0, m=0 in all selected solutions...\n');
    % fields = fieldnames(selected_solutions);
    % for i = 1:length(fields)
    %     name = fields{i};
    %     sols = selected_solutions.(name);
    %     if ~isempty(sols)
    %         sols = subs(sols, [k, l, m,n], [0, 0, 0,0]);
    %         selected_solutions.(name) = simplify(sols);  % Or use vpa(sols,6) if you prefer numbers
    %     end
    % end
    fprintf('Selected solutions saved in %s\n', selected_solutions_file);
end
fprintf('============================Taking static values from the first iteration================================================\n');

selected_solutions.phi0 = acos((-L2^2 + L1^2 + px^2 +py^2) / (2 * ((px^2+py^2)^(1/2)) * L1))+atan2(py, px);
selected_solutions.phi1 = acos(az) + 3.14159;
selected_solutions.phi2 = (acos((-px^2 - py^2 +L1^2 + L2^2) / (2 * L1 * L2)))+3.14159;
disp(selected_solutions);

% selected_solutions.dh3 = -(1.0*(1.0*dl1 + 1.0*dl2 - 1.0*pz - 1.0*D1*cos(0+ acos(az)) - 1.0*L1*sin(0+ acos(az)) + dl3*cos(0 + acos(az)) + dl4*cos(0+ acos(az)) - (0.5*sin(0+ acos(az))*(- 1.0*L1^2 - 1.0*L2^2 + px^2 + py^2 + pz^2))/L1))/cos(0+ acos(az))
disp(selected_solutions);


disp(selected_solutions);
fprintf('============================ Equations with 2 Target Variable================================================\n');
fprintf('========================================================================================\n');
for k = 1:length(eqs_two_vars)
    fprintf('===== [Equation %d/%d with 2 variables] =====\n', k, length(eqs_two_vars))
    fprintf('If no real solutions are found, please enter 0 and proceed to the next equation\n');
    eq_info = eqs_two_vars{k};
    eq = eq_info.equation;
    vars = eq_info.target_vars;
    var1_name = eq_info.var_names{1};
    var2_name = eq_info.var_names{2};
    if isfield(selected_solutions, var1_name) && ~isempty(selected_solutions.(var1_name)) && ...
       isfield(selected_solutions, var2_name) && ~isempty(selected_solutions.(var2_name))
        fprintf('Skipping equation between %s and %s: solutions already exist for both variables.\n', var1_name, var2_name);
        continue;
    end
    fprintf('Solving equation between %s and %s:\n', var1_name, var2_name);
    disp(eq);
    has_var1 = isfield(selected_solutions, var1_name) && ~isempty(selected_solutions.(var1_name));
    has_var2 = isfield(selected_solutions, var2_name) && ~isempty(selected_solutions.(var2_name));
    if has_var1 && ~has_var2
        known_var = vars(1);
        unknown_var = vars(2);
        known_name = var1_name;
        unknown_name = var2_name;
    elseif has_var2 && ~has_var1
        known_var = vars(2);
        unknown_var = vars(1);
        known_name = var2_name;
        unknown_name = var1_name;
    else
        fprintf('No solutions available yet for either of the two variables.\n');
        continue;
    end
    sols = selected_solutions.(known_name);
    for idx = 1:length(sols)
        current_val = sols(idx);
        % CHECK IF SOLUTIONS ALREADY EXIST
        if isfield(selected_solutions, unknown_name) &&~isempty(selected_solutions.(unknown_name))
            fprintf('\nSolutions already exist for %s, skipping combination %d/%d\n', ...
                unknown_name, idx, length(sols));
            continue;
        end
        fprintf('\n--- Step %d of %d for %s ---\n', ...
        idx, length(sols), known_name);
        fprintf('   Value used for %s: %s\n', known_name, char(vpa(current_val, 6)));
        eq_sub = subs(eq, known_var, current_val);
        % Standard solution
        fprintf('Standard solutions:\n');
        try
            sol_struct = solve(eq_sub, unknown_var, 'ReturnConditions', true);
            sol_std = vpa(sol_struct.(char(unknown_var)), 6);
        catch
            sol_std = [];
        end
        fprintf('Solutions by tangent substitution:\n');
        % Tangent solution
        sol_tan = [];
        try
            if has(eq_sub, sin(unknown_var)) || has(eq_sub, cos(unknown_var))
                syms u real;
                eq_tan = eq_sub;
                if has(eq_sub, sin(unknown_var))
                    eq_tan = subs(eq_tan, sin(unknown_var), (2*u)/(1 + u^2));
                end
                if has(eq_sub, cos(unknown_var))
                    eq_tan = subs(eq_tan, cos(unknown_var), (1 - u^2)/(1 + u^2));
                end
                sol_u = solve(eq_tan, u);
                sol_u = sol_u(imag(sol_u) == 0);  % Filter complex solutions
                for i = 1:length(sol_u)
                    phi_candidate = 2 * atan(sol_u(i));
                    sol_tan = [sol_tan; simplify(vpa(phi_candidate, 6))];
                end
            end
        catch ME
            fprintf('Error in tangent substitution for %s: %s\n', unknown_name, ME.message);
        end
        
        % Display standard solutions
        if isempty(sol_std)
            fprintf('No standard solutions found for %s\n', unknown_name);
        else
            fprintf('Standard solutions for %s:\n', unknown_name);
            for i = 1:length(sol_std)
                fprintf('[STD %d] %s\n', i, char(sol_std(i)));
            end
        end
        
        % Display tangent solutions
        if isempty(sol_tan)
            fprintf('No solutions by tangent substitution found for %s\n', unknown_name);
        else
            fprintf('Tangent solutions for %s:\n', unknown_name);
            for i = 1:length(sol_tan)
                fprintf('[TAN %d] %s\n', i, char(sol_tan(i)));
            end
        end
            
        % Display solutions
        all_sols = [sol_std; sol_tan];
        if isempty(all_sols)
            fprintf('No solutions found for %s\n', unknown_name);
            continue;
        end
        
        fprintf('Possible solutions for %s:\n', unknown_name);
        for i = 1:length(all_sols)
            fprintf('[%d] %s\n', i, char(all_sols(i)));
        end
        fprintf('\nSelect the valid indices for %s, , preferably choose only one, check is not imaginary (e.g., [1 2] or 0 for none):\n', unknown_name);
        sel = input('Indices: ');
        if ~isequal(sel, 0)
            % Immediate replacement of k, l, m with 0
            syms k l m n k1 real;
            selected = all_sols(sel);
            selected = subs(selected, [k, l, m,n,k1], [0, 0, 0,0,0]);
            selected = simplify(selected);  % or vpa(selected, 6) if you prefer numbers
            
            if ~isfield(selected_solutions, unknown_name) || isempty(selected_solutions.(unknown_name))
                selected_solutions.(unknown_name) = selected;
            else
                selected_solutions.(unknown_name) = [selected_solutions.(unknown_name); selected];
            end
        
            fprintf('Selected solutions (k=0, l=0, m=0) for %s:\n', unknown_name);
            disp(vpa(selected_solutions.(unknown_name), 6));
        else
            fprintf('No solutions selected for %s\n', unknown_name);
        end
    end
end
fprintf('\n................................ values entering the calculation with 3 variables ...................................\n');
disp(selected_solutions);

fprintf('============================Equations with 3 Target Variable ================================================\n');
fprintf('========================================================================================\n');
for k = 1:length(eqs_three_vars)
    fprintf('===== [Equation %d/%d with 3 variables] =====\n', k, length(eqs_three_vars));
    fprintf('If no real solutions are found, please enter 0 and proceed to the next equation\n');
    eq_info = eqs_three_vars{k};
    eq = eq_info.equation;
    vars = eq_info.target_vars;
    var_names = eq_info.var_names;
    fprintf('\nEquation with three variables: %s, %s and %s\n', var_names{1}, var_names{2}, var_names{3});
    disp(eq);
    % Count how many variables already have a solution
    known = sum([ ...
        isfield(selected_solutions, var_names{1}) && ~isempty(selected_solutions.(var_names{1})), ...
        isfield(selected_solutions, var_names{2}) && ~isempty(selected_solutions.(var_names{2})), ...
        isfield(selected_solutions, var_names{3}) && ~isempty(selected_solutions.(var_names{3})) ...
    ]);
    if known < 2
        fprintf('At least 2 known variables are needed to solve.\n');
        continue;
    end
    % Try with combinations of 2 known variables
    for i = 1:3
        for j = i+1:3
            name_i = var_names{i};
            name_j = var_names{j};
            name_k = setdiff(var_names, {name_i, name_j});
            var_unknown = sym(name_k{1});
            if isfield(selected_solutions, name_i) && isfield(selected_solutions, name_j)
                sols_i = selected_solutions.(name_i);
                sols_j = selected_solutions.(name_j);
                for si = 1:length(sols_i)
                    for sj = 1:length(sols_j)
                        if isfield(selected_solutions, name_k{1}) && ~isempty(selected_solutions.(name_k{1}))
                            fprintf('Skipping combination si=%d, sj=%d: solution already exists for %s\n', ...
                                    si, sj, name_k{1});
                            continue;
                        end
                        eq_sub = subs(eq, [sym(name_i), sym(name_j)], [sols_i(si), sols_j(sj)]);
                        % Standard solution
                        try
                            sol_struct = solve(eq_sub, var_unknown, 'ReturnConditions', true);
                            sol_std = vpa(sol_struct.(char(var_unknown)), 6);
                        catch
                            sol_std = [];
                        end
                        % Tangent solution
                        sol_tan = [];
                        try
                            if has(eq_sub, sin(var_unknown)) || has(eq_sub, cos(var_unknown))
                                syms u real;
                                eq_tan = eq_sub;
                                if has(eq_sub, sin(var_unknown))
                                    eq_tan = subs(eq_tan, sin(var_unknown), (2*u)/(1 + u^2));
                                end
                                if has(eq_sub, cos(var_unknown))
                                    eq_tan = subs(eq_tan, cos(var_unknown), (1 - u^2)/(1 + u^2));
                                end
                                sol_u = solve(eq_tan, u);
                                sol_u = sol_u(imag(sol_u) == 0);  % Only real solutions
                                for r = 1:length(sol_u)
                                    phi_candidate = 2 * atan(sol_u(r));
                                    sol_tan = [sol_tan; simplify(vpa(phi_candidate, 6))];
                                end
                            end
                        catch ME
                            fprintf('Error in tangent substitution for %s: %s\n', name_k{1}, ME.message);
                        end
                        % Display solutions
                        all_sols = [sol_std; sol_tan];
                        
                        %test
                        if isempty(all_sols)
                            fprintf('No solutions found for %s\n', name_k{1});
                            continue;
                        end
                        fprintf('--- Combination (%d/%d) x (%d/%d) ---\n', ...
                            si, length(sols_i), sj, length(sols_j));
                        fprintf('%s = %s, %s = %s\n', ...
                            name_i, char(vpa(sols_i(si),6)), ...
                            name_j, char(vpa(sols_j(sj),6)));
   
                        fprintf('Looking for solutions for %s...\n', name_k{1});
                        for idx = 1:length(all_sols)
                            fprintf('[%d] ', idx);
                            disp(all_sols(idx));
                        end
                        fprintf('Select the valid indices for %s, preferably choose only one, check is not imaginary (e.g., [1 2] or 0 for none):\n', name_k{1});
                        sel = input('Indices: ');
                        % Add this at the beginning of the block that solves each three-variable equation
                        % If a previous solution exists, do not ask again or overwrite
                        if isfield(selected_solutions, name_k{1}) && ~isempty(selected_solutions.(name_k{1}))
                            fprintf('Solution already exists for %s. Skipping assignment.\n', name_k{1});
                            continue;
                        end
                        
                        % Evaluate what the user just entered
                        if isequal(sel, 0)
                            fprintf('No solutions selected for %s in this combination.\n', name_k{1});
                            continue;
                        end
                        
                        % Save the new solutions
                        syms k l m n k1 real;
                        selected = all_sols(sel);
                        selected = subs(selected, [k, l, m, n, k1], [0, 0, 0, 0,0]);
                        selected = simplify(selected);  % or vpa if you prefer
                        
                        selected_solutions.(name_k{1}) = selected;
                        fprintf('Selected solutions for %s:\n', name_k{1});
                        disp(vpa(selected, 6));
                    end
                end
            end
        end
    end
end
disp(selected_solutions);
fprintf('\n\n=== FINAL SOLUTIONS SAVED IN selected_solutions.mat ===\n');
for k = 1:length(target_var_names)
    var_name = target_var_names{k};
    if isfield(selected_solutions, var_name)
        fprintf('%s:\n', var_name);
        disp(vpa(selected_solutions.(var_name), 6));
    else
        fprintf('%s: no solution found.\n', var_name);
    end
end

save('final_solutions.mat', 'selected_solutions');
fprintf('Final solutions successfully saved in final_solutions.mat\n');