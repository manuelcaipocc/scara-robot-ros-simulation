clc; clear;
% Load the file with symbolic solutions
selected_solutions_file = 'final_solutions.mat';
if isfile(selected_solutions_file)
    load(selected_solutions_file, 'selected_solutions');
    fprintf('Earlier solutions were loaded from %s\n', selected_solutions_file);
else
    error('The file final_solutions.mat was not found.');
end
% Define all necessary symbolic variables
syms Hs H0 L0 L1 D1 L2 DA2 L3 H3 L5 L4 KH3
syms phi0 phi1 phi2 phi4 dh3  % Target variables
syms nx ny nz ux uy uz ax ay az px py pz
syms k l m n k1 k2 k3 k4  % Possible auxiliary variables in the solutions
syms dl1 dl2 dl3 dl4  % New symbolic variables to be defined via substitution
% Symbolic definitions of auxiliary variables
subs_relations = [
    dl1 == Hs - H0;
    dl2 == L0 + H0;
    dl3 == (DA2/2) + L3 + KH3;  % New definition for dl3
    dl4 == L4/2 + L5
];
% =============================================
% FIRST PART: DEFINITION OF CONSTANTS
% =============================================
% Numerical values in radians and millimeters
values = {
    H3, 1060;
    Hs, 200;
    H0, 100;
    L0, 800;
    L1, 400;
    L2, 500;
    L3, 630;
    L4, 600;
    L5, 50;
    D1, 110;
    DA2, 100;
    KH3, 80;  % Updated value for KH3
    nx, cos(pi/6);
    ny, sin(pi/6);
    nz, 0;
    ux, cos(pi/3);
    uy, -sin(pi/3);
    uz, 0;
    ax, 0;
    ay, 0;
    az, -1;
    px, 400;
    py, 100;
    pz, 50;
    k, 0;
    l, 0;
    m, 0;
    n, 0;
    k1, 0;
    k2, 0;
    k3,0;
    k4,0
};
% Get numerical values for the necessary variables
Hs_val = values{2,2};  % Hs is the second entry in values
H0_val = values{3,2};  % H0 is the third entry
L0_val = values{4,2};  % L0 is the fourth entry
DA2_val = values{11,2}; % DA2 is the eleventh entry
L3_val = values{7,2};  % L3 is the seventh entry
L4_val = values{8,2};  % L4 is the eighth entry
L5_val = values{9,2};  % L5 is the ninth entry
KH3_val = values{12,2}; % KH3 is the twelfth entry
% Calculate dl1 to dl4 according to the updated symbolic definitions
dl1_val = Hs_val - H0_val;
dl2_val = L0_val + H0_val;
dl3_val = (DA2_val/2) + L3_val + KH3_val;  % Updated calculation
dl4_val = (L4_val/2) + L5_val;
% Add dl1 to dl4 with correct numerical values
values = [values;
    {dl1, dl1_val};
    {dl2, dl2_val};
    {dl3, dl3_val};
    {dl4, dl4_val};
];
% Display equations and values of dl1 to dl4
fprintf('\n=== Definitions and values of dl1 to dl4 ===\n');
fprintf('dl1 = Hs - H0 = %.0f - %.0f = %.0f mm\n', Hs_val, H0_val, dl1_val);
fprintf('dl2 = L0 + H0 = %.0f + %.0f = %.0f mm\n', L0_val, H0_val, dl2_val);
fprintf('dl3 = (DA2/2) + L3 + KH3 = (%.0f/2) + %.0f + %.0f = %.0f mm\n',...
        DA2_val, L3_val, KH3_val, dl3_val);
fprintf('dl4 = L4/2 + L5 = %.0f/2 + %.0f = %.0f mm\n', L4_val, L5_val, dl4_val);
% Display geometric values
fprintf('\n=== Geometric Values ===\n');
for i = 1:12  % Display up to KH3
    fprintf('%s = %.1f mm\n', char(values{i,1}), values{i,2});
end
% =============================================
% SECOND PART: EQUATION EVALUATION
% =============================================
% Create homogeneous transformation matrix T1
T1 = [
    nx, ux, ax, px;
    ny, uy, ay, py;
    nz, uz, az, pz;
    0,  0,  0,  1
];
fprintf('\nMatrix T1 (symbolic):\n');
disp(T1)
% Evaluate the matrix T1 with the defined values
T1_eval = double(subs(T1, values(:,1), values(:,2)));
fprintf('\nMatrix T1 with substituted values:\n');
disp(T1_eval);
% Names of the target variables
vars = {'phi0', 'phi1', 'phi2', 'phi4', 'dh3'};
% Evaluation of each target variable
for i = 1:length(vars)
    current_var = vars{i};
    if isfield(selected_solutions, current_var)
        expr = selected_solutions.(current_var);
        fprintf('\n=== Evaluating variable: %s ===\n', current_var);
        fprintf('Original loaded expression:\n%s\n', char(expr));
        % First substitute the dl variables with their symbolic definitions
        expr_subs_dl = subs(expr, [dl1, dl2, dl3, dl4], [Hs - H0, L0 + H0, (DA2/2) + L3 + KH3, L4/2 + L5]);
        % Then substitute all numerical values
        try
            % Create safe values (replace exact zeros with small numbers for ax, ay)
            values_safe = values;
            for j = 1:size(values_safe,1)
                name = char(values_safe{j,1});
                if ismember(name, {'ax', 'ay'}) && values_safe{j,2} == 0
                    values_safe{j,2} = 1e-6;
                end
            end
            expr_subs = subs(expr_subs_dl, values_safe(:,1), values_safe(:,2));
            value = double(expr_subs);
            
            if startsWith(current_var, 'phi')
                fprintf('%s = %.6f rad\n', current_var, value);
                fprintf('%s = %.2f°\n', current_var, rad2deg(value));
            else
                fprintf('%s = %.2f mm\n', current_var, value);
            end
            
            % Store the evaluated value
            eval([current_var '_val = value;']);
            
        catch ME
            fprintf('\nError during the evaluation of %s:\n%s\n', current_var, ME.message);
            fprintf('Expression after substitution:\n%s\n', char(expr_subs_dl));
            remaining_vars = symvar(expr_subs_dl);
            if ~isempty(remaining_vars)
                fprintf('Unsubstituted variables:\n');
                disp(remaining_vars');
            else
                fprintf('No remaining symbolic variables detected, but conversion failed.\n');
            end
        end
    else
        fprintf('\nWarning: Variable %s not found in the solutions file\n', current_var);
    end
end