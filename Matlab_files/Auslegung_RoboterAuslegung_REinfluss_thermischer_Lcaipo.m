clc;
clear;

% =============================================
% Thermal Expansion Parameters
% =============================================
alpha_St = 12.5e-6;  % Thermal expansion coefficient [K^-1]
T0 = 18;              % Initial temperature [°C]
T1 = 30;              % Final temperature [°C]
delta_T = T1 - T0;    % Temperature change [K]
expansion_factor = 1 + alpha_St * delta_T;

% =============================================
% UNIT CONVERSION (mm to m) with thermal expansion
% =============================================
% Original dimensions in mm (before expansion)
Hs_mm = 200; H0_mm = 100; L0_mm = 800;
L1_mm = 400; D1_mm = 110; L2_mm = 500;
L3_mm = 630; DH3_mm = 360; L4_mm = 600;
L5_mm = 50; H3_mm = 1060; DA2_mm = 100;
KH3_mm = 80;

% Apply thermal expansion to all linear dimensions
Hs = Hs_mm * expansion_factor / 1000;
H0 = H0_mm * expansion_factor / 1000;
L0 = L0_mm * expansion_factor / 1000;
L1 = L1_mm * expansion_factor / 1000;
D1 = D1_mm * expansion_factor / 1000;
L2 = L2_mm * expansion_factor / 1000;
L3 = L3_mm * expansion_factor / 1000;
DH3 = DH3_mm * expansion_factor / 1000;
L4 = L4_mm * expansion_factor / 1000;
L5 = L5_mm * expansion_factor / 1000;
DA2 = DA2_mm * expansion_factor / 1000;
KH3 = KH3_mm * expansion_factor / 1000;

% Print all thermal expansion information at the beginning
disp('=============================================');
disp(' Thermal Expansion Information');
disp('=============================================');
fprintf('Thermal expansion coefficient (alpha): %.2e K^-1\n', alpha_St);
fprintf('Initial temperature: %.1f°C\n', T0);
fprintf('Final temperature: %.1f°C\n', T1);
fprintf('Temperature change: %.1f K\n', delta_T);
fprintf('Thermal expansion factor: %.6f\n', expansion_factor);
disp(' ');
disp('Original dimensions (mm) and expanded dimensions (m):');
fprintf('Hs: %6.1f mm -> %.6f m\n', Hs_mm, Hs);
fprintf('H0: %6.1f mm -> %.6f m\n', H0_mm, H0);
fprintf('L0: %6.1f mm -> %.6f m\n', L0_mm, L0);
fprintf('L1: %6.1f mm -> %.6f m\n', L1_mm, L1);
fprintf('D1: %6.1f mm -> %.6f m\n', D1_mm, D1);
fprintf('L2: %6.1f mm -> %.6f m\n', L2_mm, L2);
fprintf('L3: %6.1f mm -> %.6f m\n', L3_mm, L3);
fprintf('DH3: %6.1f mm -> %.6f m\n', DH3_mm, DH3);
fprintf('L4: %6.1f mm -> %.6f m\n', L4_mm, L4);
fprintf('L5: %6.1f mm -> %.6f m\n', L5_mm, L5);
fprintf('DA2: %6.1f mm -> %.6f m\n', DA2_mm, DA2);
fprintf('KH3: %6.1f mm -> %.6f m\n', KH3_mm, KH3);
disp(' ');

% =============================================
% Robot Construction (with expanded dimensions)
% =============================================
robot = rigidBodyTree('DataFormat','row','MaxNumBodies',8);

% DH table with expanded dimensions
dhparams = [
    0, 0, Hs-H0, 0;       % 1. base to Z1
    0, pi/2, L0+H0, 0;    % 2. phi0 (revolute)
    L1, -pi/2, 0, 0;      % 3. phi1 (revolute)
    0, 0, D1, 0;          % 4. fixed
    L2, pi, 0, 0;         % 5. phi2 (revolute)
    0, 0, (DA2/2)+L3+KH3, 0; % 6. fixed
    0, 0, DH3, 0;         % 7. prismatic
    0, 0, (L4/2)+L5, 0    % 8. phi4 (revolute)
];

% Rest of your robot construction code remains the same...
joint_types = {'fixed', 'revolute', 'revolute', 'fixed', 'revolute', 'fixed', 'prismatic', 'revolute'};
activeJointBodies = [2, 3, 5, 7, 8];

for i = 1:8
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)], joint_types{i});
    setFixedTransform(joints{i}, dhparams(i,:), 'dh');
    
    if strcmp(joint_types{i}, 'prismatic')
        joints{i}.JointAxis = [0 0 1];
        joints{i}.PositionLimits = [-DH3, 0];
    elseif strcmp(joint_types{i}, 'revolute')
        switch i
            case 3  % phi0
                joints{i}.JointAxis = [0 0 1];
                joints{i}.PositionLimits = deg2rad([-150, 150]);
            case 4  % phi1
                joints{i}.JointAxis = [0 1 0];
                joints{i}.PositionLimits = deg2rad([0, 90]);
            case 5  % phi2
                joints{i}.JointAxis = [0 0 1];
                joints{i}.PositionLimits = deg2rad([-153, 153]);
            case 8  % phi4
                joints{i}.JointAxis = [0 0 1];
                joints{i}.PositionLimits = deg2rad([-180, 180]);
        end
    end
    
    bodies{i}.Joint = joints{i};
    
    if i == 1
        addBody(robot, bodies{i}, 'base');
    else
        addBody(robot, bodies{i}, bodies{i-1}.Name);
    end
end

endEffectorFrame = 'body8';
showdetails(robot);

% =============================================
% Inverse Kinematics with Expanded Robot
% =============================================
% Target pose remains the same (we want to maintain same end-effector position)
T_goal = [0.8660  0.5000  0       400/1000;
          0.5000 -0.8660  0       100/1000;
          0       0      -1.0000  50/1000;
          0       0       0       1.0000];

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 1000;
weights = [1 1 1 1 1 1];

% Find solutions with the expanded robot
numDesiredSolutions = 2;
solutionCount = 0;
solutions = {};
errors = [];

while solutionCount < numDesiredSolutions
    initialGuess = [
        deg2rad(randi([-150, 150])),    % phi0
        deg2rad(randi([0, 90])),        % phi1
        deg2rad(randi([-153, 153])),    % phi2
        -DH3*rand(),                    % dh3
        deg2rad(randi([-180, 180]))     % phi4
    ]';
    
    [configSoln, solnInfo] = ik(endEffectorFrame, T_goal, weights, initialGuess);
    configSoln = configSoln(:)';
    
    % Check solution validity
    valid = true;
    for i = 1:length(configSoln)
        jointBodyIdx = activeJointBodies(i);
        jointLimits = robot.Bodies{jointBodyIdx}.Joint.PositionLimits;
        if configSoln(i) < jointLimits(1) || configSoln(i) > jointLimits(2)
            valid = false;
            break;
        end
    end
    
    if valid
        T_solved = getTransform(robot, configSoln, endEffectorFrame);
        pos_error = norm(T_goal(1:3,4) - T_solved(1:3,4));
        rot_error = norm(T_goal(1:3,1:3) - T_solved(1:3,1:3));
        total_error = pos_error + rot_error;
        
        if total_error < 0.01
            isDuplicate = false;
            for j = 1:solutionCount
                if max(abs(solutions{j} - configSoln)) < 0.01
                    isDuplicate = true;
                    break;
                end
            end
            
            if ~isDuplicate
                solutionCount = solutionCount + 1;
                solutions{solutionCount} = configSoln;
                errors(solutionCount) = total_error;
                fprintf('Found solution %d of %d\n', solutionCount, numDesiredSolutions);
            end
        end
    end
end

% Display thermal compensation results
disp('=============================================');
disp(' Thermal Expansion Compensation Results');
disp('=============================================');
fprintf('Temperature change: %.1f K (from %.1f°C to %.1f°C)\n', delta_T, T0, T1);
fprintf('Expansion factor: %.6f\n', expansion_factor);

for i = 1:min(2, length(solutions))
    fprintf('\nSolution %d (Error: %.6f):\n', i, errors(i));
    fprintf('phi0: %9.4f rad (%9.4f°)\n', solutions{i}(1), rad2deg(solutions{i}(1)));
    fprintf('phi1: %9.4f rad (%9.4f°)\n', solutions{i}(2), rad2deg(solutions{i}(2)));
    fprintf('phi2: %9.4f rad (%9.4f°)\n', solutions{i}(3), rad2deg(solutions{i}(3)));
    fprintf('dh3:  %9.4f m (%9.4f mm)\n', solutions{i}(4), solutions{i}(4)*1000);
    fprintf('phi4: %9.4f rad (%9.4f°)\n', solutions{i}(5), rad2deg(solutions{i}(5)));
end

% Visualize solutions
for i = 1:min(4, length(solutions))
    figure;
    show(robot, solutions{i});
    hold on;
    plot3(T_goal(1,4), T_goal(2,4), T_goal(3,4), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title(sprintf('Thermal Compensation Solution %d (Error: %.6f)', i, errors(i)));
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    grid on;
end