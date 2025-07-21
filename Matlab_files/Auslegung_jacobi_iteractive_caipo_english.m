clc;
clear;
% Create the robot with correct configuration
robot = rigidBodyTree('DataFormat','row','MaxNumBodies',8);
% =============================================
% UNIT CONVERSION (mm to m) and parameters
% =============================================
Hs = 200/1000; H0 = 100/1000; L0 = 800/1000;
L1 = 400/1000; D1 = 110/1000; L2 = 500/1000;
L3 = 630/1000; DH3 = 360/1000; L4 = 600/1000;
L5 = 50/1000; H3 = 1060/1000; DA2 = 100/1000;
KH3 = 80/1000;
% Material properties (steel)
steel_density = 7750; % kg/m^3
% Custom geometry for each body (outer and inner diameters/sides)
% Format: [length, type, dimension1, dimension2, inner_dimension1, inner_dimension2]
% type: 1 = hollow cylinder, 2 = hollow square
geometries = {
    % body1: base to Z1 (cylinder)
    [Hs, 1, 0.3, 0.3,0.2,0.2]; % Note: fixed body
    
    % body2: between Z1 and phi0 (cylinder)
    [L0, 1, 0.2, 0.2, 0.15, 0.15];
    
    % body3: between phi0 and phi1 (hollow square)
    [L1, 2, 0.1, 0.1, 0.08, 0.08];
    
    % body4: between phi1 and phi2 (hollow square)
    [D1, 2, 0.1, 0.1, 0.08, 0.08];
    
    % body5: between phi2 and dh3 (cylinder)
    [L2, 1, 0.1, 0.1, 0.08, 0.08];
    
    % body6: between dh3 and phi4 (cylinder)
    [(DA2/2)+L3+KH3, 1, 0.1, 0.1, 0.08, 0.08];
    
    % body7: prismatic actuator (cylinder)
    [DH3, 1, 0.14, 0.14, 0.08, 0.08];
    
    % body8: end effector (cylinder)
    [(L4/2)+L5, 1, 0.14, 0.14, 0.08, 0.08];
};
% DH table: [a alpha d theta] (now in meters)
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
% Joint types
joint_types = {'fixed', 'revolute', 'revolute', 'fixed', 'revolute', 'fixed', 'prismatic', 'revolute'};
% =============================================
% Robot construction with realistic dynamic properties
% =============================================
% Correct joint types configuration
joint_types = {'fixed', 'revolute', 'revolute', 'fixed', 'revolute', 'fixed', 'prismatic', 'revolute'};
% Active (movable) joints:
activeJointBodies = [2, 3, 5, 7, 8]; % Indices of revolute/prismatic joints
% =============================================
% Robot construction with realistic dynamic properties
% =============================================
for i = 1:8
    % Create body and joint
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)], joint_types{i});
    
    % Configure DH transformation
    setFixedTransform(joints{i}, dhparams(i,:), 'dh');
    
    % Configure axis and limits based on joint type
    if strcmp(joint_types{i}, 'prismatic')
        joints{i}.JointAxis = [0 0 1]; % Movement in Z
        joints{i}.PositionLimits = [-DH3, 0]; % Prismatic joint moves along Z-axis
    
    elseif strcmp(joint_types{i}, 'revolute')
        % Assign limits based on the physical joint
        switch i
            case 2  % phi0 (body2, jnt2)
                joints{i}.JointAxis = [0 0 1]; % Z-axis
                joints{i}.PositionLimits = deg2rad([-150, 150]);
            case 3  % phi1 (body3, jnt3) - IMPORTANT: Rotates around Y
                joints{i}.JointAxis = [0 1 0]; % Y-axis
                joints{i}.PositionLimits = deg2rad([0, 90]);
            case 5  % phi2 (body5, jnt5)
                joints{i}.JointAxis = [0 0 1]; % Z-axis
                joints{i}.PositionLimits = deg2rad([-153, 153]);
            case 8  % phi4 (body8, jnt8)
                joints{i}.JointAxis = [0 0 1]; % Z-axis
                joints{i}.PositionLimits = deg2rad([-180, 180]);
        end
    end
    
    % =============================================
    % DYNAMIC PROPERTIES BASED ON CUSTOM GEOMETRY
    % =============================================
    % Only calculate properties for non-fixed bodies (since fixed bodies don't contribute to dynamics)
    if ~strcmp(joint_types{i}, 'fixed')
        % Get geometry parameters for this body
        geom = geometries{i};
        length = geom(1);
        type = geom(2);
        dim1 = geom(3);  % Outer radius or outer side
        dim2 = geom(4);  % For cylinder: same as dim1, for square: other side
        inner_dim1 = geom(5); % Inner radius or inner side
        inner_dim2 = geom(6); % For cylinder: same as inner_dim1, for square: other side
        
        % Mass calculation based on geometry type
        if type == 1 % Hollow cylinder
            outer_radius = dim1;
            inner_radius = inner_dim1;
            cross_section_area = pi*(outer_radius^2 - inner_radius^2);
        else % Hollow square
            outer_side = dim1;
            inner_side = inner_dim1;
            cross_section_area = outer_side^2 - inner_side^2;
        end
        
        volume = cross_section_area * length;
        mass = steel_density * volume;
        
        % Center of mass (at link center)
        center_of_mass = [length/2, 0, 0];
        
        % Inertia tensor calculation based on geometry type
        if type == 1 % Hollow cylinder
            % Inertia for cylindrical tube (X-axis along the tube)
            Ixx = (1/2)*mass*(outer_radius^2 + inner_radius^2);
            Iyy = (1/12)*mass*(3*(outer_radius^2 + inner_radius^2) + length^2);
            Izz = Iyy; % Cylindrical symmetry
        else % Hollow square
            % Inertia for hollow rectangular prism (X-axis along length)
            a_ext = dim1; b_ext = dim2; % External dimensions
            a_int = inner_dim1; b_int = inner_dim2; % Internal dimensions
            
            % Inertia calculation for hollow rectangular prism
            Ixx = (1/12)*mass*(a_ext^2 + b_ext^2 + a_int^2 + b_int^2);
            Iyy = (1/12)*mass*(3*(b_ext^2 + b_int^2) + length^2);
            Izz = (1/12)*mass*(3*(a_ext^2 + a_int^2) + length^2);
        end
        
        % Assign properties
        bodies{i}.Mass = mass;
        bodies{i}.CenterOfMass = center_of_mass;
        bodies{i}.Inertia = [Ixx, Iyy, Izz, 0, 0, 0]; % Ixx, Iyy, Izz, Iyz, Izx, Ixy
    end
    
    bodies{i}.Joint = joints{i};
    
    % Add to rigid body tree
    if i == 1
        addBody(robot, bodies{i}, 'base');
    else
        addBody(robot, bodies{i}, bodies{i-1}.Name);
    end
end
% =============================================
% GRAVITY CONFIGURATION (IMPORTANT!)
% =============================================
robot.Gravity = [0, 0, -9.81]; % Gravity in -Z direction (m/s²)
%% End Effector Configuration (body8 as end effector)
endEffectorFrame = 'body8';
% Show robot details
showdetails(robot);
% Convert given configurations from degrees to radians
configurations_deg = [
    90,0,0,-50/1000,0;
    0,90,0,-360/1000,0;
    90, 0.0000, -126.87, -50/1000, -66.87
];
configurations_rad = [
    deg2rad(90), deg2rad(0), deg2rad(0), -50/1000, deg2rad(0);
    deg2rad(0), deg2rad(90), deg2rad(90), -360/1000, deg2rad(0); % Corrected DH3 for 2nd config
    deg2rad(90), deg2rad(0.0000), deg2rad(-126.87), -50/1000, deg2rad(-66.87)
];
%% Analyze the configurations for singularities
for i = 1:3
    % For joint 7 (prismatic) in the rigidBodyTree, its position corresponds to `d` in the DH table.
    % The `q` input for `geometricJacobian` should correspond to the joint limits.
    % The `configurations_rad` array has phi0, phi1, phi2, dh3, phi4.
    % The `robot`'s joints are: jnt2 (phi0), jnt3 (phi1), jnt5 (phi2), jnt7 (dh3), jnt8 (phi4).
    % So, the q vector for geometricJacobian should be:
    q_robot = zeros(1, 5); % Initialize q for the robot's active joints
    q_robot(1) = configurations_rad(i, 1); % phi0 -> jnt2
    q_robot(2) = configurations_rad(i, 2); % phi1 -> jnt3
    q_robot(3) = configurations_rad(i, 3); % phi2 -> jnt5
    q_robot(4) = configurations_rad(i, 4); % dh3  -> jnt7 (prismatic joint value)
    q_robot(5) = configurations_rad(i, 5); % phi4 -> jnt8

    fprintf('\n=== Analyzing Configuration %d ===\n', i);
    fprintf('Joint angles (rad): [%.4f, %.4f, %.4f, %.4f, %.4f]\n', q_robot);
    fprintf('Joint angles (deg): [%.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
            rad2deg(q_robot(1)), rad2deg(q_robot(2)), rad2deg(q_robot(3)), q_robot(4)*1000, rad2deg(q_robot(5)));
    
    % Calculate Jacobian
    % geometricJacobian directly returns a 6xN Jacobian where N is the number of movable joints
    % The order of columns in J corresponds to the order of joints in the robot's 'Joints' property.
    % Our active joints are jnt2, jnt3, jnt5, jnt7, jnt8.
    J = geometricJacobian(robot, q_robot, endEffectorFrame);
    fprintf('Jacobian is:\n')
    disp(J);
    % J is already for the active joints, so J_reduced is not needed here
    
    % Singular value decomposition
    [U,S,V] = svd(J);
    singular_values = diag(S);
    
    fprintf('\nSingular values:\n');
    disp(singular_values');
    
    % Adaptive threshold for singularity detection
    tol = max(size(J)) * eps(max(singular_values));
    fprintf('\nAdaptive threshold: %e\n', tol);
    
    % Check for singularity
    rank_defect = sum(singular_values < tol);
    
    % Visualize the singular configuration
    figure('Name', sprintf('Singular Configuration %d', i));
    show(robot, q_robot, 'Frames', 'on', 'PreservePlot', false);
    hold on;
        
    % Mark end effector position
    eePos = getTransform(robot, q_robot, endEffectorFrame);
    plot3(eePos(1,4), eePos(2,4), eePos(3,4), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        
    title(sprintf('Singular Configuration %d', i));
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    view(3);
    axis equal;
    grid on;
    legend('Robot', 'TCP');
    
    if rank_defect > 0
        fprintf('WARNING! Robot is in or near a singular configuration.\n');
        fprintf('Rank deficiency: %d\n', rank_defect);
        
        % Identify lost motions
        % The columns of U corresponding to zero singular values define the directions of lost motion (null space)
        lost_motions = U(:, (end-rank_defect+1):end); 
        fprintf('\nLost motion directions (null space vectors):\n');
        disp(lost_motions);
        
        % Physical interpretation
        fprintf('\nPhysical interpretation:\n');
        for j = 1:rank_defect
            linear_part = lost_motions(1:3,j)';
            angular_part = lost_motions(4:6,j)';
            
            fprintf('Lost direction %d:\n', j);
            fprintf('  - Linear component: [%.3f, %.3f, %.3f]\n', linear_part);
            fprintf('  - Angular component: [%.3f, %.3f, %.3f]\n', angular_part);
            
            % Determine type of loss
            if norm(linear_part) > norm(angular_part) % Check which component is dominant
                fprintf('  - Dominant loss: linear motion\n');
            elseif norm(angular_part) > norm(linear_part)
                fprintf('  - Dominant loss: angular motion\n');
            else
                fprintf('  - Combined linear/angular loss or equal contribution\n');
            end
        end
        
    else
        fprintf('Robot is NOT in a singular configuration.\n');
    end

    if i==3
        F_TCP = [30; 0; 100; -50; 25; 75];  % [Fx; Fy; Fz; Mx; My; Mz]
        % MATLAB's geometricJacobian returns linear velocities first, then angular velocities (vx, vy, vz, wx, wy, wz)
        % For force/torque (wrench), it's typically (Fx, Fy, Fz, Mx, My, Mz) for consistency
        % The Jacobian transpose maps wrench to joint torques/forces: tau = J' * Wrench
        tau = J' * F_TCP;
        fprintf('tau (joint efforts) is:\n');
        % The first, second, third, and fifth values will be moments [Nm] (corresponding to revolute joints)
        % The fourth value is the required linear force in the prismatic axis 3 [N]
        disp(tau);
        fprintf('\nTau (joint efforts) interpretation:\n');
        fprintf('Tau(1): %.2f Nm  (Joint 1 - Moment for phi0)\n', tau(1));
        fprintf('Tau(2): %.2f Nm  (Joint 2 - Moment for phi1)\n', tau(2));
        fprintf('Tau(3): %.2f Nm  (Joint 3 - Moment for phi2)\n', tau(3));
        fprintf('Tau(4): %.2f  N   (Joint 4 - Linear force for dh3 prismatic axis)\n', tau(4));
        fprintf('Tau(5): %.2f Nm  (Joint 5 - Moment for phi4)\n', tau(5));
    end
end
