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
densidad_acero = 7750; % kg/m^3

% Custom geometry for each body (external/internal diameters/sides)
% Format: [length, type, dimension1, dimension2, internal_dimension1, internal_dimension2]
% type: 1 = hollow cylinder, 2 = hollow square
% Extended format: [length, type, dim1, dim2, int1, int2, axis]
% axis = 1 (X), 2 (Y), 3 (Z)
geometrias = {
    [Hs, 1, 0.3/2, 0.3/2, 0.2/2, 0.2/2, 3];   
    [L0, 3, 0.2/2, 0.2/2, 0.15/2, 0.15/2, 3]; 
    [L1, 2, 0.1, 0.1, 0.08, 0.08, 1]; 
    [D1, 4, 0.1, 0.1, 0.08, 0.08, 3];
    [L2, 1, 0.1/2, 0.1/2, 0.08/2, 0.08/2, 1];
    [(DA2/2)+L3+KH3, 1, 0.1/2, 0.1/2, 0.08/2, 0.08/2, 3];
    [DH3, 1, 0.14/2, 0.14/2, 0.08/2, 0.08/2, 3]; 
    [(L4/2)+L5, 1, 0.14/2, 0.14/2, 0.08/2, 0.08/2, 3]; 
};


% DH table: [a L alpha d theta phi] (now in meters)
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
        joints{i}.PositionLimits = [-DH3, 0];
    
    elseif strcmp(joint_types{i}, 'revolute')
        % Assign limits based on the physical joint
        switch i
            case 3  % phi0
                joints{i}.JointAxis = [0 0 1]; % Z-axis
                joints{i}.PositionLimits = deg2rad([-150, 150]);
            case 4  % phi1 - IMPORTANT: Now rotates around Y
                joints{i}.JointAxis = [0 1 0]; % Y-axis
                joints{i}.PositionLimits = deg2rad([0, 90]);
            case 5  % phi2
                joints{i}.JointAxis = [0 0 1]; % Z-axis
                joints{i}.PositionLimits = deg2rad([-153, 153]);
            case 8  % phi4
                joints{i}.JointAxis = [0 0 1]; % Z-axis
                joints{i}.PositionLimits = deg2rad([-180, 180]);
        end
    end
    
    % =============================================
    % DYNAMIC PROPERTIES BASED ON CUSTOM GEOMETRY
    % =============================================
    if ismember(joint_types{i}, {'fixed', 'revolute', 'prismatic'})
        % Get geometry parameters for this body
        geom = geometrias{i};
        longitud = geom(1);
        tipo = geom(2);
        dim1 = geom(3);  % Radio externo o lado externo
        dim2 = geom(4);  % Para cilindro: igual a dim1, para cuadrado: otro lado
        dim_int1 = geom(5); % Radio interno o lado interno
        dim_int2 = geom(6); % Para cilindro: igual a dim_int1, para cuadrado: otro lado
        
        % Mass calculation based on geometry type
        if tipo == 1 % Cilindro hueco
            radio_externo = dim1;
            radio_interno = dim_int1;
            area_seccion = pi*(radio_externo^2 - radio_interno^2);
            volumen = area_seccion * longitud;
        
        elseif tipo == 2 % Cuadrado hueco
            lado_externo = dim1;
            lado_interno = dim_int1;
            area_seccion = lado_externo^2 - lado_interno^2;
            volumen = area_seccion * longitud;
        
        elseif tipo == 3
            volumen = 0.012967447276; % CAD
        
        elseif tipo == 4
            volumen = 0.000740789794;  %  CAD
        
        else
            error('Tipo de geometría desconocido para volumen en body %d', i);
        end



        masa = densidad_acero * volumen;
        masses(i) = masa;
        
        % Center of mass (at link center)
        % Direction (1=X, 2=Y, 3=Z)
        eje_principal = geom(7);
        centro_masa = [0, 0, 0];  
        
  
        if tipo == 3
            %  CAD 
            distancia = 0.470732;  % en metros
            centro_masa(eje_principal) = distancia;
        elseif tipo == 4
            distancia = 0.111934;  % en metros
            centro_masa(eje_principal) = distancia;
        else
            switch eje_principal
                case 1
                    centro_masa = [longitud/2, 0, 0];
                case 2
                    centro_masa = [0, longitud/2, 0];
                case 3
                    centro_masa = [0, 0, longitud/2];
                otherwise
                    error('Eje no reconocido para la geometría del body %d', i);
            end
        end

        
        % Inertia tensor calculation based on geometry type
        if tipo == 1 % Cilindro hueco

            Ixx = (1/2)*masa*(radio_externo^2 + radio_interno^2);
            Iyy = (1/12)*masa*(3*(radio_externo^2 + radio_interno^2) + longitud^2);
            Izz = Iyy; % Simetría cilíndrica
        elseif tipo==2 
            a_ext = dim1; b_ext = dim2; 
            a_int = dim_int1; b_int = dim_int2; 
            
            Ixx = (1/12)*masa*(a_ext^2 + b_ext^2 + a_int^2 + b_int^2);
            Iyy = (1/12)*masa*(3*(b_ext^2 + b_int^2) + longitud^2);
            Izz = (1/12)*masa*(3*(a_ext^2 + a_int^2) + longitud^2);
        elseif tipo == 3  
            Ixx = 7749265.619e-6;
            Iyy = 776332.607e-6;
            Izz = 7749265.619e-6;
        
        elseif tipo == 4 
            Ixx = 36063.992e-6;
            Iyy = 18013.291e-6;
            Izz = 36063.991e-6;

        end
        
        % Assign properties
        bodies{i}.Mass = masa;
        bodies{i}.CenterOfMass = centro_masa;
        bodies{i}.Inertia = [Ixx, Iyy, Izz, 0, 0, 0];
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

%% End Effector configuration (body8 as end effector)
endEffectorFrame = 'body8';

% Show robot details
showdetails(robot);

%% Enhanced visualization
figure('Name','Robot with End Effector in body8');
show(robot, 'Frames', 'on', 'PreservePlot', false);
hold on;

% Mark end effector position
config = homeConfiguration(robot);
eePos = getTransform(robot, config, endEffectorFrame);
plot3(eePos(1,4), eePos(2,4), eePos(3,4), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

title('Robot with End Effector in body8');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); % Now in meters
view(3);
axis equal;
grid on;

%% Interactive interface
figure("Name","Interactive GUI test");
gui = interactiveRigidBodyTree(robot, 'MarkerScaleFactor', 0.5, 'MarkerBodyName', endEffectorFrame);

%% Dynamic calculations
% 1. Calculate Jacobian
config = homeConfiguration(robot);
jacobian = geometricJacobian(robot, config, endEffectorFrame);

fprintf('\n=== Jacobian in home configuration ===\n');
disp(jacobian);


%% Ruck-limited trajectory generation
load('ruckig_trajectories.mat');  

time_phi0 = trajectories.phi0.time;
time_phi1 = trajectories.phi1.time;
time_phi2 = trajectories.phi2.time;
time_dh3  = trajectories.dh3.time;
time_phi4 = trajectories.phi4.time;

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


n_points = length(phi0_values);


tau_eval = zeros(5, n_points);

% Forzar a vectores fila
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

whos phi0 phi1 phi2 dh3 phi4

fprintf('\n=========== ESTADÍSTICAS ORIGINALES DE LAS TRAYECTORIAS ===========\n');

% PHI0
print_stats(trajectories.phi0.position,      'phi0.position');
print_stats(trajectories.phi0.velocity,      'phi0.velocity');
print_stats(trajectories.phi0.acceleration,  'phi0.acceleration');
print_stats(trajectories.phi0.jerk,          'phi0.jerk');

% PHI1
print_stats(trajectories.phi1.position,      'phi1.position');
print_stats(trajectories.phi1.velocity,      'phi1.velocity');
print_stats(trajectories.phi1.acceleration,  'phi1.acceleration');
print_stats(trajectories.phi1.jerk,          'phi1.jerk');

% PHI2
print_stats(trajectories.phi2.position,      'phi2.position');
print_stats(trajectories.phi2.velocity,      'phi2.velocity');
print_stats(trajectories.phi2.acceleration,  'phi2.acceleration');
print_stats(trajectories.phi2.jerk,          'phi2.jerk');

% DH3
print_stats(trajectories.dh3.position,       'dh3.position');
print_stats(trajectories.dh3.velocity,       'dh3.velocity');
print_stats(trajectories.dh3.acceleration,   'dh3.acceleration');
print_stats(trajectories.dh3.jerk,           'dh3.jerk');

% PHI4
print_stats(trajectories.phi4.position,      'phi4.position');
print_stats(trajectories.phi4.velocity,      'phi4.velocity');
print_stats(trajectories.phi4.acceleration,  'phi4.acceleration');
print_stats(trajectories.phi4.jerk,          'phi4.jerk');


fprintf('\n=== Shape de variables de posición ===\n');
disp(['phi0: ', mat2str(size(phi0_values))]);
disp(['phi1: ', mat2str(size(phi1_values))]);
disp(['phi2: ', mat2str(size(phi2_values))]);
disp(['dh3:  ', mat2str(size(dh3_values))]);
disp(['phi4: ', mat2str(size(phi4_values))]);


final_time = max([
    trajectories.phi0.time(end);
    trajectories.phi1.time(end);
    trajectories.phi2.time(end);
    trajectories.dh3.time(end);
    trajectories.phi4.time(end)
]);

n_points = max([length(trajectories.phi0.time); length(trajectories.phi1.time);
                length(trajectories.phi2.time); length(trajectories.dh3.time);
                length(trajectories.phi4.time)]);

common_time = linspace(0, final_time, n_points);

pad_with_zeros = @(data, target_len) [data, zeros(1, target_len - length(data))];

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
dddh3_values   = pad_with_zeros(trajectories.dh3.acceleration, n_points);

phi4_values    = pad_with_zeros(trajectories.phi4.position, n_points);
dphi4_values   = pad_with_zeros(trajectories.phi4.velocity, n_points);
ddphi4_values  = pad_with_zeros(trajectories.phi4.acceleration, n_points);


fprintf('\n=========== ESTADÍSTICAS DESPUÉS DE LA INTERPOLACIÓN ===========\n');

% PHI0
print_stats(phi0_values,      'phi0.position (interp)');
print_stats(dphi0_values,     'phi0.velocity (interp)');
print_stats(ddphi0_values,    'phi0.acceleration (interp)');

% PHI1
print_stats(phi1_values,      'phi1.position (interp)');
print_stats(dphi1_values,     'phi1.velocity (interp)');
print_stats(ddphi1_values,    'phi1.acceleration (interp)');

% PHI2
print_stats(phi2_values,      'phi2.position (interp)');
print_stats(dphi2_values,     'phi2.velocity (interp)');
print_stats(ddphi2_values,    'phi2.acceleration (interp)');

% DH3
print_stats(dh3_values,       'dh3.position (interp)');
print_stats(ddh3_values,      'dh3.velocity (interp)');
print_stats(dddh3_values,     'dh3.acceleration (interp)');

% PHI4
print_stats(phi4_values,      'phi4.position (interp)');
print_stats(dphi4_values,     'phi4.velocity (interp)');
print_stats(ddphi4_values,    'phi4.acceleration (interp)');

time = common_time;
n_points = length(time);

fprintf('\n=== Shape de variables de posición ===\n');
disp(['phi0: ', mat2str(size(phi0_values))]);
disp(['phi1: ', mat2str(size(phi1_values))]);
disp(['phi2: ', mat2str(size(phi2_values))]);
disp(['dh3:  ', mat2str(size(dh3_values))]);
disp(['phi4: ', mat2str(size(phi4_values))]);


%% evaluation Lagrange  inverseDynamics 

tau_eval = zeros(5, n_points);


for i = 1:n_points
    q_current = [phi0_values(i), phi1_values(i), phi2_values(i), dh3_values(i), phi4_values(i)];
    qd_current = [dphi0_values(i), dphi1_values(i), dphi2_values(i), ddh3_values(i), dphi4_values(i)];
    qdd_current = [ddphi0_values(i), ddphi1_values(i), ddphi2_values(i), dddh3_values(i), ddphi4_values(i)];
    
    tau_eval(:, i) = inverseDynamics(robot, q_current, qd_current, qdd_current);

    if mod(i, 100) == 0 || i == n_points
        fprintf('Paso %d/%d - Tiempo %.3f s\n', i, n_points, time(i));
        fprintf('Tau evaluated');
        disp(tau_eval(:, i));
    end
end

fprintf('\nGenerating Torque vs Time and Position vs Time plots...\n');

joint_names = {'$\varphi_0$', '$\varphi_1$', '$\varphi_2$', '$d_{h3}$', '$\varphi_4$'};
units_tau   = {'Nm', 'Nm', 'Nm', 'N', 'Nm'};
positions   = {phi0_values, phi1_values, phi2_values, dh3_values, phi4_values};
is_angle    = [true, true, true, false, true];  % dh3 is linear

% Define colors
color_torque   = [0 0 0];       % black
color_position = [0 0 0.6];     % dark blue

for i = 1:5
    figure('Name', ['Torque and Position - Joint ', joint_names{i}], 'Color', 'w');
    file_names     = {'tau_phi0.png', 'tau_phi1.png', 'tau_phi2.png', 'tau_dh3.png', 'tau_phi4.png'};
    % LEFT Y-AXIS: Torque
    yyaxis left
    ax = gca;
    ax.YColor = color_torque;  % Match y-axis color
    plot(time, tau_eval(i, :), '-', 'LineWidth', 2, 'Color', color_torque);
    ylabel(sprintf('$\\tau_{%d}$ [%s]', i-1, units_tau{i}), ...
           'FontSize', 12, 'Interpreter', 'latex');
    ylim padded
    grid on;

    % RIGHT Y-AXIS: Position
    yyaxis right
    ax.YColor = color_position;  % Match y-axis color
    if is_angle(i)
        plot(time, rad2deg(positions{i}), '--', 'LineWidth', 1.5, 'Color', color_position);
        ylabel(sprintf('%s [deg]', joint_names{i}), ...
               'FontSize', 12, 'Interpreter', 'latex');
    else
        plot(time, positions{i}, '--', 'LineWidth', 1.5, 'Color', color_position);
        ylabel(sprintf('%s [m]', joint_names{i}), ...
               'FontSize', 12, 'Interpreter', 'latex');
    end
    ylim padded

    % X-Axis
    xlabel('Time [s]', 'FontSize', 12, 'Interpreter', 'latex');

    % Title
    title(sprintf('Movable joint %d: Torque $\\tau_{%d}$ and %s vs Time', ...
          i, i-1, joint_names{i}), ...
          'FontSize', 14, 'Interpreter', 'latex');

    % Legend
    legend({'Torque', 'Position'}, 'Location', 'best', 'Interpreter', 'latex');

    set(gca, 'FontSize', 12);
    grid on;
    box on;
    
    jpg_name = strrep(file_names{i}, '.png', '.jpg');  
    exportgraphics(gcf, jpg_name, 'Resolution', 300); 
end


function print_stats(signal, name)
    fprintf('\n--- Estadísticas para %s ---\n', name);
    fprintf('  Mínimo:    %.6f\n', min(signal));
    fprintf('  Máximo:    %.6f\n', max(signal));
    fprintf('  Promedio:  %.6f\n', mean(signal));
    fprintf('  Desv. Std: %.6f\n', std(signal));
end


%% === 8. Calculation and visualization of total robot mass ===
total_mass = sum(masses);
fprintf('\n=== Total Robot Mass ===\n');
fprintf('Total mass: %.3f kg\n', total_mass);
fprintf('Breakdown by links:\n');
for i = 1:numel(masses)
    fprintf('  Link %d: %.3f kg\n', i, masses(i));
end


