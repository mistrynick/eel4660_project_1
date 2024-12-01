% Here we are creating the robot model using rigidBodyTree
robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 6);

% Defining the joint lengths based on the assignment
L1 = 0.5; % Length of link 1
L2 = 0.5; % Length of link 2
L3 = 0.5; % Length of link 3

% Defining joint limits in radians of our robot
joint_limits = [
    -270*pi/180, 270*pi/180;   % Joint 1 limits
    -180*pi/180, 180*pi/180;   % Joint 2 limits
    -166*pi/180, 166*pi/180;   % Joint 3 limits
    -180*pi/180, 180*pi/180;   % Joint 4 limits
    -180*pi/180, 180*pi/180;   % Joint 5 limits
    -270*pi/180, 270*pi/180;   % Joint 6 limits
];

% Defining each joint's axis as in project 1
j1 = rigidBodyJoint('joint1', 'revolute');
j1.JointAxis = [0 0 1]; % Z-axis rotation

j2 = rigidBodyJoint('joint2', 'revolute');
j2.JointAxis = [0 1 0]; % Y-axis rotation

j3 = rigidBodyJoint('joint3', 'revolute');
j3.JointAxis = [0 1 0]; % Y-axis rotation

j4 = rigidBodyJoint('joint4', 'revolute');
j4.JointAxis = [1 0 0]; % X-axis rotation

j5 = rigidBodyJoint('joint5', 'revolute');
j5.JointAxis = [1 0 0]; % X-axis rotation

j6 = rigidBodyJoint('joint6', 'revolute');
j6.JointAxis = [0 0 1]; % Z-axis rotation

% Creating and adding bodies to the robot as we did in project 1
body1 = rigidBody('link1'); body1.Joint = j1; addBody(robot, body1, 'base');
body2 = rigidBody('link2'); body2.Joint = j2; addBody(robot, body2, 'link1');
body3 = rigidBody('link3'); body3.Joint = j3; addBody(robot, body3, 'link2');
body4 = rigidBody('link4'); body4.Joint = j4; addBody(robot, body4, 'link3');
body5 = rigidBody('link5'); body5.Joint = j5; addBody(robot, body5, 'link4');
body6 = rigidBody('link6'); body6.Joint = j6; addBody(robot, body6, 'link5');

% Here we are creating the inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', robot);

% Code to create the GUI figure as stated in project 2
fig = uifigure('Name', 'Robot Circle Path', 'Position', [100, 100, 400, 300]);

% Below is code for Latitude Inputs for the user
latLabel = uilabel(fig, 'Position', [20, 240, 100, 22], 'Text', 'Latitude:');
latInput = uieditfield(fig, 'numeric', 'Position', [120, 240, 100, 22]);

% Below is code for Longitude Inputs for the user
lonLabel = uilabel(fig, 'Position', [20, 200, 100, 22], 'Text', 'Longitude:');
lonInput = uieditfield(fig, 'numeric', 'Position', [120, 200, 100, 22]);

% Below is code for radius Inputs for the user
radiusLabel = uilabel(fig, 'Position', [20, 160, 100, 22], 'Text', 'Radius:');
radiusInput = uieditfield(fig, 'numeric', 'Position', [120, 160, 100, 22]);

% Here we are calculating the Button
calcBtn = uibutton(fig, 'push', 'Position', [150, 120, 100, 22], 'Text', 'Calculate', ...
                   'ButtonPushedFcn', @(btn, event) calculateCircle(latInput.Value, lonInput.Value, radiusInput.Value));

% Function to validate if the circle is within the robot's workspace
function valid = isValidCircle(lat, lon, radius)
    % Here we are assuming that the max reachable distance ofthe robot's end effector is within a sphere of radius 1.0m
    maxReach = 1.0; 

    % Code below is used to calculate the distance from the center of latitude and longitude to the origin
    distanceFromCenter = sqrt(lat^2 + lon^2);
    
    % code below is checking if the circle is within the robot's reach including radius
    if distanceFromCenter + radius <= maxReach
        valid = true;  % Here the circle is within the reach
    else
        valid = false; % Here the circle is out of the reach
    end
end

% Below is the function to solve the inverse kinematics with joint limits
% and weights as in project 1
function [config, solInfo] = solveIK(target_pos)
    % Code creating a starting configuration or the home
    q_init = robot.homeConfiguration;
    
    % Here we are setting up optimization problem with joint limits and
    % weights for the circle
    weights = [1, 1, 1, 0, 0, 0]; 
    solverOptions = ikoptions('MaxIterations', 1000, 'Solver', 'LMA', 'Tolerance', 1e-4);

    % The code below is setting up the constraint on joint limits
    jointConstraints = [];
    for i = 1:6
        jointConstraints = [jointConstraints; joint_limits(i, 1), joint_limits(i, 2)];
    end
    
    % Here we are using optimization to find the joint angles that satisfy the target position
    [config, solInfo] = ik(target_pos, q_init, weights, solverOptions);
    
    % Code below is hecking for the joint limits and clip if necessary
    for i = 1:6
        config(i) = min(max(config(i), joint_limits(i, 1)), joint_limits(i, 2));
    end
end

% Function to animate the robot tracing the circle as we did in project 1
function animateRobot(lat, lon, radius)
    % Number of points on the circle as we defined in project 1
    num_points = 100;
    theta_vals = linspace(0, 2 * pi, num_points);
    
    % Prepare the circular path points
    positions = zeros(num_points, 3);
    for i = 1:num_points
        theta = theta_vals(i);
        positions(i, :) = [radius * cos(theta) + lat, radius * sin(theta) + lon, sqrt(radius^2 - (radius * cos(theta))^2 - (radius * sin(theta))^2)];
    end

    % code to plot the circular path 
    figure;
    hold on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Omron TM20 Tracing Circular Path');
    plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'Color', 'lightgray');

    % code to plot the robot's reach
    [X, Y, Z] = sphere;
    surf(X, Y, Z, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', [0.7, 0.7, 0.7]);

    % Animation loop as we did in project 1
    for i = 1:num_points
        % Here we are getting the current position for the robot
        current_pos = positions(i, :);
        
        % Code below is creating the target transformation matrix
        target_transform = trvec2tform(current_pos); % Convert position to a transformation matrix
        
        % Solving the inverse kinematics with joint limits as we did in
        % project 1
        [config, solInfo] = solveIK(target_transform);
        
        % code to display the robot in the new configuration
        show(robot, config);
        
        % Pause for animation effect
        pause(0.1);
    end
    hold off;
end

% Button callback to handle circle calculation and animation
function calculateCircle(lat, lon, radius)
    % Checking if the circle is valid and within robot's reach
    if isValidCircle(lat, lon, radius)
        % If valid, then we will animate the robot tracing the circle
        animateRobot(lat, lon, radius);
    else
        % If not valid, then we will display an error message saying circle
        % is out of reach
        uialert(fig, 'The circle is out of reach for the robot.', 'Invalid Circle', 'Icon', 'error');
    end
end
