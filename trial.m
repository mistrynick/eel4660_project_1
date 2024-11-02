% Here we are defining the robot model using rigidBodyTree
robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 6);

% Adding the joints and bodies according to the DH parameters of the Omron TM20
L1 = 0.5; % Length of link 1 
L2 = 0.5; % Length of link 2 
L3 = 0.5; % Length of link 3 

% Below is the joint definitions of OMRON Robot with angles and axes
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

% Creating bodies and adding them to the robot
body1 = rigidBody('link1'); body1.Joint = j1; addBody(robot, body1, 'base');
body2 = rigidBody('link2'); body2.Joint = j2; addBody(robot, body2, 'link1');
body3 = rigidBody('link3'); body3.Joint = j3; addBody(robot, body3, 'link2');
body4 = rigidBody('link4'); body4.Joint = j4; addBody(robot, body4, 'link3');
body5 = rigidBody('link5'); body5.Joint = j5; addBody(robot, body5, 'link4');
body6 = rigidBody('link6'); body6.Joint = j6; addBody(robot, body6, 'link5');

% Defining the sphere parameters
r = 1.0; % Radius of the sphere
num_points = 100; % Total number of points on the circular path
theta_vals = linspace(0, 2 * pi, num_points);

% Here we aer calculating the end-effector positions along the circular path
positions = zeros(num_points, 3);
for i = 1:num_points
    theta = theta_vals(i);
    positions(i, :) = [r * cos(theta), r * sin(theta), sqrt(r^2 - (r * cos(theta))^2 - (r * sin(theta))^2)];
end

% Preparing the figure for animation
figure;
hold on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Omron TM20 Drawing Circular Path on Sphere');

% code to plot the circular path
plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'Color', 'lightgray');

% Animation loop
for i = 1:num_points
    % code to get the current position
    current_pos = positions(i, :);
    
    % Calculate inverse kinematics
    joint_angles = inverseKinematics(robot, current_pos); 
    
    % Code to update the robot's position using joint angles
    show(robot, joint_angles, 'Frames', 'off');
    
    % Pause for a brief moment for animation effect
    pause(0.1);
end

hold off;

% Function to compute inverse kinematics (implement as needed)
function joint_angles = inverseKinematics(robot, target_pos)
    joint_angles = zeros(6, 1); % Replace with actual calculations
   
end
