
clear;clc; % clearing the workspace
warning('off', 'all'); % disabling all warnings

%Here we are defining the robot DH parameters for each link
l(1) = Revolute('d',166.20/1000,'a',0,'alpha',-pi/2,'offset',0,'qlim',[-270 270]*pi/180); 
l(2) = Revolute('d',0,'a',636.10/1000,'alpha',0,'offset',-pi/2,'qlim',[-180 180]*pi/180); 
l(3) = Revolute('d',0,'a',557.90/1000,'alpha',0,'offset',0,'qlim',[-166 166]*pi/180); 
l(4) = Revolute('d',-129.70/1000,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[-180 180]*pi/180); 
l(5) = Revolute('d',106/1000,'a',0,'alpha',pi/2,'offset',0,'qlim',[-180 180]*pi/180);  
l(6) = Revolute('d',113.15/1000,'a',0,'alpha',0,'offset',0,'qlim',[-270 270]*pi/180);  
robot = SerialLink(l,'name', 'tm20');
disp("---------------------")

%Here we are defining the number of points to trace on the path
NUMBER_OF_POINTS = 100;

%Creating a vector for circle in a plane
t = linspace(0, 2*pi, NUMBER_OF_POINTS);
RADIUS = 0.9;

%generating a sphere
[x, y, z] = sphere;

%Below we are defining the center of the sphere
X_c = 0.3;
Y_c = 1.4;
Z_c = 0.78;

% Configured the circle making thing to trace a sphere intersecting a plane
% in the Y axis
% Use the parametric equation of a sphere then solve for phi using the
% plane of intersection and radius
% It is bounded by the curve of acos() and by Y_c - Y, essentially
% the plane must exist on the sphere, if it doesnt exist then no solution
% will arrive

% another bound is the reach of the robot 

Y = 0.55;
phi = acos((Y_c - Y)/RADIUS);
X = RADIUS * sin(phi) * cos(t); 
Z = RADIUS * sin(phi) * sin(t); 

x = RADIUS*x + X_c;
y = RADIUS*y + Y_c;
z = RADIUS*z + Z_c;

% Adjusting X, Z so that they have the correct offsets
X = X+X_c;
Z = Z+Z_c;

% The code below is Initializing a cell array to store joint angles for each point along the path
ARR = {};
ERROR_HIST = zeros(1, NUMBER_OF_POINTS);

% Below we are defining the initial transformation matrix based on the current position
P = [
        1 0 0 X(1);
        0 1 0 Y;
        0 0 1 Z(1);
        0 0 0 1];

% We are using inverse kinematics to calculate the initial joint configuration
Q0 = robot.ikcon(P);

% Looping over the points to calculate the joint configurations
for i = 1:NUMBER_OF_POINTS
    
    posX = X(i); % Current X position
    posY = Y;% Current y position since we are tracing a circle 
    posZ = Z(i); % Current z position
    G = [posX; posY; posZ]; 
    V = [posX; posY; posZ] - [X_c; Y_c; Z_c];
    V = V / norm(V);

    % let V be the vector that points to the center 
    % We have to compute the axis of rotation so we can do the axis/angle
    % rotation 
    % We can use the vector perpendicular to both V and the Z axis as the
    % axis of rotation
    % then the angle of rotation can just be the angle between V and Z
    % because both V and Z are unit vectors, cos(theta) = dot(V,Z) / mag(V)*mag(Z); 
    % or in other words, theta = arccos(dot(V,Z)) 

    % Then we can form a Skew Symmetric Matrix and use the Rodriguez Formula
   
    U = [0;0;1];
    
    k = cross(V,U);

    theta = acos(dot(V,U));
    % Create the skew-symmetric matrix for rotation using the Rodrigues formula
    S_k = [
      0, -k(3), k(2);
      k(3), 0, -k(1);
      -k(2), k(1), 0;
    
    ];

% Calculating the rotation matrix using Rodrigues' formula
    R = eye(3) + S_k*sin(theta)+ S_k^2*(1-cos(theta));

    P = [
        R(1,:), posX;
        R(2,:), posY;
        R(3,:), posZ;
        0, 0, 0, 1
        ];
       
    [angles, err] = robot.ikcon(P, Q0);
    err;
    ERROR_HIST(i) = err;
    ARR{end+1} = angles;
    Q0 = angles;
    % in order to speed up ikcon, we can pass in the previous joint angles
    % as a starting configuration for the numerical ik method
end


THETA_1 = zeros(1, NUMBER_OF_POINTS);
THETA_2 = zeros(1, NUMBER_OF_POINTS);
THETA_3 = zeros(1, NUMBER_OF_POINTS);
THETA_4 = zeros(1, NUMBER_OF_POINTS);
THETA_5 = zeros(1, NUMBER_OF_POINTS);
THETA_6 = zeros(1, NUMBER_OF_POINTS);
m = mean(ERROR_HIST) % just we can have an idea of how bad the ik is 

% ERROR of IK: if its large, it means that we could have certain points the robot
% cannot reach or we are approaching a singularity
for i = 1:length(ARR)
    angles = ARR{i};
    THETA_1(i) = angles(1);
    THETA_2(i) = angles(2);
    THETA_3(i) = angles(3);
    THETA_4(i) = angles(4);
    THETA_5(i) = angles(5);
    THETA_6(i) = angles(6);
end

% Below, we are combining joint angles into a single matrix
Q = [THETA_1' THETA_2' THETA_3' THETA_4' THETA_5' THETA_6'];

%code for the configuration settings for the simulation
CONFIG = { ...       
    'fps', 30, ...            
    'movie', 'tm20_video2.mp4', ... 
    'tilesize', 0.2, ...
    'trail', 'r', ...
    'view', [44 12]
};

figure;

% Creating a sphere to visualize
sph = surf(x,y,z);
hold on;

% Set the limits for the plot
xlim([-0.5, 2]);  
ylim([-0.5, 2]);  
zlim([0, 2]);

% Finally, we are plotting the robot's motion along the configurations of the joint computed
robot.plot(Q, CONFIG{:},'workspace',[-0.5,2,-0.5,2,0,2]);


hold off;
