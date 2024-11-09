
clear;clc;
warning('off', 'all');

l(1) = Revolute('d',166.20/1000,'a',0,'alpha',-pi/2,'offset',0,'qlim',[-270 270]*pi/180); 
l(2) = Revolute('d',0,'a',636.10/1000,'alpha',0,'offset',-pi/2,'qlim',[-180 180]*pi/180); 
l(3) = Revolute('d',0,'a',557.90/1000,'alpha',0,'offset',0,'qlim',[-166 166]*pi/180); 
l(4) = Revolute('d',-129.70/1000,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[-180 180]*pi/180); 
l(5) = Revolute('d',106/1000,'a',0,'alpha',pi/2,'offset',0,'qlim',[-180 180]*pi/180);  
l(6) = Revolute('d',113.15/1000,'a',0,'alpha',0,'offset',0,'qlim',[-270 270]*pi/180);  
robot = SerialLink(l,'name', 'tm20');
disp("---------------------")

NUMBER_OF_POINTS = 100;

t = linspace(0, 2*pi, NUMBER_OF_POINTS);
RADIUS = 0.9;

[x, y, z] = sphere;

X_c = 0.3;
Y_c = 1.4;
Z_c = 0.78;


Y = 0.6;
phi = acos((Y_c - Y)/RADIUS);
X = RADIUS * sin(phi) * cos(t); 
Z = RADIUS * sin(phi) * sin(t); 

x = RADIUS*x + X_c;
y = RADIUS*y + Y_c;
z = RADIUS*z + Z_c;

X = X+X_c;
Z = Z+Z_c;

ARR = {};
ERROR_HIST = zeros(1, NUMBER_OF_POINTS);
P_hist = {};

P = [
        1 0 0 X(1);
        0 1 0 Y;
        0 0 1 Z(1);
        0 0 0 1];

Q0 = robot.ikcon(P);

for i = 1:NUMBER_OF_POINTS
    
    posX = X(i);
    posY = Y;
    posZ = Z(i);
    % nicholas mistry: math for orientation of end-effector is the same thing as glm lookAt from computer
    % graphics rendering, essentially we pretend the end-effector is the
    % camera and we want to look towards the center to mimic how a pen
    % would function if you were to draw on a sphere
    % SOURCE: https://registry.khronos.org/OpenGL-Refpages/gl2.1/xhtml/gluLookAt.xml

    target = [X_c-posX; Y_c-posY; Z_c-posZ];
    U_hat = [0; 0; 1];

    f = target/norm(target);
    r = cross(f,U_hat);
    r = r / norm(r);

    u = cross(r,f);
    u = u / norm(u);

    R = [
     r(1), r(2), r(3);
     u(1), u(2), u(3);
     -f(1), -f(2),  -f(3) ;
    ];
    
    P = [
        R(1,:), posX;
        R(2,:), posY;
        R(3,:), posZ;
        0, 0, 0, 1
        ];

    
    %P = [
        %1 0 0 posX;
        %0 1 0 posY;
        %0 0 1 posZ;
        %0 0 0 1];
    P_hist{end+1} = P;

    [angles, err] = robot.ikcon(P, Q0);
    err;
    ERROR_HIST(i) = err;
    ARR{end+1} = angles;
    Q0 = angles;
    
end


THETA_1 = zeros(1, NUMBER_OF_POINTS);
THETA_2 = zeros(1, NUMBER_OF_POINTS);
THETA_3 = zeros(1, NUMBER_OF_POINTS);
THETA_4 = zeros(1, NUMBER_OF_POINTS);
THETA_5 = zeros(1, NUMBER_OF_POINTS);
THETA_6 = zeros(1, NUMBER_OF_POINTS);
m = mean(ERROR_HIST)
for i = 1:length(ARR)
    angles = ARR{i};
    THETA_1(i) = angles(1);
    THETA_2(i) = angles(2);
    THETA_3(i) = angles(3);
    THETA_4(i) = angles(4);
    THETA_5(i) = angles(5);
    THETA_6(i) = angles(6);
end

Q = [THETA_1' THETA_2' THETA_3' THETA_4' THETA_5' THETA_6'];

positions = zeros(length(P_hist), 3);
for i = 1:length(P_hist)
    positions(i, :) = P_hist{i}(1:3, 4)'; 
end

CONFIG = { ...       
    'fps', 30, ...            
    'movie', 'tm20_video1.mp4', ... 
    'tilesize', 0.2, ...
    'trail', 'r', ...
    'view', [44 12]
};
positions;

figure;

sph = surf(x,y,z);
hold on;
alpha(sph, 0.5);
%plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'b', 'LineWidth',2);

xlim([-0.5, 2]);  
ylim([-0.5, 2]);  
zlim([0, 2]);
robot.plot(Q, CONFIG{:},'workspace',[-0.5,2,-0.5,2,0,2]);


hold off;
