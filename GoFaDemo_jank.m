%GoFa10 Demo

close all;
clear all;

axis([-2.5,2.5,-2.5,2.5,0,2.5])

hold on 

m = DobotMagician

%Gofa10
L1 = Link('d',0.4,'a',0.15,'alpha',pi/2,'qlim',[deg2rad(-270) deg2rad(270)]);
L2 = Link('d',0,'a',0.707,'alpha',0,'qlim',[deg2rad(-180) deg2rad(180)]);
L3 = Link('d',0,'a',-0.11,'alpha',-pi/2,'qlim',[deg2rad(-225) deg2rad(85)]);   
L4 = Link('d',0.637,'a',0,'alpha',pi/2,'qlim',[deg2rad(-180) deg2rad(180)]);
L5 = Link('d',0,'a',-0.08,'alpha',-pi/2,'qlim',[deg2rad(-180) deg2rad(180)]);
L6 = Link('d',0,'a',0,'alpha',0,'qlim',[deg2rad(-270) deg2rad(270)]);  

arm = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot');    
q = [0 pi/2 3*pi/2 0 0 0];
scale = 0.5;
workspace = [-2 2 -2 2 -2 2];                                       
arm.plot(q,'workspace',workspace,'scale',scale);    





%arm = GoFa10
% q = [0 0 0 0 0 0];
% scale = 0.5;
% workspace = [-2 2 -2 2 -2 2];                                       
   


% Creates the left gripper
LL1 = Link('d',0,'a',0.12,'alpha',0,'qlim',[-pi pi]) 
LL2 = Link('d',0,'a',0.12,'alpha',0,'offset',pi/4,'qlim',[-pi pi]) 
LL3 = Link('d',0,'a',0.12,'alpha',0,'offset',pi/4,'qlim',[-pi pi]) 

leftG = SerialLink([LL1 LL2 LL3],'name','myRobotl')

q = zeros(1,leftG.n);

leftG.base = arm.fkine(arm.getpos()).T * transl(0,0,0.1) * trotx(pi/2);

leftG.plot(q);

drawnow();

% Creates the right gripper
LR1 = Link('d',0,'a',-0.12,'alpha',0,'qlim',[-pi pi]) 
LR2 = Link('d',0,'a',-0.12,'alpha',0,'offset',-pi/4,'qlim',[-pi pi]) 
LR3 = Link('d',0,'a',-0.12,'alpha',0,'offset',-pi/4,'qlim',[-pi pi]) 

rightG = SerialLink([LR1 LR2 LR3],'name','myRobotr')

q = zeros(1,rightG.n);

rightG.base = arm.fkine(arm.getpos()).T * transl(0,0,0.1) * trotx(pi/2);

rightG.plot(q);

drawnow();


% Placing shelf
shelf = PlaceObject('Shelf3.ply',[-1,0,0.05]);
verts = [get(shelf,'Vertices'), ones(size(get(shelf,'Vertices'),1),1)] * trotz(pi);
set(shelf,'Vertices',verts(:,1:3))


% Bag
[f,v,data] = plyread('bag.ply','tri');

% Plots the surfaces of the brick
bag = trisurf(f,v(:,1),v(:,2), v(:,3));

% Vertex count
bagVertexCount = size(v,1);

% Remap brick vert matrix so it is the same size as if it where 
% get(brick,'Vertices')
midPoint1 = sum(v)/bagVertexCount;
bagVerts = v - repmat(midPoint1,bagVertexCount,1);

% Create transfomation matrix
bagPose = eye(4);

bagPose = bagPose * transl(0,1,0.23) * trotz(pi/2);
updbag = [bagPose * [bagVerts,ones(bagVertexCount,1)]']';
    bag.Vertices = updbag(:,1:3);

% Juice box 1
[f,v,data] = plyread('Juice_box.ply','tri');

% Plots the surfaces of the brick
box1 = trisurf(f,v(:,1),v(:,2), v(:,3));

% Vertex count
box1VertexCount = size(v,1);

% Remap brick vert matrix so it is the same size as if it where 
% get(brick,'Vertices')
midPointbox1 = sum(v)/box1VertexCount;
box1Verts = v - repmat(midPointbox1,box1VertexCount,1);

% Create transfomation matrix
box1Pose = eye(4);

box1Pose = box1Pose * transl(0.04,1.07,0.17) * trotz(pi/2);
updbox1 = [box1Pose * [box1Verts,ones(box1VertexCount,1)]']';
    box1.Vertices = updbox1(:,1:3);


% Juice box 2
[f,v,data] = plyread('Juice_box.ply','tri');

% Plots the surfaces of the brick
box2 = trisurf(f,v(:,1),v(:,2), v(:,3));

% Vertex count
box2VertexCount = size(v,1);

% Remap brick vert matrix so it is the same size as if it where 
% get(brick,'Vertices')
midPointbox2 = sum(v)/box2VertexCount;
box2Verts = v - repmat(midPointbox2,box2VertexCount,1);

% Create transfomation matrix
box2Pose = eye(4);

box2Pose = box2Pose * transl(-0.04,1.07,0.17) * trotz(pi/2);
updbox2 = [box2Pose * [box2Verts,ones(box2VertexCount,1)]']';
    box2.Vertices = updbox2(:,1:3);


% Placing flooring
surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[0.01,0.01;0.01,0.01] ...
,'CData',imread('oakfloorhd.jpg'),'FaceColor','texturemap');



view(3);
camlight;

%% 

steps = 90;

% initial robot state
T1_r = arm.fkine(arm.getpos());
% transl(0,0.867,1.217) * trotx(pi);
q1_r = arm.ikcon(T1_r);

% bag pick up robot state
T2_r = transl(0,0.8,0.40) * trotx(deg2rad(-130));
q2_r = arm.ikcon(T2_r);

%robTraj1 = jtraj(q1_r,q2_r,steps);
steps=90
x1 = q1_r';
x2 = q2_r';
deltaT = 0.01; % Discrete time step
% 3.7
x = zeros(6,steps);
s = lspb(0,1,steps); % Create interpolation scalar
for i = 1:steps
x(:,i) = x1*(1-s(i)) + s(i)*x2; % Create trajectory in x-y plane
end
% 3.8
robTraj1 = nan(steps,6);
% 3.9
% UPDATE: ikine function now has different syntax when entering
% arguments into the function call. The argument must be prefaced by
% its argument name. E.g. Initial Q Guess = 'q0', q & Mask = 'mask', m.
robTraj1(1,:) = arm.ikine(T1_r, 'q0', q1_r, 'mask', [1 1 1 0 0 0]); % Solve forjoint angles
% 3.10
for i = 1:steps-1
xdot = (x(:,i+1) - x(:,i))/deltaT; % Calculate velocity at discrete time step
J = arm.jacob0(robTraj1(i,:)); % Get the Jacobian at the current state
J = J(1:6,:); % Take only first 6 rows
qdot = inv(J)*xdot; % Solve velocitities via RMRC
robTraj1(i+1,:) = robTraj1(i,:) + deltaT*qdot'; % Update next joint state
end

for i = 1:steps
    arm.animate(robTraj1(i,:));
    leftG.base = arm.fkine(arm.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
    leftG.plot(zeros(1,3));
    rightG.base = arm.fkine(arm.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
    rightG.plot(zeros(1,3));
    drawnow();
end
%%
gSteps = 30;

% initial coordinates of left gripper arm
T1_left = leftG.fkine(leftG.getpos());
q1_left = leftG.ikcon(T1_left);

% initial coordinates of right gripper arm
T1_right = rightG.fkine(rightG.getpos());
q1_right = rightG.ikcon(T1_right);

% coordinates to move left gripper arm to
% T2_left = transl(0.05,1,0.2502);
%T2_left = leftG.fkine(leftG.getpos()).T * transl(-0.05,0.1,0);
T2_left = transl(0.03,1,0.2031) * trotz(deg2rad(160));
q2_left = leftG.ikcon(T2_left);

% coordinates to move right gripper arm to
% T2_right = transl(-0.05,1,0.2502);
% T2_right = rightG.fkine(rightG.getpos()).T * transl(0.05,0.1,0);
T2_right = transl(-0.03,1,0.2044) * trotz(deg2rad(200));
q2_right = rightG.ikcon(T2_right);


leftGIn = jtraj(q1_left,q2_left,gSteps);
rightGIn = jtraj(q1_right,q2_right,gSteps);

axis([-0.5,0.5,0.5,1.5,0,0.6])

for i = 1:gSteps
    leftG.animate(leftGIn(i,:));
    rightG.animate(rightGIn(i,:));
    leftG_cl_q = leftG.getpos();
    rightG_cl_q = rightG.getpos();
    drawnow
end

axis([-2.5,2.5,-2.5,2.5,0,2.5])

% bag pick up robot state
T3_r = arm.fkine(arm.getpos());
% b_tr = T3_r.T;
% b_tr(1:3,4)=0;
% b_tr = b_tr * trotz(deg2rad(-20)) * troty(pi/2);
q3_r = arm.ikcon(T3_r);

% bag magicbot robot state
T4_r = transl(0.68,0,1.4) * trotz(-pi/2) * trotx(deg2rad(-130));
q4_r = arm.ikcon(T4_r);



%robTraj1 = jtraj(q1_r,q2_r,steps);
steps=90
x1 = q3_r';
x2 = q4_r';
deltaT = 0.01; % Discrete time step
% 3.7
x = zeros(6,steps);
s = lspb(0,1,steps); % Create interpolation scalar
for i = 1:steps
x(:,i) = x1*(1-s(i)) + s(i)*x2; % Create trajectory in x-y plane
end
% 3.8
robTraj1 = nan(steps,6);
% 3.9
% UPDATE: ikine function now has different syntax when entering
% arguments into the function call. The argument must be prefaced by
% its argument name. E.g. Initial Q Guess = 'q0', q & Mask = 'mask', m.
robTraj1(1,:) = arm.ikine(T3_r, 'q0', q3_r, 'mask', [1 1 1 0 0 0]); % Solve forjoint angles
% 3.10
for i = 1:steps-1
xdot = (x(:,i+1) - x(:,i))/deltaT; % Calculate velocity at discrete time step
J = arm.jacob0(robTraj1(i,:)); % Get the Jacobian at the current state
J = J(1:6,:); % Take only first 6 rows
qdot = inv(J)*xdot; % Solve velocitities via RMRC
robTraj1(i+1,:) = robTraj1(i,:) + deltaT*qdot'; % Update next joint state
end

for i = 1:steps
    arm.animate(robTraj1(i,:));
    leftG.base = arm.fkine(arm.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
    leftG.plot(zeros(1,3));
    rightG.base = arm.fkine(arm.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
    rightG.plot(zeros(1,3));
    drawnow();
end


axis([0.2,1.4,-0.4,0.8,1,1.7])


% box2 drop magicbot state
T1_m = m.model.fkine(m.model.getpos());
q1_m = m.model.ikcon(T1_m);

% box1 pick up magicbot state
T2_m = transl(0.96,-0.04,1.209);
q2_m = m.model.ikcon(T2_m);

magicTraj1 = jtraj(q1_m,q2_m,steps);

for i = 1:steps
    m.model.animate(magicTraj1(i,:));
    drawnow
end


% box1 pick up magicbot state
T1_m = m.model.fkine(m.model.getpos());
q1_m = m.model.ikcon(T1_m);

% box1 drop magicbot state
T3_m = transl(1.25,-0.35,1.263);
q3_m = m.model.ikcon(T3_m);

magicTraj2 = jtraj(q1_m,q3_m,steps);

for i = 1:steps
    m.model.animate(magicTraj2(i,:));
    box1Pose = m.model.fkine(m.model.getpos()).T * transl(0,0,-0.05);
    updbox1 = [box1Pose * [box1Verts,ones(box1VertexCount,1)]']';
    box1.Vertices = updbox1(:,1:3);
    drawnow
end


% box2 drop magicbot state
T1_m = m.model.fkine(m.model.getpos());
q1_m = m.model.ikcon(T1_m);

% box2 drop magicbot state
T4_m = transl(1.15,0,1.263);
q4_m = m.model.ikcon(T4_m);

magicTraj3 = jtraj(q1_m,q4_m,steps);

for i = 1:steps
    m.model.animate(magicTraj3(i,:));
    drawnow
end

% bag pick up robot state
T1_m = m.model.fkine(m.model.getpos());
q1_m = m.model.ikcon(T1_m);

% box1 magicbot state
T5_m = transl(0.96,0.04,1.209);
q5_m = m.model.ikcon(T5_m);

magicTraj4 = jtraj(q1_m,q5_m,steps);

for i = 1:steps
    m.model.animate(magicTraj4(i,:));
    drawnow
end


% box2 pick up magicbot state
T1_m = m.model.fkine(m.model.getpos());
q1_m = m.model.ikcon(T1_m);

% box2 magicbot drop state
T6_m = transl(1.25,0.35,1.263);
q6_m = m.model.ikcon(T6_m);

magicTraj5 = jtraj(q1_m,q6_m,steps);

for i = 1:steps
    m.model.animate(magicTraj5(i,:));
    box2Pose = m.model.fkine(m.model.getpos()).T * transl(0,0,-0.05);
    updbox2 = [box2Pose * [box2Verts,ones(box2VertexCount,1)]']';
    box2.Vertices = updbox2(:,1:3);
    drawnow
end

% box2 drop magicbot state
T1_m = m.model.fkine(m.model.getpos());
q1_m = m.model.ikcon(T1_m);

% box2 int magicbot state
T7_m = transl(1.15,0,1.263);
q7_m = m.model.ikcon(T7_m);

magicTraj6 = jtraj(q1_m,q7_m,steps);

for i = 1:steps
    m.model.animate(magicTraj6(i,:));
    drawnow
end

axis([-2.5,2.5,-2.5,2.5,0,2.5])

% bag magicbot robot state
T4_r = transl(0.68,0,1.4) * trotz(-pi/2) * trotx(deg2rad(-130));
q4_r = arm.ikcon(T4_r);

% bag pick up robot state
T2_r = transl(0,0.8,0.40) * trotx(deg2rad(-130));
q2_r = arm.ikcon(T2_r);

steps = 90;

x1 = q4_r';
x2 = q2_r';
deltaT = 0.01; % Discrete time step
% 3.7
x = zeros(6,steps);
s = lspb(0,1,steps); % Create interpolation scalar
for i = 1:steps
x(:,i) = x1*(1-s(i)) + s(i)*x2; % Create trajectory in x-y plane
end
% 3.8
robTraj1 = nan(steps,6);
% 3.9
% UPDATE: ikine function now has different syntax when entering
% arguments into the function call. The argument must be prefaced by
% its argument name. E.g. Initial Q Guess = 'q0', q & Mask = 'mask', m.
robTraj1(1,:) = arm.ikine(T4_r, 'q0', q4_r, 'mask', [1 1 1 0 0 0]); % Solve forjoint angles
% 3.10
for i = 1:steps-1
xdot = (x(:,i+1) - x(:,i))/deltaT; % Calculate velocity at discrete time step
J = arm.jacob0(robTraj1(i,:)); % Get the Jacobian at the current state
J = J(1:6,:); % Take only first 6 rows
qdot = inv(J)*xdot; % Solve velocitities via RMRC
robTraj1(i+1,:) = robTraj1(i,:) + deltaT*qdot'; % Update next joint state
end

for i = 1:steps
    arm.animate(robTraj1(i,:));
    leftG.base = arm.fkine(arm.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
    leftG.plot(zeros(1,3));
    rightG.base = arm.fkine(arm.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
    rightG.plot(zeros(1,3));
    drawnow();
end


