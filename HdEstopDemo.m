close all;
clear all;


axis([-3,2.2,-3.5,3.5,0,4])

hold on 


m = DobotMagician

%Gofa10
L1 = Link('d',0.4,'a',0.15,'alpha',pi/2,'qlim',[deg2rad(-270) deg2rad(270)]);
L2 = Link('d',0,'a',0.707,'alpha',0,'qlim',[deg2rad(-180) deg2rad(180)]);
L3 = Link('d',0,'a',-0.11,'alpha',-pi/2,'qlim',[deg2rad(-225) deg2rad(85)]);   
L4 = Link('d',0.637,'a',0,'alpha',pi/2,'qlim',[deg2rad(-180) deg2rad(180)]);
L5 = Link('d',0,'a',-0.08,'alpha',-pi/2,'qlim',[deg2rad(-180) deg2rad(180)]);
L6 = Link('d',0,'a',0,'alpha',0,'qlim',[deg2rad(-270) deg2rad(270)]);  

Gofa10 = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot');    
q = [1.5710    0.7277    0.0075   -0.0000   -1.3551   -1.5709];
scale = 0.5;
% workspace = [-2 2 -2 2 -2 2];                                       
Gofa10.plot(q,'scale',scale);    


% Creates the left gripper
LL1 = Link('d',0,'a',0.12,'alpha',0,'qlim',[-pi pi]) 
LL2 = Link('d',0,'a',0.12,'alpha',0,'offset',pi/4,'qlim',[-pi pi]) 
LL3 = Link('d',0,'a',0.12,'alpha',0,'offset',pi/4,'qlim',[-pi pi]) 

leftG = SerialLink([LL1 LL2 LL3],'name','myRobotl')

q = zeros(1,leftG.n);

leftG.base = Gofa10.fkine(Gofa10.getpos()).T * transl(0,0,0.1) * trotx(pi/2);

leftG.plot(q);

drawnow();

% Creates the right gripper
LR1 = Link('d',0,'a',-0.12,'alpha',0,'qlim',[-pi pi]) 
LR2 = Link('d',0,'a',-0.12,'alpha',0,'offset',-pi/4,'qlim',[-pi pi]) 
LR3 = Link('d',0,'a',-0.12,'alpha',0,'offset',-pi/4,'qlim',[-pi pi]) 

rightG = SerialLink([LR1 LR2 LR3],'name','myRobotr')

q = zeros(1,rightG.n);

rightG.base = Gofa10.fkine(Gofa10.getpos()).T * transl(0,0,0.1) * trotx(pi/2);

rightG.plot(q);

drawnow();


% Placing environment
env = PlaceObject('env2.ply',[-2.5,-1.5,-0.3]);
verts1 = [get(env,'Vertices'), ones(size(get(env,'Vertices'),1),1)] * trotz(pi);
set(env,'Vertices',verts1(:,1:3))

% Placing shelf
shelf = PlaceObject('Shelf3.ply',[-1,0,0.05]);
verts = [get(shelf,'Vertices'), ones(size(get(shelf,'Vertices'),1),1)] * trotz(pi);
set(shelf,'Vertices',verts(:,1:3))

% Placing emergency stop
stop = PlaceObject('PaintedStop.ply',[-1.9,3,1]);

% Placing fire extinguisher
fire = PlaceObject('PaintedExtinguisher.ply',[-2.5,3,0]);


% Bag
[f,v,data] = plyread('bag.ply','tri');

% Plots the surfaces of the bag
bag = trisurf(f,v(:,1),v(:,2), v(:,3));

% Vertex count
bagVertexCount = size(v,1);

% Remap bag vert matrix so it is the same size as if it where 
midPoint1 = sum(v)/bagVertexCount;
bagVerts = v - repmat(midPoint1,bagVertexCount,1);

% Create transfomation matrix
bagPose = eye(4);

bagPose = bagPose * transl(0,1,0.23) * trotz(pi/2);
updbag = [bagPose * [bagVerts,ones(bagVertexCount,1)]']';
    bag.Vertices = updbag(:,1:3);


% Juice box 1
[f,v,data] = plyread('Juice_box.ply','tri');

% Plots the surfaces of the box
box1 = trisurf(f,v(:,1),v(:,2), v(:,3));

% Vertex count
box1VertexCount = size(v,1);

% Remap box vert matrix so it is the same size as if it where 
midPointbox1 = sum(v)/box1VertexCount;
box1Verts = v - repmat(midPointbox1,box1VertexCount,1);

% Create transfomation matrix
box1Pose = eye(4);

box1Pose = box1Pose * transl(0.04,1.07,0.17) * trotz(pi/2);
updbox1 = [box1Pose * [box1Verts,ones(box1VertexCount,1)]']';
    box1.Vertices = updbox1(:,1:3);


% Juice box 2
[f,v,data] = plyread('Juice_box.ply','tri');

% Plots the surfaces of the box
box2 = trisurf(f,v(:,1),v(:,2), v(:,3));

% Vertex count
box2VertexCount = size(v,1);

% Remap box vert matrix so it is the same size as if it where 
midPointbox2 = sum(v)/box2VertexCount;
box2Verts = v - repmat(midPointbox2,box2VertexCount,1);

% Create transfomation matrix
box2Pose = eye(4);

box2Pose = box2Pose * transl(-0.04,1.07,0.17) * trotz(pi/2);
updbox2 = [box2Pose * [box2Verts,ones(box2VertexCount,1)]']';
    box2.Vertices = updbox2(:,1:3);


% Placing flooring
surf([-3.5,-3.5;3.5,3.5],[-3.5,3.5;-3.5,3.5],[0.01,0.01;0.01,0.01] ...
,'CData',imread('oakfloorhd.jpg'),'FaceColor','texturemap');


button_pin = 'D4';
port = 'COM5';
board = 'Uno';

a = arduino;

counter = 0;

view(3);
camlight;


steps = 130;

% initial robot state
T1_r = Gofa10.fkine(Gofa10.getpos());
q1_r = Gofa10.ikcon(T1_r);

% bag pick up robot state
T2_r = transl(0,0.8,0.40) * trotx(deg2rad(-130));
q2_r = Gofa10.ikcon(T2_r);

robTraj1 = jtraj(q1_r,q2_r,steps);
x = [];
x2nd = [];

diff = 0;


button_val = readDigitalPin(a, button_pin);
button_press = 0;
count = 0;
count2nd = 0;
but_tog = 1;
initial_val = 1;
rob_state = 0;
next_state = 0;
diff2nd = 0;
diff_count = 0;
round = 0;



if next_state == 0
    disp("1st")
    for i = 1:steps
        button_val = readDigitalPin(a, button_pin);
        if button_val == 1 && initial_val == 0
            % first button click that sends to loop offset recording
            if rob_state == 0
                disp("High")
                rob_state = 1;
            else
                disp("Low")
                rob_state = 0;
                % this is the second button click
                next_state = 1;
                break
            end
        end
        initial_val = button_val;
        
        % when robot state is clicked over this recordes the elapsed loops
        % to offset the other next for loop to keep animation smooth
        if rob_state == 1
            x = [x,i];
            diff = x(end) - x(1);
            disp("Stoped")
        else
            disp("Moving")
            % robot animation at start up as robot state is 0
            Gofa10.animate(robTraj1(i,:));
            leftG.base = Gofa10.fkine(Gofa10.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
            leftG.plot(zeros(1,3));
            rightG.base = Gofa10.fkine(Gofa10.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
            rightG.plot(zeros(1,3));
            drawnow();
        end
        % total amount of loops completed before break occurs
        count = count +1;
    end

end

if next_state == 1
    initial_val = 1;
    disp("2nd")
    
    % loop offset ammount
    but_diff = (steps - count) + diff;
    
    % loop begins where the other left off for smooth animation
    for j = count:count+but_diff
        button_val = readDigitalPin(a, button_pin);
        if rob_state == 0
            disp("Moving")

            % animate rest of the trajectory
            Gofa10.animate(robTraj1(j-diff,:));
            leftG.base = Gofa10.fkine(Gofa10.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
            leftG.plot(zeros(1,3));
            rightG.base = Gofa10.fkine(Gofa10.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
            rightG.plot(zeros(1,3));
            drawnow();
        end
    end
end
