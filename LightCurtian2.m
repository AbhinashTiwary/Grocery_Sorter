% light curtian

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


% Placing person
[f,v,data] = plyread('PaintedHuman1.ply','tri');

% Scale from RGB 0-255 to RGB 0-1
humanColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Plots the surfaces of the brick
human = trisurf(f,v(:,1)+0.5,v(:,2)+2, v(:,3) ...
    ,'FaceVertexCData',humanColours,'EdgeColor','interp','EdgeLighting','flat');

% Vertex count
humanVertexCount = size(v,1);

% Remap brick vert matrix so it is the same size as if it where 
% get(brick,'Vertices')
midPointhuman = sum(v)/humanVertexCount;
humanVerts = v - repmat(midPointhuman,humanVertexCount,1);

% Create transfomation matrix
humanPose = makehgtform('translate',[0.5,2,0.6]);


% Placing flooring
surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[0.01,0.01;0.01,0.01] ...
,'CData',imread('oakfloorhd.jpg'),'FaceColor','texturemap');



view(3);
camlight;



steps = 90;

% initial robot state
T1_r = Gofa10.fkine(Gofa10.getpos());
q1_r = Gofa10.ikcon(T1_r);

% end robot state
T2_r = transl(0,0.8,0.40) * trotx(deg2rad(-130));
q2_r = Gofa10.ikcon(T2_r);

robTraj1 = jtraj(q1_r,q2_r,steps);

% human transfromations
movFwHuman = makehgtform('translate',[0,-0.02,0]);
movBkHuman = makehgtform('translate',[0,0.04,0]);

% differance between human move and robot start back up need this for
% smooth animation
diff = 11;
counter = 0;

for i = 1:steps+diff

    % animate robot trajectory
    if i < 40
        Gofa10.animate(robTraj1(i,:));
        leftG.base = Gofa10.fkine(Gofa10.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
        leftG.plot(zeros(1,3));
        rightG.base = Gofa10.fkine(Gofa10.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
        rightG.plot(zeros(1,3));
        robq = Gofa10.getpos();
        disp("GO")
        drawnow();
    end
    
    % move human
    if i < 40
        humanPose = humanPose * movFwHuman;
        updHuman = [humanPose * [humanVerts,ones(humanVertexCount,1)]']';
        human.Vertices = updHuman(:,1:3);
        drawnow();  
    end
    
    % if human is in light curtian range stop movment
    if (abs(humanPose(2,4)-1.22) < 0.05)
        Gofa10.plot(robq)
        disp("STOP")
    end
    
    % moving human away from light curtian
    if (i > 45) && (i < 55)
        humanPose = humanPose * movBkHuman;
        updHuman = [humanPose * [humanVerts,ones(humanVertexCount,1)]']';
        human.Vertices = updHuman(:,1:3);
        drawnow();  
    end

    % if in the latter half of the for loop and out of the range of the
    % sensor then complete trajectory
    if i > 50 && ((abs(humanPose(2,4)-1.22) > 0.05))
        disp("GO")
        Gofa10.animate(robTraj1(i-diff,:));
        leftG.base = Gofa10.fkine(Gofa10.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
        leftG.plot(zeros(1,3));
        rightG.base = Gofa10.fkine(Gofa10.getpos()).T * transl(0,0,0.1) * trotx(pi/2);
        rightG.plot(zeros(1,3));
        robq = Gofa10.getpos();
        drawnow();
    end

end

