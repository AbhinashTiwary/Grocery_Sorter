close all;
clear all;

L1 = Link('d',0.4,'a',0.15,'alpha',pi/2,'qlim',[deg2rad(-270) deg2rad(270)]);
L2 = Link('d',0,'a',0.707,'alpha',0,'qlim',[deg2rad(-180) deg2rad(180)]);
L3 = Link('d',0,'a',-0.11,'alpha',-pi/2,'qlim',[deg2rad(-225) deg2rad(85)]);   
L4 = Link('d',0.637,'a',0,'alpha',pi/2,'qlim',[deg2rad(-180) deg2rad(180)]);
L5 = Link('d',0,'a',-0.08,'alpha',-pi/2,'qlim',[deg2rad(-180) deg2rad(180)]);
L6 = Link('d',0,'a',0,'alpha',0,'qlim',[deg2rad(-270) deg2rad(270)]);  

Gofa10 = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot');    
q = [0 pi/2 3*pi/2 0 0 0];
scale = 0.5;
workspace = [-2 2 -2 2 -2 2];                                       
Gofa10.plot(q,'workspace',workspace,'scale',scale);    


Gofa10.teach(q);




