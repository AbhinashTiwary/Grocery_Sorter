%Dobot Mov

clear all;
close all;
clc;
rosshutdown;

% Start dobot magician node
rosinit;

dobot = DobotMagician;

% Operating State
safetyStateMsg.Data = 2;

% Home
end_effector_position = [0.22,0,0.08];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(2);

% Pick up box 1
end_effector_position = [0.22,0.04,-0.04];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(2);

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);



% Drop box 1
end_effector_position = [0.08,0.2,0.09];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(2);

% Int 1
end_effector_position = [0.22,0.06,0.08];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(1.5);

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

pause(2);

% Pick up box 2
end_effector_position = [0.22,-0.04,-0.04];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(2);

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

pause(2);

% Drop box 2
end_effector_position = [0.08,-0.2,0.09];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);


pause(2);



% End
end_effector_position = [0.22,-0.06,0.08];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

pause(1.5);

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);