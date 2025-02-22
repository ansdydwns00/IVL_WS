%% ROS Node 
clear; clc

global G_bbox
global G_id
global G_cls
global G_point

% Create a node for connection between MATLAB and ROS2
Node = ros2node("/IVL");

% Create Subscribe Node
sub.avia = ros2subscriber(Node,"/livox/lidar","sensor_msgs/PointCloud2",@HelperCallbackLiDAR);
sub.lr_detection = ros2subscriber(Node,"/lr_detections","vision_msgs/Detection3DArray",@HelperCallbackPCDet);


% Create Publish Node
% pub.detections = ros2publisher(Node,'/clusters_marker', 'visualization_msgs/MarkerArray');

% Create Publish Message

%% Main 
%===================================================================================%
%-----------------------------------Visualization-----------------------------------%
%===================================================================================%
% Set x,y,z range of pcplayer
xmin = 0;      xmax = 10;
ymin = -5;     ymax = 5;
zmin = -2;      zmax = 4;


player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",6);

% Set values for frame count 
frameCount = 1;

% Set values for n frames
frame_num = 6;

% Parameter for n frame buffer
xyzPointsBuffer = [];
xyzIntensityBuffer = [];



%===================================================================================%
%----------------------------------Detection Info-----------------------------------%
%===================================================================================%


L_bbox = [];
L_id = [];
L_cls = [];



while true

    % Store [frame_num] message
    xyzPointsBuffer = vertcat(xyzPointsBuffer,G_point(:,1:3));
    xyzIntensityBuffer = vertcat(xyzIntensityBuffer,G_point(:,4));
    
    if mod(frameCount,frame_num) == 0
    
        ptCloud = pointCloud(xyzPointsBuffer,"Intensity",xyzIntensityBuffer);
        
        L_bbox = G_bbox;
        L_id = G_id;
        L_cls = G_cls;

        % Display ptCloud 
        view(player,ptCloud);
        HelperDeleteCuboid(player.Axes)


        if ~isempty(L_bbox) && ~isempty(ptCloud.Location)                 
    
            Distances = HelperComputeDistance(L_bbox,ptCloud);

            HelperDrawCuboid(player.Axes,L_bbox,Distances,L_id,L_cls) 
        end

        xyzPointsBuffer = [];
        xyzIntensityBuffer = [];
    end
    

    frameCount = frameCount + 1; 
    pause(0.01)
end

