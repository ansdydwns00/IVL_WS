%% Connect AVIA UDP Communication
clear; clc

% Connect udp data communication
Avia_UDP = udpport("datagram","LocalPort",56001);

%% ROS Node 

% 2D Detection global parameter
% global g_img
global g_bboxes

% 3D Detection global parameter
global G_bbox
global G_id
global G_cls
global G_vel
global G_isTracking


G_bbox = [];
G_id = {};
G_cls = {};
G_vel = [];
G_isTracking = []; 


% Create a node for connection between MATLAB and ROS2
Node        = ros2node("/IVL");
Sub_Node    = ros2node("/IVL_Sub");

% Create Subscribe Node
sub.Yolo_track      = ros2subscriber(Node,"/yolo/tracking","yolov8_msgs/DetectionArray",@HelperCallbackYolo_KF);
sub.lr_detection    = ros2subscriber(Sub_Node,"/lr_detections","vision_msgs/Detection3DArray",@HelperCallbackPCDet_KF);

% Create Publish Node
pub.LiDAR           = ros2publisher(Node,"/livox/lidar","sensor_msgs/PointCloud2");

%% 
clear HelperDrawCuboid_KF
%-----------------------------------------------------------------------------------%
%-------------------------------Calibration Parameter-------------------------------%
%-----------------------------------------------------------------------------------%

% Load LiDAR-Camera Calibration parameter
load("lcc_params_640.mat");
 
% lcc parameter
lidarToCam = tform;              
camToLidar = invert(tform);
%-----------------------------------------------------------------------------------%



%-----------------------------------------------------------------------------------%
%-----------------------------------Visualization-----------------------------------%
%-----------------------------------------------------------------------------------%
% Set x,y,z range of pcplayer
xmin = 0;       xmax = 40;
ymin = -7;      ymax = 7;
zmin = -2;      zmax = 1.5;

% pointCloud viewer
player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",4);


% image viewer
% vPlayer = vision.DeployableVideoPlayer;

l_bboxes = [];
% l_img = [];
%-----------------------------------------------------------------------------------%




%-----------------------------------------------------------------------------------%
%------------------------------Clustering Parameter---------------------------------%
%-----------------------------------------------------------------------------------%
% ROI setting
roi = [1, 40, -7, 7, -2, 1.3];     

% Downsampling
gridStep = 0.1;

% Cluster distance 
clusterThreshold = 0.5;   

%-----------------------------------------------------------------------------------%



%-----------------------------------------------------------------------------------%
%-----------------------------------UDP Parameter-----------------------------------%
%-----------------------------------------------------------------------------------%
% Set values for frame count 
frameCount = 1;

% Set values for n frames
frame_num = 3;

% Flag for first Run
reset_flag = single(0);

% Parameter for n frame buffer
xyzPointsBuffer = [];
xyzIntensityBuffer = [];

% Remove Buffer
flush(Avia_UDP);
%-----------------------------------------------------------------------------------%




while true
    
    % Read 1 packet datagram
    packet = read(Avia_UDP,1,"uint8");
    
    % LiDAR [x,y,z,I] data parsing
    if size(packet.Data,2) == 1362 
        [xyzCoords,xyzIntensity,isValid] = Avia_parsing_mex(single((packet.Data)'),reset_flag);
    end
    
    if isValid

        % Store [frame_num] message
        xyzPointsBuffer = vertcat(xyzPointsBuffer,xyzCoords);
        xyzIntensityBuffer = vertcat(xyzIntensityBuffer,xyzIntensity);

        if mod(frameCount,frame_num) == 0
            ptCloud = pointCloud(xyzPointsBuffer,"Intensity",xyzIntensityBuffer);
            
            % Preprocessing point clound (ROI, Downsampling, remove ground)
            if ~isempty(ptCloud.Location)
                ptCloud_ps = HelperPtCldProcessing_KF(ptCloud, roi, gridStep); 
            end
            
            % Results from 2D DL model
            l_bboxes = g_bboxes;
            
            if ~isempty(l_bboxes) && ~isempty(ptCloud_ps.Location)
                frustumIndices = HelperBboxCameraToLidar_KF(l_bboxes, ptCloud_ps, camParams, camToLidar,'ClusterThreshold', clusterThreshold, 'MaxDetectionRange', [3,60]);
            
                if ~isempty(frustumIndices{1})
    
                    allValues       = vertcat(frustumIndices{:});
                    uniqueValues    = unique(allValues);
                    
                    tmpPC                       = select(ptCloud_ps,uniqueValues);
                    msg_LiDAR                   = ros2message(pub.LiDAR);
                    msg_LiDAR.header.frame_id   = 'map';
                    msg_LiDAR                   = rosWriteXYZ(msg_LiDAR,(tmpPC.Location));
                    msg_LiDAR                   = rosWriteIntensity(msg_LiDAR,(tmpPC.Intensity));
                    send(pub.LiDAR,msg_LiDAR);
                end
            end
            
            

            %-----------------------------------------------------------------------------------%
            %-------------------------------Object Detection Info-------------------------------%
            %-----------------------------------------------------------------------------------%
            % Results from 3D DL model
            L_bbox          = G_bbox;
            L_id            = G_id;
            L_cls           = G_cls;
            L_vel           = G_vel;
            L_isTracking    = G_isTracking;

            % Calculate Object Distance & Velocity  
            [Model, ModelInfo]          = HelperComputeDistance_KF(L_bbox, L_id, L_cls, L_vel, L_isTracking, ptCloud_ps);
            [VelocityInfo, OrientInfo]  = HelperComputeVelocity_KF(ModelInfo);
            
            %-----------------------------------------------------------------------------------%
            %-----------------------------------Visualization-----------------------------------%
            %-----------------------------------------------------------------------------------%
            % Display detection results
            view(player,ptCloud)
            HelperDeleteCuboid_KF(player.Axes)
            HelperDrawCuboid_KF(player.Axes, Model, ModelInfo, VelocityInfo, OrientInfo);

            xyzPointsBuffer = [];
            xyzIntensityBuffer = [];
        end
        frameCount = frameCount + 1;
        flush(Avia_UDP);
    end    

    reset_flag = single(1);
end
