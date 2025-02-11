%% ROS Node 
clear; clc

% 3D Detection global parameter
global G_bbox
global G_id
global G_cls
global G_isTracking
global vehiclePose

vehiclePose = [0 0 0];

% Create a node for connection between MATLAB and ROS2
Pub_Node = ros2node("/IVL_Pub");
Sub_Node = ros2node("/IVL_Sub");


% Create Publish Node
pub.LiDAR = ros2publisher(Pub_Node,"/livox/lidar","sensor_msgs/PointCloud2");

% Create Subscribe Node
sub.lr_detection = ros2subscriber(Sub_Node,"/lr_detections","vision_msgs/Detection3DArray",@HelperCallbackPCDet_TF);

%% 
clear HelperComputeVelocity_TF

%-----------------------------------------------------------------------------------%
%-----------------------------------Visualization-----------------------------------%
%-----------------------------------------------------------------------------------%
% Set x,y,z range of pcplayer
xmin = 0;      xmax = 40;
ymin = -40;    ymax = 40;
zmin = -2;     zmax = 1.3;

% pointCloud viewer
player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",4);
                      
%-----------------------------------------------------------------------------------%



%-----------------------------------------------------------------------------------%
%----------------------------preprocessing Parameter--------------------------------%
%-----------------------------------------------------------------------------------%
% ROI 설정
roi = [2, 40, -40, 40, -2, 1.3];     

% Downsample
gridStep = 0.15;
%-----------------------------------------------------------------------------------
%-----------------------------------------------------------------------------------%
% folder_path = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/ai_b05F';
folder_path = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/car_move/none_repetitive';
% folder_path = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/testing/velodyne';

file_list = dir(fullfile(folder_path, '*.bin'));



% 각 bin 파일 처리하기
for i = 1:length(file_list)
    % 현재 파일 경로 구성
    current_dir = fopen(fullfile(folder_path, file_list(i).name));
    
    current_file = single(fread(current_dir,[4 inf],'single')');
    % ptCloud = pointCloud(current_file(:,1:3),"Intensity",(current_file(:,4)));
    ptCloud = pointCloud(current_file(:,1:3),"Intensity",zeros(size(current_file(:,1:3),1),1));

    ptCloud = HelperPtCldProcessing_TF(ptCloud,roi,gridStep); 

    msg_LiDAR = ros2message(pub.LiDAR);
    msg_LiDAR.header.frame_id = 'map';
    msg_LiDAR = rosWriteXYZ(msg_LiDAR,(ptCloud.Location));
    msg_LiDAR = rosWriteIntensity(msg_LiDAR,(ptCloud.Intensity));
    send(pub.LiDAR,msg_LiDAR);

    pause(0.06)

    % Results from 3D DL model
    L_bbox = G_bbox;
    L_id = G_id;
    L_cls = G_cls;
    L_isTracking = G_isTracking;
    
    %-----------------------------------------------------------------------------------%
    %-------------------------------Object Detection Info-------------------------------%
    %-----------------------------------------------------------------------------------%
    % Calculate Object Distance 
    [Model, ModelInfo] = HelperComputeDistance_TF(L_bbox, L_id, L_cls, L_isTracking, ptCloud);
   
    % Calculate Object Velocity
    [VelocityInfo, OrientInfo] = HelperComputeVelocity_TF(ModelInfo,vehiclePose);
    %-----------------------------------------------------------------------------------%
    %-----------------------------------Visualization-----------------------------------%
    %-----------------------------------------------------------------------------------%
    % Display detection results
    view(player,ptCloud)
    HelperDeleteCuboid_TF(player.Axes)
    HelperDrawCuboid_TF(player.Axes, Model, ModelInfo, VelocityInfo, OrientInfo);  
end


   



