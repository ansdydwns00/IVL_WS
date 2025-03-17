%% ROS2 Node 
clear; clc

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
Pub_Node = ros2node("/IVL_Pub");
Sub_Node = ros2node("/IVL_Sub");


% Create Publish Node
pub.LiDAR = ros2publisher(Pub_Node,"/livox/lidar","sensor_msgs/PointCloud2");

% Create Subscribe Node
sub.lr_detection = ros2subscriber(Sub_Node,"/lr_detections","vision_msgs/Detection3DArray",@KITTI_HelperCallbackPCDet_KF);

%%

% Clear memory cache
clear KITTI_HelperDrawCuboid_KF



% KITTI 3D Object Detection Dataset (실행 전 '새 볼륨' 연결 필요) - 연구실 본체 용
LiDAR_folder_path   = '/media/aiv/새 볼륨/kitti/kitti_3d/training/velodyne';
Cam_folder_path     = '/media/aiv/새 볼륨/kitti/kitti_3d/training/image_2';
Calib_folder_path   = '/media/aiv/새 볼륨/kitti/kitti_3d/training/calib';
Label_folder_path   = '/media/aiv/새 볼륨/kitti/kitti_3d/training/label_2';

output_folder = '/home/aiv/openPCD_ws/src/OpenPCDet/data/custom/kitti/training/frustum';
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end


% KITTI 3D Object Detection Dataset (실행 전 '새 볼륨' 연결 필요) - 연구실 노트북 용
% LiDAR_folder_path   = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/training/velodyne';
% Cam_folder_path     = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/training/image_2';
% Calib_folder_path   = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/training/calib';
% Label_folder_path   = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/training/label_2';

% output_folder = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/training/velodyne_frustum';
% if ~exist(output_folder, 'dir')
%     mkdir(output_folder);
% end


LiDAR_file_list = dir(fullfile(LiDAR_folder_path, '*.bin'));
Cam_file_list = dir(fullfile(Cam_folder_path, '*.png'));
Calib_file_list = dir(fullfile(Calib_folder_path,'*.txt'));
Label_file_list = dir(fullfile(Label_folder_path,'*.txt'));


% Load KITTI image data with YOLO
yolo_result = load("YOLO11m.mat").data';






%-----------------------------------------------------------------------------------%
%-----------------------------------Visualization-----------------------------------%
%-----------------------------------------------------------------------------------%
% Set x,y,z range of pcplayer
xmin = 0;       xmax = 70;
ymin = -30;     ymax = 30;
zmin = -2;      zmax = 1.5;

% pointCloud viewer
player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",2);

% image viewer
% vPlayer = vision.DeployableVideoPlayer;


G_bbox = [];
G_id = {};
G_cls = {};
G_vel = [];
G_isTracking = [];
%-----------------------------------------------------------------------------------%


%-----------------------------------------------------------------------------------%
%------------------------------Clustering Parameter---------------------------------%
%-----------------------------------------------------------------------------------%
% ROI setting
roi = [1, 70, -30, 30, -2, 1.5];     

% Downsampling
gridStep = 0.1;

% Cluster distance 
clusterThreshold = 0.4;   
%-----------------------------------------------------------------------------------%


fig = figure;


for i = 1:length(LiDAR_file_list)

    % Load and process image
    [img,h,w] = KITTI_readImage(Cam_folder_path,Cam_file_list,i);
    
    % Process KITTI GT results
    % [bbox, class] = KITTI_readBbox(Label_folder_path, Label_file_list, i);
    
    % Process YOLO results
    [bbox, class] = KITTI_scenario_test_bbox(yolo_result{i,1});

    % Load calibration data
    [camParams, lidarToCam, CamToLidar] = KITTI_readCalib(Calib_folder_path, Calib_file_list, h, w, i);

    % Load point cloud
    ptCld = KITTI_readPtCld(LiDAR_folder_path, LiDAR_file_list, i);  
   
    % Project points into the image
    [imPts, idx] = KITTI_projection(ptCld, img, class, bbox, camParams, lidarToCam,fig);
    
    % Frustum selection
    if ~isempty(bbox) && ~isempty(ptCld.Location)
        frustumIndices = KITTI_HelperBboxCameraToLidar_KF(bbox, ptCld, camParams, CamToLidar,'ClusterThreshold', clusterThreshold, 'MaxDetectionRange', [1,80]);
        uniqueValues = unique(vertcat(frustumIndices{:}));
        frustumPtCld = select(ptCld,uniqueValues);
    else
        frustumPtCld = ptCld;
    end
    
    msg_LiDAR                   = ros2message(pub.LiDAR);
    msg_LiDAR.header.frame_id   = 'map';
    msg_LiDAR                   = rosWriteXYZ(msg_LiDAR,(frustumPtCld.Location));
    msg_LiDAR                   = rosWriteIntensity(msg_LiDAR,(frustumPtCld.Intensity));
    send(pub.LiDAR,msg_LiDAR);
    

    %-----------------------------------------------------------------------------------%
    %-------------------------------Object Detection Info-------------------------------%
    %-----------------------------------------------------------------------------------%
    % Results from 3D DL model
    pause(0.1)
    L_bbox          = G_bbox;
    L_id            = G_id;
    L_cls           = G_cls;
    L_vel           = G_vel;
    L_isTracking    = G_isTracking;

    % Calculate Object Distance & Velocity 
    [Model, ModelInfo]          = KITTI_HelperComputeDistance_KF(L_bbox, L_id, L_cls, L_vel, L_isTracking, frustumPtCld);
    [VelocityInfo, OrientInfo]  = KITTI_HelperComputeVelocity_KF(ModelInfo);

    

    % Check if enough points are available for saving
    % savePtCld = frustumPtCld;
    % if isempty(frustumPtCld.Location) || size(frustumPtCld.Location, 1) < 20 
    %     savePtCld = ptCld;
    % end
    
    % % Save point cloud
    % filename = sprintf('%06d.bin', i - 1);
    % filepath = fullfile(output_folder, filename);
    % data = [savePtCld.Location, savePtCld.Intensity];
    % fileID = fopen(filepath, 'wb');
    % fwrite(fileID, data', 'float32');  % Save as float32
    % fclose(fileID);
    % 
    % fprintf("Saved point cloud %s\n", filepath);


    view(player,ptCld)
    KITTI_HelperDeleteCuboid_KF(player.Axes)
    KITTI_HelperDrawCuboid_KF(player.Axes, Model, ModelInfo, VelocityInfo, OrientInfo);


    pause(5)
end


