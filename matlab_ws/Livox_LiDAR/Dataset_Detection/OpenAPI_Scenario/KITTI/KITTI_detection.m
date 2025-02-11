%% ROS2 Node 
clear; clc

% 3D Detection global parameter
global G_bbox
global G_id
global G_cls
global G_vel
global G_isTracking


% Create a node for connection between MATLAB and ROS2
Pub_Node = ros2node("/IVL_Pub");
Sub_Node = ros2node("/IVL_Sub");


% Create Publish Node
pub.LiDAR = ros2publisher(Pub_Node,"/livox/lidar","sensor_msgs/PointCloud2");

% Create Subscribe Node
sub.lr_detection = ros2subscriber(Sub_Node,"/lr_detections","vision_msgs/Detection3DArray",@HelperCallbackPCDet_KF);

%% pointcloud dataset(.bin file)

% Clear memory cache
clear KITTI_HelperDrawCuboid_KF



% KITTI 
LiDAR_folder_path = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/training/velodyne';
Cam_folder_path = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/training/image_2';
Calib_folder_path = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/training/calib';
Label_folder_path = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/training/label_2';

LiDAR_file_list = dir(fullfile(LiDAR_folder_path, '*.bin'));
Cam_file_list = dir(fullfile(Cam_folder_path, '*.png'));
Calib_file_list = dir(fullfile(Calib_folder_path,'*.txt'));
Label_file_list = dir(fullfile(Label_folder_path,'*.txt'));



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

%-----------------------------------------------------------------------------------%


frame_times = [];


% 각 bin 파일 처리하기
for i = 1:length(LiDAR_file_list)
    tic

    % Load point cloud
    ptCld = KITTI_readPtCld(LiDAR_folder_path, LiDAR_file_list, i);  
    
    % Preprocessing point clound (ROI, Downsampling, remove ground)
    ptCloud_ps = HelperPtCldProcessing_KF(ptCld,roi, gridStep); 
    
    % Sending point cloud msg to ROS2 
    msg_LiDAR = ros2message(pub.LiDAR);
    msg_LiDAR.header.frame_id = 'map';
    msg_LiDAR = rosWriteXYZ(msg_LiDAR,(ptCloud_ps.Location));
    msg_LiDAR = rosWriteIntensity(msg_LiDAR,(ptCloud_ps.Intensity));
    send(pub.LiDAR,msg_LiDAR);

    %-----------------------------------------------------------------------------------%
    %-------------------------------Object Detection Info-------------------------------%
    %-----------------------------------------------------------------------------------%
    % Results from 3D DL model
    pause(0.1)
    L_bbox = G_bbox;
    L_id = G_id;
    L_cls = G_cls;
    L_vel = G_vel;
    L_isTracking = G_isTracking;

    % Calculate Object Distance & Velocity 
    [Model, ModelInfo] = HelperComputeDistance_KF(L_bbox, L_id, L_cls, L_vel, L_isTracking, ptCloud_ps);
    [VelocityInfo, OrientInfo] = HelperComputeVelocity_KF(ModelInfo);

    %-----------------------------------------------------------------------------------%
    %-----------------------------------Visualization-----------------------------------%
    %-----------------------------------------------------------------------------------%
    % Display detection results
    view(player,ptCld)
    HelperDeleteCuboid_KF(player.Axes)
    KITTI_HelperDrawCuboid_KF(player.Axes, Model, ModelInfo, VelocityInfo, OrientInfo);    

    % elapsed_time = toc;
    % frame_times = [frame_times, elapsed_time];
    % 
    % % 일정 시간이 지나면 Hz 계산
    % if length(frame_times) >= 100
    %     avg_time = mean(frame_times);  % 평균 프레임 처리 시간 계산
    %     avg_fps = 1 / avg_time;  % 초당 프레임 수 (Hz) 계산
    %     fprintf('Average FPS: %.2f Hz\n', avg_fps);
    %     frame_times = [];  % 시간 측정 배열 초기화
    % end
end


   



