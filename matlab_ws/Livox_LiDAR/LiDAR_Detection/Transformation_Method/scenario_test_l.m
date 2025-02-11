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

% filePath = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/test_1014/sina1/set1/rosbag2_2024_10_14-15_33_06_0.db3';
% filePath = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/test_1014/sina1/set2/rosbag2_2024_10_14-15_34_15_0.db3';

filePath = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/test_1014/sina2/set1/rosbag2_2024_10_14-15_36_49_0.db3';
% filePath = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/test_1014/sina2/set2/rosbag2_2024_10_14-15_37_45_0.db3';
% 
% filePath = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/test_1014/sina3/set1/rosbag2_2024_10_14-15_39_26_0.db3';
% filePath = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/test_1014/sina3/set2/rosbag2_2024_10_14-15_40_12_0.db3';
% 
% filePath = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/test_1014/sina4/set1/rosbag2_2024_10_14-15_41_27_0.db3';
% 
% filePath = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/test_1014/sina5/set1/rosbag2_2024_10_14-15_42_51_0.db3';

bagReader = ros2bagreader(filePath);

% lidar, camera 데이터 읽기

lidar_bag = select(bagReader,"Topic","/livox/lidar");
rgb_bag = select(bagReader,"Topic","/color/image_raw");

lidar_msg = readMessages(lidar_bag);
image_msg = readMessages(rgb_bag);

%% 
clear HelperComputeVelocity_TF



%-----------------------------------------------------------------------------------%
%-----------------------------------Visualization-----------------------------------%
%-----------------------------------------------------------------------------------%
% Set x,y,z range of pcplayer
xmin = 0;      xmax = 40;
ymin = -10;    ymax = 10;
zmin = -2;     zmax = 1.5;

% pointCloud viewer
player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",6);
                      
%-----------------------------------------------------------------------------------%



%-----------------------------------------------------------------------------------%
%----------------------------preprocessing Parameter--------------------------------%
%-----------------------------------------------------------------------------------%
% ROI 설정
roi = [1, 40, -6, 6, -2, 1.3];     

% Downsample
gridStep = 0.1;
%-----------------------------------------------------------------------------------
%-----------------------------------------------------------------------------------





% 각 bin 파일 처리하기
for i = 1:length(lidar_msg)

    % ptCloud = pointCloud(rosReadXYZ(lidar_msg{i,1}),"Intensity",(rosReadField(lidar_msg{i,1},'intensity')));
    ptCloud = pointCloud(rosReadXYZ(lidar_msg{i,1}),"Intensity",zeros(size(rosReadXYZ(lidar_msg{i,1}),1),1));

    ptCloud_ps = HelperPtCldProcessing_TF(ptCloud,roi,gridStep); 

    msg_LiDAR = ros2message(pub.LiDAR);
    msg_LiDAR.header.frame_id = 'map';
    msg_LiDAR = rosWriteXYZ(msg_LiDAR,(ptCloud_ps.Location));
    msg_LiDAR = rosWriteIntensity(msg_LiDAR,(ptCloud_ps.Intensity));
    send(pub.LiDAR,msg_LiDAR);

    pause(0.05)

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


   



