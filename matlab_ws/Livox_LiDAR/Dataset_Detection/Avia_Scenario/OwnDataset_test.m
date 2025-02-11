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

% 대양 ai센터 지하5층 실험 데이터
% folder_path = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/ai_b05F';
% folder_path = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/car_move/none_repetitive';
% folder_path = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/car_move/repetitive';
folder_path = '/home/aiv/YongJun_ws_dataset/dataset/car_stop/none_repetitive';
% folder_path = '/home/aiv/YongJun_ws/matlab_ws/Livox_LiDAR/dataset/car_stop/repetitive';



file_list = dir(fullfile(folder_path, '*.bin'));

%%

clear HelperDrawCuboid_KF

VehiclePose = [0,0,0,0,0,0];

%-----------------------------------------------------------------------------------%
%-----------------------------------Visualization-----------------------------------%
%-----------------------------------------------------------------------------------%
% Set x,y,z range of pcplayer
xmin = 0;      xmax = 40;
ymin = -40;    ymax = 40;
zmin = -2;     zmax = 1.3;

% pointCloud viewer
player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",4);
                      
L_bbox = [];
L_id = [];
L_cls = [];
%-----------------------------------------------------------------------------------%



%-----------------------------------------------------------------------------------%
%----------------------------preprocessing Parameter--------------------------------%
%-----------------------------------------------------------------------------------%
% ROI 설정
roi = [2, 40, -40, 40, -2, 1.3];     

% Downsample
gridStep = 0.1;
%-----------------------------------------------------------------------------------
%-----------------------------------------------------------------------------------%

frame_times = [];


% 각 bin 파일 처리하기
for i = 1:length(file_list)
    tic

    % 현재 파일 경로 구성
    current_dir = fopen(fullfile(folder_path, file_list(i).name));

    current_file = single(fread(current_dir,[4 inf],'single')');

    % ptCloud = pointCloud(current_file(:,1:3),"Intensity",(current_file(:,4)));
    ptCloud = pointCloud(current_file(:,1:3),"Intensity",zeros(size(current_file(:,1:3),1),1));

    % Preprocessing point clound (ROI, Downsampling, remove ground)
    ptCloud_ps = HelperPtCldProcessing_KF(ptCloud,roi,gridStep); 

    % Sending point cloud msg to ROS2 
    msg_LiDAR = ros2message(pub.LiDAR);
    msg_LiDAR.header.frame_id = 'map';
    msg_LiDAR = rosWriteXYZ(msg_LiDAR,(ptCloud_ps.Location));
    msg_LiDAR = rosWriteIntensity(msg_LiDAR,(ptCloud_ps.Intensity));
    send(pub.LiDAR,msg_LiDAR);

    % Results from 3D DL model
    pause(0.06)
    L_bbox = G_bbox;
    L_id = G_id;
    L_cls = G_cls;
    L_vel = G_vel;
    L_isTracking = G_isTracking;

    %-----------------------------------------------------------------------------------%
    %-------------------------------Object Detection Info-------------------------------%
    %-----------------------------------------------------------------------------------%
    % Calculate Object Distance 
    [Model, ModelInfo] = HelperComputeDistance_KF(L_bbox, L_id, L_cls, L_vel, L_isTracking, ptCloud_ps);

    % Calculate Object Velocity
    [VelocityInfo, OrientInfo] = HelperComputeVelocity_KF(ModelInfo);

    %-----------------------------------------------------------------------------------%
    %-----------------------------------Visualization-----------------------------------%
    %-----------------------------------------------------------------------------------%
    % Display detection results
    view(player,ptCloud)
    HelperDeleteCuboid_KF(player.Axes)
    HelperDrawCuboid_KF(player.Axes, Model, ModelInfo, VelocityInfo, OrientInfo);    

    elapsed_time = toc;
    frame_times = [frame_times, elapsed_time];

    % 일정 시간이 지나면 Hz 계산
    if length(frame_times) >= 100
        avg_time = mean(frame_times);  % 평균 프레임 처리 시간 계산
        avg_fps = 1 / avg_time;  % 초당 프레임 수 (Hz) 계산
        fprintf('Average FPS: %.2f Hz\n', avg_fps);
        frame_times = [];  % 시간 측정 배열 초기화
    end

end


