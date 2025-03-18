%% ROS Node 
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
sub.lr_detection = ros2subscriber(Sub_Node,"/lr_detections","vision_msgs/Detection3DArray",@HelperCallbackPCDet_KF);

%% 
%-----------------------------------------------------------------------------------%
% 충북대 자율주행 테스트베드(c-Track) bag 파일 - 연구실 노트북 용   
%-----------------------------------------------------------------------------------%

% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina1/set1/rosbag2_2024_10_14-15_33_06_0.db3'; % test1014/sina1/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina1/set2/rosbag2_2024_10_14-15_34_15_0.db3'; % test1014/sina1/set2

% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina2/set1/rosbag2_2024_10_14-15_36_49_0.db3'; % test1014/sina2/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina2/set2/rosbag2_2024_10_14-15_37_45_0.db3'; % test1014/sina2/set2

% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina3/set1/rosbag2_2024_10_14-15_39_26_0.db3'; % test1014/sina3/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina3/set2/rosbag2_2024_10_14-15_40_12_0.db3'; % test1014/sina3/set2

% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina4/set1/rosbag2_2024_10_14-15_41_27_0.db3'; % test1014/sina4/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina5/set1/rosbag2_2024_10_14-15_42_51_0.db3'; % test1014/sina5/set1

% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1015/set1/rosbag2_2024_10_15-16_15_00_0.db3';       % test1015/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1015/set2/rosbag2_2024_10_15-16_15_56_0.db3';       % test1015/set2


%-----------------------------------------------------------------------------------%
% 충북대 자율주행 테스트베드(c-Track) bag 파일 - 연구실 PC 용   
%-----------------------------------------------------------------------------------%

% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina1/set1/rosbag2_2024_10_14-15_33_06_0.db3';   % test1014/sina1/set1
% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina1/set2/rosbag2_2024_10_14-15_34_15_0.db3'; % test1014/sina1/set2

% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina2/set1/rosbag2_2024_10_14-15_36_49_0.db3'; % test1014/sina2/set1
filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina2/set2/rosbag2_2024_10_14-15_37_45_0.db3'; % test1014/sina2/set2

% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina3/set1/rosbag2_2024_10_14-15_39_26_0.db3'; % test1014/sina3/set1
% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina3/set2/rosbag2_2024_10_14-15_40_12_0.db3'; % test1014/sina3/set2

% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina4/set1/rosbag2_2024_10_14-15_41_27_0.db3'; % test1014/sina4/set1
% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina5/set1/rosbag2_2024_10_14-15_42_51_0.db3'; % test1014/sina5/set1

% filePath = '/media/aiv/새 볼륨/CTrack/test_1015/set1/rosbag2_2024_10_15-16_15_00_0.db3';       % test1015/set1
% filePath = '/media/aiv/새 볼륨/CTrack/test_1015/set2/rosbag2_2024_10_15-16_15_56_0.db3';       % test1015/set2






% lidar, camera 데이터 읽기
bagReader   = ros2bagreader(filePath);
lidar_bag   = select(bagReader,"Topic","/livox/lidar");
rgb_bag     = select(bagReader,"Topic","/color/image_raw");

lidar_msg   = readMessages(lidar_bag);
image_msg   = readMessages(rgb_bag);


lidar_sz = length(lidar_msg);
image_sz = length(image_msg);

if lidar_sz > image_sz
    view_sz = image_sz;
else
    view_sz = lidar_sz;
end

%% 
clear HelperDrawCuboid__KF

%-----------------------------------------------------------------------------------%
%-----------------------------------Visualization-----------------------------------%
%-----------------------------------------------------------------------------------%
% % Set x,y,z range of pcplayer
xmin = 0;      xmax = 30;
ymin = -7;    ymax = 7;
zmin = -2;     zmax = 1.5;


% pointCloud viewer
player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",2);
           

G_bbox = [];
G_id = {};
G_cls = {};
G_vel = [];
G_isTracking = [];

frame_times = [];
i = 1;
%-----------------------------------------------------------------------------------%

%-----------------------------------------------------------------------------------%
%----------------------------preprocessing Parameter--------------------------------%
%-----------------------------------------------------------------------------------%
% ROI 설정
roi = [1, 20, -6, 6, -2, 1.3];     

% Downsample
gridStep = 0.1;
%-----------------------------------------------------------------------------------
%-----------------------------------------------------------------------------------



while true

    % tic

    % Load point cloud 
    ptCloud = pointCloud(rosReadXYZ(lidar_msg{i,1}),"Intensity",zeros(size(rosReadXYZ(lidar_msg{i,1}),1),1));
    
    % Preprocessing point clound (ROI, Downsampling, remove ground)
    ptCloud_ps = HelperPtCldProcessing_KF(ptCloud,roi,gridStep); 
    
    % Sending point cloud msg to ROS2 
    msg_LiDAR                   = ros2message(pub.LiDAR);
    msg_LiDAR.header.frame_id   = 'map';
    msg_LiDAR                   = rosWriteXYZ(msg_LiDAR,(ptCloud_ps.Location));
    msg_LiDAR                   = rosWriteIntensity(msg_LiDAR,(ptCloud_ps.Intensity));
    send(pub.LiDAR,msg_LiDAR);

    pause(0.04)
    
    i = i + 1;

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

    % % 시간 측정
    % elapsed_time = toc;  % 루프 끝난 시간 기록
    % frame_times = [frame_times, elapsed_time];  % 각 프레임 처리 시간을 기록
    % 
    % % 일정 시간이 지나면 Hz 계산
    % if length(frame_times) >= 100
    %     avg_time = mean(frame_times);  % 평균 프레임 처리 시간 계산
    %     avg_fps = 1 / avg_time;  % 초당 프레임 수 (Hz) 계산
    %     fprintf('Average FPS: %.2f Hz\n', avg_fps);
    %     frame_times = [];  % 시간 측정 배열 초기화
    % end
    
    

    % 반복 재생 
    if i == view_sz + 1
        i = 1;
    end

end


   



