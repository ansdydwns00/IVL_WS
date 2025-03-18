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

% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina1/set1/rosbag2_2024_10_14-15_33_06_0.db3'; % test1014/sina1/set1
% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina1/set2/rosbag2_2024_10_14-15_34_15_0.db3'; % test1014/sina1/set2

% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina2/set1/rosbag2_2024_10_14-15_36_49_0.db3'; % test1014/sina2/set1
% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina2/set2/rosbag2_2024_10_14-15_37_45_0.db3'; % test1014/sina2/set2

% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina3/set1/rosbag2_2024_10_14-15_39_26_0.db3'; % test1014/sina3/set1
% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina3/set2/rosbag2_2024_10_14-15_40_12_0.db3'; % test1014/sina3/set2

% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina4/set1/rosbag2_2024_10_14-15_41_27_0.db3'; % test1014/sina4/set1
% filePath = '/media/aiv/새 볼륨/CTrack/test_1014/sina5/set1/rosbag2_2024_10_14-15_42_51_0.db3'; % test1014/sina5/set1

filePath = '/media/aiv/새 볼륨/CTrack/test_1015/set1/rosbag2_2024_10_15-16_15_00_0.db3';       % test1015/set1
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

% Yolo 검출 정보 (frustum을 위한)
bbox = load("test1015-scene1-set1.mat");
bbox = bbox.data';

%-----------------------------------------------------------------------------------%
%-------------------------------Calibration Parameter-------------------------------%
%-----------------------------------------------------------------------------------%

% Load LiDAR-Camera Calibration parameter - 연구실 PC 용
load('/media/aiv/새 볼륨/CTrack/cali.mat');
% Load LiDAR-Camera Calibration parameter - 연구실 노트북 용
% load('/home/aiv/YongJun_ws_dataset/dataset/cali.mat');

focalLength     = [cali.intrinsic_color.intrinsic_matrix(1,1),cali.intrinsic_color.intrinsic_matrix(2,2)];
principalPoint  = [cali.intrinsic_color.intrinsic_matrix(1,3),cali.intrinsic_color.intrinsic_matrix(2,3)];
imageSize       = [1080,1920];
radialDist      = cali.intrinsic_color.radial_distortion_coef;
tangentialDist  = cali.intrinsic_color.tangential_distortion_coef;

R               = cali.extrinsic_lidar2color(1:3,1:3);
T               = cali.extrinsic_lidar2color(1:3,4);

[U,~,V]         = svd(R); % Singular Value Decomposition (SVD)
R_orthogonal    = U * V';

scene_camParams     = cameraIntrinsics(focalLength,principalPoint,imageSize,"RadialDistortion",radialDist,"TangentialDistortion",tangentialDist);
scene_lidarToCam    = rigidtform3d(R_orthogonal,T);
scene_camToLidar    = invert(scene_lidarToCam);

%% 

% Clear memory cache
clear HelperDrawCuboid_KF




%-----------------------------------------------------------------------------------%
%-----------------------------------Visualization-----------------------------------%
%-----------------------------------------------------------------------------------%
% Set x,y,z range of pcplayer
xmin = 0;       xmax = 40;
ymin = -7;     ymax = 7;
zmin = -2;      zmax = 1.5;

% pointCloud viewer
player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",2);

% image viewer
vPlayer = vision.DeployableVideoPlayer;


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
roi = [1, 40, -7, 7, -2, 1.3];     

% Downsampling
gridStep = 0.1;

i = 1;
%-----------------------------------------------------------------------------------%



while true
    % Load point cloud
    ptCloud = pointCloud(rosReadXYZ(lidar_msg{i,1}),"Intensity",zeros(size(rosReadXYZ(lidar_msg{i,1}),1),1));
    
    % Preprocessing point clound (ROI, Downsampling, remove ground)
    ptCloud_ps = HelperPtCldProcessing_KF(ptCloud,roi,gridStep); 
    
    % Load RGB image
    img = rosReadImage(image_msg{i,1}); 
    
    % Read Yolo Detection Info
    [l_bboxes, ~, ~] = scenario_test_bbox(bbox{i,1});
    

    if ~isempty(l_bboxes) && ~isempty(ptCloud_ps.Location)
        
        % Get the projected LiDAR points
        [imPts,idx] = projectLidarPointsOnImage(ptCloud_ps,scene_camParams,scene_lidarToCam);
        
        % Initialize mask to keep track of points inside the bounding boxes
        pointsInBboxMask = false(size(imPts, 1), 1);
       
        % Initialize cell arrays to keep track of points and intensities for each bounding box
        pointsInBboxCell = cell(size(l_bboxes, 1), 1);
        intensityInBboxCell = cell(size(l_bboxes, 1), 1);

        % Loop through each bounding box and check which points are inside
        for b = 1:size(l_bboxes, 1)
            
            bbox_x = l_bboxes(b, 1);
            bbox_y = l_bboxes(b, 2);
            bbox_w = l_bboxes(b, 3);
            bbox_h = l_bboxes(b, 4);

            % Create a logical mask for points inside the current bounding box
            inBoxMask = (imPts(:, 1) >= bbox_x) & (imPts(:, 1) <= (bbox_x + bbox_w)) & ...
                        (imPts(:, 2) >= bbox_y) & (imPts(:, 2) <= (bbox_y + bbox_h));

            % Extract the points and intensities for the current bounding box
            pointsInBboxCell{b} = ptCloud_ps.Location(idx(inBoxMask), :);
            intensityInBboxCell{b} = ptCloud_ps.Intensity(idx(inBoxMask), :);

            % Update the overall mask for points in any bounding box
            pointsInBboxMask = pointsInBboxMask | inBoxMask;
        end
        
        % Combine all points and intensities from all bounding boxes
        allPointsInBbox = vertcat(pointsInBboxCell{:});
        allIntensityInBbox = vertcat(intensityInBboxCell{:});
       
        % Create a new point cloud with the combined points and intensities
        projPtCld = pointCloud(allPointsInBbox,"Intensity",allIntensityInBbox);
       

        % Publish the point cloud to the ROS topic
        msg_LiDAR                   = ros2message(pub.LiDAR);
        msg_LiDAR.header.frame_id   = 'map';
        msg_LiDAR                   = rosWriteXYZ(msg_LiDAR,projPtCld.Location);
        msg_LiDAR                   = rosWriteIntensity(msg_LiDAR,projPtCld.Intensity);
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
    end
    
    %-----------------------------------------------------------------------------------%
    %-----------------------------------Visualization-----------------------------------%
    %-----------------------------------------------------------------------------------%
    % Display image results with bounding boxes and projected points
    if ~isempty(l_bboxes)
        img = insertObjectAnnotation(img,"rectangle",l_bboxes, strcat({'Class:'},string(zeros(size(l_bboxes,1),1))'));
        img = insertShape(img, "filled-circle", [imPts(pointsInBboxMask, 1), imPts(pointsInBboxMask, 2), repmat(2, sum(pointsInBboxMask), 1)]);
    end
    vPlayer.step(img)

    % Display detection results
    view(player,ptCloud)
    HelperDeleteCuboid_KF(player.Axes)
    HelperDrawCuboid_KF(player.Axes, Model, ModelInfo, VelocityInfo, OrientInfo);
    
    % Reset index if it exceeds the size
    if i == view_sz + 1
        i = 1;
    end
end



function upsampled_ptCld = upSampling(ptCld)

points = ptCld.Location;

% 업샘플링할 포인트 개수 지정
numSamples = size(points, 1); % 원래 포인트 수
numNewPoints = round(numSamples * 0.4); % 새로 생성할 포인트 수

% 새 포인트를 저장할 행렬 초기화
newPoints = zeros(numNewPoints, 3);

% k-Nearest Neighbors를 사용하여 평균 유클리드 거리 기반 반경 R 설정
k = 5; 
idx = knnsearch(points, points, 'K', k + 1); % 자기 자신 포함 k+1개의 이웃 찾기

% 각 포인트에 대해 평균 유클리드 거리 계산
averageDistances = zeros(numSamples, 1);
for i = 1:numSamples
    neighbors = points(idx(i, 2:end), :); % 자기 자신 제외한 이웃들
    distances = vecnorm(neighbors - points(i, :), 2, 2); % 유클리드 거리 계산
    averageDistances(i) = mean(distances); % 평균 거리 계산
end

% 기존 포인트 중 임의의 두 포인트를 선택하여 구형 보간
for i = 1:numNewPoints

    % 랜덤하게 한 개의 포인트 선택
    idx1 = randi(numSamples);
    point1 = points(idx1, :);

    % 반경 R을 평균 유클리드 거리로 설정
    R = averageDistances(idx1);

    % 구면 좌표계에서 랜덤 방향 생성
    theta = 2 * pi * rand(); % 0에서 2*pi 사이의 랜덤 각도
    phi = acos(2 * rand() - 1); % 0에서 pi 사이의 랜덤 각도

    % 구면 좌표를 데카르트 좌표로 변환하여 방향 벡터 생성
    direction = [R * sin(phi) * cos(theta), R * sin(phi) * sin(theta), R * cos(phi)];

    % 새로운 포인트 생성
    newPoints(i, :) = point1 + direction;
end

% 기존 포인트와 업샘플링된 포인트 합치기
upSampledPoints = [points; newPoints];

% 업샘플링된 포인트 클라우드 시각화
upsampled_ptCld = pointCloud(upSampledPoints,"Intensity",zeros(size(upSampledPoints,1),1));

end