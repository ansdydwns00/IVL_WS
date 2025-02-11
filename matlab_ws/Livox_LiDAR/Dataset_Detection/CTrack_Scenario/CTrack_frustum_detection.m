%% ROS Node 
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
%% 

% test1014/sina1/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina1/set1/rosbag2_2024_10_14-15_33_06_0.db3';
% % test1014/sina1/set2
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina1/set2/rosbag2_2024_10_14-15_34_15_0.db3';

 
% % test1014/sina2/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina2/set1/rosbag2_2024_10_14-15_36_49_0.db3';
% % test1014/sina2/set2
filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina2/set2/rosbag2_2024_10_14-15_37_45_0.db3';


% % test1014/sina3/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina3/set1/rosbag2_2024_10_14-15_39_26_0.db3';
% % test1014/sina3/set2
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina3/set2/rosbag2_2024_10_14-15_40_12_0.db3';


% % test1014/sina4/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina4/set1/rosbag2_2024_10_14-15_41_27_0.db3';
% % test1014/sina5/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1014/sina5/set1/rosbag2_2024_10_14-15_42_51_0.db3';

% % test1015/set1
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1015/set1/rosbag2_2024_10_15-16_15_00_0.db3';
% % test1015/set2
% filePath = '/home/aiv/YongJun_ws_dataset/dataset/test_1015/set2/rosbag2_2024_10_15-16_15_56_0.db3';

bagReader = ros2bagreader(filePath);


% lidar, camera 데이터 읽기
lidar_bag = select(bagReader,"Topic","/livox/lidar");
rgb_bag = select(bagReader,"Topic","/color/image_raw");

lidar_msg = readMessages(lidar_bag);
image_msg = readMessages(rgb_bag);

lidar_sz = length(lidar_msg);
image_sz = length(image_msg);

if lidar_sz > image_sz
    view_sz = image_sz;
else
    view_sz = lidar_sz;
end

bbox = load("test1014-scene2-set2.mat");
bbox = bbox.data';
%% 
clear HelperDrawCuboid_KF
%-----------------------------------------------------------------------------------%
%-------------------------------Calibration Parameter-------------------------------%
%-----------------------------------------------------------------------------------%

% Load LiDAR-Camera Calibration parameter
load('/home/aiv/YongJun_ws_dataset/dataset/cali.mat');

focalLength = [cali.intrinsic_color.intrinsic_matrix(1,1),cali.intrinsic_color.intrinsic_matrix(2,2)];
principalPoint = [cali.intrinsic_color.intrinsic_matrix(1,3),cali.intrinsic_color.intrinsic_matrix(2,3)];
imageSize = [1080,1920];
radialDist = cali.intrinsic_color.radial_distortion_coef;
tangentialDist = cali.intrinsic_color.tangential_distortion_coef;

R = cali.extrinsic_lidar2color(1:3,1:3);
T = cali.extrinsic_lidar2color(1:3,4);

[U,~,V] = svd(R); % Singular Value Decomposition (SVD)
R_orthogonal = U * V';

scene_camParams = cameraIntrinsics(focalLength,principalPoint,imageSize,"RadialDistortion",radialDist,"TangentialDistortion",tangentialDist);
scene_lidarToCam = rigidtform3d(R_orthogonal,T);
scene_camToLidar = invert(scene_lidarToCam);


%-----------------------------------------------------------------------------------%
%-----------------------------------Visualization-----------------------------------%
%-----------------------------------------------------------------------------------%
% Set x,y,z range of pcplayer
xmin = 0;       xmax = 40;
ymin = -7;     ymax = 7;
zmin = -2;      zmax = 1.5;

% pointCloud viewer
player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",2);



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

% Cluster distance 
clusterThreshold = 0.4;   

%-----------------------------------------------------------------------------------%



i = 1;
colormap jet;
colorRange = jet(256);


while true

    ptCloud = pointCloud(rosReadXYZ(lidar_msg{i,1}),"Intensity",zeros(size(rosReadXYZ(lidar_msg{i,1}),1),1));

    ptCloud_ps = HelperPtCldProcessing_KF(ptCloud,roi,gridStep); 
    
    [l_bboxes, ~,~] = scenario_test_bbox(bbox{i,1});
    
    img = rosReadImage(image_msg{i,1}); 

    i = i + 1;

    if ~isempty(l_bboxes) && ~isempty(ptCloud_ps.Location)
        frustumIndices = HelperBboxCameraToLidar_KF(l_bboxes, ptCloud_ps, scene_camParams, scene_camToLidar,'ClusterThreshold', clusterThreshold, 'MaxDetectionRange', [1,60]);

        if ~isempty(frustumIndices{1})

            allValues = vertcat(frustumIndices{:});
            uniqueValues = unique(allValues);

            ptCloud_frustum = select(ptCloud_ps,uniqueValues);
            msg_LiDAR = ros2message(pub.LiDAR);
            msg_LiDAR.header.frame_id = 'map';
            msg_LiDAR = rosWriteXYZ(msg_LiDAR,(ptCloud_frustum.Location));
            msg_LiDAR = rosWriteIntensity(msg_LiDAR,(ptCloud_frustum.Intensity));
            send(pub.LiDAR,msg_LiDAR);
        end

        % Create Colormap of Pointcloud
        heights = ptCloud.Location(:,1);

        minHeight = min(heights); % 최소 높이 값
        maxHeight = max(heights); % 최대 높이 값
        normalizedHeights = (heights - minHeight) / (maxHeight - minHeight); % 높이 값을 0과 1 사이로 정규화
        colorIndices = ceil(normalizedHeights * 255) + 1;
        colorIndices(colorIndices > 256) = 256;
        colorIndices(colorIndices < 1) = 1;
        pointColors = colorRange(colorIndices, :);

        [imPts,idx] = projectLidarPointsOnImage(ptCloud,scene_camParams,scene_lidarToCam);

        % Initialize mask to keep track of points inside the bounding boxes
        pointsInBboxMask = false(size(imPts, 1), 1);

        % Loop through each bounding box and check which points are inside
        for b = 1:size(l_bboxes, 1)
            bbox_x = l_bboxes(b, 1);
            bbox_y = l_bboxes(b, 2);
            bbox_w = l_bboxes(b, 3);
            bbox_h = l_bboxes(b, 4);

            % Create a logical mask for points inside the current bounding box
            inBoxMask = (imPts(:, 1) >= bbox_x) & (imPts(:, 1) <= (bbox_x + bbox_w)) & ...
                        (imPts(:, 2) >= bbox_y) & (imPts(:, 2) <= (bbox_y + bbox_h));

            % Update the overall mask for points in any bounding box
            pointsInBboxMask = pointsInBboxMask | inBoxMask;
        end

        % Extract the points that are inside any bounding box
        pointsInBbox = ptCloud.Location(idx(pointsInBboxMask), :);

        % Extract the colors for those points
        colorsInBbox = pointColors(idx(pointsInBboxMask), :);
    end
    
    pause(0.04)

    % Results from 3D DL model
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
    
    if ~isempty(l_bboxes)
        img = insertObjectAnnotation(img,"rectangle",l_bboxes, repmat("Pedestrian",size(l_bboxes,1),1));
    end

   
    % Display detection results
    view(player,ptCloud)
    HelperDeleteCuboid_KF(player.Axes)
    HelperDrawCuboid_KF(player.Axes, Model, ModelInfo, VelocityInfo, OrientInfo);
    
    % imshow(img);
    % hold on;
    % % scatter(imPts(:, 1), imPts(:, 2), 2, pointColors(idx), 'filled');
    % scatter(imPts(pointsInBboxMask, 1), imPts(pointsInBboxMask, 2), 2, colorsInBbox, 'filled');
    % hold off;

    if i == view_sz + 1
        i = 1;
    end
end
