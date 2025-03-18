clear; clc
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

%% 

% Set x,y,z range of pcplayer
xmin = 0;       xmax = 40;
ymin = -7;     ymax = 7;
zmin = -2;      zmax = 1.5;

% pointCloud viewer
player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",2);


colormap jet;
colorRange = jet(256);

bbox = load("test1015-scene1-set1.mat");
bbox = bbox.data';

%-----------------------------------------------------------------------------------%
%-------------------------------Calibration Parameter-------------------------------%
%-----------------------------------------------------------------------------------%

% Load LiDAR-Camera Calibration parameter - 연구실 PC 용
load('/media/aiv/새 볼륨/CTrack/cali.mat');

% Load LiDAR-Camera Calibration parameter - 연구실 노트북 용
% load('/home/aiv/YongJun_ws_dataset/dataset/cali.mat');

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

clusterThreshold = 0.4;   


i = 1;
while true
    
    ptCloud = pointCloud(rosReadXYZ(lidar_msg{i,1}),"Intensity",zeros(size(rosReadXYZ(lidar_msg{i,1}),1),1));
    img = rosReadImage(image_msg{i,1}); 
    
    [l_bboxes, ~, ~] = scenario_test_bbox(bbox{i,1});
    
    if ~isempty(l_bboxes) && ~isempty(ptCloud.Location)
       
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
    

        % frustumIndices = HelperBboxCameraToLidar_KF(l_bboxes, ptCloud, scene_camParams, scene_camToLidar,'ClusterThreshold', clusterThreshold, 'MaxDetectionRange', [1,60]);
        % if ~isempty(frustumIndices{1})
        %     allValues = vertcat(frustumIndices{:});
        %     uniqueValues = unique(allValues);
        % 
        %     tmpPC = select(ptCloud,uniqueValues);
        % end

        img = insertObjectAnnotation(img,"rectangle",l_bboxes, strcat({'Class:'},string(zeros(size(l_bboxes,1),1))'));
       
        imshow(img);
        hold on
        % scatter(imPts(:,1), imPts(:,2),3.5, pointColors(idx), 'filled');
        scatter(imPts(pointsInBboxMask, 1), imPts(pointsInBboxMask, 2), 3.5, colorsInBbox, 'filled');
        hold off
        
        view(player,pointsInBbox)
    else
        imshow(img);
    end
   
    i = i + 1;
    if i == view_sz + 1
        i = 1;
    end
    
    pause(0.05)
end