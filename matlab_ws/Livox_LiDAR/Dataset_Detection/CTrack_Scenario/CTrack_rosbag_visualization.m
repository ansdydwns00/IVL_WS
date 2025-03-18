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


%% LiDAR and Camera rosbag data check

%-----------------------------------------------------------------------------------%
%-----------------------------------Visualization-----------------------------------%
%-----------------------------------------------------------------------------------%
% Set x,y,z range of pcplayer
xmin = 0;      xmax = 40;
ymin = -7;    ymax = 7;
zmin = -2;     zmax = 1;

% pointCloud viewer
player = pcplayer([xmin xmax],[ymin ymax],[zmin zmax],"ColorSource","X","MarkerSize",4);
vPlayer = vision.DeployableVideoPlayer;
               
%-----------------------------------------------------------------------------------%


i = 1;

while true

    img = rosReadImage(image_msg{i,1});
    pc = pointCloud(rosReadXYZ(lidar_msg{i,1}),"Intensity",rosReadField(lidar_msg{i,1},'intensity'));

    view(player,pc)
    vPlayer.step(img)

    i = i + 1;
    if i == view_sz + 1
        i = 1;
    end

    pause(0.01)
end


%% LiDAR and Camera Calibration Result check

% Load LiDAR-Camera Calibration parameter - 연구실 PC 용
load('/media/aiv/새 볼륨/CTrack/cali.mat');

% Load LiDAR-Camera Calibration parameter - 연구실 노트북 용
% load('/home/aiv/YongJun_ws_dataset/dataset/cali.mat');


focalLength = [cali.intrinsic_color.intrinsic_matrix(1,1),cali.intrinsic_color.intrinsic_matrix(2,2)];
principalPoint = [cali.intrinsic_color.intrinsic_matrix(1,3),cali.intrinsic_color.intrinsic_matrix(2,3)];
imageSize = [1080,1920];
radialDist = cali.intrinsic_color.radial_distortion_coef;
tangentialDist = cali.intrinsic_color.tangential_distortion_coef;

scene_camParams = cameraIntrinsics(focalLength,principalPoint,imageSize,"RadialDistortion",radialDist,"TangentialDistortion",tangentialDist);


R = cali.extrinsic_lidar2color(1:3,1:3);
T = cali.extrinsic_lidar2color(1:3,4);

[U,~,V] = svd(R); % Singular Value Decomposition (SVD)
R_orthogonal = U * V';

scene_lidarToCam = rigidtform3d(R_orthogonal,T);
scene_camToLidar = invert(scene_lidarToCam);

 

i = 1;
colormap jet;
colorRange = jet(256);

while true
    
    img = rosReadImage(image_msg{i,1});
    pc = pointCloud(rosReadXYZ(lidar_msg{i,1}),"Intensity",rosReadField(lidar_msg{i,1},'intensity'));
            
    % Create Colormap of Pointcloud
    heights = pc.Location(:,1);
   
    minHeight = min(heights); % 최소 높이 값
    maxHeight = max(heights); % 최대 높이 값
    normalizedHeights = (heights - minHeight) / (maxHeight - minHeight); % 높이 값을 0과 1 사이로 정규화
    colorIndices = ceil(normalizedHeights * 255) + 1;
    colorIndices(colorIndices > 256) = 256;
    colorIndices(colorIndices < 1) = 1;
    pointColors = colorRange(colorIndices, :);
   

    [imPts,idx] = projectLidarPointsOnImage(pc,scene_camParams,scene_lidarToCam);
    
    l_img = insertShape(img,"filled-circle",[imPts(:,1), imPts(:,2), repmat(2,size(imPts,1), 1)],"ShapeColor",pointColors(idx,:,:));
    
    imshow(l_img);
    hold on
    scatter(imPts(:,1), imPts(:,2),4, pointColors(idx), 'filled');
    hold off

    i = i + 1;
    if i == view_sz + 1
        i = 1;
    end

    pause(0.001)
end
