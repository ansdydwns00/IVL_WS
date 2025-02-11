function [camParams,lidarToCam,CamToLidar] = KITTI_readCalib(folder_path,file_list,h,w,idx)

calib_file = fullfile(folder_path,file_list(idx).name);

% Open the calibration file and read its contents
fileID = fopen(calib_file,'r');
calib_data = textscan(fileID, '%s', 'Delimiter', '\n');
fclose(fileID);

% Initialize empty matrices for intrinsic and extrinsic parameters
P2 = []; % Camera matrix for camera 2 (Intrinsic)
R0 = []; % Rectification matrix
Tr = []; % Extrinsic matrix from LiDAR to camera

% Parse each line and extract the necessary matrices
for i = 1:length(calib_data{1})
    line = calib_data{1}{i};
    split_line = strsplit(line, ' ');
    
    % Check for P2 (Camera intrinsic matrix)
    if strcmp(split_line{1}, 'P2:')
        P2 = reshape(str2double(split_line(2:end)), [4, 3])';
    end
    
    % Check for R0_rect (Rectification matrix)
    if strcmp(split_line{1}, 'R0_rect:')
        R0 = eye(4);
        R0(1:3,1:3) = reshape(str2double(split_line(2:end)), [3, 3])';
    end

    % Check for Tr_velo_to_cam (Extrinsic matrix from LiDAR to Camera)
    if strcmp(split_line{1}, 'Tr_velo_to_cam:')
        Tr = reshape(str2double(split_line(2:end)), [4, 3])';
        Tr(4,:) = [0 0 0 1]; % Add last row for homogeneous coordinates
    end
end

camParams = cameraIntrinsics([P2(1,1),P2(2,2)],[P2(1,3),P2(2,3)],[h,w]);

R = Tr(1:3, 1:3);
T = Tr(1:3,4);

[U, ~, V] = svd(R);
R_orthogonal = U * V';

% if det(R) < 0
%     R = U * diag([1, 1, -1]) * V';
% end
% 
% Tr(1:3, 1:3) = R;
% 
% lidarToCam = rigidtform3d(Tr(1:3,1:3), Tr(1:3,4));
lidarToCam = rigidtform3d(R_orthogonal,T);
CamToLidar = invert(lidarToCam);

end
