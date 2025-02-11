function ptCld = KITTI_readPtCld(folder_path, file_list, idx)
 
lidar_file = fullfile(folder_path,file_list(idx).name);
fileID = fopen(lidar_file,'r');

raw_ptCloud = fread(fileID,[4, inf], 'single')';
ptCld = pointCloud(raw_ptCloud(:,1:3),"Intensity",raw_ptCloud(:,4));
fclose(fileID);

end