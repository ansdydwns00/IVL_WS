function [img,h,w] = KITTI_readImage(folder_path,file_list,idx)

img_file = fullfile(folder_path, file_list(idx).name);
img = imread(img_file);

h = size(img,1);
w = size(img,2);

end