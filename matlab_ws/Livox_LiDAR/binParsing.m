%% ptCloud to bin file

% 저장하고자 하는 bin 파일의 이름
fld = fopen("ivl_nds.bin",'w');

% 저장하고자 하는 ptCloud 객체의 Location, Intensity
xyzi_point = [ptCloud.Location ptCloud.Intensity]';

fwrite(fld,xyzi_point,'single');
fclose(fld);
%% bin file to ptCloud

pt = fopen('000001.bin');
pt_raw = single(fread(pt,[4 inf],'single')');

ptCld = pointCloud(pt_raw(:,1:3),"Intensity",pt_raw(:,4));

figure;
pcshow(ptCld)
%% 


% 특정 경로의 파일 목록 가져오기
folder_path = '/home/aiv/pcdet_ws/src/OpenPCDet/data/kitti/training/velodyne';
file_list = dir(fullfile(folder_path, '*.bin'));
player = pcplayer([-70 70],[-40 40],[-3 1],"MarkerSize",10,"ColorSource","X");


% 각 bin 파일 처리하기
for i = 1:length(file_list)

    % 현재 파일 경로 구성
    current_dir = fopen(fullfile(folder_path, file_list(i).name));
    current_file = single(fread(current_dir,[4 inf],'single')');
    
    ptCloud = pointCloud(current_file(:,1:3),"Intensity",current_file(:,4));
    
    
    % 포인트 클라우드 플롯
    view(player,ptCloud);
    pause(1.5)
end



