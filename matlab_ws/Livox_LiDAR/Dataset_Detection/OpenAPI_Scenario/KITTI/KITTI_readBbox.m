function [bbox_filtered, class_filtered] = KITTI_readBbox(folder_path, file_list, idx)
    % 파일 경로 설정
    label_file = fullfile(folder_path, file_list(idx).name);
    
    % 파일 열기 및 레이블 데이터 읽기
    fileID = fopen(label_file, 'r');
    labels = textscan(fileID, '%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
    fclose(fileID);
    
    % 전체 클래스와 바운딩 박스 정보 가져오기
    class_all = labels{1};
    bbox_all = [labels{5}, labels{6}, labels{7}, labels{8}]; % x_min, y_min, x_max, y_max
    
    % 필터링할 클래스 정의
    target_classes = {'Car', 'Pedestrian', 'Cyclist'};
    
    % 클래스와 bbox 초기화
    class_filtered = {};
    bbox_filtered = [];
    
    % 클래스 필터링
    for i = 1:length(class_all)
        if ismember(class_all{i}, target_classes)
            class_filtered{end+1, 1} = class_all{i};  % 해당 클래스 추가

            % 좌표 변환: (x_min, y_min, x_max, y_max) -> (x, y, width, height)
            x_min = bbox_all(i, 1);
            y_min = bbox_all(i, 2);
            x_max = bbox_all(i, 3);
            y_max = bbox_all(i, 4);
            
            width = x_max - x_min;
            height = y_max - y_min;
            
            bbox_filtered(end+1, :) = [x_min, y_min, width, height];  % 변환된 bbox 추가
        end
    end
end