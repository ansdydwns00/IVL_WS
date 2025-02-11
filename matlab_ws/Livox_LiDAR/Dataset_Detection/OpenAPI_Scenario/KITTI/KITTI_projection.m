function [imPts, idx] = KITTI_projection(ptCld, img, class, bbox, camParams, lidarToCam,figHandle)


% Classes of interest
target_classes = {'Car', 'Pedestrian', 'Cyclist'};


colormap jet;
colorRange = jet(256);


% 거리 계산 및 색상 매핑
distances = sqrt(sum(ptCld.Location(:, 1:3).^2, 2));  % 각 포인트의 유클리드 거리 계산
minDist = min(distances);
maxDist = max(distances);
normalizedDistances = (distances - minDist) / (maxDist - minDist);  % 거리값을 0-1로 정규화

colorIndices = ceil(normalizedDistances * 255) + 1;
colorIndices(colorIndices > 256) = 256;
colorIndices(colorIndices < 1) = 1;
pointColors = colorRange(colorIndices, :);


[imPts, idx] = projectLidarPointsOnImage(ptCld, camParams, lidarToCam);

% Figure가 이미 있으면 초기화, 없으면 새로 생성
if isvalid(figHandle)
    figure(figHandle);
    clf; % 기존 figure 초기화
else
    figHandle = figure; % 새로운 figure 생성
end

imshow(img);
hold on;
scatter(imPts(:, 1), imPts(:, 2), 5, pointColors(idx, :), 'filled');
hold off;

% Draw bounding boxes
for i = 1:size(bbox,1)
    if ismember(class{i}, target_classes)
        % Define the bounding box rectangle
        rectangle('Position', [bbox(i,1), bbox(i,2), bbox(i,3), bbox(i,4)], ...
                  'EdgeColor', 'g', 'LineWidth', 2);
        % Display class label
        text(bbox(i,1), bbox(i,2) - 10, class{i}, 'Color', 'white', 'FontSize', 8, 'FontWeight', 'bold');
    end
end


end