function KITTI_HelperDrawCuboid_KF(ax, cuboids, modelInfo, velInfo, orientInfo)

persistent idColorMap colorIndex colorList idBBoxMap idFrameCounterMap


if isempty(orientInfo) || isempty(modelInfo) || (length(cuboids) ~= length(modelInfo.Distance))
    return;
end


%--------------------------
% VelInfo / Rotation Matrix 캐시
%--------------------------
hasVelInfo = ~isempty(velInfo) && (size(velInfo, 1) == size(cuboids, 1));
RxCache = zeros(3, 3, size(cuboids, 1));
RyCache = zeros(3, 3, size(cuboids, 1));
RzCache = zeros(3, 3, size(cuboids, 1));


for i = 1:size(cuboids, 1)
    x_rot = cuboids{i}.Orientation(1);
    y_rot = cuboids{i}.Orientation(2);
    z_rot = cuboids{i}.Orientation(3);

    RxCache(:, :, i) = [1 0 0; 0 cos(x_rot) -sin(x_rot); 0 sin(x_rot) cos(x_rot)];
    RyCache(:, :, i) = [cos(y_rot) 0 sin(y_rot); 0 1 0; -sin(y_rot) 0 cos(y_rot)];
    RzCache(:, :, i) = [cos(z_rot) -sin(z_rot) 0; sin(z_rot) cos(z_rot) 0; 0 0 1];
end



%------------------------------
% (D) ID-Color Map 초기화
%------------------------------
if isempty(idColorMap)
    idColorMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
    colorIndex = 1;

    maxColors = 100;
    colorList = lines(maxColors); 
end


%------------------------------
% (E) ID별 BBox 크기/프레임 카운터 관리 맵 초기화
%------------------------------
if isempty(idBBoxMap)
    idBBoxMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
    idFrameCounterMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
end

updateInterval = 10;  % 10프레임 주기로 크기 업데이트


%------------------------------
% (F) 루프 전에, 전체 좌표축 레퍼런스 라인(빨강/초록/파랑) 1번만 그림
%     - 매 bbox마다 반복할 필요가 전혀 없으므로, 루프 밖으로 이동
%------------------------------
line(ax, 'XData', [0, 0.8], 'YData', [0, 0],    'ZData', [0, 0],    'Color', 'r', 'LineWidth', 1, 'Tag', 'detAxis');
line(ax, 'XData', [0, 0],   'YData', [0, 0.8],  'ZData', [0, 0],    'Color', 'g', 'LineWidth', 1, 'Tag', 'detAxis');
line(ax, 'XData', [0, 0],   'YData', [0, 0],    'ZData', [0, 0.8],  'Color', 'b', 'LineWidth', 1, 'Tag', 'detAxis');


for i = 1:size(cuboids, 1)

    cuboid      = cuboids{i};
    dist        = modelInfo.Distance(i);
    id          = modelInfo.ID{i};
    classNum    = modelInfo.Class{i};

    % ID에 따른 색상 할당
    if isKey(idColorMap, id)
        cuboidColor = idColorMap(id);
    else
        % 새로운 ID인 경우 새로운 색상 할당
        cuboidColor = colorList(colorIndex, :);
        idColorMap(id) = cuboidColor;

        colorIndex = colorIndex + 1;

        if colorIndex > size(colorList, 1)
            % 색상 인덱스가 색상 리스트를 초과하면 초기화 또는 다른 처리
            colorIndex = 1; 
        end
    end


    % 프레임 카운터 가져오기 또는 초기화
    if isKey(idFrameCounterMap, id)
        frameCounter = idFrameCounterMap(id);
    else
        frameCounter = 0;
    end


    % 바운딩 박스 크기 업데이트 여부 결정
    if mod(frameCounter, updateInterval) == 0 || ~isKey(idBBoxMap, id)
        
        % 주기에 도달했거나 새로운 객체인 경우 현재 크기로 업데이트
        dimensions = cuboid.Dimensions;
        idBBoxMap(id) = dimensions;
    else
        % 이전에 저장된 크기 사용
        dimensions = idBBoxMap(id);
    end

    % 프레임 카운터 증가 및 저장
    frameCounter = frameCounter + 1;
    idFrameCounterMap(id) = frameCounter;


    % Cuboid center and length
    x_ctr = cuboid.Center(1);
    y_ctr = cuboid.Center(2);
    z_ctr = cuboid.Center(3);
    x_len = dimensions(1);
    y_len = dimensions(2);
    z_len = dimensions(3);

    % Vertex calculation
    vertices = [x_len/2, -y_len/2,  z_len/2;
                x_len/2,  y_len/2,  z_len/2;
               -x_len/2,  y_len/2,  z_len/2;
               -x_len/2, -y_len/2,  z_len/2;
                x_len/2, -y_len/2, -z_len/2;
                x_len/2,  y_len/2, -z_len/2;
               -x_len/2,  y_len/2, -z_len/2;
               -x_len/2, -y_len/2, -z_len/2];

    % Apply rotation matrix
    R = RzCache(:, :, i) * RyCache(:, :, i) * RxCache(:, :, i);
    vertices = (R * vertices')';

    % Translation to original location

    vertices = vertices + [x_ctr, y_ctr, z_ctr];


    

    % Index of the line connecting the vertices
    edges = [1,2; 2,3; 3,4; 4,1;    % up side
             5,6; 6,7; 7,8; 8,5;    % down side
             1,5; 2,6; 3,7; 4,8];   % col

    %---- Draw cuboid model using line
    for j = 1:size(edges, 1)
        line(ax, 'XData', vertices(edges(j,:), 1), ...
                 'YData', vertices(edges(j,:), 2), ...
                 'ZData', vertices(edges(j,:), 3), ...
                 'Color', cuboidColor, 'Linewidth', 1, 'Tag', 'detBox');
    end

    
    
    % % Notation Orientation
    % bottomCenter = mean(vertices(5:8,:),1);
    % markerLength = 0.1; % 마커 길이 (중심을 표현하는 작은 선)
    % line(ax, 'XData', [bottomCenter(1) - markerLength, bottomCenter(1) + markerLength], ...
    %          'YData', [bottomCenter(2), bottomCenter(2)], ...
    %          'ZData', [bottomCenter(3), bottomCenter(3)], ...
    %          'Color', 'y', 'LineWidth', 2); % X축 방향으로 작은 노란색 선
    % 
    % line(ax, 'XData', [bottomCenter(1), bottomCenter(1)], ...
    %          'YData', [bottomCenter(2) - markerLength, bottomCenter(2) + markerLength], ...
    %          'ZData', [bottomCenter(3), bottomCenter(3)], ...
    %          'Color', 'y', 'LineWidth', 2); % Y축 방향으로 작은 노란색 선
    % 
    % 
    % x_dir = orientInfo(i, 1); % orientInfo의 x 방향 벡터 값
    % y_dir = orientInfo(i, 2); % orientInfo의 y 방향 벡터 값
    % 
    % directionVector = [x_dir; y_dir; 0];
    % directionVector = directionVector / norm(directionVector); % 방향 벡터를 단위 벡터로 정규화
    % 
    % lineLength = 1.0; % 직선의 길이 (1m)
    % lineEndPoint = bottomCenter + (lineLength * directionVector');
    % 
    % % 노란색 선을 bottom center로부터 방향 벡터를 따라 그리기
    % line(ax, 'XData', [bottomCenter(1), lineEndPoint(1)], ...
    %          'YData', [bottomCenter(2), lineEndPoint(2)], ...
    %          'ZData', [bottomCenter(3), lineEndPoint(3)], ...
    %          'Color', 'y', 'LineWidth', 1.5);



    % Notation distance and velocity
    topCenter = [mean(vertices(1:4, 1)), mean(vertices(1:4, 2)), max(vertices(1:4, 3))];
    textData = {id + " " + classNum + " " + num2str(round(dist, 2)) + "m"};


    line(ax, 'XData', [0, 0.8],'YData', [0, 0],'Color', 'r', 'LineWidth', 1);



    %---- Notation Orientation
    bottomCenter = mean(vertices(5:8,:), 1);
    
    % X,Y축 방향으로 작은 노란색 선
    markerLength = 0.1; % 마커 길이 (중심을 표현하는 작은 선)
    line(ax, 'XData', [bottomCenter(1) - markerLength, bottomCenter(1) + markerLength], ...
             'YData', [bottomCenter(2), bottomCenter(2)], ...
             'ZData', [bottomCenter(3), bottomCenter(3)], ...
             'Color', 'y', 'LineWidth', 2, 'Tag', 'detBox'); 

    line(ax, 'XData', [bottomCenter(1), bottomCenter(1)], ...
             'YData', [bottomCenter(2) - markerLength, bottomCenter(2) + markerLength], ...
             'ZData', [bottomCenter(3), bottomCenter(3)], ...
             'Color', 'y', 'LineWidth', 2, 'Tag', 'detBox'); 

    % orientInfo의 방향 벡터 
    x_dir = orientInfo(i, 1); 
    y_dir = orientInfo(i, 2); 

    directionVector = [x_dir; y_dir; 0];
    directionVector = directionVector / norm(directionVector + 1e-9); % 방향 벡터를 단위 벡터로 정규화(0나눗셈 방지)
    lineLength = 1.0; 
    lineEndPoint = bottomCenter + (lineLength * directionVector');
    line(ax, 'XData', [bottomCenter(1), lineEndPoint(1)], ...
             'YData', [bottomCenter(2), lineEndPoint(2)], ...
             'ZData', [bottomCenter(3), lineEndPoint(3)], ...
             'Color', 'y', 'LineWidth', 1.5, 'Tag', 'detBox');

    %---- Notation distance and velocity
    topCenter = [mean(vertices(1:4, 1)), mean(vertices(1:4, 2)), max(vertices(1:4, 3))];
    textData  = string(sprintf('%s %s %.2fm', id, classNum, dist));

    if hasVelInfo
        textData = string(sprintf('%s %.2fm/s', textData, velInfo(i)));
    end

    text(ax,topCenter(1),       topCenter(2),       topCenter(3)+0.3,...
             textData,...
             'HorizontalAlignment', 'center',...
             'VerticalAlignment', 'bottom', ...
             'Color', 'w',...
             'FontSize', 6,...
             'FontWeight', 'bold', ...
             'Tag', 'detText');

end
end

