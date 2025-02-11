function [bboxes, cls] = scenario_test_bbox(bbox_info)
    
    % Class mapping
    classNames = containers.Map({'0', '1', '2'}, {'Pedestrian', 'Bicycle', 'Car'});
    
    bboxes = [];
    cls = {};
    
    % bounding box info
    for idx = 1:size(bbox_info,1)

        r_bbox = bbox_info{idx,1};

        x = r_bbox(1);
        y = r_bbox(2);
        w = r_bbox(3);
        h = r_bbox(4);
        
        % Bounding box info
        bbox = [x-w/2, y-h/2, w, h];
        bboxes(idx,:) = bbox;

        % Object info
        classNum = num2str(bbox_info{idx,2});


        % 클래스 번호를 이름으로 변환
        if isKey(classNames, classNum)
            className = classNames(classNum);
        else
            className = 'Unknown';
        end

        cls{idx} = className;

    end
end