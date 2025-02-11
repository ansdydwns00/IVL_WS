function [bboxes, id, cls] = scenario_test_bbox(bbox_info)
    
    
    bboxes = [];
    id = {};
    cls = {};
    
    % bounding box info
    for idx = 1:size(bbox_info,1)
        x = bbox_info(idx,1);
        y = bbox_info(idx,2);
        w = bbox_info(idx,3);
        h = bbox_info(idx,4);
        
        % Bounding box info
        bbox = [x-w/2, y-h/2, w, h];
        bboxes(idx,:) = bbox;


    end
end