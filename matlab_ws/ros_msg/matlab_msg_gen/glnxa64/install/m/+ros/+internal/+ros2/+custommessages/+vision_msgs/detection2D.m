function [data, info] = detection2D
%Detection2D gives an empty data for vision_msgs/Detection2D
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'vision_msgs/Detection2D';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.results, info.results] = ros.internal.ros2.custommessages.vision_msgs.objectHypothesisWithPose;
info.results.MLdataType = 'struct';
info.results.MaxLen = NaN;
info.results.MinLen = 0;
[data.bbox, info.bbox] = ros.internal.ros2.custommessages.vision_msgs.boundingBox2D;
info.bbox.MLdataType = 'struct';
[data.source_img, info.source_img] = ros.internal.ros2.messages.sensor_msgs.image;
info.source_img.MLdataType = 'struct';
[data.is_tracking, info.is_tracking] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
[data.tracking_id, info.tracking_id] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'vision_msgs/Detection2D';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,41);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'results';
info.MatPath{7} = 'results.id';
info.MatPath{8} = 'results.score';
info.MatPath{9} = 'results.pose';
info.MatPath{10} = 'results.pose.pose';
info.MatPath{11} = 'results.pose.pose.position';
info.MatPath{12} = 'results.pose.pose.position.x';
info.MatPath{13} = 'results.pose.pose.position.y';
info.MatPath{14} = 'results.pose.pose.position.z';
info.MatPath{15} = 'results.pose.pose.orientation';
info.MatPath{16} = 'results.pose.pose.orientation.x';
info.MatPath{17} = 'results.pose.pose.orientation.y';
info.MatPath{18} = 'results.pose.pose.orientation.z';
info.MatPath{19} = 'results.pose.pose.orientation.w';
info.MatPath{20} = 'results.pose.covariance';
info.MatPath{21} = 'bbox';
info.MatPath{22} = 'bbox.center';
info.MatPath{23} = 'bbox.center.x';
info.MatPath{24} = 'bbox.center.y';
info.MatPath{25} = 'bbox.center.theta';
info.MatPath{26} = 'bbox.size_x';
info.MatPath{27} = 'bbox.size_y';
info.MatPath{28} = 'source_img';
info.MatPath{29} = 'source_img.header';
info.MatPath{30} = 'source_img.header.stamp';
info.MatPath{31} = 'source_img.header.stamp.sec';
info.MatPath{32} = 'source_img.header.stamp.nanosec';
info.MatPath{33} = 'source_img.header.frame_id';
info.MatPath{34} = 'source_img.height';
info.MatPath{35} = 'source_img.width';
info.MatPath{36} = 'source_img.encoding';
info.MatPath{37} = 'source_img.is_bigendian';
info.MatPath{38} = 'source_img.step';
info.MatPath{39} = 'source_img.data';
info.MatPath{40} = 'is_tracking';
info.MatPath{41} = 'tracking_id';
