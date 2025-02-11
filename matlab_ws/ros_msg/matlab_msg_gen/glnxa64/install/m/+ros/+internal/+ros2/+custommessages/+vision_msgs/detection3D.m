function [data, info] = detection3D
%Detection3D gives an empty data for vision_msgs/Detection3D
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'vision_msgs/Detection3D';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.results, info.results] = ros.internal.ros2.custommessages.vision_msgs.objectHypothesisWithPose;
info.results.MLdataType = 'struct';
info.results.MaxLen = NaN;
info.results.MinLen = 0;
[data.bbox, info.bbox] = ros.internal.ros2.custommessages.vision_msgs.boundingBox3D;
info.bbox.MLdataType = 'struct';
[data.source_cloud, info.source_cloud] = ros.internal.ros2.messages.sensor_msgs.pointCloud2;
info.source_cloud.MLdataType = 'struct';
[data.is_tracking, info.is_tracking] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
[data.tracking_id, info.tracking_id] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'vision_msgs/Detection3D';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,63);
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
info.MatPath{23} = 'bbox.center.position';
info.MatPath{24} = 'bbox.center.position.x';
info.MatPath{25} = 'bbox.center.position.y';
info.MatPath{26} = 'bbox.center.position.z';
info.MatPath{27} = 'bbox.center.orientation';
info.MatPath{28} = 'bbox.center.orientation.x';
info.MatPath{29} = 'bbox.center.orientation.y';
info.MatPath{30} = 'bbox.center.orientation.z';
info.MatPath{31} = 'bbox.center.orientation.w';
info.MatPath{32} = 'bbox.size';
info.MatPath{33} = 'bbox.size.x';
info.MatPath{34} = 'bbox.size.y';
info.MatPath{35} = 'bbox.size.z';
info.MatPath{36} = 'source_cloud';
info.MatPath{37} = 'source_cloud.header';
info.MatPath{38} = 'source_cloud.header.stamp';
info.MatPath{39} = 'source_cloud.header.stamp.sec';
info.MatPath{40} = 'source_cloud.header.stamp.nanosec';
info.MatPath{41} = 'source_cloud.header.frame_id';
info.MatPath{42} = 'source_cloud.height';
info.MatPath{43} = 'source_cloud.width';
info.MatPath{44} = 'source_cloud.fields';
info.MatPath{45} = 'source_cloud.fields.INT8';
info.MatPath{46} = 'source_cloud.fields.UINT8';
info.MatPath{47} = 'source_cloud.fields.INT16';
info.MatPath{48} = 'source_cloud.fields.UINT16';
info.MatPath{49} = 'source_cloud.fields.INT32';
info.MatPath{50} = 'source_cloud.fields.UINT32';
info.MatPath{51} = 'source_cloud.fields.FLOAT32';
info.MatPath{52} = 'source_cloud.fields.FLOAT64';
info.MatPath{53} = 'source_cloud.fields.name';
info.MatPath{54} = 'source_cloud.fields.offset';
info.MatPath{55} = 'source_cloud.fields.datatype';
info.MatPath{56} = 'source_cloud.fields.count';
info.MatPath{57} = 'source_cloud.is_bigendian';
info.MatPath{58} = 'source_cloud.point_step';
info.MatPath{59} = 'source_cloud.row_step';
info.MatPath{60} = 'source_cloud.data';
info.MatPath{61} = 'source_cloud.is_dense';
info.MatPath{62} = 'is_tracking';
info.MatPath{63} = 'tracking_id';
