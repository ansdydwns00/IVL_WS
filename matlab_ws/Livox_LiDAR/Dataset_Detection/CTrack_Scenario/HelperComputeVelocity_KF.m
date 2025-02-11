function [VelocityInfo, OrientInfo] = HelperComputeVelocity_KF(ModelInfo)
 

    VelocityInfo = zeros(0,1);
    OrientInfo = zeros(0,2);
    
    for i = 1:size(ModelInfo.Obj_Vel,1)
    
        vx = ModelInfo.Obj_Vel(i,1);
        vy = ModelInfo.Obj_Vel(i,2);
        vz = ModelInfo.Obj_Vel(i,3);

        vel = floor(sqrt(vx^2 + vy^2 + vz^2) * 10) / 10;
        % vel = round(sqrt(vx^2 + vy^2 + vz^2),2);

        VelocityInfo = [VelocityInfo; vel];
        OrientInfo = [OrientInfo; [vx,vy]];      
    end

end

