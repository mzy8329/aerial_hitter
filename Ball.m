classdef Ball

    properties
        pose;
        r;
        pose_0;
        vel_0;
        acc_0;
        
        t_0;
    end
    
    methods
        function obj = Ball(origin_pose, r)
            obj.pose = origin_pose;
            obj.r = r;

            obj.pose_0 = origin_pose;
            obj.vel_0 = [0; 0; 0];
            obj.acc_0 = [0; 0; -9.8];
        end

        
        function updata_draw(obj,pose_0, vel_0, acc_0, t, t_0)
            obj.t_0 = t_0;
            obj.vel_0 = vel_0;
            obj.acc_0 = acc_0;

            obj.pose = obj.pose_0 + obj.vel_0.*(t-t_0) + 1./2.0.*obj.acc_0.*(t-t_0).^2;
            if obj.pose(3) <= obj.r
                obj.pose(3) = obj.r;
            end

            plot3(obj.pose(1), obj.pose(2), obj.pose(3), 'ro')

        end

    end
end
