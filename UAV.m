classdef UAV

    properties
        pt_base_link;
        pt_base_link_0;
        pt_base_link_1;
        l_base_link;

        pose;

        pt_arm_0_end;
        pt_arm_1_end;
        
        l_arm_0;
        l_arm_1;
        
        pos_arm_0;
        pos_arm_1;
        vel_arm_0;
        vel_arm_1;
        
        pos_arm_0_offset;
        pos_arm_1_offset;

        XLim;
        YLim;
        ZLim;

        virtual;
        
    end
    
    methods
        % pt_base_link: x, y, z, r, p, y
        function obj = UAV(base_pose, l0, l1, p0_offset, p1_offset, XLim, YLim, ZLim, virtual)
            obj.pt_base_link = base_pose(1:3);
            obj.pose = base_pose;
            obj.l_arm_0 = l0;
            obj.l_arm_1 = l1;
            obj.pos_arm_0_offset = p0_offset;
            obj.pos_arm_1_offset = p1_offset;

            obj.pos_arm_0 = -2.3 - p0_offset;
            obj.pos_arm_1 = -1.8 - p1_offset;
            obj.vel_arm_0 = 0;
            obj.vel_arm_1 = 0;
            obj.l_base_link = 0.2;


            y = obj.pose(4);
            obj.pt_base_link_0 = obj.pt_base_link + obj.l_base_link*0.5*[cos(y);sin(y);0];
            obj.pt_base_link_1 = obj.pt_base_link - obj.l_base_link*0.5*[cos(y);sin(y);0];

            obj.pt_arm_0_end = obj.pt_base_link + obj.l_arm_0*[cos(obj.pos_arm_0+obj.pos_arm_0_offset)*cos(y);cos(obj.pos_arm_0+obj.pos_arm_0_offset)*sin(y);sin(obj.pos_arm_0+obj.pos_arm_0_offset)];
            obj.pt_arm_1_end = obj.pt_arm_0_end + obj.l_arm_1*[cos(obj.pos_arm_1+obj.pos_arm_1_offset)*cos(y);cos(obj.pos_arm_1+obj.pos_arm_1_offset)*sin(y);sin(obj.pos_arm_1+obj.pos_arm_1_offset)];

            obj.XLim = XLim;
            obj.YLim = YLim;
            obj.ZLim = ZLim;

            obj.virtual = virtual;
        end
     
        
        function obj = updata(obj)
            obj.pt_base_link = obj.pose(1:3);
            y = obj.pose(4);            

            obj.pt_base_link_0 = obj.pt_base_link + obj.l_base_link*0.5*[cos(y);sin(y);0];
            obj.pt_base_link_1 = obj.pt_base_link - obj.l_base_link*0.5*[cos(y);sin(y);0];

            obj.pt_arm_0_end = obj.pt_base_link + obj.l_arm_0*[cos(obj.pos_arm_0+obj.pos_arm_0_offset)*cos(y); cos(obj.pos_arm_0+obj.pos_arm_0_offset)*sin(y); sin(obj.pos_arm_0+obj.pos_arm_0_offset)];
            obj.pt_arm_1_end = obj.pt_arm_0_end + obj.l_arm_1*[cos(obj.pos_arm_1+obj.pos_arm_1_offset)*cos(y); cos(obj.pos_arm_1+obj.pos_arm_1_offset)*sin(y); sin(obj.pos_arm_1+obj.pos_arm_1_offset)];
    
        end

        function obj = set(obj, uav_pos, arm_pos)
            obj.pt_base_link = uav_pos(1:3);
            y = uav_pos(4);            

            obj.pt_base_link_0 = obj.pt_base_link + obj.l_base_link*0.5*[cos(y);sin(y);0];
            obj.pt_base_link_1 = obj.pt_base_link - obj.l_base_link*0.5*[cos(y);sin(y);0];

            obj.pt_arm_0_end = obj.pt_base_link + obj.l_arm_0*[cos(arm_pos(1)+obj.pos_arm_0_offset)*cos(y); cos(arm_pos(1)+obj.pos_arm_0_offset)*sin(y); sin(arm_pos(1)+obj.pos_arm_0_offset)];
            obj.pt_arm_1_end = obj.pt_arm_0_end + obj.l_arm_1*[cos(arm_pos(2)+obj.pos_arm_1_offset)*cos(y); cos(arm_pos(2)+obj.pos_arm_1_offset)*sin(y); sin(arm_pos(2)+obj.pos_arm_1_offset)];

        end

        function draw1(obj)
                
            set(gca, "XLim", obj.XLim);
            set(gca, "YLim", obj.YLim);
            set(gca, "ZLim", obj.ZLim);
            hold on;
            grid on;

            line_base = [obj.pt_base_link_0, obj.pt_base_link_1];
            line_arm_0 = [obj.pt_base_link, obj.pt_arm_0_end];
            line_arm_1 = [obj.pt_arm_0_end, obj.pt_arm_1_end];

            line(line_base(1,:), line_base(2,:), line_base(3,:), 'Color', 'r', 'LineWidth', 3);
            line(line_arm_0(1,:), line_arm_0(2,:), line_arm_0(3,:), 'Color', 'g', 'LineWidth', 3);
            line(line_arm_1(1,:), line_arm_1(2,:), line_arm_1(3,:), 'Color', 'b', 'LineWidth', 3);
        end

        function draw2(obj)
                
            set(gca, "XLim", obj.XLim);
            set(gca, "YLim", obj.YLim);
            set(gca, "ZLim", obj.ZLim);
            hold on;
            grid on;

            line_base = [obj.pt_base_link_0, obj.pt_base_link_1];
            line_arm_0 = [obj.pt_base_link, obj.pt_arm_0_end];
            line_arm_1 = [obj.pt_arm_0_end, obj.pt_arm_1_end];

            line(line_base(1,:), line_base(2,:), line_base(3,:), 'Color', 'r', 'LineWidth', 3, 'LineStyle', '-.');
            line(line_arm_0(1,:), line_arm_0(2,:), line_arm_0(3,:), 'Color', 'g', 'LineWidth', 3, 'LineStyle', '-.');
            line(line_arm_1(1,:), line_arm_1(2,:), line_arm_1(3,:), 'Color', 'b', 'LineWidth', 3, 'LineStyle', '-.');
        end


    end
end
