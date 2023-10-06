classdef Drone < handle
    %% Drone properties
    properties
        index
        position
        velocity
        heading
        path
        vrs
    end
    
    %% Control parameters
    properties
        kf = 0.8;
        kg = 0.6;
        kc = 0.12;
        ko = 1.0;
    end
    
    %% Zone
    properties
        ra = 0.3;
        rs = 2.0;
    end
    
    %% Drone methods
    methods
        % Constructor
        function obj = Drone(index_, position_)
            obj.index = index_;
            obj.position = position_;
            obj.velocity = [0,0];
            obj.heading = 0;
            
            obj.path = [obj.path; [obj.position, obj.heading]];
            obj.vrs = 0;
        end
        
        % Update position
        function obj = UpdatePosition(obj, vel, dt)
            obj.position = obj.position + vel*dt;
            obj.heading = atan2(vel(2), vel(1));
            obj.velocity = vel;
            obj.path = [obj.path; [obj.position, obj.heading]];
        end
        
        %% Behaviors
        function [vel, reached] = Behavior(obj,drones,model)
            l = (model.n+1)/2;
            if obj.index == l
                [vt, reached] = obj.Tracking(model);
                [vc, vr] = obj.Collision(drones, model);
                vo = obj.Obstacle(model);
                vel = vt + vc + vo;
            else
                vf = obj.Formation(drones, model);
                [vc, vr] = obj.Collision(drones, model);
                vo = obj.Obstacle(model);
                reached = false;
                vel = vf + vc + vo;
            end
            
            obj.vrs = [obj.vrs; norm(vr)];
            if norm(vel) > 2.0
                vel = 2.0*vel/norm(vel);
            end
        end
        
        %% Formation behavior
        function vf = Formation(obj, drones, model)
%             beta = 0.5;
            l = (model.n+1)/2;
            dis = model.d*abs(l-obj.index);
            if obj.index < l
                ang = drones(l).heading + model.alpha;
            else
                ang = drones(l).heading - model.alpha;
            end
            
            pd = drones(l).position + dis*[cos(ang), sin(ang)];
            
            vf = -obj.kf*(obj.position - pd) + drones(l).velocity;
%             vf = -obj.kf*abs(norm(obj.position)-norm(pd))^beta*(obj.position-pd)/norm(obj.position-pd) + drones(l).velocity;
        end
        
        %% Tracking behavior
        function [vt, reached] = Tracking(obj, model)
            epsilon = 0.1;
            if norm(model.goal-obj.position) > epsilon
                vt = -obj.kg*(obj.position - model.goal);
                % Normalization
                if norm(vt) > 1.0
                    vt = vt/norm(vt);
                end
                reached = false;
            else
                vt = [0,0];
                reached = true;
            end
        end
        
        %% Collision avoidance
        function [vc, vr] = Collision(obj, drones, model)
            beta = 2.5;
            l = (model.n+1)/2;
            vc = [0,0];
            vr = [0,0];
            for i = 1:model.n
                if obj.index == i
                    continue;
                end
                dis = model.d*abs(i-obj.index);
                pij = obj.position - drones(i).position;
                if (obj.index <= l && i <= l) || (obj.index > l && i >= l)
%                     if norm(pij) < obj.rs
                        vc = vc + obj.kc*abs(norm(pij)-dis)^beta*(1/(norm(pij)-obj.ra)^2)*pij/norm(pij);
                        vr = vr + obj.kc*abs(norm(pij)-dis)^beta*(1/(norm(pij)-obj.ra)^2)*pij/norm(pij);
%                     end
                else
                    if norm(pij) < obj.rs
                        vc = vc + obj.kc*exp(-beta*(norm(pij) - obj.ra))/(norm(pij)-obj.ra)*pij/norm(pij);
                    end
                end
            end
        end
        
        %% Obstacle avoidance
        function vo = Obstacle(obj, model)
            vo = [0,0];
            for j = 1:size(model.obstacles,2)
                obstacle = model.obstacles{j};
                dis = inf;
                voj = [0,0];
                for i=1:size(obstacle,1)
                    per = Perpendicular(obj.position, obstacle(i,:), obstacle(mod(i,4)+1,:));
                    dis_per = norm(obj.position-per);
                    if dis_per < dis
                        dis = dis_per;
                        n_ = per - obj.position;
                    end
                    if dis <= obj.rs
                        voj = - 1/2*(1/dis^2 - 1/obj.rs^2)*n_/norm(n_);
                    end
                end
                vo = vo + obj.ko*voj;
            end
        end
    end
end
    