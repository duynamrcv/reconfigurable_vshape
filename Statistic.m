close all;

%% Distance
xmax = size(drones(1).path,1);
min_dis = inf;
for i = 1:model.n-1
    heading = 0;
    for j = i+1:model.n
        err = drones(i).path(:,1:2) - drones(j).path(:,1:2);
        dis = sqrt(sum(err.^2,2));
        if min_dis > min(dis)
            min_dis = min(dis);
        end
    end
end
disp(['Min distance: ', num2str(min_dis), ', Alert: ', num2str(drones(1).ra)]);
% return;
%% Mean
dis = [];
for i = 1:model.n-1
    di = drones(i).path(:,1:2);
    dj = drones(i+1).path(:,1:2);
    dis = [dis; sqrt(sum((di-dj).^2,2))];
end
ave = sum(dis)/length(dis);
disp(['Ave distance: ', num2str(ave), ', Desired: ', num2str(model.d)]);

%% Error
err = 0.5*(sum(dis)/4-model.d)/length(dis);
disp(['Ave error: ', num2str(err)]);