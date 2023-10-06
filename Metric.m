close all;

%% Heading disturbance
headings = [];
for i = 1:size(drones(1).path,1)
    heading = 0;
    for j = 1:model.n
        x = drones(j).path(i,3);
        heading = heading + [cos(x) sin(x)];
    end
    headings = [headings, norm(heading)/model.n];
end
figure();
hold on;
grid on;
steps = 1:size(headings,2);
% plot(headings, '-b', 'LineWidth', 2);
scatter(steps, headings, 50, '.');
xlabel('Time step');
ylabel('Order');
xlim([0 size(headings,2)]);

%% Distance
xmax = size(drones(1).path,1);
figure();
hold on;
grid on;
line([x,xmax],[drones(1).ra,drones(1).ra], 'Color', 'k',...
        'LineStyle' ,'--', 'LineWidth', 2, 'DisplayName', 'Alert Zone');
for i = 1:model.n-1
    heading = 0;
    for j = i+1:model.n
        dis = drones(i).path(:,1:2) - drones(j).path(:,1:2);
        plot(sqrt(sum(dis.^2,2)), 'LineWidth',2,...
            'DisplayName', ['UAV', num2str(i),'-UAV',num2str(j)]);
    end
end
xlabel('Time step');
ylabel('Value [m]');
xlim([0 xmax]);
legend({},'NumColumns',2);
% return;
%% Mean
dis = [];
for i = 1:model.n-1
    di = drones(i).path(:,1:2);
    dj = drones(i+1).path(:,1:2);
    dis = [dis, sqrt(sum((di-dj).^2,2))];
end

figure();
boxplot(dis,'Widths',0.5,'symbol', '',...
        'Labels',{'UAV1 - UAV2','UAV2 - UAV3','UAV3 - UAV4', 'UAV4 - UAV5'});
ylabel('Distance [m]');

%% Error
figure();
hold on;
grid on;
plot(0.5*(sum(dis,2)/4-model.d), 'LineWidth', 2);
xlabel('Time step');
ylabel('Evarage error [m]');
xlim([0 xmax]);