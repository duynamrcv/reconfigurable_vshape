clc
close all
clear

% Load model
model = CreateModel2();    

% Init drones
drones = [];
for i = 1:model.n
    drone = Drone(i, model.start+unifrnd(-1,1,[1,2]));
    drones = [drones; drone];
end
dt = 0.02;
iter = 0;
% while norm(drone.position - model.goal) > 0.1 && iter < 1000
reached = false;
while ~reached
    for i = 1:model.n
        [vel, reach] = drones(i).Behavior(drones, model);
        reached = reached + reach;
        drones(i).UpdatePosition(vel, dt);
    end
    iter = iter + 1;
end

figure();
hold on;
% Start location
xs=model.start(1);
ys=model.start(2);
plot(xs,ys,'bs','MarkerSize',10,'MarkerFaceColor','b');

% End location
xf=model.goal(1);
yf=model.goal(2);
plot(xf,yf,'rp','MarkerSize',10,'MarkerFaceColor','r');

% Obstacles
for i = 1:size(model.obstacles,2)
    obs = model.obstacles{i};
    pgon = polyshape(obs(:,1), obs(:,2));
    plot(pgon, 'FaceColor','black');
end

% Plot path
for i = 1:model.n
    plot(drones(i).path(:,1), drones(i).path(:,2), 'LineWidth', 2);
end

num = 6;
step = floor((size(drones(1).path,1))/num);
for i = 0:num
    idx = i*step;
    if idx == 0
        idx = 1;
    end
    
    % Plot formation graph
    gr = [];
    for j = 1:size(drones,1)
        gr = [gr; drones(j).path(idx,:)];
    end
    plot(gr(:,1), gr(:,2), '-k', 'LineWidth', 2);
    
    % Plot formation agent
    for j = 1:size(drones,1)
        scatter(drones(j).path(idx,1),drones(j).path(idx,2),...
                'Marker', 'o',...
                'MarkerEdgeColor',[0 .5 .5],...
                'MarkerFaceColor',[0 .7 .7],...
                'LineWidth',1.5);
    end
end
xlabel('x [m]');
ylabel('y [m]');
axis('equal');
xlim([model.xmin model.xmax]);
ylim([model.ymin model.ymax]);