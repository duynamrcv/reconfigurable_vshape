function PlotResults(scen)
%PLOTRESULTS
% figure();
% hold on;

% Draw scenario
scatter(scen.start(1), scen.start(2), 80, 'go', 'filled');
scatter(scen.goal(1), scen.goal(2), 80, 'rp', 'filled');
for i = 1:size(scen.obstacles,2)
    obs = scen.obstacles{i};
    pgon = polyshape(obs(:,1), obs(:,2));
    plot(pgon, 'FaceColor','black');
end
axis('equal');
xlim([scen.xmin scen.xmax]);
ylim([scen.ymin scen.ymax]);
end

