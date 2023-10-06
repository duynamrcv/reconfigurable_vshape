step = 10;
length = size(drones(1).path);
newcolors = [[0 0.4470 0.7410];...
             [0.8500 0.3250 0.0980];...
             [0.9290 0.6940 0.1250];...
             [0.4940 0.1840 0.5560];...
             [0.4660 0.6740 0.1880];...
             [0.3010 0.7450 0.9330];...
             [0.6350 0.0780 0.1840]];

figure();
set(gcf, 'Position', get(0, 'Screensize'));
iter = 0;

for idx = 1:step:length
    cla;
    iter = iter + 1;
    hold on;
    grid on;
    gr = [];
    
    PlotResults(model);
    % Start location
    xs=model.start(1);
    ys=model.start(2);
    plot(xs,ys,'bs','MarkerSize',10,'MarkerFaceColor','b');

    % End location
    xf=model.goal(1);
    yf=model.goal(2);
    plot(xf,yf,'rp','MarkerSize',10,'MarkerFaceColor','r');

    % Plot path
    for i = 1:model.n
        plot(drones(i).path(1:idx,1),drones(i).path(1:idx,2),...
            'LineWidth',2,'Color',newcolors(i,:));
        
        scatter(drones(i).path(idx,1),drones(i).path(idx,2),...
                'Marker', 'o',...
                'MarkerEdgeColor',[0 .5 .5],...
                'MarkerFaceColor',[0 .7 .7],...
                'LineWidth',1.5);
        gr = [gr; drones(i).path(idx,:)];
    end
    
    plot(gr(:,1), gr(:,2), '-k', 'LineWidth', 2);
    axis('equal');
    xlim([model.xmin model.xmax]);
    ylim([model.ymin model.ymax]);
    drawnow;
    F(iter) = getframe(gcf);
    hold off;
end

video = VideoWriter('result.avi','Motion JPEG AVI');
video.FrameRate = 20;  % (frames per second) this number depends on the sampling time and the number of frames you have
open(video);
writeVideo(video,F);
close(video);