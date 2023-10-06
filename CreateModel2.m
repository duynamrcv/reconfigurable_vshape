function model = CreateModel2()
%CREATEMODEL
%% Formation description
d = 0.8;        % The desired distance
alpha = 3*pi/4; % The desired angle
n = 5;          % Number of drone 

%% Start and Goal
start = [-20.0 3.5];
goal = [22.0 3.5];

%% Obstacles set
obs1 = [-10.0 0.0; -10.0 1.0; 0.0 3.15; 15.0 3.15; 20.0 0.0];
obs2 = [-10.0 7.0; -10.0 6.0; 10.0 3.85; 15.0 3.85; 20.0 7.0];

%% Limit
xmax = 22.0;
xmin = -24.0;
ymax = 7.0;
ymin = 0.0;

%% Scenario
model.d = d;
model.alpha = alpha;
model.n = n;

model.start = start;
model.goal = goal;

model.obstacles = {obs1, obs2};

model.xmax = xmax;
model.xmin = xmin;
model.ymax = ymax;
model.ymin = ymin;
end

