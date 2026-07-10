function cfg = getSimulationConfig(mode)
%GETSIMULATIONCONFIG Build configuration for baseline or enhanced simulations.

if nargin < 1 || isempty(mode)
    mode = 'enhanced';
end

cfg.mode = lower(char(mode));
cfg.dt = 0.1;
cfg.Tmax = 40;
cfg.predictTime = 2.0;
cfg.goalTol = 0.3;

cfg.robot.radius = 0.135;
cfg.robot.wheelRadius = 0.05;
cfg.robot.wheelBase = 0.33;

cfg.safetyMargin = 0.12;

cfg.params.v_min = 0.0;
cfg.params.v_max = 1.2;
cfg.params.w_min = -1.5;
cfg.params.w_max = 1.5;
cfg.params.a_v   = 0.6;
cfg.params.a_w   = 1.2;
cfg.params.v_res = 0.05;
cfg.params.w_res = 0.1;

cfg.controller.useLQI = strcmp(cfg.mode, 'enhanced');
cfg.controller.maxUv = 2.5;
cfg.controller.maxUw = 4.0;
cfg.controller.intLimitV = 2.0;
cfg.controller.intLimitW = 2.0;
cfg.controller.Q = diag([15 10 4 3]);
cfg.controller.R = diag([0.7 0.5]);

cfg.dynamics.tau_v = 0.35;
cfg.dynamics.tau_w = 0.25;
cfg.dynamics.k_v = 1.0;
cfg.dynamics.k_w = 1.0;

cfg.globalPlanner.enabled = strcmp(cfg.mode, 'enhanced');
cfg.globalPlanner.resolution = 0.25;
cfg.globalPlanner.inflation = cfg.robot.radius + cfg.safetyMargin + 0.10;
cfg.globalPlanner.replanPeriod = 0.5;
if strcmp(cfg.mode, 'enhanced')
    cfg.globalPlanner.lookAheadIdx = 12;
    cfg.globalPlanner.minWaypointDist = 1.00;
else
    cfg.globalPlanner.lookAheadIdx = 10;
    cfg.globalPlanner.minWaypointDist = 1.00;
end
cfg.globalPlanner.xLimits = [-2 14];
cfg.globalPlanner.yLimits = [-3.5 3.5];
cfg.globalPlanner.dwaPathBlend = 0.55;
cfg.globalPlanner.pathTrackingDistScale = 1.00;

cfg.cost.lambda1 = 1/40;
cfg.cost.lambda2 = 1/15;
cfg.cost.lambda3 = 1/5;
cfg.cost.lambda4 = 1/10;
end
