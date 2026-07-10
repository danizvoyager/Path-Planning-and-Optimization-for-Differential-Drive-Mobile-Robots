function [J, sim] = evaluateWeights(weights, doPlot, scenario, cfg)

if nargin < 2
    doPlot = false;
end

if nargin < 3 || isempty(scenario)
    scenario = generateScenarioFixed();
end

if nargin < 4 || isempty(cfg)
    cfg = getSimulationConfig('enhanced');
end

weights = max(weights, 0);
if sum(weights) == 0
    weights = [1 1 1];
end
weights = weights / sum(weights);

dt          = cfg.dt;
Tmax        = cfg.Tmax;
predictTime = cfg.predictTime;
goalTol     = cfg.goalTol;

x = [0; 0; 0; 0; 0];
goal = scenario.goal;

robot = cfg.robot;
safetyMargin = cfg.safetyMargin;
params = cfg.params;

staticObs  = scenario.staticObs;
dynamicObs = scenario.dynamicObs;

hist_state = [];
hist_uRef  = [];
hist_uAct  = [];
hist_time  = [];
hist_dmin  = [];
hist_goal  = [];
plannerPathLog = {};

plannerState = [];
controllerState = [];
time = 0;
reached = false;
lastPlanTime = -inf;
bestTraj = [];

while time <= Tmax
    if norm(x(1:2) - goal) < goalTol
        reached = true;
        break;
    end

    for i = 1:numel(dynamicObs)
        dynamicObs(i).pos = dynamicObs(i).pos + dynamicObs(i).vel * dt;
    end

    trackingGoal = goal;
    globalPath = [];
    if cfg.globalPlanner.enabled && (time - lastPlanTime >= cfg.globalPlanner.replanPeriod || isempty(plannerState))
        [trackingGoal, plannerState] = selectTrackingGoal(x, goal, staticObs, dynamicObs, plannerState, cfg);
        lastPlanTime = time;
        if isfield(plannerState, 'waypoints')
            plannerPathLog{end+1} = plannerState.waypoints; %#ok<AGROW>
        end
    elseif cfg.globalPlanner.enabled && ~isempty(plannerState) && isfield(plannerState, 'waypoints') && ~isempty(plannerState.waypoints)
        trackingGoal = plannerState.waypoints(:, min(size(plannerState.waypoints,2), cfg.globalPlanner.lookAheadIdx));
    end
    if cfg.globalPlanner.enabled && ~isempty(plannerState) && isfield(plannerState, 'waypoints')
        globalPath = plannerState.waypoints;
    end

    [uRef, bestTraj, evalData] = dwaControl(x, trackingGoal, staticObs, dynamicObs, ...
        robot, params, weights, predictTime, dt, safetyMargin, cfg, controllerState, globalPath);

    [x, controllerState, dynDebug] = ddmrDynamics(x, uRef, dt, cfg.dynamics, cfg.controller, controllerState);

    hist_state = [hist_state x]; %#ok<AGROW>
    hist_uRef  = [hist_uRef uRef]; %#ok<AGROW>
    hist_uAct  = [hist_uAct dynDebug.uAct]; %#ok<AGROW>
    hist_time  = [hist_time time]; %#ok<AGROW>
    hist_dmin  = [hist_dmin evalData.bestDmin]; %#ok<AGROW>
    hist_goal  = [hist_goal trackingGoal]; %#ok<AGROW>

    if doPlot
        clf; hold on; grid on; axis equal;
        xlim([-2 14]); ylim([-3 3]);

        plot(goal(1), goal(2), 'gp', 'MarkerSize', 14, 'LineWidth', 2);
        plot(trackingGoal(1), trackingGoal(2), 'mp', 'MarkerSize', 12, 'LineWidth', 1.5);

        for i = 1:numel(staticObs)
            drawCircle(staticObs(i).pos(1), staticObs(i).pos(2), staticObs(i).radius, [0.85 0.2 0.2]);
        end

        for i = 1:numel(dynamicObs)
            drawCircle(dynamicObs(i).pos(1), dynamicObs(i).pos(2), dynamicObs(i).radius, [0.2 0.2 0.85]);
            quiver(dynamicObs(i).pos(1), dynamicObs(i).pos(2), dynamicObs(i).vel(1), dynamicObs(i).vel(2), ...
                0, 'Color', [0.2 0.2 0.85], 'LineWidth', 2);
        end

        if cfg.globalPlanner.enabled && ~isempty(plannerState) && isfield(plannerState, 'waypoints') && ~isempty(plannerState.waypoints)
            plot(plannerState.waypoints(1,:), plannerState.waypoints(2,:), 'c-.', 'LineWidth', 1.5);
        end

        drawCircle(x(1), x(2), robot.radius, [0.1 0.6 0.1]);
        quiver(x(1), x(2), 0.6*cos(x(3)), 0.6*sin(x(3)), 0, 'k', 'LineWidth', 2);

        if ~isempty(hist_state)
            plot(hist_state(1,:), hist_state(2,:), 'k-', 'LineWidth', 1.5);
        end

        if ~isempty(bestTraj)
            plot(bestTraj(1,:), bestTraj(2,:), 'm--', 'LineWidth', 2);
        end

        title(sprintf('t = %.1f s, v_{ref}=%.2f, w_{ref}=%.2f, v=%.2f, w=%.2f', time, uRef(1), uRef(2), x(4), x(5)));
        drawnow;
    end

    time = time + dt;
end

if isempty(hist_time)
    T_goal = Tmax;
else
    T_goal = hist_time(end);
end

L = 0;
for k = 2:size(hist_state, 2)
    dx = hist_state(1,k) - hist_state(1,k-1);
    dy = hist_state(2,k) - hist_state(2,k-1);
    L = L + hypot(dx, dy);
end

if isempty(hist_dmin)
    Dmin_global = 0;
else
    Dmin_global = min(hist_dmin);
end

if isempty(hist_uAct)
    omega_sq_int = 1e6;
else
    omega_sq_int = sum(hist_uAct(2,:).^2) * dt;
end

penalty = 0;
if ~reached
    penalty = 50;
end

J = globalCost(T_goal, L, Dmin_global, omega_sq_int, ...
    cfg.cost.lambda1, cfg.cost.lambda2, cfg.cost.lambda3, cfg.cost.lambda4) + penalty;

sim.reached       = reached;
sim.T_goal        = T_goal;
sim.L             = L;
sim.Dmin_global   = Dmin_global;
sim.omega_sq_int  = omega_sq_int;
sim.hist_state    = hist_state;
sim.hist_u        = hist_uAct;
sim.hist_uRef     = hist_uRef;
sim.hist_time     = hist_time;
sim.hist_dmin     = hist_dmin;
sim.hist_goal     = hist_goal;
sim.guidancePath  = hist_goal;
sim.weights       = weights;
sim.staticObs     = staticObs;
sim.dynamicObs    = dynamicObs;
sim.goal          = goal;
sim.bestTraj      = bestTraj;
sim.plannerPathLog = plannerPathLog;
sim.mode          = char(cfg.mode);

end

function drawCircle(xc, yc, r, c)
th = linspace(0, 2*pi, 100);
x = xc + r*cos(th);
y = yc + r*sin(th);
fill(x, y, c, 'FaceAlpha', 0.35, 'EdgeColor', c, 'LineWidth', 1.5);
end
