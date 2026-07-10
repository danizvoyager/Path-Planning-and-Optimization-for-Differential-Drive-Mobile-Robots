function [u_best, bestTraj, evalData] = dwaControl(x, goal, staticObs, dynamicObs, ...
    robot, params, weights, predictTime, dt, safetyMargin, cfg, controllerState, globalPath)

if nargin < 11
    cfg = [];
end
if nargin < 12
    controllerState = [];
end
if nargin < 13
    globalPath = [];
end

dw = calcDynamicWindow(x, params, dt);

bestScore = -inf;
u_best = [0; 0];
bestTraj = [];
bestDmin = inf;

candidates = [];

for v = dw(1):params.v_res:dw(2)
    for w = dw(3):params.w_res:dw(4)

        if isempty(cfg)
            traj = predictTrajectory(x, [v; w], predictTime, dt);
        else
            traj = predictTrajectory(x, [v; w], predictTime, dt, cfg, controllerState);
        end

        [H, D, V, dmin, collision] = evaluateTrajectory( ...
            traj, goal, v, staticObs, dynamicObs, robot, dt, params.v_max, safetyMargin, globalPath, cfg);

        if ~collision
            candidates = [candidates; v, w, H, D, V, dmin]; %#ok<AGROW>
        end
    end
end

if isempty(candidates)
    u_best = [0; 0.5];
    if isempty(cfg)
        bestTraj = predictTrajectory(x, u_best, predictTime, dt);
    else
        bestTraj = predictTrajectory(x, u_best, predictTime, dt, cfg, controllerState);
    end
    evalData.bestScore = -inf;
    evalData.bestDmin = 0;
    return;
end

Hn = normalizeColumn(candidates(:,3));
Dn = normalizeColumn(candidates(:,4));
Vn = normalizeColumn(candidates(:,5));

alpha = weights(1);
beta  = weights(2);
gamma = weights(3);

for i = 1:size(candidates,1)
    v = candidates(i,1);
    w = candidates(i,2);

    score = alpha * Hn(i) + beta * Dn(i) + gamma * Vn(i);

    if score > bestScore
        bestScore = score;
        u_best = [v; w];
        if isempty(cfg)
            bestTraj = predictTrajectory(x, u_best, predictTime, dt);
        else
            bestTraj = predictTrajectory(x, u_best, predictTime, dt, cfg, controllerState);
        end
        bestDmin = candidates(i,6);
    end
end

evalData.bestScore = bestScore;
evalData.bestDmin  = bestDmin;

end

function dw = calcDynamicWindow(x, params, dt)
v_now = x(4);
w_now = x(5);

v_low  = max(params.v_min, v_now - params.a_v * dt);
v_high = min(params.v_max, v_now + params.a_v * dt);

w_low  = max(params.w_min, w_now - params.a_w * dt);
w_high = min(params.w_max, w_now + params.a_w * dt);

dw = [v_low, v_high, w_low, w_high];
end

function [H, D, V, dmin, collision] = evaluateTrajectory( ...
    traj, goal, v, staticObs, dynamicObs, robot, dt, vmax, safetyMargin, globalPath, cfg)

startPose = traj(:,1);
finalPose = traj(:,end);
clearanceCap = max(vmax * max((size(traj,2) - 1) * dt, dt), robot.radius + safetyMargin);
theta_r = finalPose(3);
theta_target = atan2(goal(2) - finalPose(2), goal(1) - finalPose(1));
dtheta = wrapToPiLocal(theta_target - theta_r);

headingScore = 1 - abs(dtheta) / pi;
headingScore = max(0, headingScore);

dist0 = norm(goal - startPose(1:2));
distf = norm(goal - finalPose(1:2));
progressScore = max(0, dist0 - distf) / max(dist0, eps);

baseGuidance = min(1, max(0, 0.5 * headingScore + 0.5 * progressScore));
pathGuidance = [];
if ~isempty(globalPath) && size(globalPath,2) >= 2
    pathGuidance = scoreTrajectoryToPath(startPose, finalPose, globalPath, cfg);
end
if isempty(pathGuidance)
    H = baseGuidance;
else
    pathBlend = clampValue(cfg.globalPlanner.dwaPathBlend, 0, 1);
    H = min(1, max(0, (1 - pathBlend) * baseGuidance + pathBlend * pathGuidance));
end

dmin = clearanceCap;
collision = false;

for k = 1:size(traj,2)
    t = (k - 1) * dt;
    pos = traj(1:2, k);

    d_here = clearanceCap;
    for i = 1:numel(staticObs)
        ds = norm(pos - staticObs(i).pos) - ...
            (robot.radius + staticObs(i).radius + safetyMargin);
        d_here = min(d_here, ds);
    end
    for i = 1:numel(dynamicObs)
        dynPos = dynamicObs(i).pos + dynamicObs(i).vel * t;
        dd = norm(pos - dynPos) - ...
            (robot.radius + dynamicObs(i).radius + safetyMargin);
        d_here = min(d_here, dd);
    end

    dmin = min(dmin, d_here);

    if d_here <= 0
        collision = true;
        break;
    end
end

D = min(clearanceCap, max(0, dmin));

if size(traj,2) < 2
    avgSpeed = 0;
else
    segLens = sqrt(sum(diff(traj(1:2,:), 1, 2).^2, 1));
    avgSpeed = sum(segLens) / max((size(traj,2) - 1) * dt, eps);
end
V = min(1, avgSpeed / vmax);

end

function score = scoreTrajectoryToPath(startPose, finalPose, globalPath, cfg)
startPos = startPose(1:2);
finalPos = finalPose(1:2);

dStart = vecnorm(globalPath - startPos, 2, 1);
[~, idxStart] = min(dStart);

dEnd = vecnorm(globalPath - finalPos, 2, 1);
[minDist, idxEnd] = min(dEnd);

if idxEnd >= size(globalPath,2)
    tanVec = globalPath(:,idxEnd) - globalPath(:,max(1, idxEnd - 1));
else
    tanVec = globalPath(:,idxEnd + 1) - globalPath(:,idxEnd);
end
if norm(tanVec) < 1e-9
    score = [];
    return;
end

pathHeading = atan2(tanVec(2), tanVec(1));
pathHeadingScore = 1 - abs(wrapToPiLocal(pathHeading - finalPose(3))) / pi;
pathHeadingScore = max(0, pathHeadingScore);

progressIdx = max(0, idxEnd - idxStart);
pathProgressScore = progressIdx / max(size(globalPath,2) - 1, 1);

distScale = max(cfg.globalPlanner.pathTrackingDistScale, cfg.globalPlanner.resolution);
pathProximityScore = max(0, 1 - minDist / distScale);

score = min(1, max(0, ...
    0.45 * pathProximityScore + ...
    0.30 * pathHeadingScore + ...
    0.25 * pathProgressScore));
end

function y = normalizeColumn(x)
xmin = min(x);
xmax = max(x);

if abs(xmax - xmin) < 1e-9
    y = ones(size(x));
else
    y = (x - xmin) ./ (xmax - xmin);
end
end

function ang = wrapToPiLocal(ang)
ang = mod(ang + pi, 2*pi) - pi;
end

function y = clampValue(x, lo, hi)
y = min(max(x, lo), hi);
end
