function [trackingGoal, plannerState] = selectTrackingGoal(x, goal, staticObs, dynamicObs, plannerState, cfg)
%SELECTTRACKINGGOAL Choose a local tracking goal from the D* Lite global path.

trackingGoal = goal;
if ~cfg.globalPlanner.enabled
    return;
end

[waypoints, plannerState] = planWithDStarLite(x(1:2), goal, staticObs, dynamicObs, plannerState, cfg);
plannerState.waypoints = waypoints;

if isempty(waypoints)
    trackingGoal = goal;
    return;
end

lookAheadIdx = min(size(waypoints,2), cfg.globalPlanner.lookAheadIdx);
distIdx = [];
for i = 1:size(waypoints,2)
    if norm(waypoints(:,i) - x(1:2)) > cfg.globalPlanner.minWaypointDist
        distIdx = i;
        break;
    end
end

if isempty(distIdx)
    targetIdx = size(waypoints,2);
else
    targetIdx = max(distIdx, lookAheadIdx);
end

trackingGoal = waypoints(:,targetIdx);
end
