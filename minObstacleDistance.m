function dmin = minObstacleDistance(pos, t, staticObs, dynamicObs, robot)
dmin = inf;

for i = 1:numel(staticObs)
    d = norm(pos - staticObs(i).pos) - robot.radius - staticObs(i).radius;
    dmin = min(dmin, d);
end

for i = 1:numel(dynamicObs)
    obsPos = dynamicObs(i).pos + dynamicObs(i).vel * t;
    d = norm(pos - obsPos) - robot.radius - dynamicObs(i).radius;
    dmin = min(dmin, d);
end
end