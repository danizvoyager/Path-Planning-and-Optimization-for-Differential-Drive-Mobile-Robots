function scenario = generateScenarioRandom()

scenario.goal = [12; 0];

nStatic  = 7;
nDynamic = 6;

robotRadius = 0.135;
minSep = 0.7;

xMin = 2.0;  xMax = 11.0;
yMin = -2.3; yMax = 2.3;

scenario.staticObs = struct([]);
scenario.dynamicObs = struct([]);
%-----------------------------------------------------------------------------
% Static obstacles
%-----------------------------------------------------------------------------
count = 0;
attempts = 0;
while count < nStatic && attempts < 5000
    attempts = attempts + 1;

    pos = [xMin + (xMax - xMin) * rand();
           yMin + (yMax - yMin) * rand()];
    rad = 0.18 + 0.10 * rand();

    if isValidObstacle(pos, rad, scenario, robotRadius, minSep)
        count = count + 1;
        scenario.staticObs(count).pos = pos;
        scenario.staticObs(count).radius = rad;
    end
end
%-----------------------------------------------------------------------------
% Dynamic obstacles
%-----------------------------------------------------------------------------
count = 0;
attempts = 0;
while count < nDynamic && attempts < 5000
    attempts = attempts + 1;

    pos = [xMin + (xMax - xMin) * rand();
           yMin + (yMax - yMin) * rand()];
    rad = 0.14 + 0.06 * rand();
    vel = [-0.6 + 1.2 * rand();
           -0.6 + 1.2 * rand()];

    if norm(vel) < 0.15
        continue;
    end

    if isValidObstacle(pos, rad, scenario, robotRadius, minSep)
        count = count + 1;
        scenario.dynamicObs(count).pos = pos;
        scenario.dynamicObs(count).vel = vel;
        scenario.dynamicObs(count).radius = rad;
    end
end

end

function ok = isValidObstacle(pos, rad, scenario, robotRadius, minSep)

ok = true;

startPos = [0; 0];
goalPos  = [12; 0];

if norm(pos - startPos) < (rad + robotRadius + minSep)
    ok = false;
    return;
end

if norm(pos - goalPos) < (rad + robotRadius + minSep)
    ok = false;
    return;
end

for i = 1:numel(scenario.staticObs)
    d = norm(pos - scenario.staticObs(i).pos);
    if d < (rad + scenario.staticObs(i).radius + minSep)
        ok = false;
        return;
    end
end

for i = 1:numel(scenario.dynamicObs)
    d = norm(pos - scenario.dynamicObs(i).pos);
    if d < (rad + scenario.dynamicObs(i).radius + minSep)
        ok = false;
        return;
    end
end

end