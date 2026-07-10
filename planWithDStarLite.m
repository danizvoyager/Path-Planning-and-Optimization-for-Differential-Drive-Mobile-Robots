function [waypoints, plannerState] = planWithDStarLite(startXY, goalXY, staticObs, dynamicObs, plannerState, cfg)
%PLANWITHDSTARLITE Grid-based D* Lite global planner.
% Rebuilds the local occupancy grid around the scene and computes / repairs a
% shortest path when the map changes.

if nargin < 5 || isempty(plannerState)
    plannerState = struct();
end

occGrid = buildOccupancyGrid(staticObs, dynamicObs, cfg.globalPlanner);
startIdx = worldToGrid(startXY, occGrid);
goalIdx = worldToGrid(goalXY, occGrid);

startIdx = clampIndexToFree(startIdx, occGrid.occ);
goalIdx = clampIndexToFree(goalIdx, occGrid.occ);

needInit = ~isfield(plannerState, 'initialized') || ~plannerState.initialized;
sameGeometry = false;
mapChanged = true;
if ~needInit && isfield(plannerState, 'occ') && isfield(plannerState, 'occGrid')
    sameGeometry = isequal(size(plannerState.occ), size(occGrid.occ)) && ...
        isequal(plannerState.occGrid.xv, occGrid.xv) && ...
        isequal(plannerState.occGrid.yv, occGrid.yv);
    if sameGeometry
        mapChanged = ~isequal(plannerState.occ, occGrid.occ);
    end
end

goalChanged = needInit || ~isfield(plannerState, 'goal') || any(plannerState.goal ~= goalIdx);

if needInit || ~sameGeometry || goalChanged
    plannerState = initializeDStarLite(startIdx, goalIdx, occGrid);
else
    plannerState = updateStartState(plannerState, startIdx, goalIdx, occGrid);
    if mapChanged
        plannerState = updateChangedEdges(plannerState, occGrid);
    end
end

plannerState = computeShortestPath(plannerState, occGrid);
idxPath = extractPath(plannerState, occGrid, startIdx, goalIdx);
waypoints = gridPathToWorld(idxPath, occGrid);
plannerState.occ = occGrid.occ;
plannerState.occGrid = occGrid;
plannerState.initialized = true;
end

function planner = initializeDStarLite(startIdx, goalIdx, occGrid)
N = numel(occGrid.occ);
planner.g = inf(N,1);
planner.rhs = inf(N,1);
planner.km = 0;
planner.start = startIdx;
planner.last = startIdx;
planner.goal = goalIdx;
planner.open = [];
planner.rhs(sub2ind(size(occGrid.occ), goalIdx(1), goalIdx(2))) = 0;
planner = insertOrUpdateOpen(planner, goalIdx, calculateKey(planner, goalIdx, occGrid));
planner.occ = occGrid.occ;
planner.initialized = true;
end

function planner = updateStartState(planner, startIdx, goalIdx, occGrid)
planner.km = planner.km + heuristic(planner.last, startIdx, occGrid);
planner.start = startIdx;
planner.goal = goalIdx;
planner.last = startIdx;
end

function planner = updateChangedEdges(planner, occGrid)
oldOcc = planner.occ;
changed = find(oldOcc ~= occGrid.occ);
for idxLin = changed(:)'
    [r,c] = ind2sub(size(occGrid.occ), idxLin);
    planner = updateVertex(planner, [r c], occGrid);
    nbrs = getNeighbors([r c], occGrid);
    for k = 1:size(nbrs,1)
        planner = updateVertex(planner, nbrs(k,:), occGrid);
    end
end
end

function planner = computeShortestPath(planner, occGrid)
startLin = sub2ind(size(occGrid.occ), planner.start(1), planner.start(2));
iter = 0;
maxIter = numel(occGrid.occ) * 20;
while true
    topKey = [inf inf];
    if ~isempty(planner.open)
        [~,idx] = minrows(planner.open(:,3:4));
        topKey = planner.open(idx,3:4);
    end
    startKey = calculateKey(planner, planner.start, occGrid);
    if (~keyLess(topKey, startKey)) && planner.rhs(startLin) == planner.g(startLin)
        break;
    end
    if isempty(planner.open)
        break;
    end

    [~,idx] = minrows(planner.open(:,3:4));
    u = planner.open(idx,1:2);
    kOld = planner.open(idx,3:4);
    planner.open(idx,:) = [];
    kNew = calculateKey(planner, u, occGrid);
    uLin = sub2ind(size(occGrid.occ), u(1), u(2));

    if keyLess(kOld, kNew)
        planner = insertOrUpdateOpen(planner, u, kNew);
    elseif planner.g(uLin) > planner.rhs(uLin)
        planner.g(uLin) = planner.rhs(uLin);
        preds = getNeighbors(u, occGrid);
        for i = 1:size(preds,1)
            planner = updateVertex(planner, preds(i,:), occGrid);
        end
    else
        planner.g(uLin) = inf;
        planner = updateVertex(planner, u, occGrid);
        preds = getNeighbors(u, occGrid);
        for i = 1:size(preds,1)
            planner = updateVertex(planner, preds(i,:), occGrid);
        end
    end

    iter = iter + 1;
    if iter > maxIter
        break;
    end
end
end

function planner = updateVertex(planner, u, occGrid)
uLin = sub2ind(size(occGrid.occ), u(1), u(2));
if ~isequal(u, planner.goal)
    succ = getNeighbors(u, occGrid);
    rhsMin = inf;
    for i = 1:size(succ,1)
        s = succ(i,:);
        sLin = sub2ind(size(occGrid.occ), s(1), s(2));
        rhsMin = min(rhsMin, costBetween(u, s, occGrid) + planner.g(sLin));
    end
    planner.rhs(uLin) = rhsMin;
end
planner = removeFromOpen(planner, u);
if planner.g(uLin) ~= planner.rhs(uLin)
    planner = insertOrUpdateOpen(planner, u, calculateKey(planner, u, occGrid));
end
end

function key = calculateKey(planner, node, occGrid)
lin = sub2ind(size(occGrid.occ), node(1), node(2));
minGR = min(planner.g(lin), planner.rhs(lin));
key = [minGR + heuristic(planner.start, node, occGrid) + planner.km, minGR];
end

function h = heuristic(a, b, occGrid)
d = double(a - b);
h = norm(d) * occGrid.resolution;
end

function c = costBetween(a, b, occGrid)
if isBlocked(b, occGrid.occ)
    c = inf;
else
    step = double(a - b);
    c = norm(step) * occGrid.resolution;
end
end

function tf = isBlocked(idx, occ)
tf = occ(idx(1), idx(2));
end

function nbrs = getNeighbors(idx, occGrid)
dirs = [1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1];
nbrs = zeros(0,2);
for i = 1:size(dirs,1)
    n = idx + dirs(i,:);
    if n(1) >= 1 && n(1) <= occGrid.size(1) && n(2) >= 1 && n(2) <= occGrid.size(2)
        if ~occGrid.occ(n(1), n(2))
            nbrs(end+1,:) = n; %#ok<AGROW>
        end
    end
end
end

function planner = insertOrUpdateOpen(planner, node, key)
planner = removeFromOpen(planner, node);
planner.open = [planner.open; node, key]; %#ok<AGROW>
end

function planner = removeFromOpen(planner, node)
if isempty(planner.open)
    return;
end
mask = planner.open(:,1) == node(1) & planner.open(:,2) == node(2);
planner.open(mask,:) = [];
end

function tf = keyLess(a, b)
tf = (a(1) < b(1)) || (a(1) == b(1) && a(2) < b(2));
end

function [row, idx] = minrows(M)
[~, idx] = min(M(:,1) + 1e-9*M(:,2));
row = M(idx,:);
end

function idxPath = extractPath(planner, occGrid, startIdx, goalIdx)
idxPath = startIdx;
current = startIdx;
maxSteps = numel(occGrid.occ);
if all(startIdx == goalIdx)
    return;
end
for k = 1:maxSteps
    if all(current == goalIdx)
        break;
    end
    nbrs = getNeighbors(current, occGrid);
    if isempty(nbrs)
        break;
    end
    bestVal = inf;
    bestNbr = [];
    for i = 1:size(nbrs,1)
        s = nbrs(i,:);
        sLin = sub2ind(size(occGrid.occ), s(1), s(2));
        val = costBetween(current, s, occGrid) + planner.g(sLin);
        if val < bestVal
            bestVal = val;
            bestNbr = s;
        end
    end
    if isempty(bestNbr) || isinf(bestVal)
        break;
    end
    if size(idxPath,1) > 1 && all(bestNbr == idxPath(end-1,:))
        break;
    end
    current = bestNbr;
    idxPath(end+1,:) = current; %#ok<AGROW>
end
end

function waypoints = gridPathToWorld(idxPath, occGrid)
waypoints = zeros(2, size(idxPath,1));
for i = 1:size(idxPath,1)
    rc = idxPath(i,:);
    waypoints(:,i) = [occGrid.xv(rc(2)); occGrid.yv(rc(1))];
end
end

function occGrid = buildOccupancyGrid(staticObs, dynamicObs, plannerCfg)
allPts = [0 0; 12 0];
for i = 1:numel(staticObs)
    allPts(end+1,:) = staticObs(i).pos(:)'; %#ok<AGROW>
end
for i = 1:numel(dynamicObs)
    allPts(end+1,:) = dynamicObs(i).pos(:)'; %#ok<AGROW>
end
margin = 1.5;
res = plannerCfg.resolution;
if isfield(plannerCfg, 'xLimits') && numel(plannerCfg.xLimits) == 2
    minX = plannerCfg.xLimits(1);
    maxX = plannerCfg.xLimits(2);
else
    minX = floor((min(allPts(:,1)) - margin)/res)*res;
    maxX = ceil((max(allPts(:,1)) + margin)/res)*res;
end
if isfield(plannerCfg, 'yLimits') && numel(plannerCfg.yLimits) == 2
    minY = plannerCfg.yLimits(1);
    maxY = plannerCfg.yLimits(2);
else
    minY = floor((min(allPts(:,2)) - margin)/res)*res;
    maxY = ceil((max(allPts(:,2)) + margin)/res)*res;
end

xv = minX:res:maxX;
yv = minY:res:maxY;
occ = false(numel(yv), numel(xv));

for r = 1:numel(yv)
    for c = 1:numel(xv)
        p = [xv(c); yv(r)];
        for i = 1:numel(staticObs)
            if norm(p - staticObs(i).pos) <= staticObs(i).radius + plannerCfg.inflation
                occ(r,c) = true;
                break;
            end
        end
        if occ(r,c)
            continue;
        end
        for i = 1:numel(dynamicObs)
            if norm(p - dynamicObs(i).pos) <= dynamicObs(i).radius + plannerCfg.inflation
                occ(r,c) = true;
                break;
            end
        end
    end
end

occGrid.occ = occ;
occGrid.resolution = res;
occGrid.xv = xv;
occGrid.yv = yv;
occGrid.size = size(occ);
end

function idx = worldToGrid(p, occGrid)
[~, c] = min(abs(occGrid.xv - p(1)));
[~, r] = min(abs(occGrid.yv - p(2)));
idx = [r c];
end

function idx = clampIndexToFree(idx, occ)
if ~occ(idx(1), idx(2))
    return;
end
[rr,cc] = find(~occ);
D = (rr - idx(1)).^2 + (cc - idx(2)).^2;
[~,k] = min(D);
idx = [rr(k) cc(k)];
end
