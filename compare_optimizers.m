clc; clear; close all;

rng('shuffle');
%-----------------------------------------------------------------------------
% Settings
%-----------------------------------------------------------------------------
nvars = 3;
lb = [0 0 0];
ub = [1 1 1];

nRuns = 5;
maxIter = 30;
popSize = 20;
fixedWeights = normalizeWeights([0.8 0.1 0.4]);
methodNames = {'Fixed DWA','GA','GWO','ACO','PSO'};
nMethods = numel(methodNames);

simMode = 'enhanced'; % 'baseline' or 'enhanced'
cfg = getSimulationConfig(simMode);

allCosts = nan(nRuns, nMethods);
allTimes = nan(nRuns, nMethods);
allPathLengths = nan(nRuns, nMethods);
allConvIters95 = nan(nRuns, nMethods);
allReached = nan(nRuns, nMethods);

bestConv = cell(1, nMethods);
bestWeights = cell(1, nMethods);
bestScenario = cell(1, nMethods);
bestFinalCost = inf(1, nMethods);

for runIdx = 1:nRuns
    fprintf('\n------------------- RUN %d / %d ----------------\n', runIdx, nRuns);
    fprintf('Simulation architecture: %s\n', upper(simMode));

    scenario = generateScenarioRandom();
    objFun = @(w) evaluateWeights(normalizeWeights(w), false, scenario, cfg);

    fprintf('\nRunning fixed-weight DWA reference...\n');
    tStart = tic;
    [J_fixed, ~] = evaluateWeights(fixedWeights, false, scenario, cfg);
    tFixed = toc(tStart);
    allCosts(runIdx, 1) = J_fixed;
    allTimes(runIdx, 1) = tFixed;
    allConvIters95(runIdx, 1) = NaN;
    if J_fixed < bestFinalCost(1)
        bestFinalCost(1) = J_fixed;
        bestConv{1} = repmat(J_fixed, maxIter, 1);
        bestWeights{1} = fixedWeights;
        bestScenario{1} = scenario;
    end

    fprintf('\nRunning GA...\n');
    tStart = tic;
    [w_ga_raw, J_ga, conv_ga] = runGA(objFun, nvars, lb, ub, popSize, maxIter);
    tGA = toc(tStart);
    w_ga = normalizeWeights(w_ga_raw);
    allCosts(runIdx, 2) = J_ga;
    allTimes(runIdx, 2) = tGA;
    allConvIters95(runIdx, 2) = getConvIter95(conv_ga);
    if J_ga < bestFinalCost(2)
        bestFinalCost(2) = J_ga;
        bestConv{2} = conv_ga;
        bestWeights{2} = w_ga;
        bestScenario{2} = scenario;
    end

    fprintf('\nRunning GWO...\n');
    tStart = tic;
    [w_gwo_raw, J_gwo, conv_gwo] = runGWO(objFun, nvars, lb, ub, popSize, maxIter);
    tGWO = toc(tStart);
    w_gwo = normalizeWeights(w_gwo_raw);
    allCosts(runIdx, 3) = J_gwo;
    allTimes(runIdx, 3) = tGWO;
    allConvIters95(runIdx, 3) = getConvIter95(conv_gwo);
    if J_gwo < bestFinalCost(3)
        bestFinalCost(3) = J_gwo;
        bestConv{3} = conv_gwo;
        bestWeights{3} = w_gwo;
        bestScenario{3} = scenario;
    end

    fprintf('\nRunning ACO...\n');
    tStart = tic;
    [w_aco_raw, J_aco, conv_aco] = runACO(objFun, nvars, lb, ub, popSize, maxIter);
    tACO = toc(tStart);
    w_aco = normalizeWeights(w_aco_raw);
    allCosts(runIdx, 4) = J_aco;
    allTimes(runIdx, 4) = tACO;
    allConvIters95(runIdx, 4) = getConvIter95(conv_aco);
    if J_aco < bestFinalCost(4)
        bestFinalCost(4) = J_aco;
        bestConv{4} = conv_aco;
        bestWeights{4} = w_aco;
        bestScenario{4} = scenario;
    end

    fprintf('\nRunning PSO...\n');
    tStart = tic;
    [w_pso_raw, J_pso, conv_pso] = runPSO(objFun, nvars, lb, ub, popSize, maxIter);
    tPSO = toc(tStart);
    w_pso = normalizeWeights(w_pso_raw);
    allCosts(runIdx, 5) = J_pso;
    allTimes(runIdx, 5) = tPSO;
    allConvIters95(runIdx, 5) = getConvIter95(conv_pso);
    if J_pso < bestFinalCost(5)
        bestFinalCost(5) = J_pso;
        bestConv{5} = conv_pso;
        bestWeights{5} = w_pso;
        bestScenario{5} = scenario;
    end

    figure('Name', sprintf('Run %d - Convergence', runIdx), 'NumberTitle', 'off', 'Color', 'w');
    hold on; grid on;
    plot(repmat(J_fixed, maxIter, 1), 'LineWidth', 2);
    plot(conv_ga,  'LineWidth', 2);
    plot(conv_gwo, 'LineWidth', 2);
    plot(conv_aco, 'LineWidth', 2);
    plot(conv_pso, 'LineWidth', 2);
    xlabel('Iteration'); ylabel('Best Cost');
    title(sprintf('Convergence Comparison - Scenario - %d', runIdx));
    legend(methodNames, 'Location', 'northeast');
    hold off;
    saveFigureA4Landscape(gcf, sprintf('Run_%02d_Convergence_%s.fig', runIdx, simMode));

    [Jfixed_path, simFixed] = evaluateWeights(fixedWeights, false, scenario, cfg);
    [Jga_path,  simGA]      = evaluateWeights(w_ga,  false, scenario, cfg);
    [Jgwo_path, simGWO]     = evaluateWeights(w_gwo, false, scenario, cfg);
    [Jaco_path, simACO]     = evaluateWeights(w_aco, false, scenario, cfg);
    [Jpso_path, simPSO]     = evaluateWeights(w_pso, false, scenario, cfg);
    trajFixed = getTrajectory(simFixed);
    trajGA  = getTrajectory(simGA);
    trajGWO = getTrajectory(simGWO);
    trajACO = getTrajectory(simACO);
    trajPSO = getTrajectory(simPSO);

    allPathLengths(runIdx, 1) = computePathLength(trajFixed);
    allPathLengths(runIdx, 2) = computePathLength(trajGA);
    allPathLengths(runIdx, 3) = computePathLength(trajGWO);
    allPathLengths(runIdx, 4) = computePathLength(trajACO);
    allPathLengths(runIdx, 5) = computePathLength(trajPSO);

    allReached(runIdx, 1) = simFixed.reached;
    allReached(runIdx, 2) = simGA.reached;
    allReached(runIdx, 3) = simGWO.reached;
    allReached(runIdx, 4) = simACO.reached;
    allReached(runIdx, 5) = simPSO.reached;

    fprintf('\nRun %d final re-evaluation:\n', runIdx);
    fprintf('Fixed DWA : J = %.6f, reached = %d\n', Jfixed_path, simFixed.reached);
    fprintf('GA        : J = %.6f, reached = %d\n', Jga_path,    simGA.reached);
    fprintf('GWO       : J = %.6f, reached = %d\n', Jgwo_path,   simGWO.reached);
    fprintf('ACO       : J = %.6f, reached = %d\n', Jaco_path,   simACO.reached);
    fprintf('PSO       : J = %.6f, reached = %d\n', Jpso_path,   simPSO.reached);

    figure('Name', sprintf('Run %d - Paths', runIdx), 'NumberTitle', 'off', 'Color', 'w');
    hold on; grid on; axis equal;
    colors = lines(nMethods);
    h = []; legendText = {};
    x0 = [0; 0];

    if isfield(scenario, 'goal') && ~isempty(scenario.goal)
        goal = scenario.goal;
        h(end+1) = plot(goal(1), goal(2), 'gp', 'MarkerSize', 14, 'LineWidth', 2);
        legendText{end+1} = 'Goal';
    end
    h(end+1) = plot(x0(1), x0(2), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    legendText{end+1} = 'Start';

    if isfield(scenario, 'staticObs') && ~isempty(scenario.staticObs)
        for i = 1:numel(scenario.staticObs)
            drawCircleLocal(scenario.staticObs(i).pos(1), scenario.staticObs(i).pos(2), scenario.staticObs(i).radius, [0.85 0.2 0.2]);
        end
        h(end+1) = plot(nan, nan, 'o', 'MarkerFaceColor', [0.85 0.2 0.2], 'MarkerEdgeColor', [0.85 0.2 0.2]);
        legendText{end+1} = 'Static Obstacles';
    end

    if isfield(simGA, 'dynamicObs') && ~isempty(simGA.dynamicObs)
        for i = 1:numel(simGA.dynamicObs)
            drawCircleLocal(simGA.dynamicObs(i).pos(1), simGA.dynamicObs(i).pos(2), simGA.dynamicObs(i).radius, [0.2 0.2 0.85]);
            quiver(simGA.dynamicObs(i).pos(1), simGA.dynamicObs(i).pos(2), simGA.dynamicObs(i).vel(1), simGA.dynamicObs(i).vel(2), 0, 'Color', [0.2 0.2 0.85], 'LineWidth', 1.5);
        end
        h(end+1) = plot(nan, nan, 'o', 'MarkerFaceColor', [0.2 0.2 0.85], 'MarkerEdgeColor', [0.2 0.2 0.85]);
        legendText{end+1} = 'Dynamic Obstacles';
    end

    traj = getTrajectory(simFixed); if ~isempty(traj), h(end+1)=plot(traj(:,1), traj(:,2), 'LineWidth', 2, 'Color', colors(1,:)); legendText{end+1}='Fixed DWA'; end
    traj = getTrajectory(simGA);    if ~isempty(traj), h(end+1)=plot(traj(:,1), traj(:,2), 'LineWidth', 2, 'Color', colors(2,:)); legendText{end+1}='GA'; end
    traj = getTrajectory(simGWO);   if ~isempty(traj), h(end+1)=plot(traj(:,1), traj(:,2), 'LineWidth', 2, 'Color', colors(3,:)); legendText{end+1}='GWO'; end
    traj = getTrajectory(simACO);   if ~isempty(traj), h(end+1)=plot(traj(:,1), traj(:,2), 'LineWidth', 2, 'Color', colors(4,:)); legendText{end+1}='ACO'; end
    traj = getTrajectory(simPSO);   if ~isempty(traj), h(end+1)=plot(traj(:,1), traj(:,2), 'LineWidth', 2, 'Color', colors(5,:)); legendText{end+1}='PSO'; end

    xlabel('X'); ylabel('Y');
    title(sprintf('Path Comparison - Scenario - %d', runIdx));
    legend(h, legendText, 'Location', 'best'); hold off;
    saveFigureA4Landscape(gcf, sprintf('Run_%02d_Paths_%s.fig', runIdx, simMode));

    figure('Name', sprintf('Run %d - Control Outputs', runIdx), 'NumberTitle', 'off', 'Color', 'w');
    tiledlayout(2,1);

    nexttile;
    hold on; grid on;
    plotControlSeries(simFixed, colors(1,:), 'Fixed DWA', 1);
    plotControlSeries(simGA,    colors(2,:), 'GA',        1);
    plotControlSeries(simGWO,   colors(3,:), 'GWO',       1);
    plotControlSeries(simACO,   colors(4,:), 'ACO',       1);
    plotControlSeries(simPSO,   colors(5,:), 'PSO',       1);
    ylabel('u_v');
    title(sprintf('Controller Output - Scenario - %d', runIdx));
    legend(methodNames, 'Location', 'best');
    hold off;

    nexttile;
    hold on; grid on;
    plotControlSeries(simFixed, colors(1,:), 'Fixed DWA', 2);
    plotControlSeries(simGA,    colors(2,:), 'GA',        2);
    plotControlSeries(simGWO,   colors(3,:), 'GWO',       2);
    plotControlSeries(simACO,   colors(4,:), 'ACO',       2);
    plotControlSeries(simPSO,   colors(5,:), 'PSO',       2);
    xlabel('Time [s]');
    ylabel('u_w');
    hold off;

    saveFigureA4Landscape(gcf, sprintf('Run_%02d_Control_%s.fig', runIdx, simMode));
end

meanCost = mean(allCosts, 1, 'omitnan');
stdCost  = std(allCosts, 0, 1, 'omitnan');
meanTime = mean(allTimes, 1, 'omitnan');
stdTime  = std(allTimes, 0, 1, 'omitnan');

fprintf('\n\n------------------ SUMMARY --------------------------\n');
for m = 1:nMethods
    fprintf('%s:\n', methodNames{m});
    fprintf('  Mean Cost   = %.6f\n', meanCost(m));
    fprintf('  Std Cost    = %.6f\n', stdCost(m));
    fprintf('  Mean Time   = %.4f s\n', meanTime(m));
    fprintf('  Std Time    = %.4f s\n', stdTime(m));
    fprintf('  Best Cost   = %.6f\n\n', bestFinalCost(m));
end

meanPath = mean(allPathLengths, 1, 'omitnan');
stdPath  = std(allPathLengths, 0, 1, 'omitnan');
meanConv95 = mean(allConvIters95, 1, 'omitnan');
stdConv95  = std(allConvIters95, 0, 1, 'omitnan');
reachRate = 100 * mean(allReached, 1, 'omitnan');
minCostEach = min(allCosts, [], 1);
minPathEach = min(allPathLengths, [], 1);
minTimeEach = min(allTimes, [], 1);

fprintf('\n\n-------------- NUMERICAL PERFORMANCE ANALYSIS ----------------\n');
for m = 1:nMethods
    fprintf('%s:\n', methodNames{m});
    fprintf('  Minimum Cost            = %.6f\n', minCostEach(m));
    fprintf('  Mean Path Length        = %.6f\n', meanPath(m));
    fprintf('  Std Path Length         = %.6f\n', stdPath(m));
    fprintf('  Minimum Path Length     = %.6f\n', minPathEach(m));
    fprintf('  Mean Runtime            = %.6f s\n', meanTime(m));
    fprintf('  Minimum Runtime         = %.6f s\n', minTimeEach(m));
    fprintf('  Mean Convergence Iter   = %.2f\n', meanConv95(m));
    fprintf('  Std Convergence Iter    = %.2f\n', stdConv95(m));
    fprintf('  Goal Reached Rate       = %.2f %%\n\n', reachRate(m));
end

[worstMeanCost, idxWorstCost] = max(meanCost);
[worstMeanPath, idxWorstPath] = max(meanPath);
[worstMeanTime, idxWorstTime] = max(meanTime);

fprintf('\n-------------------- PERCENTAGE IMPROVEMENT -----------------------\n');
fprintf('Reference for cost improvement: %s\n', methodNames{idxWorstCost});
for m = 1:nMethods
    impCost = 100 * (worstMeanCost - meanCost(m)) / worstMeanCost;
    fprintf('  %s cost improvement = %.2f %%\n', methodNames{m}, impCost);
end
fprintf('\nReference for path-length improvement: %s\n', methodNames{idxWorstPath});
for m = 1:nMethods
    impPath = 100 * (worstMeanPath - meanPath(m)) / worstMeanPath;
    fprintf('  %s path improvement = %.2f %%\n', methodNames{m}, impPath);
end
fprintf('\nReference for runtime improvement: %s\n', methodNames{idxWorstTime});
for m = 1:nMethods
    impTime = 100 * (worstMeanTime - meanTime(m)) / worstMeanTime;
    fprintf('  %s runtime improvement = %.2f %%\n', methodNames{m}, impTime);
end

figure('Name','Final Cost Boxplot','NumberTitle','off','Color','w');
boxplot(allCosts, 'Labels', methodNames); ylabel('Final Cost J');
title(sprintf('Optimizer Comparison - Final Cost (%s)', upper(simMode))); grid on;
saveFigureA4Landscape(gcf, sprintf('Summary_Final_Cost_Boxplot_%s.fig', simMode));

figure('Name','Mean Final Cost','NumberTitle','off','Color','w');
bar(meanCost); set(gca, 'XTick', 1:nMethods, 'XTickLabel', methodNames);
ylabel('Mean Final Cost'); title(sprintf('Mean Final Cost Comparison (%s)', upper(simMode))); grid on;
saveFigureA4Landscape(gcf, sprintf('Summary_Mean_Final_Cost_%s.fig', simMode));

figure('Name','Mean Runtime','NumberTitle','off','Color','w');
bar(meanTime); set(gca, 'XTick', 1:nMethods, 'XTickLabel', methodNames);
ylabel('Mean Runtime (s)'); title(sprintf('Mean Runtime Comparison (%s)', upper(simMode))); grid on;
saveFigureA4Landscape(gcf, sprintf('Summary_Mean_Runtime_%s.fig', simMode));

figure('Name','Best Convergence Curves','NumberTitle','off','Color','w');
hold on; grid on;
for m = 1:nMethods, plot(bestConv{m}, 'LineWidth', 2); end
xlabel('Iteration'); ylabel('Best Cost');
title(sprintf('Best Convergence Curve of Each Optimizer (%s)', upper(simMode)));
legend(methodNames, 'Location', 'northeast'); hold off;
saveFigureA4Landscape(gcf, sprintf('Summary_Best_Convergence_Curves_%s.fig', simMode));

function w = normalizeWeights(w)
    w = max(w, 0);
    if sum(w) <= 0, w = [1 1 1]; end
    w = w / sum(w);
end

function traj = getTrajectory(simStruct)
    traj = [];
    if isfield(simStruct, 'hist_state') && ~isempty(simStruct.hist_state)
        traj = simStruct.hist_state(1:2, :)';
    end
end

function L = computePathLength(traj)
    if isempty(traj) || size(traj,1) < 2
        L = 0; return;
    end
    diffs = diff(traj,1,1);
    L = sum(sqrt(sum(diffs.^2,2)));
end

function k95 = getConvIter95(conv)
    if isempty(conv)
        k95 = NaN; return;
    end
    c0 = conv(1); cf = conv(end);
    thr = cf + 0.05*(c0 - cf);
    idx = find(conv <= thr, 1, 'first');
    if isempty(idx), k95 = numel(conv); else, k95 = idx; end
end

function drawCircleLocal(xc, yc, r, c)
    th = linspace(0, 2*pi, 100);
    x = xc + r*cos(th); y = yc + r*sin(th);
    fill(x, y, c, 'FaceAlpha', 0.35, 'EdgeColor', c, 'LineWidth', 1.2);
end

function saveFigureA4Landscape(figHandle, fileName)
    if nargin < 1 || isempty(figHandle)
        figHandle = gcf;
    end

    if nargin < 2 || isempty(fileName)
        fileName = 'figure.fig';
    end

    outDir = 'fig_results';
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    [~, name, ~] = fileparts(fileName);
    filePath = fullfile(outDir, [name '.fig']);

    % Save only as MATLAB editable .fig file
    savefig(figHandle, filePath);
end

function plotControlSeries(simStruct, c, labelText, idx)
    if ~isfield(simStruct, 'hist_time') || isempty(simStruct.hist_time) || ...
            ~isfield(simStruct, 'hist_u') || isempty(simStruct.hist_u)
        return;
    end
    plot(simStruct.hist_time, simStruct.hist_u(idx,:), 'LineWidth', 1.5, ...
        'Color', c, 'DisplayName', labelText);
end

function [bestPos, bestCost, conv] = runGA(objFun, nvars, lb, ub, popSize, maxIter)
    conv = nan(maxIter, 1);
    opts = optimoptions('ga', 'PopulationSize', popSize, 'MaxGenerations', maxIter, 'Display', 'off', 'EliteCount', 2, 'CrossoverFraction', 0.8, 'MutationFcn', @mutationadaptfeasible, 'OutputFcn', @gaOutFcn);
    [bestPos, bestCost] = ga(objFun, nvars, [], [], [], [], lb, ub, [], opts);
    lastValid = find(~isnan(conv), 1, 'last');
    if isempty(lastValid), conv(:) = bestCost; elseif lastValid < maxIter, conv(lastValid+1:end) = conv(lastValid); end
    function [state, options, optchanged] = gaOutFcn(options, state, flag)
        optchanged = false;
        switch flag
            case 'iter'
                idx = max(1, min(maxIter, state.Generation + 1));
                conv(idx) = state.Best(end);
        end
    end
end

function [Alpha_pos, Alpha_score, Convergence_curve] = runGWO(objFun, nvars, lb, ub, nAgents, Max_iter)
    Positions = rand(nAgents, nvars) .* repmat((ub - lb), nAgents, 1) + repmat(lb, nAgents, 1);
    Alpha_pos = zeros(1, nvars); Alpha_score = inf;
    Beta_pos = zeros(1, nvars); Beta_score = inf;
    Delta_pos = zeros(1, nvars); Delta_score = inf;
    Convergence_curve = zeros(Max_iter, 1);
    for l = 1:Max_iter
        for i = 1:nAgents
            Positions(i,:) = max(Positions(i,:), lb);
            Positions(i,:) = min(Positions(i,:), ub);
            fitness = objFun(Positions(i,:));
            if fitness < Alpha_score
                Delta_score = Beta_score; Delta_pos = Beta_pos;
                Beta_score = Alpha_score; Beta_pos = Alpha_pos;
                Alpha_score = fitness; Alpha_pos = Positions(i,:);
            elseif fitness < Beta_score
                Delta_score = Beta_score; Delta_pos = Beta_pos;
                Beta_score = fitness; Beta_pos = Positions(i,:);
            elseif fitness < Delta_score
                Delta_score = fitness; Delta_pos = Positions(i,:);
            end
        end
        a = 2 - l * (2 / Max_iter);
        for i = 1:nAgents
            for j = 1:nvars
                r1 = rand(); r2 = rand(); A1 = 2*a*r1 - a; C1 = 2*r2;
                D_alpha = abs(C1*Alpha_pos(j) - Positions(i,j)); X1 = Alpha_pos(j) - A1*D_alpha;
                r1 = rand(); r2 = rand(); A2 = 2*a*r1 - a; C2 = 2*r2;
                D_beta = abs(C2*Beta_pos(j) - Positions(i,j)); X2 = Beta_pos(j) - A2*D_beta;
                r1 = rand(); r2 = rand(); A3 = 2*a*r1 - a; C3 = 2*r2;
                D_delta = abs(C3*Delta_pos(j) - Positions(i,j)); X3 = Delta_pos(j) - A3*D_delta;
                Positions(i,j) = (X1 + X2 + X3) / 3;
            end
        end
        Convergence_curve(l) = Alpha_score;
    end
end

function [bestSol, bestCost, convCurve] = runACO(objFun, nvars, lb, ub, nAnts, maxIter)
    tau = ones(1,nvars); alpha = 1; beta = 2; rho = 0.25; Q = 1;
    bestSol = zeros(1,nvars); bestCost = inf; convCurve = nan(maxIter,1);
    for it = 1:maxIter
        sols = zeros(nAnts,nvars); costs = inf(nAnts,1);
        for k = 1:nAnts
            eta = rand(1,nvars);
            p = (tau.^alpha) .* (eta.^beta); p = p ./ max(sum(p), eps);
            sols(k,:) = lb + (ub-lb) .* ((p + rand(1,nvars))/2);
            costs(k) = objFun(sols(k,:));
        end
        [iterBestCost, idx] = min(costs); iterBestSol = sols(idx,:);
        if iterBestCost < bestCost, bestCost = iterBestCost; bestSol = iterBestSol; end
        tau = (1-rho)*tau + Q ./ (1 + abs(iterBestSol));
        convCurve(it) = bestCost;
    end
end

function [gBest, gBestCost, convCurve] = runPSO(objFun, nvars, lb, ub, nPop, maxIter)
    w = 0.7; c1 = 1.5; c2 = 1.5;
    X = rand(nPop,nvars) .* (ub-lb) + lb; V = zeros(nPop,nvars);
    pBest = X; pBestCost = inf(nPop,1); gBest = zeros(1,nvars); gBestCost = inf; convCurve = nan(maxIter,1);
    for it = 1:maxIter
        for i = 1:nPop
            cost = objFun(X(i,:));
            if cost < pBestCost(i), pBestCost(i) = cost; pBest(i,:) = X(i,:); end
            if cost < gBestCost, gBestCost = cost; gBest = X(i,:); end
        end
        for i = 1:nPop
            V(i,:) = w*V(i,:) + c1*rand(1,nvars).*(pBest(i,:)-X(i,:)) + c2*rand(1,nvars).*(gBest-X(i,:));
            X(i,:) = min(max(X(i,:) + V(i,:), lb), ub);
        end
        convCurve(it) = gBestCost;
    end
end
