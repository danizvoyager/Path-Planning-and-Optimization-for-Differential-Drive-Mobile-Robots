clc; clear; close all;
nvars = 3;
lb = [0 0 0];
ub = [1 1 1];

objFun = @(w) evaluateWeights(normalizeWeights(w), false);
%-----------------------------------------------------------------------------
% ACOR parameters
%-----------------------------------------------------------------------------
maxIter     = 30;  
archiveSize = 20;   
numAnts     = 20;  
q           = 0.5;  
zeta        = 1.0;  
%-----------------------------------------------------------------------------
% Initialize solution archive
%-----------------------------------------------------------------------------
archivePos  = repmat(lb, archiveSize, 1) + rand(archiveSize, nvars) .* repmat((ub - lb), archiveSize, 1);
archiveCost = zeros(archiveSize, 1);

for i = 1:archiveSize
    archiveCost(i) = objFun(archivePos(i,:));
end

[archiveCost, sortIdx] = sort(archiveCost);
archivePos = archivePos(sortIdx,:);

bestCost = zeros(maxIter, 1);
%-----------------------------------------------------------------------------
% Main ACOR loop
%-----------------------------------------------------------------------------
for it = 1:maxIter
    %-----------------------------------------------------------------------------
    % Rank-based Gaussian kernel weights
    %-----------------------------------------------------------------------------
    ranks = (1:archiveSize)';
    probs = 1 ./ (sqrt(2*pi) * q * archiveSize) .* ...
        exp(-0.5 * ((ranks - 1) ./ (q * archiveSize)).^2);
    probs = probs / sum(probs);

    newPos  = zeros(numAnts, nvars);
    newCost = zeros(numAnts, 1);

    for k = 1:numAnts
        candidate = zeros(1, nvars);

        for j = 1:nvars
            selected = rouletteWheelSelection(probs);

            if archiveSize == 1
                sigma = zeta * (ub(j) - lb(j));
            else
                sigma = zeta * sum(abs(archivePos(:,j) - archivePos(selected,j))) / (archiveSize - 1);
            end
            %-----------------------------------------------------------------------------
            % Avoid zero-variance collapse
            %-----------------------------------------------------------------------------
            sigma = max(sigma, 1e-3 * (ub(j) - lb(j)));

            candidate(j) = archivePos(selected,j) + sigma * randn();
        end
        %-----------------------------------------------------------------------------
        % Bound handling
        %-----------------------------------------------------------------------------
        candidate = max(candidate, lb);
        candidate = min(candidate, ub);

        newPos(k,:)  = candidate;
        newCost(k,1) = objFun(candidate);
    end
    %-----------------------------------------------------------------------------
    % Merge old archive and new ants, then keep best archiveSize
    %-----------------------------------------------------------------------------
    allPos  = [archivePos; newPos];
    allCost = [archiveCost; newCost];

    [allCost, sortIdx] = sort(allCost);
    allPos = allPos(sortIdx,:);

    archivePos  = allPos(1:archiveSize,:);
    archiveCost = allCost(1:archiveSize);

    bestCost(it) = archiveCost(1);

    fprintf('Iteration %2d/%2d, Best Cost = %.6f\n', it, maxIter, bestCost(it));
end
%-----------------------------------------------------------------------------
% Best solution
%-----------------------------------------------------------------------------
w_best_raw = archivePos(1,:);
w_best     = normalizeWeights(w_best_raw);
J_best     = archiveCost(1);

fprintf('\nBest weights found using ACOR:\n');
fprintf('alpha = %.4f\n', w_best(1));
fprintf('beta  = %.4f\n', w_best(2));
fprintf('gamma = %.4f\n', w_best(3));
fprintf('Best cost J = %.4f\n', J_best);
%-----------------------------------------------------------------------------
% Convergence plot
%-----------------------------------------------------------------------------
figure('Name','ACO-DWA Convergence Plot','NumberTitle','off','Color','w');
clf;
plot(1:maxIter, bestCost, 'LineWidth', 2);
grid on;
xlabel('Iteration');
ylabel('Best cost');
title('ACOR convergence');
%-----------------------------------------------------------------------------
% Run best solution with animation
%-----------------------------------------------------------------------------
figure('Name','ACO-DWA animation','NumberTitle','off','Color','w');
clf;
[J_final, sim] = evaluateWeights(w_best, true);

fprintf('\nSimulation with optimized weights:\n');
fprintf('J = %.4f\n', J_final);
fprintf('Reached goal: %d\n', sim.reached);

function idx = rouletteWheelSelection(p)
r = rand();
c = cumsum(p);
idx = find(r <= c, 1, 'first');
if isempty(idx)
    idx = numel(p);
end
end

function w = normalizeWeights(w)
w = max(w, 0);
if sum(w) <= 0
    w = [1 1 1];
end
w = w / sum(w);
end
