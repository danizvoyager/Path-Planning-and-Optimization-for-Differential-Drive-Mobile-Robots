clc; clear; close all;
objFun = @(w) evaluateWeights(normalizeWeights(w), false);

nvars = 3;
lb = [0 0 0];
ub = [1 1 1];
max_iter=30;
pop_size=20;
opts = optimoptions('ga', ...
    'PopulationSize', pop_size, ...
    'MaxGenerations', max_iter, ...
    'EliteCount', 2, ...
    'CrossoverFraction', 0.8, ...
    'MutationFcn', @mutationadaptfeasible, ...
    'FunctionTolerance', 1e-6, ...
    'Display', 'iter');
opts = optimoptions(opts,'PlotFcn',@gaplotbestf);
[w_best_raw, J_best] = ga(objFun, nvars, [], [], [], [], lb, ub, [], opts);

w_best = normalizeWeights(w_best_raw);

fprintf('Best weights found using GA:\n');
fprintf('alpha = %.4f\n', w_best(1));
fprintf('beta  = %.4f\n', w_best(2));
fprintf('gamma = %.4f\n', w_best(3));
fprintf('Best cost J = %.4f\n', J_best);
%-----------------------------------------------------------------------------
% Run best solution with animation
%-----------------------------------------------------------------------------
figure('Name','GA-DWA animation','NumberTitle','off');
clf;
[J_final, sim] = evaluateWeights(w_best, true);
fprintf('\nSimulation with optimized weights:\n');
fprintf('J = %.4f\n', J_final);
fprintf('Reached goal: %d\n', sim.reached);

function w = normalizeWeights(w)
w = max(w, 0);
if sum(w) <= 0
    w = [1 1 1];
end
w = w / sum(w);
end
