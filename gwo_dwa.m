clc; clear; close all;

objFun = @(w) evaluateWeights(normalizeWeights(w), false);

nvars = 3;
lb = [0 0 0];
ub = [1 1 1];
%-----------------------------------------------------------------------------
% GWO parameters
%-----------------------------------------------------------------------------
SearchAgents_no = 20;
Max_iter        = 30;
%-----------------------------------------------------------------------------
% Initialize wolf positions
%-----------------------------------------------------------------------------
Positions = rand(SearchAgents_no, nvars) .* repmat((ub - lb), SearchAgents_no, 1) + repmat(lb, SearchAgents_no, 1);

Alpha_pos = zeros(1, nvars);
Alpha_score = inf;

Beta_pos = zeros(1, nvars);
Beta_score = inf;

Delta_pos = zeros(1, nvars);
Delta_score = inf;

Convergence_curve = zeros(Max_iter, 1);
%-----------------------------------------------------------------------------
% Main loop
%-----------------------------------------------------------------------------
for l = 1:Max_iter

    for i = 1:SearchAgents_no
        Positions(i,:) = max(Positions(i,:), lb);
        Positions(i,:) = min(Positions(i,:), ub);

        fitness = objFun(Positions(i,:));

        if fitness < Alpha_score
            Delta_score = Beta_score;
            Delta_pos = Beta_pos;

            Beta_score = Alpha_score;
            Beta_pos = Alpha_pos;

            Alpha_score = fitness;
            Alpha_pos = Positions(i,:);
        elseif fitness < Beta_score
            Delta_score = Beta_score;
            Delta_pos = Beta_pos;

            Beta_score = fitness;
            Beta_pos = Positions(i,:);
        elseif fitness < Delta_score
            Delta_score = fitness;
            Delta_pos = Positions(i,:);
        end
    end

    a = 2 - l * (2 / Max_iter);

    for i = 1:SearchAgents_no
        for j = 1:nvars
            r1 = rand(); r2 = rand();
            A1 = 2 * a * r1 - a;
            C1 = 2 * r2;
            D_alpha = abs(C1 * Alpha_pos(j) - Positions(i,j));
            X1 = Alpha_pos(j) - A1 * D_alpha;

            r1 = rand(); r2 = rand();
            A2 = 2 * a * r1 - a;
            C2 = 2 * r2;
            D_beta = abs(C2 * Beta_pos(j) - Positions(i,j));
            X2 = Beta_pos(j) - A2 * D_beta;

            r1 = rand(); r2 = rand();
            A3 = 2 * a * r1 - a;
            C3 = 2 * r2;
            D_delta = abs(C3 * Delta_pos(j) - Positions(i,j));
            X3 = Delta_pos(j) - A3 * D_delta;

            Positions(i,j) = (X1 + X2 + X3) / 3;
        end
    end

    Convergence_curve(l) = Alpha_score;
    fprintf('Iteration %2d/%2d, Best Cost = %.6f\n', l, Max_iter, Alpha_score);
end
%-----------------------------------------------------------------------------
% Best solution
%-----------------------------------------------------------------------------
w_best_raw = Alpha_pos;
w_best = normalizeWeights(w_best_raw);
J_best = Alpha_score;

fprintf('\nBest weights found using GWO:\n');
fprintf('alpha = %.4f\n', w_best(1));
fprintf('beta  = %.4f\n', w_best(2));
fprintf('gamma = %.4f\n', w_best(3));
fprintf('Best cost J = %.4f\n', J_best);

%-----------------------------------------------------------------------------
% Convergence plot
%-----------------------------------------------------------------------------
hConv = figure('Name','GWO-DWA Convergence Plot','NumberTitle','off','Color','w');
clf(hConv);
plot(1:Max_iter, Convergence_curve, 'LineWidth', 1.5);
grid on;
xlabel('Iteration');
ylabel('Best cost');
title('GWO convergence');

%-----------------------------------------------------------------------------
% Run best solution with animation
%-----------------------------------------------------------------------------
hAnim = figure('Name','GWO-DWA animation','NumberTitle','off','Color','w');
clf(hAnim);
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