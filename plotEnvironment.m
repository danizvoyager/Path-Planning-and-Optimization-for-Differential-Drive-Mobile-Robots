function plotEnvironment(env, robotState)
%-----------------------------------------------------------------------------
% Draw robot, goal, static obstacle, and dynamic obstacle
%-----------------------------------------------------------------------------
clf;
hold on;
grid on;
axis equal;

xlim(env.map.xlim);
ylim(env.map.ylim);

xlabel('x [m]');
ylabel('y [m]');
title('Environment Visualization');
%-----------------------------------------------------------------------------
% Draw goal
%-----------------------------------------------------------------------------
hGoal = plot(env.goal(1), env.goal(2), 'gp', ...
    'MarkerSize', 14, 'LineWidth', 2);
%-----------------------------------------------------------------------------
% Draw static obstacles
%-----------------------------------------------------------------------------
for i = 1:numel(env.staticObs)
    hStatic = drawCircle(env.staticObs(i).pos(1),env.staticObs(i).pos(2), env.staticObs(i).radius, [0.18 0.12 0.12]);
end
%-----------------------------------------------------------------------------
% Draw dynamic obstacles
%-----------------------------------------------------------------------------
for i = 1:numel(env.dynamicObs)
    hDyn = drawCircle(env.dynamicObs(i).pos(1), env.dynamicObs(i).pos(2), ...
        env.dynamicObs(i).radius, [0.12 0.12 0.185]);

    hVel = quiver(env.dynamicObs(i).pos(1), env.dynamicObs(i).pos(2), ...
        env.dynamicObs(i).vel(1), env.dynamicObs(i).vel(2), 0, 'Color', [0.12 0.12 0.185], 'LineWidth', 2);
end
%-----------------------------------------------------------------------------
% Draw robot
%-----------------------------------------------------------------------------
hRobot = drawCircle( robotState(1), robotState(2), env.robot.radius, [0.11 0.16 0.11]);
%-----------------------------------------------------------------------------
% Draw heading direction
%-----------------------------------------------------------------------------
hHeading = quiver( robotState(1), robotState(2), ...
    0.6*cos(robotState(3)), ...
    0.6*sin(robotState(3)), ...
    0, 'k', 'LineWidth', 2, 'MaxHeadSize', 2);
%-----------------------------------------------------------------------------
% Legend (clean handles)
%-----------------------------------------------------------------------------
legend([hGoal, hStatic, hDyn, hVel, hRobot, hHeading], ...
    {'Goal', 'Static obstacle', 'Dynamic obstacle', ...
     'Dynamic obstacle velocity', 'Robot', 'Robot heading'}, ...
    'Location', 'northoutside', ...
    'Orientation', 'horizontal');

drawnow;

end

function h = drawCircle(xc, yc, r, c)
th = linspace(0, 2*pi, 100);
x = xc + r*cos(th);
y = yc + r*sin(th);

h = fill(x, y, c, ...
    'FaceAlpha', 0.35, ...
    'EdgeColor', c, ...
    'LineWidth', 1.5);
end