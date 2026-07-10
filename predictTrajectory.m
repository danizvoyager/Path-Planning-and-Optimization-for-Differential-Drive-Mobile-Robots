function traj = predictTrajectory(x0, u, predictTime, dt, cfg, controllerState)

if nargin < 5
    cfg = [];
end
if nargin < 6
    controllerState = [];
end

x = x0;
traj = x(1:3);
N = floor(predictTime / dt);

for k = 1:N
    if isempty(cfg)
        x = motionModel(x, u, dt);
    else
        [x, controllerState] = ddmrDynamics(x, u, dt, cfg.dynamics, cfg.controller, controllerState);
    end
    traj = [traj, x(1:3)]; %#ok<AGROW>
end

end
