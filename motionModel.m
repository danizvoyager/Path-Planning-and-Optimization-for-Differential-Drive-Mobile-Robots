function x_next = motionModel(x, u, dt, cfg, controllerState)
%MOTIONMODEL Backward-compatible motion model.
% If cfg is omitted, use the original kinematic update.
% If cfg is provided, use the differential-drive dynamic model and optional LQI.

if nargin < 4 || isempty(cfg)
    v = u(1);
    w = u(2);

    x_next = x;
    x_next(1) = x(1) + v*cos(x(3))*dt;
    x_next(2) = x(2) + v*sin(x(3))*dt;
    x_next(3) = wrapToPiLocal(x(3) + w*dt);
    x_next(4) = v;
    x_next(5) = w;
    return;
end

if nargin < 5
    controllerState = [];
end

[x_next, ~] = ddmrDynamics(x, u, dt, cfg.dynamics, cfg.controller, controllerState);
end

function ang = wrapToPiLocal(ang)
ang = mod(ang + pi, 2*pi) - pi;
end
