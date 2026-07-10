function [x_next, controllerState, debug] = ddmrDynamics(x, uRef, dt, dyn, controller, controllerState)
%DDMRDYNAMICS Differential-drive body dynamics with optional LQI tracking.
% state x = [x; y; theta; v; w]
% dynamics:
%   x_dot     = v cos(theta)
%   y_dot     = v sin(theta)
%   theta_dot = w
%   v_dot     = -(1/tau_v) v + (k_v/tau_v) u_v
%   w_dot     = -(1/tau_w) w + (k_w/tau_w) u_w

if nargin < 6 || isempty(controllerState)
    controllerState.intErr = [0; 0];
    controllerState.K = [];
end

if nargin < 5 || isempty(controller)
    controller.useLQI = false;
end

vRef = uRef(1);
wRef = uRef(2);

if isfield(controller, 'useLQI') && controller.useLQI
    [uAct, controllerState] = lqiTwistController(x(4:5), [vRef; wRef], dt, dyn, controller, controllerState);
else
    uAct = [vRef; wRef];
end

v = x(4);
w = x(5);

vDot = -(1/dyn.tau_v) * v + (dyn.k_v/dyn.tau_v) * uAct(1);
wDot = -(1/dyn.tau_w) * w + (dyn.k_w/dyn.tau_w) * uAct(2);

x_next = x;
x_next(1) = x(1) + v * cos(x(3)) * dt;
x_next(2) = x(2) + v * sin(x(3)) * dt;
x_next(3) = wrapToPiLocal(x(3) + w * dt);
x_next(4) = v + vDot * dt;
x_next(5) = w + wDot * dt;

x_next(4) = max(0, x_next(4));

debug.uRef = [vRef; wRef];
debug.uAct = uAct;
debug.vDot = vDot;
debug.wDot = wDot;
end

function [uAct, controllerState] = lqiTwistController(xvw, ref, dt, dyn, controller, controllerState)
A = [-1/dyn.tau_v, 0; 0, -1/dyn.tau_w];
B = [dyn.k_v/dyn.tau_v, 0; 0, dyn.k_w/dyn.tau_w];
C = eye(2);

Aa = [A zeros(2); -C zeros(2)];
Ba = [B; zeros(2)];

if ~isfield(controllerState, 'K') || isempty(controllerState.K)
    controllerState.K = lqr(Aa, Ba, controller.Q, controller.R);
end

err = xvw - ref;
trackingErr = ref - xvw;
controllerState.intErr = controllerState.intErr + trackingErr * dt;
controllerState.intErr(1) = clamp(controllerState.intErr(1), -controller.intLimitV, controller.intLimitV);
controllerState.intErr(2) = clamp(controllerState.intErr(2), -controller.intLimitW, controller.intLimitW);

augErr = [err; controllerState.intErr];
uEq = [ref(1) / dyn.k_v; ref(2) / dyn.k_w];
uAct = uEq - controllerState.K * augErr;
uAct(1) = clamp(uAct(1), 0, controller.maxUv);
uAct(2) = clamp(uAct(2), -controller.maxUw, controller.maxUw);
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end

function ang = wrapToPiLocal(ang)
ang = mod(ang + pi, 2*pi) - pi;
end
