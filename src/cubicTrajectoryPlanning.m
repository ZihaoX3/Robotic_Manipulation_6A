function [q, qd, qdd] = cubicTrajectoryPlanning(n, q0, qf, qd0, qdf)
% q0 intial pos, 
% qf final pos, 
% qd0 initial velocity,
% qdf final velocity,
% m no of time steps

% q vector of angle positions
% qd velocity
% qdd acceleration
tf = 1;  %final time

a0 = q0;%copy(q0);
a1 = qd0;%copy(qd0);

a2 = 3/tf^2 *(qf - q0) - 2*qd0 - qdf;
a3 = -2/tf^2 *(qf - q0) + qd0 + qdf;

timestep = linspace(0, tf, n);

q = zeros(1,n);
qd = zeros(1,n);
qdd = zeros(1,n);

for i = 1:length(timestep)
    t = timestep(i);
    t2 = t^2;
    t3 = t^3;

    q(i) = a0 + a1*t + a2*t2 + a3*t3;
    qd(i) = a1 +2*a2*t + 3*a3*t2;
    qdd(i) = 2*a2 + 6*a3*t;

end