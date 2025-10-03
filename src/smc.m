function Qc = smc(P)
% Rectify control inputs using Sliding Mode Control
lambda = 0.1;
u_min = [-25*pi/180; -25*pi/180; -30*pi/180; 0.5*pi/180; 0.5*pi/180];
u_max = [25*pi/180; 10*pi/180; 30*pi/180; 10*pi/180; 10*pi/180];
Qc = zeros(5,1);

for i = 1:5
    if P(i) < u_min(i)
        u_error = P(i) - u_min(i);
        u_sat = sign(u_error)*min(abs(lambda*u_error), abs(u_max(i)-u_min(i)));
        Qc(i) = u_sat + u_min(i);
    elseif P(i) > u_max(i)
        u_error = P(i) - u_max(i);
        u_sat = sign(u_error)*min(abs(lambda*u_error), abs(u_max(i)-u_min(i)));
        Qc(i) = u_sat + u_max(i);
    else
        Qc(i) = P(i);
    end
end
end
