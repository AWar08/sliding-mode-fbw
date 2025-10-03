function V_n = navigation_eq(A)
% Converts body frame velocities to Earth frame
phi = A(4); theta = A(5); psi = A(6);
C_b2 = [1,0,0;0,cos(phi),sin(phi);0,-sin(phi),cos(phi)];
C_21 = [cos(theta),0,-sin(theta);0,1,0;sin(theta),0,cos(theta)];
C_1v = [cos(psi),sin(psi),0;-sin(psi),cos(psi),0;0,0,1];
C_vb = C_b2*C_21*C_1v;

V_b = A(1:3);
V_n = C_vb*V_b;
end
