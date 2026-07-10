function J = globalCost(T_goal, L, Dmin_global, omega_sq_int, lambda1, lambda2, lambda3, lambda4)

J = lambda1*T_goal + lambda2*L - lambda3*Dmin_global + lambda4*omega_sq_int;

end