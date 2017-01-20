function y = RK4(funX, tinc, t, x, x_dot)

K1 = funX(t, x);
K2 = funX(t+tinc/2, x+(tinc/2)*K1, x_dot);
K3 = funX(t+tinc/2, x+(tinc/2)*K2, x_dot);
K4 = funX(t+tinc, x+tinc*K3, x_dot);

y = x + (tinc/6)*(K1 + 2*K2 + 2*K3 + K4);

end
