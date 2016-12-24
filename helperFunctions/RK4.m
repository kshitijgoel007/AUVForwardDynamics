function y = RK4(h, t, x, x_dot)

K1 = funX(t, x, x_dot);
K2 = funX(t+h/2, x+(h/2)*K1, x_dot);
K3 = funX(t+h/2, x+(h/2)*K2, x_dot);
K4 = funX(t+h, x+h*K3, x_dot);

y = x + (h/6)*(K1 + 2*K2 + 2*K3 + K4);

end