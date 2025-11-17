function [Wm, Wc, lambda] = ukf_weights(n, alpha, beta, kappa)
% UKF_WEIGHTS  Compute sigma point weights for dimension n

lambda = alpha^2 * (n + kappa) - n;
c = 0.5 / (n + lambda);

Wm = ones(2*n+1, 1) * c;
Wc = Wm;
Wm(1) = lambda / (n + lambda);
Wc(1) = Wm(1) + (1 - alpha^2 + beta);
end
