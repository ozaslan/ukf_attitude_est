function X = sigma_points(x, P, lambda)
% SIGMA_POINTS  Generate UKF sigma points for mean x, covariance P
%
%   X : n x (2n+1) matrix of sigma points

n = numel(x);
X = zeros(n, 2*n+1);
X(:,1) = x;

% Scaled square-root of covariance
S = chol((n + lambda) * P, 'lower');

for i = 1:n
    X(:,i+1)    = x + S(:,i);
    X(:,i+1+n)  = x - S(:,i);
end
end
