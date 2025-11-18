function X = sigma_points(x, P, lambda)
% SIGMA_POINTS  Generate UKF sigma points for mean x, covariance P
%
%   X : n x (2n+1) matrix of sigma points

n = numel(x);
X = zeros(n, 2*n+1);
X(:,1) = x;

% Scaled square-root of covariance
P = (P + P.') / 2;
scale = (n + lambda) * P;

try
    S = chol(scale, 'lower');
catch ME
    % Retry with a small diagonal jitter to enforce positive definiteness
    jitter = max(eps(max(diag(scale))), 1e-9);
    scale_jittered = (scale + jitter * eye(n));
    scale_jittered = (scale_jittered + scale_jittered.') / 2;
    try
        S = chol(scale_jittered, 'lower');
    catch cholME
        throw(addCause(MException('ukf:sigma_points', ...
            'Covariance not positive definite for sigma point generation'), cholME));
    end
end

for i = 1:n
    X(:,i+1)    = x + S(:,i);
    X(:,i+1+n)  = x - S(:,i);
end
end
