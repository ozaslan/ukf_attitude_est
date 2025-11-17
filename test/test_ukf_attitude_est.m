classdef test_ukf_attitude_est < matlab.unittest.TestCase
    % Unit tests for the attitude UKF helper functions

    properties
        OriginalPath
    end

    methods (TestMethodSetup)
        function addMatlabFolder(testCase)
            testCase.OriginalPath = path;
            addpath(fullfile(fileparts(mfilename('fullpath')),'..','matlab'));
        end
    end

    methods (TestMethodTeardown)
        function restorePath(testCase)
            path(testCase.OriginalPath);
        end
    end

    methods (Test)
        function testQuatMultiplyIdentity(testCase)
            q_identity = [1; 0; 0; 0];
            angle = pi/2;
            axis = [0; 0; 1];
            q_z = [cos(angle/2); axis .* sin(angle/2)];

            result = quat_multiply(q_identity, q_z);

            testCase.verifyEqual(result, q_z, 'AbsTol', 1e-12);
        end

        function testQuatMultiplyComposition(testCase)
            angle_z = pi/4;
            angle_x = pi/6;
            q_z = [cos(angle_z/2); 0; 0; sin(angle_z/2)];
            q_x = [cos(angle_x/2); sin(angle_x/2); 0; 0];

            q_combined = quat_multiply(q_z, q_x);
            R_expected = [cos(angle_z) -sin(angle_z) 0; sin(angle_z) cos(angle_z) 0; 0 0 1] * ...
                [1 0 0; 0 cos(angle_x) -sin(angle_x); 0 sin(angle_x) cos(angle_x)];

            R_combined = quat_to_rotm(q_combined);

            testCase.verifyEqual(R_combined, R_expected, 'AbsTol', 1e-12);
        end

        function testRotmConversionsRoundTrip(testCase)
            yaw = 0.3; pitch = -0.2; roll = 0.1;
            Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
            Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
            R_original = Rz * Ry * Rx;

            q = rotm_to_quat(R_original);
            R_recovered = quat_to_rotm(q);

            testCase.verifyEqual(R_recovered, R_original, 'AbsTol', 1e-12);
            testCase.verifyEqual(norm(q), 1, 'AbsTol', 1e-12);
        end

        function testSigmaPointsSymmetry(testCase)
            n = 3;
            alpha = 0.5;
            beta = 2;
            kappa = 0;
            [~, ~, lambda] = ukf_weights(n, alpha, beta, kappa);
            x = zeros(n, 1);
            P = eye(n);

            X = sigma_points(x, P, lambda);

            testCase.verifyEqual(X(:,1), x, 'AbsTol', 1e-12);
            S = chol((n + lambda) * P, 'lower');
            for i = 1:n
                testCase.verifyEqual(X(:, i+1), x + S(:, i), 'AbsTol', 1e-12);
                testCase.verifyEqual(X(:, i+1+n), x - S(:, i), 'AbsTol', 1e-12);
            end
        end

        function testUkfWeightsSumToOne(testCase)
            n = 13;
            alpha = 0.3;
            beta = 2;
            kappa = 0;

            [Wm, Wc, lambda] = ukf_weights(n, alpha, beta, kappa);

            testCase.verifyEqual(sum(Wm), 1, 'AbsTol', 1e-12);
            testCase.verifyEqual(Wc(1), Wm(1) + (1 - alpha^2 + beta), 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(n + lambda, 0);
        end

        function testFStateGyroPropagation(testCase)
            q0 = [1; 0; 0; 0];
            bg0 = zeros(3,1);
            ba0 = zeros(3,1);
            bm0 = zeros(3,1);
            x = [q0; bg0; ba0; bm0];
            omega_body = [0; 0; 1];
            dt = 1.0;

            x_next = f_state(x, omega_body, dt);

            angle = norm(omega_body) * dt;
            q_expected = [cos(angle/2); 0; 0; sin(angle/2)];
            testCase.verifyEqual(x_next(1:4), q_expected, 'AbsTol', 1e-12);
            testCase.verifyEqual(x_next(5:end), x(5:end), 'AbsTol', 1e-12);
        end

        function testHAccMatchesGravity(testCase)
            g = 9.81;
            q_identity = [1; 0; 0; 0];
            ba = [0.05; -0.02; 0.01];
            x = [q_identity; zeros(3,1); ba; zeros(3,1)];

            z_pred = h_acc(x, g);

            testCase.verifyEqual(z_pred, [0; 0; -g] + ba, 'AbsTol', 1e-12);
        end

        function testHMagAppliesRotationAndBias(testCase)
            yaw = pi/6;
            q = [cos(yaw/2); 0; 0; sin(yaw/2)];
            bm = [0.02; -0.01; 0.03];
            x = [q; zeros(6,1); bm];
            m_ref = [0.2; 0.1; -0.05];

            expected = quat_to_rotm(q) * m_ref + bm;
            z_pred = h_mag(x, m_ref);

            testCase.verifyEqual(z_pred, expected, 'AbsTol', 1e-12);
        end

        function testUkfPredictStateKeepsQuaternionNormalized(testCase)
            x = [1; zeros(12,1)];
            P = eye(13) * 1e-6;
            Q = eye(13) * 1e-4;
            z_g = zeros(3,1);
            dt = 0.0;
            alpha = 0.3; beta = 2; kappa = 0;
            [Wm, Wc, lambda] = ukf_weights(numel(x), alpha, beta, kappa);

            [x_pred, P_pred] = ukf_predict_state(x, P, Q, z_g, dt, Wm, Wc, lambda);

            testCase.verifyEqual(norm(x_pred(1:4)), 1, 'AbsTol', 1e-10);
            testCase.verifyEqual(x_pred(5:end), x(5:end), 'AbsTol', 1e-8);
            testCase.verifyLessThanOrEqual(abs(P_pred - (P + Q)), 1e-6 + zeros(size(P_pred)));
        end

        function testUkfUpdateReducesUncertainty(testCase)
            g = 9.81;
            x = [1; zeros(12,1)];
            P = eye(13) * 0.01;
            R = eye(3) * 0.1;
            z_meas = h_acc(x, g);
            alpha = 0.4; beta = 2; kappa = 0;
            [Wm, Wc, lambda] = ukf_weights(numel(x), alpha, beta, kappa);

            [x_upd, P_upd] = ukf_update(x, P, z_meas, R, @h_acc, g, Wm, Wc, lambda);

            testCase.verifyEqual(norm(x_upd(1:4)), 1, 'AbsTol', 1e-10);
            testCase.verifyEqual(x_upd(5:end), x(5:end), 'AbsTol', 1e-8);
            testCase.verifyLessThan(trace(P_upd), trace(P));
        end

        function testUkfInitStateProducesNormalizedQuaternion(testCase)
            g = 9.81;
            N = 20;
            acc_init = repmat([0; 0; g], 1, N);
            gyro_init = zeros(3, N);
            mag_init = zeros(3, N);

            [x, P, m_ref] = ukf_init_state(acc_init, gyro_init, mag_init, g);

            testCase.verifyEqual(norm(x(1:4)), 1, 'AbsTol', 1e-12);
            testCase.verifyEqual(x(5:7), zeros(3,1), 'AbsTol', 1e-12);
            testCase.verifyEqual(P(1:4,1:4), eye(4) * 1e-2, 'AbsTol', 1e-15);
            testCase.verifyEqual(m_ref, zeros(3,1), 'AbsTol', 1e-12);
        end
    end
end
