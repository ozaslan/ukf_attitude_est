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

        function testQuatMultiplyVectorizedColumns(testCase)
            % Mix an identity quaternion with a 90-deg Z rotation and
            % a 90-deg X rotation in column-wise form to verify vectorization
            q_identity = [1; 0; 0; 0];
            q_z = [cos(pi/4); 0; 0; sin(pi/4)];
            q_x = [cos(pi/4); sin(pi/4); 0; 0];

            q_batch_left = [q_identity, q_z];
            q_batch_right = [q_z, q_x];

            result = quat_multiply(q_batch_left, q_batch_right);

            testCase.verifyEqual(result(:, 1), q_z, 'AbsTol', 1e-12);
            testCase.verifyEqual(result(:, 2), quat_multiply(q_z, q_x), 'AbsTol', 1e-12);
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

        function testRotm2EulZyxMatchesInputAngles(testCase)
            yaw = 0.7; pitch = -0.4; roll = 0.25;
            R = rotation_matrix_from_ypr(yaw, pitch, roll);

            eul = rotm2eul_zyx(R);

            testCase.verifyEqual(eul, [yaw; pitch; roll], 'AbsTol', 1e-12);
        end

        function testQuaternionsToEulerZyxRoundTrip(testCase)
            yaw_values = [0.0, 0.45, -1.7, 2.8];
            pitch_values = [0.1, -0.25, 0.6, -0.4];
            roll_values = [-0.05, 0.35, -0.7, 0.9];
            num_samples = numel(yaw_values);

            q_series = zeros(4, num_samples);
            for k = 1:num_samples
                q_series(:, k) = euler_zyx_to_quat(yaw_values(k), pitch_values(k), roll_values(k));
            end

            eul = quaternions_to_euler_zyx(q_series);

            expected = [yaw_values; pitch_values; roll_values];
            testCase.verifyEqual(eul, expected, 'AbsTol', 1e-12);
        end

        function testQuaternionsToEulerZyxUnwrapsYawContinuously(testCase)
            yaw_values = [pi - 0.05, -pi + 0.05];
            num_samples = numel(yaw_values);

            q_series = zeros(4, num_samples);
            for k = 1:num_samples
                q_series(:, k) = euler_zyx_to_quat(yaw_values(k), 0, 0);
            end

            eul = quaternions_to_euler_zyx(q_series);

            expected_yaw = [pi - 0.05, pi + 0.05];
            expected = [expected_yaw; zeros(2, num_samples)];
            testCase.verifyEqual(eul, expected, 'AbsTol', 1e-12);
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

function R = rotation_matrix_from_ypr(yaw, pitch, roll)
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
R = Rz * Ry * Rx;
end

function q = euler_zyx_to_quat(yaw, pitch, roll)
cy = cos(yaw/2); sy = sin(yaw/2);
cp = cos(pitch/2); sp = sin(pitch/2);
cr = cos(roll/2); sr = sin(roll/2);

qw = cy*cp*cr + sy*sp*sr;
qx = cy*cp*sr - sy*sp*cr;
qy = cy*sp*cr + sy*cp*sr;
qz = sy*cp*cr - cy*sp*sr;

q = [qw; qx; qy; qz];
q = q / norm(q);
end
