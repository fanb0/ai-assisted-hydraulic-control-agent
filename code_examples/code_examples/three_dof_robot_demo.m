%% three_dof_robot_demo.m
% A directly runnable MATLAB demo for a 3-DOF planar robot manipulator.
% This example is used as a desensitized code sample for AI-assisted
% control research and simulation analysis.
%
% Model: 3-DOF rigid-body planar manipulator
% Controller: computed torque control with PD feedback
% Output: tracking curves, tracking errors, RMS and max absolute errors

clear; clc; close all;

%% Robot parameters
p.n  = 3;
p.g  = 9.81;

% Link length, mass, center of mass, and inertia
p.l  = [0.60; 0.45; 0.30];      % link length, m
p.lc = p.l / 2;                 % center of mass position, m
p.m  = [6.0; 4.0; 2.5];         % link mass, kg
p.I  = (1/12) * p.m .* p.l.^2;  % link inertia, kg*m^2

% Controller gains
p.Kp = diag([80, 70, 50]);
p.Kd = diag([18, 16, 12]);

%% Simulation setting
tspan = linspace(0, 15, 1501);

% Initial state: [q1 q2 q3 dq1 dq2 dq3]'
x0 = [0.00; -0.10; 0.10; 0; 0; 0];

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

[t, x] = ode45(@(t, x) robot_ode(t, x, p), tspan, x0, opts);

q  = x(:, 1:3);
dq = x(:, 4:6);

%% Desired trajectory and error calculation
qd   = zeros(length(t), 3);
dqd  = zeros(length(t), 3);
ddqd = zeros(length(t), 3);

for i = 1:length(t)
    [qd_i, dqd_i, ddqd_i] = desired_trajectory(t(i));
    qd(i, :)   = qd_i';
    dqd(i, :)  = dqd_i';
    ddqd(i, :) = ddqd_i';
end

error = qd - q;

rms_error = sqrt(mean(error.^2, 1));
max_error = max(abs(error), [], 1);

%% Print performance indexes
fprintf('\nTracking Performance of 3-DOF Manipulator\n');
fprintf('------------------------------------------\n');
fprintf('Joint\tRMS Error(rad)\tMax Abs Error(rad)\n');
for i = 1:3
    fprintf('%d\t%.6f\t\t%.6f\n', i, rms_error(i), max_error(i));
end

%% Plot joint tracking results
figure('Name', 'Joint Position Tracking');
for i = 1:3
    subplot(3, 1, i);
    plot(t, qd(:, i), 'LineWidth', 1.5); hold on;
    plot(t, q(:, i), '--', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel(['q_', num2str(i), ' (rad)']);
    legend('Desired', 'Actual');
    title(['Joint ', num2str(i), ' Position Tracking']);
end

%% Plot tracking errors
figure('Name', 'Joint Tracking Errors');
for i = 1:3
    subplot(3, 1, i);
    plot(t, error(:, i), 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel(['e_', num2str(i), ' (rad)']);
    title(['Joint ', num2str(i), ' Tracking Error']);
end

%% Plot end-effector trajectory
ee = zeros(length(t), 2);
ee_d = zeros(length(t), 2);

for i = 1:length(t)
    ee(i, :) = forward_kinematics(q(i, :)', p)';
    ee_d(i, :) = forward_kinematics(qd(i, :)', p)';
end

figure('Name', 'End-Effector Trajectory');
plot(ee_d(:, 1), ee_d(:, 2), 'LineWidth', 1.5); hold on;
plot(ee(:, 1), ee(:, 2), '--', 'LineWidth', 1.5);
grid on;
axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Desired', 'Actual');
title('End-Effector Trajectory Tracking');

%% ===================== Local Functions =====================

function dx = robot_ode(t, x, p)
    q  = x(1:3);
    dq = x(4:6);

    [qd, dqd, ddqd] = desired_trajectory(t);

    M = inertia_matrix(q, p);
    C = coriolis_vector(q, dq, p);
    G = gravity_vector(q, p);

    e  = qd - q;
    de = dqd - dq;

    % Computed torque control
    v = ddqd + p.Kd * de + p.Kp * e;
    tau = M * v + C + G;

    % External disturbance for simulation
    disturbance = [1.5 * sin(1.5 * t);
                   1.0 * cos(1.2 * t);
                   0.7 * sin(1.7 * t)];

    % Robot dynamics
    ddq = M \ (tau + disturbance - C - G);

    dx = [dq; ddq];
end

function [qd, dqd, ddqd] = desired_trajectory(t)
    qd = [0.30 * sin(0.60 * t);
         -0.25 + 0.20 * sin(0.70 * t + 0.50);
          0.20 + 0.15 * cos(0.50 * t)];

    dqd = [0.30 * 0.60 * cos(0.60 * t);
           0.20 * 0.70 * cos(0.70 * t + 0.50);
          -0.15 * 0.50 * sin(0.50 * t)];

    ddqd = [-0.30 * 0.60^2 * sin(0.60 * t);
            -0.20 * 0.70^2 * sin(0.70 * t + 0.50);
            -0.15 * 0.50^2 * cos(0.50 * t)];
end

function M = inertia_matrix(q, p)
    n = p.n;
    M = zeros(n, n);

    theta = cumsum(q);

    for i = 1:n
        Jv = zeros(2, n);
        Jw = zeros(1, n);

        for j = 1:n
            if j <= i
                dx = 0;
                dy = 0;

                for k = j:i-1
                    dx = dx - p.l(k) * sin(theta(k));
                    dy = dy + p.l(k) * cos(theta(k));
                end

                dx = dx - p.lc(i) * sin(theta(i));
                dy = dy + p.lc(i) * cos(theta(i));

                Jv(:, j) = [dx; dy];
                Jw(j) = 1;
            end
        end

        M = M + p.m(i) * (Jv' * Jv) + p.I(i) * (Jw' * Jw);
    end
end

function C = coriolis_vector(q, dq, p)
    n = p.n;
    C = zeros(n, 1);

    eps_val = 1e-6;

    dM = zeros(n, n, n);

    for k = 1:n
        q_plus = q;
        q_minus = q;

        q_plus(k) = q_plus(k) + eps_val;
        q_minus(k) = q_minus(k) - eps_val;

        M_plus = inertia_matrix(q_plus, p);
        M_minus = inertia_matrix(q_minus, p);

        dM(:, :, k) = (M_plus - M_minus) / (2 * eps_val);
    end

    for i = 1:n
        temp = 0;
        for j = 1:n
            for k = 1:n
                c_ijk = 0.5 * (dM(i, j, k) + dM(i, k, j) - dM(j, k, i));
                temp = temp + c_ijk * dq(j) * dq(k);
            end
        end
        C(i) = temp;
    end
end

function G = gravity_vector(q, p)
    n = p.n;
    G = zeros(n, 1);

    theta = cumsum(q);

    for j = 1:n
        temp = 0;
        for i = j:n
            dy = 0;

            for k = j:i-1
                dy = dy + p.l(k) * cos(theta(k));
            end

            dy = dy + p.lc(i) * cos(theta(i));
            temp = temp + p.m(i) * p.g * dy;
        end
        G(j) = temp;
    end
end

function pos = forward_kinematics(q, p)
    theta = cumsum(q);

    x = 0;
    y = 0;

    for i = 1:p.n
        x = x + p.l(i) * cos(theta(i));
        y = y + p.l(i) * sin(theta(i));
    end

    pos = [x; y];
end
