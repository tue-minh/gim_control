%% gim_arm_circle_trajectory.m
% Circular end-effector trajectory for the 3-DOF GIM upper-limb rehab arm.
%
% Steps:
%   1. Import gim_arm.urdf (with STL meshes) into a rigidBodyTree.
%   2. Add a virtual end-effector frame ("ee_link") at the tip of the forearm,
%      because the URDF stops at lower_arm_link.
%   3. Choose a circle (centre, plane, radius) inside the workspace.
%   4. Solve position-only IK for every point on the circle.
%   5. Build a smooth timed trajectory: home -> circle start -> N laps -> home.
%   6. Plot joints / EE path, animate the model and export the trajectory.
%
% Requires: Robotics System Toolbox (R2020b+ recommended; tested on R2022b)

clear; clc; close all;

%% ======================= USER PARAMETERS ===============================
urdfFile = fullfile(fileparts(mfilename('fullpath')), 'gim_arm.urdf');
meshDir  = fullfile(fileparts(mfilename('fullpath')), 'meshes');

% End-effector offset expressed in the lower_arm_link frame [m].
% Measured from lower_arm_link.STL: forearm tip is around x = 0.38..0.40 m.
% Change this if your handle / wrist point is somewhere else.
eeOffset = [0.384; 0.049; -0.032];

% Circle definition
radius      = 0.05;        % [m]
nLaps       = 2;           % number of revolutions
lapTime     = 6.0;         % [s] time per lap (at cruise speed)
planeMode   = 'auto';      % 'auto'  : plane of best manipulability
                           % 'xy','yz','xz' : world-aligned plane
                           % 'custom': use customNormal below
customNormal = [0; 0; 1];  % used only when planeMode = 'custom'
centerMode  = 'auto';      % 'auto'  : centre at a well-conditioned pose
                           % 'custom': use customCenter below (world frame)
customCenter = [0.06; -0.06; 0.38];

% Timing
approachTime = 3.0;        % [s] home -> first circle point
returnTime   = 3.0;        % [s] last circle point -> home
dt           = 0.01;       % [s] sample time (100 Hz)

% Safety margin from joint limits [rad]
limitMargin = deg2rad(5);

% Output
doAnimate  = true;
exportCSV  = true;
csvFile    = fullfile(fileparts(mfilename('fullpath')), 'gim_arm_circle_traj.csv');

%% ======================= 1. LOAD ROBOT =================================
robot = importrobot(urdfFile, 'MeshPath', {meshDir}, 'DataFormat', 'column');
robot.Gravity = [0 0 -9.81];

jointNames = {'base_joint', 'shoulder_joint', 'elbow_joint'};
nJ = numel(jointNames);

% Joint limits (read from the model)
qLim = zeros(nJ, 2);
for i = 1:nJ
    b = robot.Bodies{cellfun(@(bb) strcmp(bb.Joint.Name, jointNames{i}), robot.Bodies)};
    qLim(i, :) = b.Joint.PositionLimits;
end
qLimSafe = [qLim(:,1) + limitMargin, qLim(:,2) - limitMargin];
qHome    = homeConfiguration(robot);      % all zeros

%% ======================= 2. ADD END EFFECTOR ===========================
ee = rigidBody('ee_link');
setFixedTransform(ee.Joint, trvec2tform(eeOffset'));
addBody(robot, ee, 'lower_arm_link');
eeName = 'ee_link';

fkPos = @(q) tform2trvec(getTransform(robot, q, eeName))';
jacP  = @(q) posJacobian(robot, q, eeName);

%% ======================= 3. CHOOSE THE CIRCLE ==========================
% Search a grid of configurations for the most well-conditioned pose
% (largest minimum singular value of the 3x3 position Jacobian).
% Only joints well inside their limits are considered so the full circle fits.
inner = [qLimSafe(:,1) + 0.25*diff(qLimSafe,1,2), qLimSafe(:,2) - 0.25*diff(qLimSafe,1,2)];
g = 9;
[A, B, C] = ndgrid(linspace(inner(1,1), inner(1,2), g), ...
                   linspace(inner(2,1), inner(2,2), g), ...
                   linspace(inner(3,1), inner(3,2), g));
bestSig = -inf; qCenter = mean(qLimSafe, 2);
for k = 1:numel(A)
    q = [A(k); B(k); C(k)];
    s = svd(jacP(q));
    if s(end) > bestSig
        bestSig = s(end); qCenter = q;
    end
end

switch lower(centerMode)
    case 'auto',   pc = fkPos(qCenter);
    case 'custom', pc = customCenter(:);
    otherwise, error('Unknown centerMode');
end

switch lower(planeMode)
    case 'auto'
        % The two directions in which the EE moves most easily span the plane.
        [U, ~, ~] = svd(jacP(qCenter));
        n = U(:, 3);
    case 'xy',     n = [0; 0; 1];
    case 'yz',     n = [1; 0; 0];
    case 'xz',     n = [0; 1; 0];
    case 'custom', n = customNormal(:);
    otherwise, error('Unknown planeMode');
end
n = n / norm(n);
% Orthonormal basis (u, v) of the circle plane
tmp = [1; 0; 0]; if abs(dot(tmp, n)) > 0.9, tmp = [0; 1; 0]; end
u = cross(n, tmp); u = u / norm(u);
v = cross(n, u);

fprintf('Circle centre  [m] : [%.4f  %.4f  %.4f]\n', pc);
fprintf('Circle normal      : [%.4f  %.4f  %.4f]\n', n);
fprintf('Circle radius  [m] : %.3f,  laps: %d\n', radius, nLaps);

%% ======================= 4. TIME LAW + CARTESIAN PATH ==================
% Circle phase phi(t) uses a smooth trapezoidal profile:
% quintic ramp-up (1/2 lap), constant speed, quintic ramp-down (1/2 lap).
Tcircle = nLaps * lapTime + lapTime;     % ramps cost an extra half lap each
tC  = (0:dt:Tcircle)';
phi = smoothPhase(tC, Tcircle, 2*pi*nLaps, lapTime);

Pcirc = pc + radius * (u * cos(phi') + v * sin(phi'));   % 3 x N

%% ======================= 5. INVERSE KINEMATICS =========================
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 300;
ik.SolverParameters.AllowRandomRestart = false;
w = [0 0 0 1 1 1];   % position only (3-DOF arm cannot control orientation)

% Temporarily tighten the limits used by IK to the safe range
origLim = qLim;
setLimits(robot, jointNames, qLimSafe);

Nc = size(Pcirc, 2);
qCirc = zeros(nJ, Nc);
errC  = zeros(1, Nc);
qGuess = qCenter;
for k = 1:Nc
    Td = trvec2tform(Pcirc(:,k)');
    [qSol, ~] = ik(eeName, Td, w, qGuess);
    qCirc(:,k) = qSol;
    errC(k) = norm(fkPos(qSol) - Pcirc(:,k));
    qGuess = qSol;               % warm start -> continuous solution
end
setLimits(robot, jointNames, origLim);

fprintf('IK max position error: %.3f mm\n', 1e3*max(errC));
if max(errC) > 1e-3
    warning(['Some circle points are not reachable (error > 1 mm). ' ...
             'Reduce the radius or change the centre/plane.']);
end

%% ======================= 6. FULL JOINT TRAJECTORY ======================
% Approach and return segments use quintic (minimum-jerk) joint interpolation
tA = (0:dt:approachTime)';
tR = (0:dt:returnTime)';
qApp = quinticJoint(qHome, qCirc(:,1),  tA, approachTime);
qRet = quinticJoint(qCirc(:,end), qHome, tR, returnTime);

q = [qApp, qCirc(:,2:end), qRet(:,2:end)];
t = (0:size(q,2)-1)' * dt;
segIdx = [1, numel(tA), numel(tA)+Nc-1, size(q,2)];   % segment boundaries

qd  = gradient(q,  dt);
qdd = gradient(qd, dt);

% Check limits
velLim = [15.708; 1.963; 15.708];        % from URDF <limit velocity=...>
for i = 1:nJ
    if any(q(i,:) < qLim(i,1) - 1e-6 | q(i,:) > qLim(i,2) + 1e-6)
        warning('%s exceeds position limits!', jointNames{i});
    end
    if max(abs(qd(i,:))) > velLim(i)
        warning('%s exceeds velocity limit!', jointNames{i});
    end
    fprintf('%-15s  q:[%6.3f %6.3f] rad   |qd|max: %.3f rad/s\n', ...
        jointNames{i}, min(q(i,:)), max(q(i,:)), max(abs(qd(i,:))));
end

% Actual EE path from forward kinematics
P = zeros(3, size(q,2));
for k = 1:size(q,2), P(:,k) = fkPos(q(:,k)); end

%% ======================= 7. PLOTS ======================================
figure('Name', 'Joint trajectories', 'Color', 'w');
labels = {'q', 'qd', 'qdd'}; data = {q, qd, qdd};
units  = {'[rad]', '[rad/s]', '[rad/s^2]'};
for r = 1:3
    subplot(3,1,r); plot(t, data{r}', 'LineWidth', 1.4); grid on; hold on;
    xline(t(segIdx(2:3)), '--k');
    ylabel([labels{r} ' ' units{r}]);
    if r == 1, legend(jointNames, 'Interpreter', 'none', 'Location', 'best'); end
end
xlabel('time [s]');

figure('Name', 'End-effector path', 'Color', 'w');
plot3(Pcirc(1,:), Pcirc(2,:), Pcirc(3,:), 'k--', 'LineWidth', 1); hold on;
plot3(P(1,:), P(2,:), P(3,:), 'r', 'LineWidth', 1.5);
plot3(pc(1), pc(2), pc(3), 'b+', 'MarkerSize', 10);
axis equal; grid on; xlabel('X'); ylabel('Y'); zlabel('Z');
legend('desired circle', 'FK of joint trajectory', 'centre');
title('End-effector trajectory');

%% ======================= 8. ANIMATION ==================================
if doAnimate
    fig = figure('Name', 'GIM arm - circle', 'Color', 'w');
    ax = show(robot, q(:,1), 'Frames', 'off', 'PreservePlot', false);
    hold(ax, 'on');
    plot3(ax, Pcirc(1,:), Pcirc(2,:), Pcirc(3,:), 'k--', 'LineWidth', 1);
    hTrace = plot3(ax, nan, nan, nan, 'r', 'LineWidth', 2);
    view(ax, 135, 20); axis(ax, 'equal');
    xlim(ax, [-0.4 0.7]); ylim(ax, [-0.9 0.3]); zlim(ax, [-0.3 1.0]);
    title(ax, 'GIM arm - circular end-effector trajectory');

    skip = 4;                                % draw every 4th sample
    rc = rateControl(1/(dt*skip));
    for k = 1:skip:size(q,2)
        if ~isvalid(fig), break; end
        show(robot, q(:,k), 'Parent', ax, 'Frames', 'off', ...
             'PreservePlot', false, 'FastUpdate', true);
        set(hTrace, 'XData', P(1,1:k), 'YData', P(2,1:k), 'ZData', P(3,1:k));
        drawnow limitrate;
        waitfor(rc);
    end
end

%% ======================= 9. EXPORT =====================================
if exportCSV
    T = array2table([t, q', qd', qdd', P'], 'VariableNames', ...
        {'t', 'q_base', 'q_shoulder', 'q_elbow', ...
         'qd_base', 'qd_shoulder', 'qd_elbow', ...
         'qdd_base', 'qdd_shoulder', 'qdd_elbow', ...
         'ee_x', 'ee_y', 'ee_z'});
    writetable(T, csvFile);
    fprintf('Trajectory saved to %s (%d samples, %.1f s)\n', csvFile, numel(t), t(end));
end

%% ======================= LOCAL FUNCTIONS ===============================
function J = posJacobian(robot, q, eeName)
% 3xN linear-velocity Jacobian of the end-effector point
    Jg = geometricJacobian(robot, q, eeName);   % [angular; linear]
    J  = Jg(4:6, :);
end

function setLimits(robot, jointNames, lim)
% Overwrite joint position limits (used to keep IK away from hard limits)
    for i = 1:numel(jointNames)
        for b = 1:robot.NumBodies
            if strcmp(robot.Bodies{b}.Joint.Name, jointNames{i})
                jnt = copy(robot.Bodies{b}.Joint);
                jnt.PositionLimits = lim(i, :);
                replaceJoint(robot, robot.Bodies{b}.Name, jnt);
            end
        end
    end
end

function phi = smoothPhase(t, T, phiTotal, Tramp)
% Phase with quintic ramp-up/down of duration Tramp and constant speed between.
% Ramp covers phiRamp = w*Tramp/2 of angle, w = cruise angular speed.
    w = phiTotal / (T - Tramp);              % cruise speed so total angle matches
    phiR = w * Tramp / 2;
    phi = zeros(size(t));
    for k = 1:numel(t)
        tk = t(k);
        if tk < Tramp                          % accelerate
            s = tk / Tramp;
            % integral of the speed profile w*(10s^3 - 15s^4 + 6s^5)
            phi(k) = w * Tramp * (2.5*s^4 - 3*s^5 + s^6);
        elseif tk <= T - Tramp                 % cruise
            phi(k) = phiR + w * (tk - Tramp);
        else                                   % decelerate (mirror)
            s = (T - tk) / Tramp;
            phi(k) = phiTotal - w * Tramp * (2.5*s^4 - 3*s^5 + s^6);
        end
    end
end

function q = quinticJoint(q0, q1, t, T)
% Minimum-jerk joint interpolation, zero vel/acc at both ends
    s = t(:)' / T;
    h = 10*s.^3 - 15*s.^4 + 6*s.^5;
    q = q0 + (q1 - q0) .* h;
end
