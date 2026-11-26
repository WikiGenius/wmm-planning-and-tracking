function results = reproduce(outputDir)
% From the repository root: matlab -batch "reproduce"
root = fileparts(mfilename('fullpath'));
if nargin == 0, outputDir = fullfile(root,'results'); end
oldPath = path;
restorePath = onCleanup(@() path(oldPath));
addpath(fullfile(root,'adaptive-control'));
baseline = readtable(fullfile(root,'adaptive-control','results.csv'),'ReadRowNames',true);
results = run_demo(false,'',outputDir);
assert(isequal(results.Properties.VariableNames,baseline.Properties.VariableNames) && ...
    isequal(results.Properties.RowNames,baseline.Properties.RowNames),'Unexpected results schema.');
% Relative tolerance plus absolute floor for floating-point and solver variation.
assert(all(abs(results{:,:}-baseline{:,:}) <= 1e-6 + 1e-5*abs(baseline{:,:}),'all'), ...
    'Adaptive benchmark differs from the committed baseline.');
addpath(fullfile(root,'wheeled_manipulator_framework','control','controllers'));
run = mm_main_controller(false);
assert(abs(run.work(end)-89.7296172)<0.02,'Legacy net work differs from the reference run.');
% Work is a scalar integral. Check where the robot actually ended as well.
expectedPose = [2.01491119 2.02025115 1.59512433]; % mount x, y (m), yaw (rad)
assert(all(abs(run.state(end,1:3)-expectedPose)<1e-3), ...
    'Legacy terminal mount pose differs from the reference run.');
trajectory = array2table([run.time run.state run.torque run.work],'VariableNames',...
    {'t_s','mountX','mountY','yaw','wheelR','wheelL','arm1','arm2','arm3',...
     'wheelRateR','wheelRateL','armRate1','armRate2','armRate3',...
     'tauWheelR','tauWheelL','tauArm1','tauArm2','tauArm3','netWork_J'});
writetable(trajectory,fullfile(outputDir,'legacy-trajectory.csv'));
fprintf('Legacy duration %.6g s, net actuator work %.9g J.\n',run.time(end),run.work(end));
fprintf('Reproduction passed. Outputs in %s\n',outputDir);
end
