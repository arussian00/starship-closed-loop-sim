function cfg = closed_loop_config()
%CLOSED_LOOP_CONFIG  Where the two source repos live, and how the loop runs.
%
%   cfg.controlsBranch defaults to main, where pitch-controller steps 1-2
%   are merged. setup_paths reads that ref from a separate folder
%   (rocket-controls-simulink-main). It does not checkout, pull, or reset
%   a rocket-controls-simulink folder that is already on disk.
%
%   cfg.simBranch empty means "use the matlab-rocket-sim checkout already
%   on disk". setup_paths never commits or pushes either source repo.
%
%   cfg.figureMode is 'maximized' or 'docked'. The figure tag is reused
%   on the next run. cfg.controlHz is the fixed rate of the discrete
%   pitch controller (the gimbal command is held between samples).

cfg.simRepo = 'matlab-rocket-sim';
cfg.controlsRepo = 'rocket-controls-simulink';
cfg.simUrl = 'https://github.com/arussian00/matlab-rocket-sim.git';
cfg.controlsUrl = 'https://github.com/arussian00/rocket-controls-simulink.git';
cfg.simBranch = '';
cfg.controlsBranch = 'main';
cfg.driveRoot = '/MATLAB Drive/Repositories';
cfg.figureMode = 'maximized';
cfg.figureTag = 'starshipClosedLoop';
cfg.controlHz = 100;
cfg.openLoopCatch_m = -0.08;
cfg.openLoopSplash_m = -1.47;
end
