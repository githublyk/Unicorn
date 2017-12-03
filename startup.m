function [] = startup()
% startup adds paths required to run demos

localDir = fileparts(mfilename('fullpath'));
addpath(fullfile(localDir));
addpath(fullfile(localDir, 'camera'));
addpath(fullfile(localDir, 'camera', 'HebiCam'));
addpath(fullfile(localDir, 'HEBI_API_2009'));
addpath(fullfile(localDir, 'matlab_SEA'));
addpath(fullfile(localDir,'matlab_SEA', 'armControl'));
addpath(fullfile(localDir,'matlab_SEA', 'plottingTools'));
addpath(fullfile(localDir,'matlab_SEA', 'plottingTools','animate'));
addpath(fullfile(localDir,'matlab_SEA', 'plottingTools','stl'));
addpath(fullfile(localDir,'matlab_SEA', 'plottingTools','utils'));
addpath(fullfile(localDir,'matlab_SEA', 'poseEstimation'));
addpath(fullfile(localDir,'matlab_SEA', 'utils','computation'));
addpath(fullfile(localDir,'matlab_SEA', 'utils','rotation'));
addpath(fullfile(localDir,'matlab_SEA', 'utils','SEAsnakeTools'));
addpath(fullfile(localDir,'matlab_SEA', 'utils','SEAsnakeTools','kinematics'));
addpath(fullfile(localDir,'matlab_SEA', 'utils','SEAsnakeTools','virtual_chassis'));
addpath(fullfile(localDir,'nullSpaceCompliance'));
addpath(fullfile(localDir,'rosWrapper'));
addpath(fullfile(localDir,'scripts'));
addpath(fullfile(localDir,'workingVolumeAnalysis'));
end