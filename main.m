%
%  main.m
%  ADfSM: All-around Depth from Small Motion with A Spherical Panoramic Cameras
% 
%
%  Created by Sunghoon Im on 2017. 1. 23..
%  Copyright @ 2017 Sunghoon Im. All rights reserved.
%

clc; clear; close all;

addpath('harris');
setting.dataset_root='Ricoh_db'; % Dataset root
setting.video_format='mp4'; % Input video format
setting.max_iter=5; % Maximum iteration of bundle adjustment
setting.scaling=0.5; % Factor of image resizing
setting.num_label = 64; % The number of labels
setting.sam_rate = 3; % sampling rate

flist=dir(fullfile(setting.dataset_root,['*.' setting.video_format]));
for i=1:length(flist)
    setting.dataset_name=flist(i).name(1:end-4);
    A = ADfSM(setting);
    A.SMBA360();
    A.SphereSweep360();
    A.Sphericaldepth();
end