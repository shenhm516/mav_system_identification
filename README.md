# mav_system_identification

Matlab scripts to perform system identification for DJI 100, forked from https://github.com/ethz-asl/mav_system_identification. We uses built-in functions of MATLAB to read and parse ROS messages rather than matlab_rosbag package. Also, this repository provides option to split one file into two files for estimation and validation.

## Running the tests

Run in MATLAB:

'''
m100_sysid_roll_pitch
'''

## Deployment

Change parameters in file m100_sysid_roll_pitch.m:

'''
np = 1; % number of poles (1 or 2)
use_one_file = false;
bag1_ratio = 0.5;% 50% for bag 1, 50% for bag 2, ignore if use_one_file is false
bagfile_exp1 =  'ouster_cut1.bag';
bagfile_exp2 =  'ouster_cut2.bag'; % ignore if use_one_file is true
'''
