% --------------------------------------------------------------------------
% This script uses OpenSimAD to generate a CasADi external function. Given
% an OpenSim model provided as an .osim file, this script generates a C++
% file with a function F building the musculoskeletal model programmatically 
% and running inverse dynamics. The C++ file is then compiled as an .exe, 
% which when run generates the expression graph underlying F. From this
% expression graph, CasADi can generate C code containing the function F
% and its Jacobian in a format understandable by CasADi. This code is 
% finally compiled as a .dll that can be imported when formulating 
% trajectory optimization problems with CasADi.
%
% The function F takes as:
%     - INPUTS: 
%         - joint positions and velocities (intertwined)
%         - joint accelerations
%     - OUTPUTS:
%         - joint torques
%         - (optional) other variables exported from the model
%
% Reference: 
%   Falisse A, Serrancolí G, et al. (2019) Algorithmic differentiation 
%   improves the computational efficiency of OpenSim-based trajectory 
%   optimization of human movement. PLoS ONE 14(10): e0217730. 
%   https://doi.org/10.1371/journal.pone.0217730
%
% Original author: Lars D'Hondt (based on code by Antoine Falisse)
% Original date: 8/May/2023
%
% Last edit by: 
% Last edit date: 
% --------------------------------------------------------------------------

clear
clc


% Get current directory
pathMain = pwd;
addpath(fullfile(pathMain,'utilities'))

%% User inputs

% Provide path to OpenSim model.
pathOpenSimModel = fullfile(pathMain, 'examples', 'Hamner_modified.osim');

% Output file name
outputFilename = 'F_test';

% Compiler
compiler = 'Visual Studio 17 2022';

% Print information to the command window
verbose_mode = true;

% Verify the generated function
verify_ID = true;

% Order of joints and coordinates. There is no need to provide joints or 
% coordinates, they are retrieved from the .osim file.
jointsOrder = [];
coordinatesOrder = [];

% By default, the external function returns the joint torques. However, you
% can also export other variables that you may want to use when formulating
% your problem. Here, we provide a few examples of variables we typically use
% in our problems.

% Export 3D positions and velocities of points w.r.t. ground reference.
% Leave empty to not export those variables.
export3DPositions = [];
export3DVelocities = [];

% Export total GRFs.
% If true, right and left 3D GRFs (in this order) are exported.
exportGRFs = true;

% Export separate GRFs.
% If true, right and left 3D GRFs (in this order) are exported for each of the
% 6 contact spheres. 
exportSeparateGRFs = true;

% Export GRMs.
% If true, right and left 3D GRMs (in this order) are exported.
exportGRMs = true;

% Export contact sphere vertical deformation power.
% If true, right and left vertical deformation power of all contact spheres 
% are exported. 
exportContactPowers = true;



%% Call generateExternalFunction function
generateExternalFunction(pathOpenSimModel, pathExample, jointsOrder,...
    coordinatesOrder, export3DPositions, export3DVelocities, exportGRFs,...
    exportGRMs, exportSeparateGRFs, exportContactPowers, outputFilename, compiler,...
    verbose_mode, verify_ID);
