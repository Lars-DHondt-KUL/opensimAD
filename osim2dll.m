function [S] = osim2dll(S,osim_path)
% --------------------------------------------------------------------------
% osim2dll
%   This functions uses the OpenSim model to construct a CasADi external
%   function (.dll file) that contains an implicit formulation of the
%   skeletal- and contact dynamics. More information can be found here:
%   https://github.com/Lars-DHondt-KUL/opensimAD 
% 
% INPUT:
%   - S -
%   * setting structure S
%
%   - osim_path -
%   * path to the OpenSim model file (.osim)
% 
% OUTPUT:
%   - S -
%   * setting structure S
% 
% Original author: Maarten Afschrift
% Original date: 16/May/2022
%
% Last edit by: 
% Last edit date: 
% --------------------------------------------------------------------------

% path to the opensim-AD submodule
pathFunction = mfilename('fullpath');
[pathPreProcessing, ~, ~] = fileparts(pathFunction);
[pathMain, ~] = fileparts(pathPreProcessing);
PathOsimAD = fullfile(pathMain,'opensimAD');
addpath(PathOsimAD)

% extract name of opensim model file
[~,osim_file_name,~] = fileparts(osim_path);

% external function name
S.misc.external_function = ['F_' osim_file_name '.dll'];

if ~isfile(fullfile(S.misc.subject_path,S.misc.external_function))

    % disp
    disp('Convert .osim to .dll file started ...');     
    % list with all input arguments to create the .dll file
    pathOpenSimModel = osim_path;
    outputFilename = ['F_' osim_file_name];
    compiler = S.Cpp2Dll.compiler;
    export3DPositions = S.Cpp2Dll.export3DPositions;
    export3DVelocities = S.Cpp2Dll.export3DVelocities;
    jointsOrder = S.Cpp2Dll.jointsOrder;
    coordinatesOrder = S.Cpp2Dll.coordinatesOrder;
    exportGRFs = S.Cpp2Dll.exportGRFs;
    exportSeparateGRFs = S.Cpp2Dll.exportSeparateGRFs;
    exportGRMs = S.Cpp2Dll.exportGRMs;
    exportContactPowers = S.Cpp2Dll.exportContactPowers;
    CppDir = S.misc.subject_path;

    % Create the .cpp file using an executable
    if isempty(S.Cpp2Dll.PathCpp2Dll_Exe)
        error(['You should specify the path to the executables to convert', ...
            'osim file to an .dll file. You can install the exectubale using the' ...
            'function: InstallOsim2Dll_Exe']);
    else
        PathCpp2Dll_Exe = S.Cpp2Dll.PathCpp2Dll_Exe;
    end

    % create folders if needed
    if ~isfolder(CppDir)
        mkdir(CppDir);
    end

    MainPath = pwd;

fooPath = generateExternalFunction(pathOpenSimModel, CppDir, fullfile(PathOsimAD,'inverseDynamics'),...
    jointsOrder,coordinatesOrder, export3DPositions, export3DVelocities, exportGRFs, exportGRMs,...
    exportSeparateGRFs, exportContactPowers, outputFilename, compiler);

    % load .mat file with input and outputs
    load(fullfile(CppDir,[outputFilename '_IO.mat']),'IO');

    % create foo_jac (temporary solution using executable);
    disp('   converting foo.py to foo_jac.c using casadi');
    cd(PathCpp2Dll_Exe)
    command = ['GenF.exe ' fooPath ' ' num2str(IO.nCoordinates*3)]; % ToDo: Add nInputes to IO structure
    if S.Cpp2Dll.verbose_mode == 0
        [~,~] = system(command);
    else
        system(command);
    end
    cd(MainPath);

    % createDll
    disp('   creating .dll file using cmake');
    CreateDllFromFoo_Jac(outputFilename, CppDir, compiler, PathOsimAD,...
        S.Cpp2Dll.verbose_mode);
    cd(MainPath);


    % display message
    disp('... convert .osim to .dll file finished'); 
else
    disp(['Using existing .dll file: ', fullfile(S.misc.subject_path,S.misc.external_function)]);
end



