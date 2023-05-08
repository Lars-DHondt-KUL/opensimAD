function [pathBuildExternalFunction] = buildExpressionGraph(outputFilename,...
    outputDir, compiler, verbose_mode)
% --------------------------------------------------------------------------
% buildExpressionGraph
%   Generates an expression graph and saves it as python code (foo.py)
%
%
% INPUT:
%   - outputFilename
%   * name of the generated file [char]
%
%   - outputDir
%   * full path to directory where the generated file shuld be saved [char]
%
%   - compiler -
%   * command prompt argument for the compiler. [char]
%   Example inputs:
%       Visual studio 2015: 'Visual Studio 14 2015 Win64'
%       Visual studio 2017: 'Visual Studio 15 2017 Win64'
%       Visual studio 2017: 'Visual Studio 16 2019'
%       Visual studio 2017: 'Visual Studio 17 2022'
%
%   - verbose_mode -
%   * outputs from windows command prompt are printed to matlab command 
%   window if true. [bool]
%
%
% OUTPUT:
%   - pathBuildExternalFunction -
%   * path to the folder where foo.py is
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

[pathUtilities,~,~] = fileparts(mfilename('fullpath'));
[pathMain,~,~] = fileparts(pathUtilities);
pathBuildExpressionGraph = fullfile(pathMain, 'buildExpressionGraph');
pathBuild = fullfile(pathMain, 'buildExpressionGraph', outputFilename);
mkdir(pathBuild);
pathBuildExternalFunction = fullfile(pathMain, 'buildExternalFunction');

OpenSimAD_DIR = fullfile(pathMain, 'OpenSimAD-install');
SDK_DIR = fullfile(OpenSimAD_DIR, 'sdk');
BIN_DIR = fullfile(OpenSimAD_DIR, 'bin');

cd(pathBuild);
cmd1 = ['cmake "', pathBuildExpressionGraph, '" -G "', compiler, '" -DTARGET_NAME:STRING="',...
    outputFilename, '" -DSDK_DIR:PATH="', SDK_DIR, '" -DCPP_DIR:PATH="', outputDir, '"'];
if verbose_mode
    system(cmd1);
else
    [~,~] = system(cmd1);
end
cmd2 = 'cmake --build . --config RelWithDebInfo';
if verbose_mode
    system(cmd2);
else
    [~,~] = system(cmd2);
end

cd(BIN_DIR);
path_EXE = fullfile(pathBuild, 'RelWithDebInfo', [outputFilename '.exe']);
system(path_EXE);

path_external_filename_foo = fullfile(BIN_DIR, 'foo.py');
copyfile(path_external_filename_foo, pathBuildExternalFunction);

delete(path_external_filename_foo);
rmdir(pathBuild, 's');

end