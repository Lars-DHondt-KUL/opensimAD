function [] = buildExternalFunction(outputFilename, outputDir, compiler, verbose_mode)
% --------------------------------------------------------------------------
% buildExternalFunction
%   Compiles the expression graph (foo_jac.c) into an external function (.dll)
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
%   - (This function does not return output arguments) -
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
pathBuildExternalFunction = fullfile(pathMain, 'buildExternalFunction');
path_external_functions_filename_install = fullfile(pathMain, 'installExternalFunction', outputFilename);
mkdir(path_external_functions_filename_install);
path_external_functions_filename_build = fullfile(pathMain, 'buildExternalFunction', outputFilename);
mkdir(path_external_functions_filename_build);

addpath(pathBuildExternalFunction);

cd(path_external_functions_filename_build);
cmd1 = ['cmake "', pathBuildExternalFunction, '" -G "', compiler, '" -DTARGET_NAME:STRING="',...
    outputFilename, '" -DINSTALL_DIR:PATH="', path_external_functions_filename_install, '"'];
if verbose_mode
    system(cmd1);
else
    [~,~] = system(cmd1);
end
cmd2 = 'cmake --build . --config RelWithDebInfo --target install';
if verbose_mode
    system(cmd2);
else
    [~,~] = system(cmd2);
end
cd(pathMain);

copyfile(fullfile(path_external_functions_filename_install, 'bin', [outputFilename '.dll']), outputDir);
copyfile(fullfile(path_external_functions_filename_install, 'lib', [outputFilename '.lib']), outputDir);

delete(fullfile(pathBuildExternalFunction, 'foo_jac.c'));
delete(fullfile(pathBuildExternalFunction, 'foo.py'));
rmdir(path_external_functions_filename_install, 's');
rmdir(path_external_functions_filename_build, 's');

end
