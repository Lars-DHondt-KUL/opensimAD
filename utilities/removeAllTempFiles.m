function [] = removeAllTempFiles()
% --------------------------------------------------------------------------
% removeAllTempFiles
%   Removes all temporary files that arecreaten when running opensimAD.
%   These should be removed automatically if all goes well, but files can
%   persist if an error occurred.
%
%
% INPUT:
%   -- (This function does not take input arguments) -
%
%
% OUTPUT:
%   - (This function does not return output arguments) -
%
% Original author: Lars D'Hondt
% Original date: 15/May/2023
%
% Last edit by: 
% Last edit date: 
% --------------------------------------------------------------------------

[pathUtilities,~,~] = fileparts(mfilename('fullpath'));
[pathMain,~,~] = fileparts(pathUtilities);


dir1 = dir(fullfile(pathMain, 'buildExpressionGraph'));
for i=1:length(dir1)
    if ~strcmp(dir1(i).name,'.') && ~strcmp(dir1(i).name,'..') && dir1(i).isdir
        rmdir(fullfile(dir1(i).folder,dir1(i).name), 's');
    end
end

dir1 = dir(fullfile(pathMain, 'buildExternalFunction/'));
for i=1:length(dir1)
    if ~strcmp(dir1(i).name,'.') && ~strcmp(dir1(i).name,'..') && dir1(i).isdir
        rmdir(fullfile(dir1(i).folder,dir1(i).name), 's');
    end
end

dir1 = dir(fullfile(pathMain, 'installExternalFunction'));
for i=1:length(dir1)
    if ~strcmp(dir1(i).name,'.') && ~strcmp(dir1(i).name,'..') && dir1(i).isdir
        rmdir(fullfile(dir1(i).folder,dir1(i).name), 's');
    end
end

lockFile = fullfile(pathMain, 'OpenSimAD-install', 'bin', 'lockFile.txt');
if isfile(lockFile)
    delete(lockFile)
end

path_bin_foo = fullfile(pathMain, 'OpenSimAD-install', 'bin', 'foo.py');
if isfile(path_bin_foo)
    delete(path_bin_foo)
end

end