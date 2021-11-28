% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This function provides an interface to call generateMotFile.mat from
% Python.
%
% Author: Lars D'Hondt
% November 2021
%
%--------------------------------------------------------------------------
%
% Inputs:   dataMatrixPy = numeric array to write to file
%                       (first column should be time, following columns are coordinate positions)
%           colnamesPy = list of column name strings
%           filenamePy = string containing the output filename (must include extension)
%

function generateMotFileFromPython(dataVecPy, colnamesPy, filenamePy)

filename = string(filenamePy);
colnames_coords = string(cell(colnamesPy));
% data = double(cell(dataVecPy));


colnames = {'time',colnames_coords{:}};
ncols = length(colnames);
dataMatrix = zeros(10,ncols);
dataMatrix(:,1) = 0.01:0.01:0.1;
for i=2:ncols
    if strcmp(colnames{i},'pelvis_ty')
        dataMatrix(:,i) = -0.05;
    else
        dataMatrix(:,i) = 0.05;
    end

%         dataMatrix(:,i) = data(i-1);
end


generateMotFile(dataMatrix, colnames, filename)


