
import os
import sys
import opensim
import numpy as np
import casadi as ca
import shutil
import importlib
import pandas as pd
import scipy.io as sio
import fileinput
from utilities import writeCppFile
from utilities import modelinfo2matfile

# Read all settings from the matfile
MatFile = sys.argv[1]
matInfo = sio.loadmat(MatFile, simplify_cells=1)

# test if we can output from this
pathOpenSimModel = matInfo['pathOpenSimModel']
outputFilename = matInfo['outputFilename']
pathOut = matInfo['pathOut']

# get current path
pathMain = os.getcwd()

# Provide path to the InverseDynamics folder.
pathID = ''

# Joints and coordinates.
# There is no need to provide joints or coordinates, they are retrieved from the
# .osim file. The indices corresponding to the outputs of the generated external
# function are saved as a .mat file.

# If you want to choose the order of the outputs, you still can.
jointsOrder= matInfo['jointsOrder']
jointsOrder = jointsOrder.tolist()

coordinatesOrder= matInfo['coordinatesOrder']
coordinatesOrder = coordinatesOrder.tolist()

# %% Optional user inputs.
# Compiler (default is "Visual Studio 15 2017 Win64").
# compiler = "Visual Studio 15 2017 Win64"
compiler = ''

# By default, the external function returns the joint torques. However, you
# can also export other variables that you may want to use when formulating
# your problem. Here, we provide a few examples of variables we typically use
# in our problems.

# Export 3D segment origins.
# Leave empty or do not pass as argument to not export those variables.
export3DSegmentOrigins = matInfo['export3DSegmentOrigins']
export3DSegmentOrigins = export3DSegmentOrigins.tolist()

# Export total GRFs.
# If True, right and left 3D GRFs (in this order) are exported. Set False or
# do not pass as argument to not export those variables.
exportGRFs= bool(matInfo['exportGRFs'])

# Export separate GRFs.
# If True, right and left 3D GRFs (in this order) are exported for each of the
# 6 contact spheres.
# Set False or do not pass as argument to not export those variables.
exportSeparateGRFs = bool(matInfo['exportSeparateGRFs'])

# Export GRMs.
# If True, right and left 3D GRMs (in this order) are exported. Set False or
# do not pass as argument to not export those variables.
exportGRMs = bool(matInfo['exportGRMs'])

# Export contact sphere vertical deformation power.
# If True, right and left vertical deformation power of all contact spheres
# are exported.
# Set False or do not pass as argument to not export those variables.
exportContactPowers = bool(matInfo['exportContactPowers'])

# create a .cpp file
IO_indices = writeCppFile(pathOpenSimModel, pathOut,
                         jointsOrder=jointsOrder,
                         coordinatesOrder=coordinatesOrder,
                         exportGRFs=exportGRFs,
                         export3DSegmentOrigins=export3DSegmentOrigins,
                         exportGRMs=exportGRMs,
                         exportSeparateGRFs=exportSeparateGRFs,
                         exportContactPowers=exportContactPowers,
                         outputFilename=outputFilename)

# write model information to a .mat file
modelinfo2matfile(pathOut, IO_indices, outputFilename)