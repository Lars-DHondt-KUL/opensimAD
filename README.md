# OpenSimAD
Windows libraries for OpenSimAD - OpenSim with support for Algorithmic Differentiation.

## How to generate an external function for use with CasADi?
OpenSimAD is used to formulate trajectory optimization problems with OpenSim musculoskeletal models. To leverage the benefits of algorithmic differentiation, we use [CasADi external functions](https://web.casadi.org/docs/#casadi-s-external-function). In our case, the external functions typically take as inputs the multi-body model states (joint positions and speeds) and controls (join accelerations) and return the joint torques after solving inverse dynamics. The external functions can then be called when formulating trajectory optimization problems (e.g., https://github.com/antoinefalisse/3dpredictsim and https://github.com/antoinefalisse/predictsim_mtp).

## Automating the workflow
The code in this branch is adapted to require less manual inputs. Main differences in workflow are:
1) Entering all joint and coordinate names in the appropriate order is no longer necessary. They are automatically retrieved from the given OpenSim model file.
2) The name of every coordinate is saved (in a .mat file), along with their index in the input/output of the external function. This file also holds the sets of indices corresponding to each joint, groups of joints, all rotational coordinates, all translational coordinates. It also has the indices to the additional outputs such as ground reaction forces (x,y,z), or body origin locations (x,y,z).
3) The code generates a suitable motion (.mot file) to perform an inverse dynamic analysis of the model. The result of this analysis is needed to check the generated external function.

The joint groups are defined as:
- *floating_base:* the joint that has its parent coordinate frame connected to the ground
- *leg_r:* the joint with name "hip_r", and every joint further down this kinematic chain
- *arm_r:* the joint with name 'acromial_r" and every joint further down this kinematic chain
- *leg_l:* the joint with name "hip_l", and every joint further down this kinematic chain
- *arm_l:* the joint with name 'acromial_l" and every joint further down this kinematic chain
- *torso:* all joints that do not fall in an aforementioned group
These groups can be used te generalise full body models. 

This workflow is not limited to full body models. [Any OpenSim model](https://user-images.githubusercontent.com/71920801/143950905-9ef6263e-c763-409a-bf7e-905efd8d28b8.png) within the given [limitations](#Limitations) can be used to generate an external function.

## Getting started
Here we provide code and examples to generate external functions automatically given an OpenSim musculoskeletal model (.osim file).

### Install requirements (Windows)
#### If you have the main version of OpenSimAD
  - Get adapted files:
    - Swap the existing *main.py* and *utilities.py* for those from this branch
  - conda environment:
    - Open an Anaconda prompt
    - Activate environment: `conda activate opensimAD`
    - Install required packages: `pip install scipy`
   
#### If you do not have the main version of OpenSimAD
  - Third-party software:
    - CMake (make sure cmake.exe is in your path)
    - Visual studio (tested with Visual Studio 2017 Community only)
    - Anaconda
  - conda environment:
    - Open an Anaconda prompt
    - Create environment: `conda create -n opensimAD pip spyder python=3.8`
    - Activate environment: `conda activate opensimAD`
    - Navigate to the folder where you want to download the code: eg. `cd Documents`
    - Download code: `git clone https://github.com/antoinefalisse/opensimAD.git`
    - Navigate to the folder: `cd opensimAD`
    - Install required packages: `python -m pip install -r requirements.txt`
    - Install OpenSim by following the instructions [here](https://simtk-confluence.stanford.edu:8443/display/OpenSim/Scripting+in+Python)

### Example
  - run `main.py`
      - You should get as output a few files in the example folder. Among them: `F.cpp` and `F.dll`. The .cpp file contains the source code of the external function, whereas the .dll file is the [dynamically linked library](https://web.casadi.org/docs/#casadi-s-external-function) that can be called when formulating your trajectory optimization problem.
      - More details in the comments of `main.py` about what inputs are necessary and optional.

### Limitations
  - Not all OpenSim models are supported:
    - Your model **should not have locked joints**. Please replace them with weld joints (locked joints would technically require having kinematic constraints, which is possible but makes the problem more complicated).
    - **Constraints will be ignored** (eg, coupling constraints).
    - **SimmSplines are not supported**, as their implementation in OpenSim is not really compatible with algorithmic differentiation. See how we replaced the splines of the [LaiArnold_modifed model](https://simtk.org/projects/model-high-flex) with polynomials.
  - OpenSimAD does not support all features of OpenSim. **Make sure you verify what you are doing**. We have only used OpenSimAD for specific applications.

### Troubleshooting
- On KU Leuven computers, run opensimAD from the \_MyPrograms folder to prevent issues with group policy.
- When getting an error message containing "opensim" or "simbody", check your [OpenSim installation](https://simtk-confluence.stanford.edu:8443/display/OpenSim/Scripting+in+Python) and path settings are correct.

## Tutorial
  - TODO: You can find here a tutorial describing how to generate a predictive simulation of walking. The tutorial describes all the steps required, including the use of OpenSimAD to general external functions for use when formulating the trajectory optimization problem underlying the predictive simulation. 

## Citation
Please cite this paper in your publications if OpenSimAD helps your research:
  - Falisse A, Serrancolí G, et al. (2019) Algorithmic differentiation improves the computational efficiency of OpenSim-based trajectory optimization of human movement. PLoS ONE 14(10): e0217730. https://doi.org/10.1371/journal.pone.0217730

Please cite this paper in your publications if you used OpenSimAD for simulations of human walking:
  - Falisse A, et al. (2019) Rapid predictive simulations with complex musculoskeletal models suggest that diverse healthy and pathological human gaits can emerge from similar control strategies. J. R. Soc. Interface.162019040220190402. http://doi.org/10.1098/rsif.2019.0402

## Source code
The libraries were compiled from [here](https://github.com/antoinefalisse/opensim-core/tree/AD-recorder-work-py-install).
