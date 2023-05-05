function [varargout] = generateExternalFunction(pathOpenSimModel, outputDir, pathID, jointsOrder, coordinatesOrder,...
    export3DPositions, export3DVelocities, exportGRFs, exportGRMs, exportSeparateGRFs, exportContactPowers, outputFilename, compiler)

    % write the cpp file. This functions creates a struct with various
    % information related to the indexing in the opensim model
    IO_indices = writeCppFile(pathOpenSimModel, outputDir, jointsOrder, coordinatesOrder, export3DPositions, export3DVelocities, exportGRFs, exportGRMs, exportSeparateGRFs, exportContactPowers, outputFilename);
    all_coordi = IO_indices.coordi; % struct with indexing coordinates in model
    jointi = IO_indices.jointi;
    joint_isTra = jointi.translations; % translation dofs
    nCoordinates = IO_indices.nCoordinates; % number of coordinates in model
    coordinatesOrder = IO_indices.coordinatesOrder; % order of the joint coordinates

    % Build external Function (.dll file).
    if nargout==0
        buildExternalFunction(outputFilename, outputDir, 3 * nCoordinates, compiler);
    else
        [fooPath] = buildExternalFunction(outputFilename, outputDir, 3 * nCoordinates, compiler);
        varargout{1} = fooPath;
    end


    % Verification
    % Run ID with the .osim file and verify that we can get the same torques
    % as with the external function.
    if nargout==0
        VerifyInverseDynamics(pathOpenSimModel, outputDir, pathID, coordinatesOrder, outputFilename, all_coordi, joint_isTra, nCoordinates);
    end

    % write a .mat file with all input-output information
    IO = IO_indices;
    save(fullfile(outputDir, [outputFilename, '_IO.mat']),'IO')

end


function VerifyInverseDynamics(pathOpenSimModel, outputDir, pathID, coordinatesOrder, outputFilename, all_coordi, joint_isTra, nCoordinates)
import org.opensim.modeling.*;
import casadi.*

% Run ID with the .osim file and verify that we can get the same torques as with the external function.
model = Model(pathOpenSimModel);
model.initSystem();
coordinateSet = model.getCoordinateSet();

% Extract torques from external function.
F = external('F', replace(fullfile(outputDir, [outputFilename, '.dll']),'\','/'));
vec1 = zeros(nCoordinates * 2, 1);
vec1(1:2:end) = 0.05;
if isfield(all_coordi, 'pelvis_ty')
    vec1((all_coordi.pelvis_ty - 1) * 2 + 1) = -0.05;
end
vec2 = zeros(nCoordinates, 1);
vec3 = [vec1; vec2];
ID_F = full(F(vec3)).flatten(1:nCoordinates);

% Generate .mot file with same position inputs
mot_file = ['Verify_', outputFilename, '.mot'];
path_mot = fullfile(pathID, mot_file);

if ~exist(path_mot, 'file')
    labels = ['time', coordinatesOrder];
    vec4 = vec1(1:2:end);
    data_coords = repmat(vec4, 1, 10);
    data_time = zeros(1, 10);
    data_time(1, :) = 0.01:0.01:0.1;
    data = [data_time', data_coords'];
    numpy2storage(labels, data, path_mot);
end

pathGenericIDSetupFile = fullfile(pathID, 'SetupID.xml');
idTool = InverseDynamicsTool(pathGenericIDSetupFile);
idTool.setName('ID_withOsimAndIDTool');
idTool.setModelFileName(pathOpenSimModel);
idTool.setResultsDir(outputDir);
idTool.setCoordinatesFileName(path_mot);
idTool.setOutputGenForceFileName('ID_withOsimAndIDTool.sto');
pathSetupID = fullfile(outputDir, 'SetupID.xml');
idTool.printToXML(pathSetupID);

command = ['opensim-cmd', ' run-tool ', pathSetupID];
system(command);

% Extract torques from .osim + ID tool.
headers = {};
nCoordinatesAll = coordinateSet.getSize();
for coord = 0:nCoordinatesAll-1
    if any(all_coordi.(char(coordinateSet.get(coord).getName())) == joint_isTra)
        suffix_header = '_force';
    else
        suffix_header = '_moment';
    end
    headers{end+1} = [char(coordinateSet.get(coord).getName()), suffix_header];
end

ID_osim_df = storage2df(fullfile(outputDir, 'ID_withOsimAndIDTool.sto'), headers);
ID_osim = zeros(nCoordinates, 1);
for count = 1:numel(coordinatesOrder)
    coordinateOrder = coordinatesOrder{count};
    if any(all_coordi.(coordinateOrder) == joint_isTra)
        suffix_header = '_force';
    else
        suffix_header = '_moment';
    end
    ID_osim(count) = ID_osim_df.(coordinateOrder)(1) + ID_osim_df.(coordinateOrder + suffix_header)(1);
end

% Assert we get the same torques.
assert(max(abs(ID_osim - ID_F)) < 1e-6, 'error F vs ID tool & osim');
end

function [IO_indices] = writeCppFile(pathOpenSimModel, outputDir, jointsOrder, coordinatesOrder, export3DPositions, export3DVelocities, exportGRFs, exportGRMs, exportSeparateGRFs, exportContactPowers, outputFilename)

% Paths.
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
pathOutputFile = fullfile(outputDir, strcat(outputFilename, '.cpp'));

% Generate external Function (.cpp file).
import org.opensim.modeling.*;

model = Model(pathOpenSimModel);
model.initSystem();
bodySet = model.getBodySet();
jointSet = model.getJointSet();
nJoints = jointSet.getSize();
geometrySet = model.getContactGeometrySet();
forceSet = model.getForceSet();
coordinateSet = model.getCoordinateSet();
nCoordinates = coordinateSet.getSize();
coordinates = {};
for coor = 0:nCoordinates-1
    coordinates{coor+1} = char(coordinateSet.get(coor).getName());
end

sides = {'r', 'l'};
for i = 1:length(sides)
    side = sides{i};
    if ismember(sprintf('knee_angle_%s_beta', side), coordinates)
        nCoordinates = nCoordinates - 1;
        nJoints = nJoints - 1;
    end
end

if isempty(jointsOrder) || isempty(coordinatesOrder)
    jointsOrder = {};
    coordinatesOrder = {};
    for i = 0:nJoints-1
        joint_i = jointSet.get(i);
        joint_name_i = char(joint_i.getName());
        if ~contains(joint_name_i, 'patel')
            jointsOrder{end+1} = joint_name_i;
            joint_i_Nc = joint_i.numCoordinates();
            for j = 0:joint_i_Nc-1
                joint_i_coor = joint_i.get_coordinates(j);
                if joint_i_coor.get_locked()
                    nCoordinates = nCoordinates - 1;
                else
                    coordinatesOrder{end+1} = char(joint_i_coor.getName());
                end
            end
        end
    end
end

nContacts = 0;
for i = 0:forceSet.getSize()-1
    c_force_elt = forceSet.get(i);
    if strcmp(c_force_elt.getConcreteClassName(), 'SmoothSphereHalfSpaceForce')
        nContacts = nContacts + 1;
    end
end

fid = fopen(pathOutputFile,'w');

% TODO: only include those that are necessary (model-specific).
fprintf(fid, '#include <OpenSim/Simulation/Model/Model.h>\n');
fprintf(fid, '#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>\n');
fprintf(fid, '#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>\n');
fprintf(fid, '#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>\n');
fprintf(fid, '#include <OpenSim/Simulation/SimbodyEngine/Joint.h>\n');
fprintf(fid, '#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>\n');
fprintf(fid, '#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>\n');
fprintf(fid, '#include <OpenSim/Common/LinearFunction.h>\n');
fprintf(fid, '#include <OpenSim/Common/PolynomialFunction.h>\n');
fprintf(fid, '#include <OpenSim/Common/MultiplierFunction.h>\n');
fprintf(fid, '#include <OpenSim/Common/Constant.h>\n');
fprintf(fid, '#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>\n');
fprintf(fid, '#include "SimTKcommon/internal/recorder.h"\n\n');

fprintf(fid, '#include <iostream>\n');
fprintf(fid, '#include <iterator>\n');
fprintf(fid, '#include <random>\n');
fprintf(fid, '#include <cassert>\n');
fprintf(fid, '#include <algorithm>\n');
fprintf(fid, '#include <vector>\n');
fprintf(fid, '#include <fstream>\n\n');

fprintf(fid, 'using namespace SimTK;\n');
fprintf(fid, 'using namespace OpenSim;\n\n');


nOutputs = nCoordinates;

if exportGRFs
    nOutputs = nOutputs + 6;
end
if exportSeparateGRFs
    nOutputs = nOutputs + 3*nContacts;
end
if exportContactPowers
    nOutputs = nOutputs + nContacts;
end
if exportGRMs
    nOutputs = nOutputs + 6;
end
if ~isempty(export3DPositions)
    nOutputs = nOutputs + 3*length(export3DPositions);
end
if ~isempty(export3DVelocities)
    nOutputs = nOutputs + 3*length(export3DVelocities);
end

fprintf(fid, 'constexpr int n_in = 2; \n');
fprintf(fid, 'constexpr int n_out = 1; \n');
fprintf(fid, 'constexpr int nCoordinates = %i; \n',nCoordinates);
fprintf(fid, 'constexpr int NX = nCoordinates*2; \n');
fprintf(fid, 'constexpr int NU = nCoordinates; \n');
fprintf(fid, 'constexpr int NR = %i; \n\n', nOutputs);

fprintf(fid, 'template<typename T> \n');
fprintf(fid, 'T value(const Recorder& e) { return e; }; \n');
fprintf(fid, 'template<> \n');
fprintf(fid, 'double value(const Recorder& e) { return e.getValue(); }; \n\n');

fprintf(fid, 'SimTK::Array_<int> getIndicesOSInSimbody(const Model& model) { \n');
fprintf(fid, '\tauto s = model.getWorkingState(); \n');
fprintf(fid, '\tconst auto svNames = model.getStateVariableNames(); \n');
fprintf(fid, '\tSimTK::Array_<int> idxOSInSimbody(s.getNQ()); \n');
fprintf(fid, '\ts.updQ() = 0; \n');
fprintf(fid, '\tfor (int iy = 0; iy < s.getNQ(); ++iy) { \n');
fprintf(fid, '\t\ts.updQ()[iy] = SimTK::NaN; \n');
fprintf(fid, '\t\tconst auto svValues = model.getStateVariableValues(s); \n');
fprintf(fid, '\t\tfor (int isv = 0; isv < svNames.size(); ++isv) { \n');
fprintf(fid, '\t\t\tif (SimTK::isNaN(svValues[isv])) { \n');
fprintf(fid, '\t\t\t\ts.updQ()[iy] = 0; \n');
fprintf(fid, '\t\t\t\tidxOSInSimbody[iy] = isv/2; \n');
fprintf(fid, '\t\t\t\tbreak; \n');
fprintf(fid, '\t\t\t} \n');
fprintf(fid, '\t\t} \n');
fprintf(fid, '\t} \n');
fprintf(fid, '\treturn idxOSInSimbody; \n');
fprintf(fid, '} \n\n');

fprintf(fid, 'SimTK::Array_<int> getIndicesSimbodyInOS(const Model& model) { \n');
fprintf(fid, '\tauto idxOSInSimbody = getIndicesOSInSimbody(model); \n');
fprintf(fid, '\tauto s = model.getWorkingState(); \n');
fprintf(fid, '\tSimTK::Array_<int> idxSimbodyInOS(s.getNQ()); \n');
fprintf(fid, '\tfor (int iy = 0; iy < s.getNQ(); ++iy) { \n');
fprintf(fid, '\t\tfor (int iyy = 0; iyy < s.getNQ(); ++iyy) { \n');
fprintf(fid, '\t\t\tif (idxOSInSimbody[iyy] == iy) { \n');
fprintf(fid, '\t\t\t\tidxSimbodyInOS[iy] = iyy; \n');
fprintf(fid, '\t\t\t\tbreak; \n');
fprintf(fid, '\t\t\t} \n');
fprintf(fid, '\t\t} \n');
fprintf(fid, '\t} \n');
fprintf(fid, '\treturn idxSimbodyInOS; \n');
fprintf(fid, '} \n\n');

fprintf(fid, 'template<typename T>\n');
fprintf(fid, 'int F_generic(const T** arg, T** res) {\n\n');

% Model
fprintf(fid, '\t// Definition of model.\n');
fprintf(fid, '\tOpenSim::Model* model;\n');
fprintf(fid, '\tmodel = new OpenSim::Model();\n\n');

% Bodies
fprintf(fid, '\t// Definition of bodies.\n');
for i = 0:bodySet.getSize()-1
    c_body = bodySet.get(i);
    c_body_name = char(c_body.getName());

    if (strcmp(c_body_name, 'patella_l') || strcmp(c_body_name, 'patella_r'))
        continue;
    end

    c_body_mass = c_body.get_mass();
    c_body_mass_center = c_body.get_mass_center().getAsMat();
    c_body_inertia = c_body.get_inertia();
    c_body_inertia_vec3 = [c_body_inertia.get(0), c_body_inertia.get(1), c_body_inertia.get(2)];
    fprintf(fid, '\tOpenSim::Body* %s;\n', c_body_name);
    fprintf(fid, '\t%s = new OpenSim::Body(\"%s\", %.20f, Vec3(%.20f, %.20f, %.20f), Inertia(%.20f, %.20f, %.20f, 0., 0., 0.));\n', ...
            c_body_name, c_body_name, c_body_mass, c_body_mass_center(1), c_body_mass_center(2), ...
            c_body_mass_center(3), c_body_inertia_vec3(1), c_body_inertia_vec3(2), c_body_inertia_vec3(3));
    fprintf(fid, '\tmodel->addBody(%s);\n', c_body_name);
    fprintf(fid, '\n');

end


% Loop through joint set
fprintf(fid, '\t// Definition of joints.\n');
for i = 0:jointSet.getSize()-1
    c_joint = jointSet.get(i);
    c_joint_type = char(c_joint.getConcreteClassName());

    c_joint_name = char(c_joint.getName());
    if strcmp(c_joint_name, 'patellofemoral_l') || strcmp(c_joint_name, 'patellofemoral_r')
        continue
    end

    parent_frame = c_joint.get_frames(0);
    parent_frame_name = char(parent_frame.getParentFrame().getName());
    parent_frame_trans = parent_frame.get_translation().getAsMat();
    parent_frame_or = parent_frame.get_orientation().getAsMat();

    child_frame = c_joint.get_frames(1);
    child_frame_name = char(child_frame.getParentFrame().getName());
    child_frame_trans = child_frame.get_translation().getAsMat();
    child_frame_or = child_frame.get_orientation().getAsMat();

    % Custom joints
    if strcmp(c_joint_type, 'CustomJoint')

        fprintf(fid, '\tSpatialTransform st_%s;\n', c_joint.getName());

        cObj = CustomJoint.safeDownCast(c_joint);
        spatialtransform = cObj.get_SpatialTransform();

        nCoords = spatialtransform.getCoordinateNames().getSize();

        for iCoord = 1:6
            if iCoord == 1
                dofSel = spatialtransform.get_rotation1();
            elseif iCoord == 2
                dofSel = spatialtransform.get_rotation2();
            elseif iCoord == 3
                dofSel = spatialtransform.get_rotation3();
            elseif iCoord == 4
                dofSel = spatialtransform.get_translation1();
            elseif iCoord == 5
                dofSel = spatialtransform.get_translation2();
            elseif iCoord == 6
                dofSel = spatialtransform.get_translation3();
            end
            coord = iCoord-1;

            % Transform axis.
            dofSel_axis = dofSel.get_axis().getAsMat();
            dofSel_f = dofSel.get_function();
            if strcmp(char(dofSel_f.getConcreteClassName()), 'LinearFunction')
                dofSel_f_obj = LinearFunction.safeDownCast(dofSel_f);
                dofSel_f_slope = dofSel_f_obj.getSlope();
                dofSel_f_intercept = dofSel_f_obj.getIntercept();
                %c_coord_name = c_joint.get_coordinates(coord).getName();
                c_coord_name = char(dofSel.get_coordinates(0));
                fprintf(fid, '\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n', ...
                    c_joint.getName(), coord, c_coord_name);
                fprintf(fid, '\tst_%s[%i].setFunction(new LinearFunction(%.4f, %.4f));\n', ...
                    c_joint.getName(), coord, dofSel_f_slope, dofSel_f_intercept);
            elseif strcmp(char(dofSel_f.getConcreteClassName()), 'PolynomialFunction')
                fprintf(fid, '\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n', c_joint.getName(), coord, c_coord_name);
                dofSel_f_obj = PolynomialFunction.safeDownCast(dofSel_f);
                dofSel_f_coeffs = dofSel_f_obj.getCoefficients().getAsMat();
                c_nCoeffs = size(dofSel_f_coeffs, 1);
                if c_nCoeffs == 2
                    fprintf(fid, '\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n', c_joint.getName(), coord, c_nCoeffs, dofSel_f_coeffs(1), dofSel_f_coeffs(2));
                elseif c_nCoeffs == 3
                    fprintf(fid, '\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n', c_joint.getName(), coord, c_nCoeffs, dofSel_f_coeffs(1), dofSel_f_coeffs(2), dofSel_f_coeffs(3));
                elseif c_nCoeffs == 4
                    fprintf(fid, '\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n', c_joint.getName(), coord, c_nCoeffs, dofSel_f_coeffs(1), dofSel_f_coeffs(2), dofSel_f_coeffs(3), dofSel_f_coeffs(4));
                elseif c_nCoeffs == 5
                    fprintf(fid, '\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n', c_joint.getName(), coord, c_nCoeffs, dofSel_f_coeffs(1), dofSel_f_coeffs(2), dofSel_f_coeffs(3), dofSel_f_coeffs(4), dofSel_f_coeffs(5));
                elseif c_nCoeffs == 7
                    fprintf(fid, '\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f, %.20f, %.20f}; \n', c_joint.getName(), coord, c_nCoeffs, dofSel_f_coeffs(1), dofSel_f_coeffs(2), dofSel_f_coeffs(3), dofSel_f_coeffs(4), dofSel_f_coeffs(5), dofSel_f_coeffs(6), dofSel_f_coeffs(7));
                else
                    error('TODO');
                end
                fprintf(fid, '\tVector st_%s_%i_coeffs_vec(%i); \n' , c_joint.getName(), coord, c_nCoeffs);
                fprintf(fid, '\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n', c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord);
                fprintf(fid, '\tst_%s[%i].setFunction(new PolynomialFunction(st_%s_%i_coeffs_vec));\n', c_joint.getName(), coord, c_joint.getName(), coord);

            elseif strcmp(dofSel_f.getConcreteClassName(), 'MultiplierFunction')
                dofSel_f_obj = opensim.MultiplierFunction.safeDownCast(dofSel_f);
                dofSel_f_obj_scale = dofSel_f_obj.getScale();
                dofSel_f_obj_f = dofSel_f_obj.getFunction();
                dofSel_f_obj_f_name = dofSel_f_obj_f.getConcreteClassName();
                if strcmp(dofSel_f_obj_f_name, 'Constant')
                    dofSel_f_obj_f_obj = opensim.Constant.safeDownCast(dofSel_f_obj_f);
                    dofSel_f_obj_f_obj_value = dofSel_f_obj_f_obj.getValue();
                    fprintf(fid, '\tst_%s[%i].setFunction(new MultiplierFunction(new Constant(%.20f), %.20f));\n', c_joint.getName(), coord, dofSel_f_obj_f_obj_value, dofSel_f_obj_scale);
                elseif strcmp(dofSel_f_obj_f_name, 'PolynomialFunction')
                    fprintf(fid, '\tst_%s[%i].setCoordinateNames(OpenSim::Arraystd::string("%s", 1, 1));\n', c_joint.getName(), coord, c_coord_name);
                    dofSel_f_obj_f_obj = opensim.PolynomialFunction.safeDownCast(dofSel_f_obj_f);
                    dofSel_f_obj_f_coeffs = dofSel_f_obj_f_obj.getCoefficients().getAsMat();
                    c_nCoeffs = size(dofSel_f_obj_f_coeffs,1);
                    if c_nCoeffs == 2
                        fprintf(fid, '\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n', c_joint.getName(), coord, c_nCoeffs, dofSel_f_obj_f_coeffs(1), dofSel_f_obj_f_coeffs(2));
                    elseif c_nCoeffs == 3
                        fprintf(fid, '\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n', c_joint.getName(), coord, c_nCoeffs, dofSel_f_obj_f_coeffs(1), dofSel_f_obj_f_coeffs(2), dofSel_f_obj_f_coeffs(3));
                    elseif c_nCoeffs == 4
                        fprintf(fid, '\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n', c_joint.getName(), coord, c_nCoeffs, dofSel_f_obj_f_coeffs(1), dofSel_f_obj_f_coeffs(2), dofSel_f_obj_f_coeffs(3), dofSel_f_obj_f_coeffs(4));
                    elseif c_nCoeffs == 5
                        fprintf(fid, '\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n', c_joint.getName(), coord, c_nCoeffs, dofSel_f_obj_f_coeffs(1), dofSel_f_obj_f_coeffs(2), dofSel_f_obj_f_coeffs(3), dofSel_f_obj_f_coeffs(4), dofSel_f_obj_f_coeffs(5));
                    else
                        error('TODO')
                    end
                    fprintf(fid, '\tVector st_%s_%i_coeffs_vec(%i); \n', c_joint.getName(), coord, c_nCoeffs);
                    fprintf(fid, '\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n', c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord);
                    fprintf(fid,  '\tst_%s[%i].setFunction(new MultiplierFunction(new PolynomialFunction(st_%s_%i_coeffs_vec), %.20f));\n', c_joint.getName(), coord, c_joint.getName(), coord, dofSel_f_obj_scale);

                else
                    error('Not supported')
                end

            elseif strcmp(dofSel_f.getConcreteClassName(), 'Constant')
                dofSel_f_obj = Constant.safeDownCast(dofSel_f);
                dofSel_f_obj_value = dofSel_f_obj.getValue();
                fprintf(fid, '\tst_%s[%i].setFunction(new Constant(%.20f));\n', ...
                        c_joint.getName(), coord, dofSel_f_obj_value);
            else
                error('Not supported');
            end
            fprintf(fid, '\tst_%s[%i].setAxis(Vec3(%.20f, %.20f, %.20f));\n', c_joint.getName(), coord, dofSel_axis(1), dofSel_axis(2), dofSel_axis(3));
        end

        % Joint.
        fprintf(fid, '\tOpenSim::%s* %s;\n', c_joint_type, c_joint.getName());
        if strcmp(parent_frame_name, "ground")
            fprintf(fid, '\t%s = new OpenSim::%s("%s", model->getGround(), Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), st_%s);\n', ...
            c_joint.getName(), c_joint_type, c_joint.getName(), parent_frame_trans(1), parent_frame_trans(2), parent_frame_trans(3), ...
            parent_frame_or(1), parent_frame_or(2), parent_frame_or(3), child_frame_name, child_frame_trans(1), child_frame_trans(2), ...
            child_frame_trans(3), child_frame_or(1), child_frame_or(2), child_frame_or(3), c_joint.getName());
        else
            fprintf(fid, '\t%s = new OpenSim::%s("%s", *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), st_%s);\n', ...
            c_joint.getName(), c_joint_type, c_joint.getName(), parent_frame_name, parent_frame_trans(1), parent_frame_trans(2), ...
            parent_frame_trans(3), parent_frame_or(1), parent_frame_or(2), parent_frame_or(3), child_frame_name, child_frame_trans(1), ...
            child_frame_trans(2), child_frame_trans(3), child_frame_or(1), child_frame_or(2), child_frame_or(3), c_joint.getName());
        end

    elseif strcmp(c_joint_type, 'PinJoint') || strcmp(c_joint_type, 'WeldJoint') || strcmp(c_joint_type, 'PlanarJoint')
        if strcmp(c_joint_type, 'PinJoint') && c_joint.get_coordinates(0).get_locked()
            c_joint_type = 'WeldJoint';
        end
        fprintf(fid, '\tOpenSim::%s* %s;\n', c_joint_type, c_joint.getName());
        if strcmp(parent_frame_name, 'ground')
            fprintf(fid, '\t%s = new OpenSim::%s(\"%s\", model->getGround(), Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f));\n', c_joint.getName(), c_joint_type, c_joint.getName(), parent_frame_trans(1), parent_frame_trans(2), parent_frame_trans(3), parent_frame_or(1), parent_frame_or(2), parent_frame_or(3), child_frame_name, child_frame_trans(1), child_frame_trans(2), child_frame_trans(3), child_frame_or(1), child_frame_or(2), child_frame_or(3));
        else
            fprintf(fid, '\t%s = new OpenSim::%s(\"%s\", *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f));\n', c_joint.getName(), c_joint_type, c_joint.getName(), parent_frame_name, parent_frame_trans(1), parent_frame_trans(2), parent_frame_trans(3), parent_frame_or(1), parent_frame_or(2), parent_frame_or(3), child_frame_name, child_frame_trans(1), child_frame_trans(2), child_frame_trans(3), child_frame_or(1), child_frame_or(2), child_frame_or(3));
        end
    else
        disp(['joint type error: ', c_joint_type]);
        error('TODO: joint type not yet supported');
    end
    
    fprintf(fid, '\n');

end

% Add joints to model in pre-defined order
jointi = [];
all_coordi = [];
joint_isRot = [];
joint_isTra = [];
outputCount = 1;

if ~isempty(jointsOrder)
    for i = 1:length(jointsOrder)
        jointOrder = jointsOrder{i};
        fprintf(fid, '\tmodel->addJoint(%s);\n', jointOrder);
        coordi = [];
        
        try
            c_joint = jointSet.get(jointOrder);
            c_joint_name = char(c_joint.getName());
            parent_frame = c_joint.get_frames(0);
            parent_frame_name = char(parent_frame.getParentFrame().getName());
            
            for j = 0:c_joint.numCoordinates()-1
                c_joint_coor = c_joint.get_coordinates(j);
                if ~c_joint_coor.get_locked()
                    all_coordi.(char(c_joint_coor.getName())) = outputCount;
                    coordi = [coordi, outputCount];
                    
                    if strcmp(c_joint_coor.getMotionType(),'Translational')
                        joint_isTra = [joint_isTra, outputCount];
                    else
                        joint_isRot = [joint_isRot, outputCount];
                    end
                    
                    outputCount = outputCount + 1;
                end
            end
            jointi.(c_joint_name) = coordi;
            
        catch
            error("Joint from jointOrder not in jointSet");
        end
    end
    
    assert(length(jointsOrder) == nJoints, 'jointsOrder and jointSet have different sizes');
end

fprintf(fid, '\n');


% Definition of contacts.
for i = 0:(forceSet.getSize()-1)
    c_force_elt = forceSet.get(i);
    if strcmp(c_force_elt.getConcreteClassName(),'SmoothSphereHalfSpaceForce')
        c_force_elt_obj = SmoothSphereHalfSpaceForce.safeDownCast(c_force_elt);

        socket0Name = c_force_elt.getSocketNames().get(0);
        socket0 = c_force_elt.getSocket(socket0Name);
        socket0_obj = socket0.getConnecteeAsObject();
        socket0_objName = socket0_obj.getName();
        geo0 = geometrySet.get(socket0_objName);
        geo0_loc = geo0.get_location().getAsMat();
        geo0_or = geo0.get_orientation().getAsMat();
        geo0_frameName = char(geo0.getFrame().getName());

        socket1Name = c_force_elt.getSocketNames().get(1);
        socket1 = c_force_elt.getSocket(socket1Name);
        socket1_obj = socket1.getConnecteeAsObject();
        socket1_objName = socket1_obj.getName();
        geo1 = geometrySet.get(socket1_objName);
        geo1_loc = geo1.get_location().getAsMat();
        % geo1_or = geo1.get_orientation().toMatlab();
        geo1_frameName = char(geo1.getFrame().getName());
        obj = ContactSphere.safeDownCast(geo1);
        geo1_radius = obj.getRadius();

        fprintf(fid, '\tOpenSim::%s* %s;\n',c_force_elt.getConcreteClassName(),c_force_elt.getName());
        if strcmp(geo0_frameName,'ground')
            fprintf(fid, '\t%s = new %s("%s", *%s, model->getGround());\n',c_force_elt.getName(),c_force_elt.getConcreteClassName(),c_force_elt.getName(),geo1_frameName);
        else
            fprintf(fid, '\t%s = new %s("%s", *%s, *%s);\n',c_force_elt.getName(),c_force_elt.getConcreteClassName(),c_force_elt.getName(),geo1_frameName,geo0_frameName);
        end
        fprintf(fid, '\tVec3 %s_location(%.20f, %.20f, %.20f);\n', c_force_elt.getName(), geo1_loc(1), geo1_loc(2), geo1_loc(3));
        fprintf(fid, '\t%s->set_contact_sphere_location(%s_location);\n', c_force_elt.getName(), c_force_elt.getName());
        fprintf(fid, '\tdouble %s_radius = (%.20f);\n', c_force_elt.getName(), geo1_radius);
        fprintf(fid, '\t%s->set_contact_sphere_radius(%s_radius );\n', c_force_elt.getName(), c_force_elt.getName());
        fprintf(fid, '\t%s->set_contact_half_space_location(Vec3(%.20f, %.20f, %.20f));\n', c_force_elt.getName(), geo0_loc(1), geo0_loc(2), geo0_loc(3));
        fprintf(fid, '\t%s->set_contact_half_space_orientation(Vec3(%.20f, %.20f, %.20f));\n', c_force_elt.getName(), geo0_or(1), geo0_or(2), geo0_or(3));
        
        fprintf(fid, '\t%s->set_stiffness(%.20f);\n', c_force_elt.getName(), c_force_elt_obj.get_stiffness());
        fprintf(fid, '\t%s->set_dissipation(%.20f);\n', c_force_elt.getName(), c_force_elt_obj.get_dissipation());
        fprintf(fid, '\t%s->set_static_friction(%.20f);\n', c_force_elt.getName(), c_force_elt_obj.get_static_friction());
        fprintf(fid, '\t%s->set_dynamic_friction(%.20f);\n', c_force_elt.getName(), c_force_elt_obj.get_dynamic_friction());
        fprintf(fid, '\t%s->set_viscous_friction(%.20f);\n', c_force_elt.getName(), c_force_elt_obj.get_viscous_friction());
        fprintf(fid, '\t%s->set_transition_velocity(%.20f);\n', c_force_elt.getName(), c_force_elt_obj.get_transition_velocity());
        
        fprintf(fid, '\t%s->connectSocket_sphere_frame(*%s);\n', c_force_elt.getName(), geo1_frameName);
        if strcmp(geo0_frameName, 'ground')
            fprintf(fid, '\t%s->connectSocket_half_space_frame(model->getGround());\n', c_force_elt.getName());
        else
            fprintf(fid, '\t%s->connectSocket_half_space_frame(*%s);\n', c_force_elt.getName(), geo0_frameName);
        end
        
        fprintf(fid, '\tmodel->addComponent(%s);\n', c_force_elt.getName());
        fprintf(fid, '\n');
    end
end


fprintf(fid, '\t// Initialize system.\n');
fprintf(fid, '\tSimTK::State* state;\n');
fprintf(fid, '\tstate = new State(model->initSystem());\n\n');

fprintf(fid, '\t// Read inputs.\n');
fprintf(fid, '\tstd::vector<T> x(arg[0], arg[0] + NX);\n');
fprintf(fid, '\tstd::vector<T> u(arg[1], arg[1] + NU);\n\n');

fprintf(fid, '\t// States and controls.\n');
fprintf(fid, '\tT ua[NU];\n');
fprintf(fid, '\tVector QsUs(NX);\n');
fprintf(fid, '\t/// States\n');
fprintf(fid, '\tfor (int i = 0; i < NX; ++i) QsUs[i] = x[i];\n');
fprintf(fid, '\t/// Controls\n');
fprintf(fid, '\t/// OpenSim and Simbody have different state orders.\n');
fprintf(fid, '\tauto indicesOSInSimbody = getIndicesOSInSimbody(*model);\n');
fprintf(fid, '\tfor (int i = 0; i < NU; ++i) ua[i] = u[indicesOSInSimbody[i]];\n\n');

fprintf(fid, '\t// Set state variables and realize.\n');
fprintf(fid, '\tmodel->setStateVariableValues(*state, QsUs);\n');
fprintf(fid, '\tmodel->realizeVelocity(*state);\n\n');

fprintf(fid, '\t// Compute residual forces.\n');
fprintf(fid, '\t/// Set appliedMobilityForces (# mobilities).\n');
fprintf(fid, '\tVector appliedMobilityForces(nCoordinates);\n');
fprintf(fid, '\tappliedMobilityForces.setToZero();\n');
fprintf(fid, '\t/// Set appliedBodyForces (# bodies + ground).\n');
fprintf(fid, '\tVector_<SpatialVec> appliedBodyForces;\n');
fprintf(fid, '\tint nbodies = model->getBodySet().getSize() + 1;\n');
fprintf(fid, '\tappliedBodyForces.resize(nbodies);\n');
fprintf(fid, '\tappliedBodyForces.setToZero();\n');
fprintf(fid, '\t/// Set gravity.\n');
fprintf(fid, '\tVec3 gravity(0);\n');
model_gravity = model.get_gravity().getAsMat();
fprintf(fid, '\tgravity[1] = %.20f;\n' , model_gravity(2));
fprintf(fid, '\t/// Add weights to appliedBodyForces.\n');
fprintf(fid, '\tfor (int i = 0; i < model->getBodySet().getSize(); ++i) {\n');
fprintf(fid, '\t\tmodel->getMatterSubsystem().addInStationForce(*state,\n');
fprintf(fid, '\t\tmodel->getBodySet().get(i).getMobilizedBodyIndex(),\n');
fprintf(fid, '\t\tmodel->getBodySet().get(i).getMassCenter(),\n');
fprintf(fid, '\t\tmodel->getBodySet().get(i).getMass()*gravity, appliedBodyForces);\n');
fprintf(fid, '\t}\n');
fprintf(fid, '\t/// Add contact forces to appliedBodyForces.\n');

count = 0;
for i = 0:forceSet.getSize()-1
    c_force_elt = forceSet.get(i);
    
    if strcmp(char(c_force_elt.getConcreteClassName()), "SmoothSphereHalfSpaceForce")
        c_force_elt_name = char(c_force_elt.getName());

        fprintf(fid, '\tArray<osim_double_adouble> Force_%d = %s->getRecordValues(*state);\n', count, c_force_elt_name);
        fprintf(fid, '\tSpatialVec GRF_%d;\n', count);

        fprintf(fid, '\tGRF_%i[0] = Vec3(Force_%i[3], Force_%i[4], Force_%i[5]);\n', count, count, count, count);
        fprintf(fid, '\tGRF_%i[1] = Vec3(Force_%i[0], Force_%i[1], Force_%i[2]);\n', count, count, count, count);

        socket1Name = char(c_force_elt.getSocketNames().get(1));
        socket1 = c_force_elt.getSocket(socket1Name);
        socket1_obj = socket1.getConnecteeAsObject();
        socket1_objName = char(socket1_obj.getName());
        geo1 = geometrySet.get(socket1_objName);
        geo1_frameName = char(geo1.getFrame().getName());

        fprintf(fid, '\tint c_idx_%d = model->getBodySet().get("%s").getMobilizedBodyIndex();\n', count, geo1_frameName);
        fprintf(fid, '\tappliedBodyForces[c_idx_%d] += GRF_%d;\n', count, count);
        count = count + 1;
        fprintf(fid, '\n');
    end
end

fprintf(fid, '\t/// knownUdot.\n');
fprintf(fid, '\tVector knownUdot(nCoordinates);\n');
fprintf(fid, '\tknownUdot.setToZero();\n');
fprintf(fid, '\tfor (int i = 0; i < nCoordinates; ++i) knownUdot[i] = ua[i];\n');
fprintf(fid, '\t/// Calculate residual forces.\n');
fprintf(fid, '\tVector residualMobilityForces(nCoordinates);\n');
fprintf(fid, '\tresidualMobilityForces.setToZero();\n');
fprintf(fid, '\tmodel->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,\n');
fprintf(fid, '\t\t\tappliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);\n\n');

if ~isempty(export3DPositions)
    fprintf(fid, '\t/// Station locations.\n');
    for i = 1:length(export3DPositions)
        segment = export3DPositions(i).body;
        station = export3DPositions(i).point_in_body;
        name = export3DPositions(i).name;
        fprintf(fid, '\tVec3 %s_posInGround = %s->findStationLocationInGround(*state, Vec3(%.20f, %.20f, %.20f));\n', name, segment, station(1), station(2), station(3));
    end
    fprintf(fid, '\n');
end

if ~isempty(export3DVelocities)
    fprintf(fid, '\t/// Station velocities.\n');
    for i = 1:length(export3DVelocities)
        segment = export3DVelocities(i).body;
        station = export3DVelocities(i).point_in_body;
        name = export3DVelocities(i).name;
        fprintf(fid, '\tVec3 %s_velInGround = %s->findStationVelocityInGround(*state, Vec3(%.20f, %.20f, %.20f));\n', name, segment, station(1), station(2), station(3));
    end
    fprintf(fid, '\n');
end

if exportGRFs
    fprintf(fid, '\t/// Ground reaction forces.\n');
    fprintf(fid, '\tSpatialVec GRF_r;\n');
    fprintf(fid, '\tSpatialVec GRF_l;\n');
    fprintf(fid, '\tGRF_r.setToZero();\n');
    fprintf(fid, '\tGRF_l.setToZero();\n\n');
    
    count = 0;
    for i =1:forceSet.getSize()
        c_force_elt = forceSet.get(i-1);
        if strcmp(c_force_elt.getConcreteClassName(),'SmoothSphereHalfSpaceForce')
            c_force_elt_name = char(c_force_elt.getName());
            if strcmp(c_force_elt_name(end-1:end),'_r')
                fprintf(fid, '\tGRF_r += GRF_%s;\n', num2str(count));
            elseif strcmp(c_force_elt_name(end-1:end),'_l')
                fprintf(fid, '\tGRF_l += GRF_%s;\n', num2str(count));
            else
                error("Cannot identify contact side");
            end
            count = count+1;
        end
    end
    fprintf(fid, '\n');
end


if exportGRMs
    % Ground reaction moments.
    fprintf(fid, '\t/// Ground reaction moments.\n');
    fprintf(fid, '\tSpatialVec GRM_r;\n');
    fprintf(fid, '\tSpatialVec GRM_l;\n');
    fprintf(fid, '\tGRM_r.setToZero();\n');
    fprintf(fid, '\tGRM_l.setToZero();\n');
    fprintf(fid, '\tVec3 normal(0, 1, 0);\n\n');

    count = 0;
    geo1_frameNames = {};
    for i = 1:forceSet.getSize()
        c_force_elt = forceSet.get(i-1);
        if strcmp(c_force_elt.getConcreteClassName(), 'SmoothSphereHalfSpaceForce')
            c_force_elt_name = char(c_force_elt.getName());
            socket1Name = c_force_elt.getSocketNames().get(1);
            socket1 = c_force_elt.getSocket(socket1Name);
            socket1_obj = socket1.getConnecteeAsObject();
            socket1_objName = socket1_obj.getName();
            geo1 = geometrySet.get(socket1_objName);
            geo1_frameName = char(geo1.getFrame().getName());

            if isempty(geo1_frameNames) || ~sum(contains(geo1_frameNames, geo1_frameName))
                fprintf(fid, '\tSimTK::Transform TR_GB_%s = %s->getMobilizedBody().getBodyTransform(*state);\n', geo1_frameName, geo1_frameName);
                geo1_frameNames{end+1} = geo1_frameName;
            end

            fprintf(fid, '\tVec3 %s_location_G = %s->findStationLocationInGround(*state, %s_location);\n', c_force_elt_name, geo1_frameName, c_force_elt_name);
            fprintf(fid, '\tVec3 %s_locationCP_G = %s_location_G - %s_radius * normal;\n', c_force_elt_name, c_force_elt_name, c_force_elt_name);
            fprintf(fid, '\tVec3 locationCP_G_adj_%i = %s_locationCP_G - 0.5*%s_locationCP_G[1] * normal;\n', count, c_force_elt_name, c_force_elt_name);
            fprintf(fid, '\tVec3 %s_locationCP_B = model->getGround().findStationLocationInAnotherFrame(*state, locationCP_G_adj_%i, *%s);\n', c_force_elt_name, count, geo1_frameName);
            fprintf(fid, '\tVec3 GRM_%i = (TR_GB_%s*%s_locationCP_B) %% GRF_%i[1];\n', count, geo1_frameName, c_force_elt_name, count);

            if strcmp(c_force_elt_name(end-1:end), '_r')
                fprintf(fid, '\tGRM_r += GRM_%i;\n', count);
            elseif strcmp(c_force_elt_name(end-1:end), '_l')
                fprintf(fid, '\tGRM_l += GRM_%i;\n', count);
            else
                error('Cannot identify contact side');
            end
            fprintf(fid, '\n');
            count = count + 1;
        end
    end
end

if exportContactPowers
    fprintf(fid, '\t/// Contact spheres deformation power.\n');
    count = 0;
    geo1_frameNames = {};
    for i = 1:forceSet.getSize()
        c_force_elt = forceSet.get(i-1);
        if strcmp(c_force_elt.getConcreteClassName(), 'SmoothSphereHalfSpaceForce')
            c_force_elt_name = c_force_elt.getName();
            socket1Name = c_force_elt.getSocketNames().get(1);
            socket1 = c_force_elt.getSocket(socket1Name);
            socket1_obj = socket1.getConnecteeAsObject();
            socket1_objName = socket1_obj.getName();
            geo1 = geometrySet.get(socket1_objName);
            geo1_frameName = char(geo1.getFrame().getName());

            fprintf(fid, '\tVec3 %s_velocity_G = %s->findStationVelocityInGround(*state, %s_location);\n', c_force_elt_name, geo1_frameName, c_force_elt_name);
            fprintf(fid, '\tosim_double_adouble P_HC_y_%i = %s_velocity_G[1]*GRF_%i[1][1];', count, c_force_elt_name, count);
            fprintf(fid, '\n');
            count = count + 1;
        end
    end
end

fprintf(fid, '\t/// Outputs.\n');
fprintf(fid, '\t/// Residual forces (OpenSim and Simbody have different state orders).\n');
fprintf(fid, '\tauto indicesSimbodyInOS = getIndicesSimbodyInOS(*model);\n');
fprintf(fid, '\tfor (int i = 0; i < NU; ++i) res[0][i] =\n');
fprintf(fid, '\t\t\tvalue<T>(residualMobilityForces[indicesSimbodyInOS[i]]);\n');
count_acc = 0;
jointi.rotations = joint_isRot;
jointi.translations = joint_isTra;
IO_indices.jointi = jointi;
IO_indices.coordi = all_coordi;
IO_indices.nCoordinates = nCoordinates;
IO_indices.coordinatesOrder = coordinatesOrder;


if ~isempty(export3DPositions)
    % 3D segment origins.
    IO_point_pos = struct();
    for c_seg = 1:length(export3DPositions)
        name = export3DPositions(c_seg).name;
        fprintf(fid, '\tfor (int i = 0; i < 3; ++i) res[0][i + NU + %i] = value<T>(%s_posInGround[i]);\n', count_acc + (c_seg-1) * 3, name);
        tmp = outputCount + count_acc + (c_seg - 1) * 3;
        segment_i = tmp : tmp + 2;
        IO_point_pos.(name) = segment_i;
    end
    count_acc = count_acc + 3 * length(export3DPositions);
    IO_indices.position = IO_point_pos;
end

if ~isempty(export3DVelocities)
    % 3D segment origins.
    IO_point_vel = struct();
    for c_seg = 1:length(export3DVelocities)
        name = export3DVelocities(c_seg).name;
        fprintf(fid, '\tfor (int i = 0; i < 3; ++i) res[0][i + NU + %i] = value<T>(%s_velInGround[i]);\n', count_acc + (c_seg-1) * 3, name);
        tmp = outputCount + count_acc + (c_seg - 1) * 3;
        segment_i = tmp : tmp + 2;
        IO_point_vel.(name) = segment_i;
    end
    count_acc = count_acc + 3 * length(export3DVelocities);
    IO_indices.velocity = IO_point_vel;
end

IO_GRFs = {};
if exportGRFs
    fprintf(fid, '\t/// Ground reaction forces.\n');
    fprintf(fid, '\tfor (int i = 0; i < 3; ++i) res[0][i + NU + %i] = value<T>(GRF_r[1][i]);\n', count_acc);
    fprintf(fid, '\tfor (int i = 0; i < 3; ++i) res[0][i + NU + %i] = value<T>(GRF_l[1][i]);\n', count_acc + 3);
    tmp = outputCount + count_acc;
    IO_GRFs.right_foot = tmp:tmp+2;
    IO_GRFs.left_foot = tmp+3:tmp+5;
    count_acc = count_acc + 6;
end

if exportSeparateGRFs
    fprintf(fid, '\t/// Separate Ground reaction forces.\n');
    count_GRF = 1;
    for i_GRF = 1:nContacts
        fprintf(fid, '\tfor (int i = 0; i < 3; ++i) res[0][i + NU + %i] = value<T>(GRF_%s[1][i]);\n', count_acc, num2str(i_GRF-1));
        tmp = outputCount + count_acc;
        IO_GRFs.contact_sphere_{i_GRF} = tmp:tmp+2;
        count_acc = count_acc + 3;
        count_GRF = count_GRF + 1;
    end
end

if (exportGRFs || exportSeparateGRFs)
    IO_indices.GRFs = IO_GRFs;
end

if exportGRMs
    IO_GRMs = struct();
    fprintf(fid, '\t/// Ground reaction moments.\n');
    fprintf(fid, '\tfor (int i = 0; i < 3; ++i) res[0][i + NU + %i] = value<T>(GRM_r[1][i]);\n', count_acc);
    fprintf(fid, '\tfor (int i = 0; i < 3; ++i) res[0][i + NU + %i] = value<T>(GRM_l[1][i]);\n', count_acc + 3);
    tmp = outputCount + count_acc;
    IO_GRMs.right_total = tmp:tmp+2;
    IO_GRMs.left_total = tmp+3:tmp+5;
    count_acc = count_acc + 6;
    IO_indices.GRMs = IO_GRMs;
end


if exportContactPowers
    IO_P_HC_ys = struct();
    fprintf(fid, '\t/// Contact spheres deformation power.\n');
    for i_GRF = 1:nContacts
        fprintf(fid, '\tres[0][NU + %i] = value<T>(P_HC_y_%s);\n', count_acc, num2str(i_GRF-1));
        tmp = outputCount + count_acc;
        IO_P_HC_ys.(['contact_sphere_', num2str(i_GRF-1)]) = tmp;
        count_acc = count_acc + 1;
    end
    IO_indices.P_contact_deformation_y = IO_P_HC_ys;
end

fprintf(fid, '\n');
fprintf(fid, '\treturn 0;\n');
fprintf(fid, '}\n\n');

fprintf(fid, 'int main() {\n');
fprintf(fid, '\tRecorder x[NX];\n');
fprintf(fid, '\tRecorder u[NU];\n');
fprintf(fid, '\tRecorder tau[NR];\n');
fprintf(fid, '\tfor (int i = 0; i < NX; ++i) x[i] <<= 0;\n');
fprintf(fid, '\tfor (int i = 0; i < NU; ++i) u[i] <<= 0;\n');
fprintf(fid, '\tconst Recorder* Recorder_arg[n_in] = { x,u };\n');
fprintf(fid, '\tRecorder* Recorder_res[n_out] = { tau };\n');
fprintf(fid, '\tF_generic<Recorder>(Recorder_arg, Recorder_res);\n');
fprintf(fid, '\tdouble res[NR];\n');
fprintf(fid, '\tfor (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];\n');
fprintf(fid, '\tRecorder::stop_recording();\n');
fprintf(fid, '\treturn 0;\n');
fprintf(fid, '}\n');

fclose(fid);
end

% Generate c-code with external function (and its Jacobian).
function [] = generateF(dim)
import casadi.*
cg = CodeGenerator('foo_jac');
arg = SX.sym('arg', dim);
[y, ~, ~] = foo(arg);
F = Function('F', {arg}, {y});
cg.add(F);
cg.add(F.jacobian());
cg.generate();

end

function [varargout] = buildExternalFunction(filename, CPP_DIR, nInputs, compiler)

% Set default value for compiler argument
if nargin < 4
    compiler = 'Visual Studio 15 2017 Win64';
end

% Part 1: build expression graph (i.e., generate foo.py).
[pathMain,~,~] = fileparts(mfilename('fullpath'));
pathBuildExpressionGraph = fullfile(pathMain, 'buildExpressionGraph');
pathBuild = fullfile(pathMain, 'buildExpressionGraph', filename);
mkdir(pathBuild);

OpenSimAD_DIR = fullfile(pathMain, 'OpenSimAD-install');
SDK_DIR = fullfile(OpenSimAD_DIR, 'sdk');
BIN_DIR = fullfile(OpenSimAD_DIR, 'bin');

cd(pathBuild);
cmd1 = ['cmake "', pathBuildExpressionGraph, '" -G "', compiler, '" -DTARGET_NAME:STRING="', filename, '" -DSDK_DIR:PATH="', SDK_DIR, '" -DCPP_DIR:PATH="', CPP_DIR, '"'];
system(cmd1);
cmd2 = 'cmake --build . --config RelWithDebInfo';
system(cmd2);

cd(BIN_DIR);
path_EXE = fullfile(pathBuild, 'RelWithDebInfo', [filename '.exe']);
system(path_EXE);

% Part 2: build external function (i.e., build .dll).
fooName = 'foo.py';
pathBuildExternalFunction = fullfile(pathMain, 'buildExternalFunction');
path_external_filename_foo = fullfile(BIN_DIR, fooName);
path_external_functions_filename_build = fullfile(pathMain, 'buildExternalFunction', filename);
path_external_functions_filename_install = fullfile(pathMain, 'installExternalFunction', filename);
mkdir(path_external_functions_filename_build);
mkdir(path_external_functions_filename_install);
copyfile(path_external_filename_foo, pathBuildExternalFunction);

addpath(pathBuildExternalFunction);
cd(pathBuildExternalFunction);

if nargout == 1
    varargout{1} = pathBuildExternalFunction;
    return
end

generateF(nInputs);

cd(path_external_functions_filename_build);
cmd1 = ['cmake "', pathBuildExternalFunction, '" -G "', compiler, '" -DTARGET_NAME:STRING="', filename, '" -DINSTALL_DIR:PATH="', path_external_functions_filename_install, '"'];
system(cmd1);
cmd2 = 'cmake --build . --config RelWithDebInfo --target install';
system(cmd2);
cd(pathMain);

copyfile(fullfile(path_external_functions_filename_install, 'bin', [filename '.dll']), CPP_DIR);
delete(fullfile(pathBuildExternalFunction, 'foo_jac.c'));
delete(fullfile(pathBuildExternalFunction, fooName));
delete(path_external_filename_foo);
rmdir(pathBuild, 's');
rmdir(path_external_functions_filename_install, 's');
rmdir(path_external_functions_filename_build, 's');

end

% From .sto file to numpy array.
function data = storage2numpy(storage_file, excess_header_entries)
    if nargin < 2
        excess_header_entries = 0;
    end
    
    % What's the line number of the line containing 'endheader'?
    fid = fopen(storage_file, 'r');
    line_number_of_line_containing_endheader = -1;
    header_line = false;
    while ~feof(fid) && line_number_of_line_containing_endheader == -1
        line = fgetl(fid);
        if header_line
            column_names = strsplit(line);
            break;
        end
        if contains(line, 'endheader')
            line_number_of_line_containing_endheader = ftell(fid);
            header_line = true;
        end
    end
    fclose(fid);
    
    % With this information, go get the data.
    if excess_header_entries == 0
        skip_header = line_number_of_line_containing_endheader;
        names = true;
    else
        names = column_names(1:end-excess_header_entries);
        skip_header = line_number_of_line_containing_endheader + 1;
    end
    data = importdata(storage_file, ' ', skip_header);
    if isstruct(data)
        data = data.data;
    end
    if iscell(names)
        for i = 1:numel(names)
            data = setfield(data, names{i}, data(:, i+1));
        end
        data = rmfield(data, 'colheaders');
    end
end

% From .sto file to DataFrame.
function df = storage2df(storage_file, headers)
    % Extract data
    data = storage2numpy(storage_file);
    
    % Convert to table
    df = table(data(:, 1), 'VariableNames', {'time'});
    for i = 1:numel(headers)
        df.(headers{i}) = data(:, i+1);
    end
end

function numpy2storage(labels, data, storage_file)
    assert(size(data, 2) == length(labels), "# labels doesn't match columns");
    assert(strcmp(labels(1), "time"));

    fid = fopen(storage_file, 'w');
    fprintf(fid, 'name %s\n', storage_file);
    fprintf(fid, 'version=1\n');
    fprintf(fid, 'datacolumns %d\n', size(data, 2));
    fprintf(fid, 'datarows %d\n', size(data, 1));
    fprintf(fid, 'range %f %f\n', min(data(:, 1)), max(data(:, 1)));
    fprintf(fid, 'endheader \n');

    for i = 1:length(labels)
        fprintf(fid, '%s\t', labels{i});
    end
    fprintf(fid, '\n');

    for i = 1:size(data, 1)
        for j = 1:size(data, 2)
            fprintf(fid, '%.2f\t', data(i, j));
        end
        fprintf(fid, '\n');
    end

    fclose(fid);
end