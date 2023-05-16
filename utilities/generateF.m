function [] = generateF(nInputs, fooPath, secondOrderDerivatives)
% --------------------------------------------------------------------------
% generateF
%   Generates an expression graph of the function and its derivative
%   (foo_jac.c) based on the expression graph (foo.py).
%
%
% INPUT:
%   - nInputs -
%   * number of input arguments for the external function [double]
%
%   - fooPath -
%   * path to foo.py [char]
%
%   - secondOrderDerivatives -
%   * do you want to calculate 2nd derivatives of external function outputs 
%   w.r.t. inputs? [bool]
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

% import casadi.*
% cg = CodeGenerator('foo_jac');
% arg = SX.sym('arg', dim);
% [y, ~, ~] = foo(arg);
% F = Function('F', {arg}, {y});
% cg.add(F);
% cg.add(F.jacobian());
% % % Generate also forward, reverse, and forward-over-reverse to use a exact Hessian
% % Fr = F.reverse(1);
% % cg.add(Fr);
% % for i=0:6
% %     cg.add(F.forward(2^i));
% %     cg.add(Fr.forward(2^i));
% % end
% cg.generate();


% Evaluating foo.py with SX symbolics does not work from matlab. Need to
% change recorder to create foo.m instead. For now, we use the compiled
% version of genF.py.

[pathUtilities,~,~] = fileparts(mfilename('fullpath'));
cd(fullfile(pathUtilities,'genF'))

% Make sure that booleans is compatible with python
if secondOrderDerivatives
    secondOrderDerivatives = 'True';
else
    secondOrderDerivatives = 'False';
end

command = ['genF.exe "' fooPath '" ' num2str(nInputs) ' "' secondOrderDerivatives '"'];
system(command);

cd ..


end