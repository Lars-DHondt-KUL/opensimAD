
import casadi as ca
import importlib
import sys
import os

dim = int(sys.argv[2])
PathFoo = sys.argv[1]
print(dim)
os.chdir(PathFoo)
sys.path.append(PathFoo)
import foo
importlib.reload(foo)
cg = ca.CodeGenerator('foo_jac')
arg = ca.SX.sym('arg', dim)
y, _, _ = foo.foo(arg)
F = ca.Function('F', [arg], [y])
cg.add(F)
cg.add(F.jacobian())
cg.generate()