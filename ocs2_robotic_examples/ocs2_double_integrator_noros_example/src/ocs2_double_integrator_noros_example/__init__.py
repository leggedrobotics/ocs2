import sys

if sys.version_info > (3,):
  from DoubleIntegratorPyBindings3 import mpc_interface
else:
  from DoubleIntegratorPyBindings2 import mpc_interface
del sys

