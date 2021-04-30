import numpy
import ctypes

name = "drone_mpc_compiled"
requires_callback = True
lib = "lib/libdrone_mpc_compiled.so"
lib_static = "lib/libdrone_mpc_compiled.a"
c_header = "include/drone_mpc_compiled.h"

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, ( 55,   1),   55),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  7,   1),    7),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, ( 30,   1),   30)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x1"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x2"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x3"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x4"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x5"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
('it2opt', ctypes.c_int),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('rsnorm', ctypes.c_double),
('rcompnorm', ctypes.c_double),
('pobj', ctypes.c_double),
('dobj', ctypes.c_double),
('dgap', ctypes.c_double),
('rdgap', ctypes.c_double),
('mu', ctypes.c_double),
('mu_aff', ctypes.c_double),
('sigma', ctypes.c_double),
('lsit_aff', ctypes.c_int),
('lsit_cc', ctypes.c_int),
('step_aff', ctypes.c_double),
('step_cc', ctypes.c_double),
('solvetime', ctypes.c_double),
('fevalstime', ctypes.c_double)
]