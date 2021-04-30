import numpy
import ctypes

name = "FORCESNLPsolver"
requires_callback = True
lib = "lib/libFORCESNLPsolver.so"
lib_static = "lib/libFORCESNLPsolver.a"
c_header = "include/FORCESNLPsolver.h"

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (110,   1),  110),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  7,   1),    7),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, ( 60,   1),   60)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11)]

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