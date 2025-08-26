import ctypes, os, sys

# Load library
lib = "./build/libquaternion_shared.so"
if not os.path.exists(lib):
    lib = "../build/libquaternion_shared.so"
qlib = ctypes.CDLL(lib)

class Quaternion(ctypes.Structure):
    _fields_ = [("w", ctypes.c_float),
                ("x", ctypes.c_float),
                ("y", ctypes.c_float),
                ("z", ctypes.c_float)]
    def __init__(self, w=1, x=0, y=0, z=0):
        super().__init__(w, x, y, z)
    def __repr__(self):
        return f"Q({self.w:.3f},{self.x:.3f},{self.y:.3f},{self.z:.3f})"

# Bind C functions
qlib.cf_update_from_gyro.argtypes = [ctypes.POINTER(Quaternion),
                                     ctypes.POINTER(ctypes.c_float),
                                     ctypes.c_float]
qlib.cf_update_from_accel.argtypes = [ctypes.POINTER(Quaternion),
                                      ctypes.POINTER(ctypes.c_float)]

qlib.cf_update_from_gyro.restype = None
qlib.cf_update_from_accel.restype = None

