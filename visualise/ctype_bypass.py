# quaternion_ctypes.py
import ctypes
import sys
import os

class Quaternion(ctypes.Structure):
    """Python mirror of your C quaternion struct"""
    _fields_ = [
        ("w", ctypes.c_float),
        ("x", ctypes.c_float), 
        ("y", ctypes.c_float),
        ("z", ctypes.c_float)
    ]
    
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        super().__init__()
        self.w = w
        self.x = x
        self.y = y
        self.z = z
    
    def __repr__(self):
        return f"Q(w={self.w:.3f}, x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"
qlib = ctypes.CDLL("./build/libquaternion_shared.so")


# Your C function: void updateQuaternionFromRate(struct quaternion* q, float rate[3], float dt)
qlib.updateQuaternionFromRate.argtypes = [
    ctypes.POINTER(Quaternion),           # struct quaternion*
    ctypes.POINTER(ctypes.c_float),       # float rate 
    ctypes.c_float                        # float dt
]
qlib.updateQuaternionFromRate.restype = None  # void

# Your C function: void updateQuaternionFromAcc(struct quaternion* q, float acc)
qlib.updateQuaternionFromAcc.argtypes = [
    ctypes.POINTER(Quaternion),           # struct quaternion*
    ctypes.POINTER(ctypes.c_float)        # float acc
]
qlib.updateQuaternionFromAcc.restype = None

# Your C function: float norm(const struct quaternion* q)
qlib.norm.argtypes = [ctypes.POINTER(Quaternion)]
qlib.norm.restype = ctypes.c_float

def test_quaternion_functions():
    """Test your C quaternion functions from Python"""
    
    # Create a quaternion (identity)
    q = Quaternion(1.0, 0.0, 0.0, 0.0)
    print(f"Initial: {q}")
    
    # Test norm function
    n = qlib.norm(ctypes.byref(q))
    print(f"Norm: {n} (should be ~1.0)")
    
    # Test gyro update
    gyro_rates = (ctypes.c_float * 3)(0.1, 0.2, 0.3)  # rad/s
    qlib.updateQuaternionFromRate(ctypes.byref(q), gyro_rates, 0.01)
    print(f"After gyro: {q}")
    
    # Test accel update  
    accel_data = (ctypes.c_float * 3)(0.0, 0.0, 9.807)  # m/s²
    qlib.updateQuaternionFromAcc(ctypes.byref(q), accel_data)
    print(f"After accel: {q}")
    
    # Check norm again
    n = qlib.norm(ctypes.byref(q))
    print(f"Final norm: {n}")

if __name__ == "__main__":
    test_quaternion_functions()


