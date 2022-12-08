import omni
from omni.kit.viewport.utility import get_active_viewport
from omni.ui import scene as sc
import omni.usd
import carb
from pxr import Usd, UsdLux, UsdGeom, UsdShade, Sdf, Gf, UsdPhysics
import numpy as np

PREFERED_X_OFFSET = 4000.0
PREFERED_Y_OFFSET = 1500.0
PREFERED_Z_OFFSET = 3000.0
PREFERED_OFFSET = Gf.Vec3d( PREFERED_X_OFFSET, PREFERED_Y_OFFSET, PREFERED_Z_OFFSET) 

START_X_OFFSET = 1600.0
START_Y_OFFSET = 350.0
START_Z_OFFSET = 500.0

START_OFFSET = Gf.Vec3d( START_X_OFFSET, START_Y_OFFSET, START_Z_OFFSET) 


MID_RAMPS_Z = 3500.0
MID_RAMPS_Y_ADDED_OFFSET = 800
MID_RAMPS_X_ADDED_OFFSET = 800
# VEL_X_MAX = 500.0
# VEL_Y_MAX 

CAM_VEL = 500.0
CAM_LERP_STRENGTH = 0.7275

class JumperCam():
   
    def __init__(self):
        
        self.camera_prim = None
        self.camera_target_pos = Gf.Vec3d()
        self.car_started_moving = False

    def setup_camera(self, **kwargs): 
        
        expected = ['camera_prim_path', "vehicle_prim", 'stage' ] 
        failed = False
        for arg_key in expected:
            if arg_key in kwargs:
                self.__dict__[arg_key] = kwargs[arg_key]
            else:
                failed = True
                print(f"setup_camera: Missing kwarg: {arg_key}")
                
        if failed:
            print("JumperCam: not enough args to setup")
            return

        if self.stage is not None:
            self.camera_prim = self.stage.GetPrimAtPath(self.camera_prim_path)
            
            
        viewport = get_active_viewport()
        if viewport:
            viewport.camera_path = self.camera_prim_path    
        
        self.car_started_moving = False
        
        self.set_initial_position()
        self.set_view()
        
    def set_initial_position(self):
        
        vehicle_mat = UsdGeom.Xformable(self.vehicle_prim).GetLocalTransformation()
        vehicle_pos = vehicle_mat.ExtractTranslation()
 
        # camera_pos = Gf.Vec3d(  vehicle_pos[0] + START_X_OFFSET,
        #                         vehicle_pos[1] + START_Y_OFFSET,
        #                         vehicle_pos[2] + START_Z_OFFSET) 
        self.camera_target_pos = Gf.Vec3d(vehicle_pos + START_OFFSET) 
        
     
        
    def set_view(self):
         
        # Snap to target for now...

        
        
        
        vehicle_mat = UsdGeom.Xformable(self.vehicle_prim).GetLocalTransformation()
        vehicle_pos = vehicle_mat.ExtractTranslation()
 
        camera_pos = self.camera_target_pos 
        
        new_cam_mat = Gf.Matrix4d(1.0)
        new_cam_mat.SetLookAt(camera_pos, vehicle_pos, Gf.Vec3d(0,1,0))
        destXformAttr = self.camera_prim.GetAttribute('xformOp:transform')
        destXformAttr.Set(new_cam_mat.GetInverse())
                        
    
    def update_camera(self, e: carb.events.IEvent):
        if self.car_started_moving == True:
            vehicle_mat = UsdGeom.Xformable(self.vehicle_prim).GetLocalTransformation()
            vehicle_pos = vehicle_mat.ExtractTranslation() 
            new_pos = Gf.Vec3d(vehicle_pos + PREFERED_OFFSET) 
            
            if new_pos[2] > MID_RAMPS_Z:
                new_pos[2] = MID_RAMPS_Z
                new_pos[0] += MID_RAMPS_X_ADDED_OFFSET
                new_pos[1] += MID_RAMPS_Y_ADDED_OFFSET                     

            dt = e.payload["dt"]
            exp = 1.0 - (CAM_LERP_STRENGTH ** dt)
            amt = Gf.Lerp(exp, 0, 1.0)
            to_vect = new_pos - self.camera_target_pos
            self.camera_target_pos += to_vect * amt

        # else # assume we're still at start pos        
        self.set_view()
 