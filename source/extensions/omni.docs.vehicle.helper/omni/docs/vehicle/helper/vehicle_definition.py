import omni
from pxr import UsdGeom, Sdf

 
VEHICLE_DEF_ATTRIBUTE = "vehicle_helper"  
VEH_PHYS_VEH_ATTR = VEHICLE_DEF_ATTRIBUTE + ":physx_veh"
VEH_MESH_CHASSIS_ATTR = VEHICLE_DEF_ATTRIBUTE + ":mesh_chassis"
VEH_MESH_WHEELS_ATTR = VEHICLE_DEF_ATTRIBUTE + ":mesh_wheels"

class VehicleDefinition():
    def __init__(self):
        self.definition_prim = None
        self.physx_veh_prim = None
        self.mesh_chassis_prim = None
        self.wheel_pairs = {}
   
    def load_from_definition_prim(self, stage, definition_prim) -> bool:
        #self.stage = stage
        self.definition_prim = definition_prim
        try:
            physx_veh_path = definition_prim.GetAttribute(VEH_PHYS_VEH_ATTR).Get()
            mesh_chassis_path = definition_prim.GetAttribute(VEH_MESH_CHASSIS_ATTR).Get()
            mesh_wheels_paths = definition_prim.GetAttribute(VEH_MESH_WHEELS_ATTR).Get()
            
            # relative_path = PathUtils.compute_relative_path(definition_prim.GetPath(),physx_veh_path)
            pass
        except Exception as inst:
            print(f"{self} load_from_definition_prim: Exception {inst}")
            return False
        

        args = {    'physx_veh_path' : physx_veh_path,
                    'mesh_chassis_path' : mesh_chassis_path,
                    'mesh_wheel_paths' : mesh_wheels_paths }
        
        return self.setup_vehicle(stage, **args)
        
    def setup_vehicle(self, stage, **kwargs) -> bool:
        kwarg_keys = [  'physx_veh_path',
                        'mesh_chassis_path',
                        'mesh_wheel_paths' ] #,
                        #'stage' ] 
        failed = False
        for arg_key in kwarg_keys:
            if arg_key in kwargs:
                self.__dict__[arg_key] = kwargs[arg_key]
            else:
                failed = True
                print(f"VehicleDef __init__: Missing kwarg: {arg_key}")
                
        if failed:
            return False
    
        try:
            self.physx_veh_prim = stage.GetPrimAtPath(self.physx_veh_path)             
            self.mesh_chassis_prim = stage.GetPrimAtPath(self.mesh_chassis_path)      
        
            is_left = True
            axle_num = 1
            self.wheel_pairs = {}       
            for mw in self.mesh_wheel_paths:
                wheel_ref_name = "/LeftWheel" if is_left else "/RightWheel"
                wheel_ref_name = wheel_ref_name + str(axle_num) + "References"     
                source_path = self.physx_veh_path + wheel_ref_name
                source_prim = stage.GetPrimAtPath(source_path) 
                if source_prim is None:
                    print(f"Can't find physX ref wheel: {source_path}")
                    return None  
                                
                dest_prim = stage.GetPrimAtPath(mw)
                if dest_prim is None:
                    print(f"Can't find Mesh wheel: {dest_prim}")
                    return None 
                
                print(f"source_path = {source_path}")
                print(f"dest_path = {mw}")
                self.wheel_pairs[source_prim] = dest_prim
            
                # cue up next wheel
                is_left = not is_left
                if is_left:
                    axle_num += 1
                    
        except Exception as inst:
            print(f"{self} setup_vehicle: Exception {inst}")
            return False
        
        return True
    
    def make_physx_wheel_ref_list(self, phys_veh_path, wheel_count):
        is_left = True
        axle_num = 1
        wheel_list = [] 
        for wr_num in range(wheel_count):
            wheel_ref_name = "/LeftWheel" if is_left else "/RightWheel"
            wheel_ref_name = wheel_ref_name + str(axle_num) + "References"     
            ref_path = phys_veh_path + wheel_ref_name
            wheel_list.append(ref_path)
            # cue up next wheel
            is_left = not is_left
            if is_left:
                axle_num += 1
                
        return wheel_list
    
    def ensure_prim_has_xform_op(self, prim_path) -> bool:
        try:
            usd_context = omni.usd.get_context()
            my_stage = usd_context.get_stage()
            prim = my_stage.GetPrimAtPath(prim_path)
            xform_op = prim.GetAttribute('xformOp:transform')
            if xform_op is None or not xform_op.IsValid():
                xform_op = UsdGeom.Xformable(prim) 
                new_op = xform_op.AddTransformOp()
                if new_op is None:
                    return False
        except Exception:
            print(f"ensure_prim_has_xform_op : {Exception}")
            return False
                   
        return True
    
    def set_prim_property(self, prim_path, attr_name, prop_type, prop_value):
        try:
            usd_context = omni.usd.get_context()
            my_stage = usd_context.get_stage()
            prim = my_stage.GetPrimAtPath(prim_path)
            prop = prim.GetAttribute(attr_name)
            if prop:
                prop.Set(prop_value)
            else:
                prim.CreateAttribute(attr_name, prop_type, custom=True).Set(prop_value)
                
        except Exception as inst:
            print(f"set_prim_property: {inst}")
            return False
                
        return True            
        
    
    def create_definition_from_paths(self, **kwargs) -> bool:
        expected = [    'veh_def_prim_path', 'physx_veh_path',
                        'mesh_chassis_path', 'mesh_wheel_paths'] 
        
        for key in expected:
            if not key in kwargs:
                print(f"create_definition_from_paths missing: {key}")
                return False
        
        veh_def_prim_path = kwargs['veh_def_prim_path']
        physx_veh_path = kwargs['physx_veh_path']
        mesh_chassis_path = kwargs['mesh_chassis_path']
        mesh_wheel_paths = kwargs['mesh_wheel_paths'] 
        
        if not self.ensure_prim_has_xform_op(physx_veh_path):
            return False
        
        if not self.ensure_prim_has_xform_op(mesh_chassis_path):
            return False        
                
        wheel_ref_list = self.make_physx_wheel_ref_list(physx_veh_path, len(mesh_wheel_paths))
        for wheel_ref_path in wheel_ref_list:
            if not self.ensure_prim_has_xform_op(wheel_ref_path):
                return False
            
        for wheel_mesh in mesh_wheel_paths:
            if not self.ensure_prim_has_xform_op(wheel_mesh):
                return False            
            
        try:
            
            if not self.set_prim_property(  veh_def_prim_path,
                                            VEH_PHYS_VEH_ATTR,
                                            Sdf.ValueTypeNames.String,
                                            physx_veh_path):
                return False
            
            if not self.set_prim_property(  veh_def_prim_path,
                                            VEH_MESH_CHASSIS_ATTR,
                                            Sdf.ValueTypeNames.String,
                                            mesh_chassis_path):
                return False
            
            if not self.set_prim_property(  veh_def_prim_path,
                                            VEH_MESH_WHEELS_ATTR,
                                            Sdf.ValueTypeNames.StringArray,
                                            mesh_wheel_paths):
                return False
            
        
            self.veh_def_prim = stage.GetPrimAtPath(veh_def_prim_path)             

        except Exception as inst:
            print(f"create_definition_from_paths: {inst}")
            return False
        
        return True
                    