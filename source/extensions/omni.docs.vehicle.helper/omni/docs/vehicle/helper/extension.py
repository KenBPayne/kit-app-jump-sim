import omni.ext
import omni.usd
import omni.ui as ui
import omni.kit.app
import carb
import carb.settings
import carb.dictionary
from pxr import UsdGeom 
from functools import partial
from .vehicle_definition import *
 
# Vehicle definition prim name starts with prefix "veh_"
VEHICLE_DEF_PREFIX = "veh_"
       

# give parent ext's ability to tell us
# to reload vehicles or run without UI
_extension_instance = None
def get_instance():
    return _extension_instance
 
class WheelRefTestExtension(omni.ext.IExt):
 
    def on_startup(self, ext_id):
        print("[omni.docs.vehicle.helper] vehicle helper startup")
        
        # attempt to load any vehicles if there's an existing stage
        self.force_load = True
        
        # assume ui unless parent signals to suppress 
        self.headless = False
        self.headless_signal_timeout = 1.0
        
        global _extension_instance
        _extension_instance = self        
        
        self.vehicle_list = []
        self.stage_dirty = False
        self.stage_event_sub = None
        
        usd_context = omni.usd.get_context()
        events = usd_context.get_stage_event_stream()
        self.stage_event_sub = events.create_subscription_to_pop(self.on_stage_event,name='stage_update')         
  
        self.load_stage_and_run()
                       
                            
    def load_stage_and_run(self):
        self.find_vehicles()                    
        self.start_sim()
   
    def find_vehicles(self):
        usd_context = omni.usd.get_context()
        my_stage = usd_context.get_stage()  
        if my_stage is None:
            return
        
        found_veh_prims = [x for x in my_stage.Traverse() if x.GetName().startswith(VEHICLE_DEF_PREFIX)]
        self.vehicle_list = []
        for veh_prim in found_veh_prims:
            if veh_prim:
                print(f"found vehicle def: {veh_prim.GetPath()}")             
                new_veh = VehicleDefinition()
                if new_veh.load_from_definition_prim(my_stage, veh_prim):
                    self.vehicle_list.append(new_veh)
                        
        
    def start_sim(self):
        update_event_stream = omni.kit.app.get_app().get_update_event_stream()       
        self._pop_event_steam_sub_id = update_event_stream.create_subscription_to_pop(self.tick_vehicle_list,name="tickupdate")  
        
     
    def stop_test(self):
        self.vehicle_list = []
        self._pop_event_steam_sub_id = None
        
    def on_stage_event(self, e: carb.events.IEvent):
        if self.headless:
            etype:omni.usd.StageEventType = e.type
                
                #print(f"on_stage_event: etype={string(etype)}")
            if etype == int(omni.usd.StageEventType.OPENED):
                #self.load_stage_and_run()
                print(f"on_stage_event: etype=OPENED")
                self.force_load = True
            
            elif e.type == int(omni.usd.StageEventType.CLOSING):
                print(f"on_stage_event: etype=CLOSING")
                self.stop_test()
            elif e.type == int(omni.usd.StageEventType.CLOSED):
                print(f"on_stage_event: etype=CLOSED")
                self.stop_test()
            elif e.type == int(omni.usd.StageEventType.DIRTY_STATE_CHANGED):
                print(f"on_stage_event: etype=DIRTY_STATE_CHANGED")
                self.force_load = True
            elif e.type == int(omni.usd.StageEventType.ASSETS_LOADED):
                print(f"on_stage_event: etype=ASSETS_LOADED")
                self.force_load = True
                self.start_sim()
                # update vehicle definitions (resave prim paths?)
                
            #CLOSED: kill vehicle list?
            
            #DIRTY_STATE_CHANGED: set flag to save on close?

            # wait for this to load, instead of OPENED?
            # elif e.type == int(omni.usd.StageEventType.ASSETS_LOADED):
            #     pass


            
    ####################### UI ###########################
            
    def make_ui(self):
        
        self.new_vehicle_axles_num_model = None
        self.chassis_mesh_path_model = None
        self.physx_vehicle_path_model = None
        self.def_prim_path_model = None  
        self.wheel_path_models = None   

        self._window = ui.Window("Vehicle Helper", width=500, height=500)
        with self._window.frame:
            with ui.VStack():
                with ui.HStack():#height=50):
                    ui.Label("Axles", height=25)
                    self.new_vehicle_axles_num_model = ui.SimpleIntModel()
                    self.new_vehicle_axles_num_model.as_int = 2
                    ui.IntField(model=self.new_vehicle_axles_num_model, height=25, width=30) 
                    ui.Separator()
                    ui.Button("New vehicle definition", clicked_fn=self.on_click_new_vehicle_def, height=25)   
                ui.Separator(height=25)
                ui.Separator(height=10)
                with ui.HStack():
                    ui.Button("Load+Run", clicked_fn=self.load_stage_and_run)
                    ui.Button("Start", clicked_fn=self.start_sim)
                    ui.Button("Reset", clicked_fn=self.stop_test)
                    ui.Button("find veh prims", clicked_fn=self.find_vehicles)
    
                    
    # create new vehicle definition window                
    def on_click_new_vehicle_def(self):
        win_height = 200 + 50 + (80 * self.new_vehicle_axles_num_model.as_int)
        self.new_vehicle_window = ui.Window("New Vehicle", width=700, height=win_height)
        with self.new_vehicle_window.frame:
            with ui.VStack():
                wheel_num = 0
                self.wheel_path_models = {}
                for axle_num in range(1, self.new_vehicle_axles_num_model.as_int + 1):
                    for side in ['Left', 'Right']:
                        with ui.HStack():
                            wh_label = f"axle {axle_num} : " + str(side)
                            ui.Label(wh_label, width=80)
                            str_model = ui.SimpleStringModel()
                            self.wheel_path_models[wheel_num] = str_model 
                            ui.Button("Select", clicked_fn=partial(self.on_click_mesh_wheel, wheel_num,str_model), height=25, width=20) 
                            ui.StringField(model=str_model, height=25)
                            wheel_num += 1
                            
                    ui.Separator(height=10)
                ui.Separator(height=10)
                
                with ui.HStack():
                    str_model = ui.SimpleStringModel()
                    self.chassis_mesh_path_model = str_model                    
                    ui.Button("Select Chassis Mesh", clicked_fn=partial(self.on_click_selected_prim, str_model), height=25, width=120) 
                    ui.StringField(model=str_model, height=25)
                
                with ui.HStack():
                    str_model = ui.SimpleStringModel()
                    self.physx_vehicle_path_model = str_model
                    ui.Button("Select PhysX Vehicle", clicked_fn=partial(self.on_click_selected_prim, str_model), height=25, width=120)  
                    ui.StringField(model=str_model, height=25)
                
                with ui.HStack():
                    str_model = ui.SimpleStringModel()
                    self.def_prim_path_model = str_model                    
                    ui.Button("Select Vehicle definition prim", clicked_fn=partial(self.on_click_selected_prim, str_model), height=25, width=120)            
                    ui.StringField(model=str_model, height=25)
                 
                with ui.HStack():
                    ui.Separator(width=150, height=50)
                    ui.Button("Done", clicked_fn=self.on_click_vehicle_done, height=40, width=70, alignment=ui.Alignment.CENTER )   
                    ui.Separator(width=150)
            
            
    def on_click_mesh_wheel(self, wheel_num, string_model):
        print(f"on_click_mesh_wheel: {wheel_num}")
        path = self.get_selected_prim_path()
        string_model.as_string = path

    def on_click_selected_prim(self, string_model):
        print(f"on_click_chassis_mesh")
        path = self.get_selected_prim_path()
        string_model.as_string = path
        
    def on_click_vehicle_done(self):
        wheel_count = self.new_vehicle_axles_num_model.as_int * 2
        wheel_paths_list = []
        for wheel_num in range(wheel_count):
            print(f"wheel num: {wheel_num}")
            print(f"{self.wheel_path_models[wheel_num].as_string }")
            wheel_paths_list.append(self.wheel_path_models[wheel_num].as_string)
        
        new_veh = VehicleDefinition() 

        veh_def_prim_path = self.def_prim_path_model.as_string
        physx_veh_path = self.physx_vehicle_path_model.as_string
        mesh_chassis_path = self.chassis_mesh_path_model.as_string 
        
        args = {    'veh_def_prim_path' : veh_def_prim_path,
                    'physx_veh_path' : physx_veh_path,
                    'mesh_chassis_path' : mesh_chassis_path,
                    'mesh_wheel_paths' : wheel_paths_list }
        
        new_veh.create_definition_from_paths(**args)
 

    def get_selected_prim_path(self):
        usd_context = omni.usd.get_context()
        selection = usd_context.get_selection()
        paths = selection.get_selected_prim_paths()
        if paths:
            return paths[0]

    def on_shutdown(self):
        global _extension_instance
        _extension_instance = None
        print("[omni.docs.vehicle.helper] shutdown")

     
    def tick_vehicle_list(self, e: carb.events.IEvent): 
            
        if self.force_load:
            self.force_load = False
            print(f"**** FORCE LOAD ****")
            self.find_vehicles()
        
        if self.headless_signal_timeout > 0:
            self.headless_signal_timeout -= e.payload["dt"]
            if self.headless_signal_timeout <= 0:
                if self.headless == False:
                    # then we never got signal from parent to run headless
                    self.make_ui()
        
        for veh in self.vehicle_list:
            for source_prim in veh.wheel_pairs:
                if source_prim and source_prim.IsValid():
                    dest_prim = veh.wheel_pairs[source_prim]
                    if dest_prim and dest_prim.IsValid():
                        srcXform = UsdGeom.Xformable(source_prim) 
                        mtrx_world = srcXform.ComputeLocalToWorldTransform(0)
                        destXformAttr = dest_prim.GetAttribute('xformOp:transform')
                        destXformAttr.Set(mtrx_world)
            
            srcXform = UsdGeom.Xformable(veh.physx_veh_prim)
            if srcXform: #.IsValid():
                mtrx_world = srcXform.ComputeLocalToWorldTransform(0)
                destXformAttr = veh.mesh_chassis_prim.GetAttribute('xformOp:transform')
                destXformAttr.Set(mtrx_world)