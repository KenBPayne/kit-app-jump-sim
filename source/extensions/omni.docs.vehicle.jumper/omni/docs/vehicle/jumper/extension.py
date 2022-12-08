import omni.ext
import omni.ui as ui
import omni.kit.commands
import omni.kit.app
import omni.usd
import omni.appwindow
import omni.timeline	
from pxr import Sdf, Usd
from omni.physx.scripts.physicsUtils import *
from omni.physx import get_physx_interface, get_physx_simulation_interface
from omni.physx.bindings._physx import SimulationEvent
from pxr import PhysxSchema

from .run_test_rounds import *
from .vehicle_audio import VehicleAudio

import omni.docs.vehicle.helper

contact_sensitivity = 2000.0
from carb import Float3 

MY_STAGE_NAME = "omniverse://content.ov.nvidia.com/Users/kpayne@nvidia.com/Tutorials/JumpTestKitApp/VehJumpTest.usd"
 
#VEHICLE_PRIM_PATH = "/World/dune_jumping_car_driveable/WizardVehicle1/Vehicle"
VEHICLE_PRIM_PATH = "/World/WizardVehicle1/Vehicle"

  
VEHICLE_COLLISION_PRIM = "/body_collision"
VEHICLE_COLLISION_PRIM_PATH = VEHICLE_PRIM_PATH + VEHICLE_COLLISION_PRIM
VEHICLE_ROOF_PRIM_PATH = VEHICLE_PRIM_PATH + "/Cube_roof"

 
#WHEEL_FRICTION_PRIM_PATH = "/World/dune_jumping_car_driveable/WizardSharedVehicleData/SummerTireFrictionTable"
WHEEL_FRICTION_PRIM_PATH = "/World/WizardSharedVehicleData/SummerTireFrictionTable"


WHEEL_LIST = [  "/LeftWheel1References",
                "/RightWheel1References",
                "/LeftWheel2References",
                "/RightWheel2References"]


ENGINE_TORQUE_ATTR_NAME = "physxVehicleEngine:peakTorque"
ENGINE_MAX_ROT_SPEED_ATTR_NAME = "physxVehicleEngine:maxRotationSpeed"

LANDING_RAMP_PRIM_PATH = "/World/ramp_01/ramp/ramp_surface"
END_GOAL_PRIM_PATH = "/World/goal_trigger"
LOWER_RAMP_PRIM_PATH = "/World/Cube_landing_blocked"

MATERIAL_SAFE_ROAD = "/World/PhysicsMaterial_safe_road"
MATERIAL_SAFE_RAMP = "/World/PhysicsMaterial_safe_ramp"
MATERIAL_OUT_OF_BOUNDS = "/World/PhysicsMaterial_out_of_bounds"

STAGE_START_TIME = 0.0
STAGE_END_TIME = 200.0

 
SOUND_VEHICLE_ENGINE_PATH = VEHICLE_PRIM_PATH + "/car_engine"
SOUND_ENGINE_REV_PATH = VEHICLE_PRIM_PATH + "/car_engine_rev"

SOUND_TIRE_SKID_PATH = VEHICLE_PRIM_PATH + "/tire_skid"
SOUND_SUSPENSION_SLAM_PATH = VEHICLE_PRIM_PATH + "/suspension_slam"

SOUND_WIN_PATH = "/World/Audio/good_bell"
SOUND_FAIL_PATH = "/World/Audio/buzzer"
SOUND_START_RACE_PATH = "/World/Audio/race_start"
SOUND_GOOD_TEST_PATH = "/World/Audio/good_test_over"
SOUND_BAD_TEST_PATH = "/World/Audio/bad_test_over"
 
SOUND_IMPACT_LARGE_PATH_1 = VEHICLE_PRIM_PATH + "/impacts/impact_large_1"
SOUND_IMPACT_LARGE_PATH_2 = VEHICLE_PRIM_PATH + "/impacts/impact_large_2"
SOUND_IMPACT_LARGE_PATH_3 = VEHICLE_PRIM_PATH + "/impacts/impact_large_3"

HEADLIGHTS = [  "/World/veh_jumper/car_jumper/headlight_glass/headlight_driver",
                "/World/veh_jumper/car_jumper/headlight_glass/headlight_pass_" ]


JUMPER_CAMERA_PATH = "/World/jumper_cam"

BUTTON_TEXT_START_TEST = "Start Test"
BUTTON_TEXT_STOP_TEST = "STOP Test"

LOAD_STAGE_FLASH_PERIOD = 0.5
TORQUE_SPREAD_DEFAULT = 30.0

class SimData():
   
    def __init__(self):
        # sim test params
        self.sim_torque_steps = None
        self.sim_min_torque = None
        self.sim_max_torque = None  
        # current round stats
        self.round_step_model = ui.SimpleIntModel()
        self.round_torque_model = ui.SimpleFloatModel()
        self.round_largest_susp_force_model = ui.SimpleFloatModel()
        self.round_largest_body_impulse_model = ui.SimpleFloatModel()
        # stats from best round
        self.best_torque_model = ui.SimpleFloatModel()
        self.best_landing_body_impulse_model = ui.SimpleFloatModel()
        self.best_landing_susp_force_model = ui.SimpleFloatModel()
        self.best_round_idx_model = ui.SimpleIntModel()        
        # stage prims
        self.test_stage = None
        self.vehicle_prim_path = VEHICLE_PRIM_PATH
        self.vehicle_collision_prim_path = VEHICLE_COLLISION_PRIM_PATH
        self.vehicle_roof_prim_path = VEHICLE_ROOF_PRIM_PATH
        self.wheel_list = WHEEL_LIST
        self.landing_ramp_prim_path = LANDING_RAMP_PRIM_PATH
        self.end_goal_prim_path = END_GOAL_PRIM_PATH        
        self.material_safe_ramp = MATERIAL_SAFE_RAMP
        self.material_out_of_bounds = MATERIAL_OUT_OF_BOUNDS
        self.engine_max_rotation_speed = 800.0
        self.wheel_friction_prim_path = WHEEL_FRICTION_PRIM_PATH
        self.vehicle_camera_path = JUMPER_CAMERA_PATH
        self.vehicle_headlights = HEADLIGHTS
        
        
class UI_Data():
   
    def __init__(self):     
        self.test_round_event_fn = None
        #self.end_of_round_report_fn = None 
        self.test_done_report_fn = None
        self.set_stage_params_fn = None
        self.engine_rpm_model = None
    

class MyExtension(omni.ext.IExt):
    
    def on_startup(self, ext_id):
        print("[omni.docs.vehicle.jumper] Vehicle Jump Test: Extension startup")

        self.sim_data = SimData() 
        self.ui_data = UI_Data()
        
        #self._test_running = False
        self._test_round = None
        self._stage_loaded = False
        
        wr_inst = omni.docs.vehicle.helper.get_instance()
        if wr_inst:
            # tell vehicle helper to run headless 
            wr_inst.headless = True
            # check for vehicles in case we just hot reloaded,
            # (normally from load stage)
            wr_inst.force_load = True    

        self._window = ui.Window("Vehicle Jump Test", width=600, height=1000)
        with self._window.frame:
            with ui.VStack():
                with ui.VStack(height=160):
                    self._stage_button = ui.Button("Load Stage", clicked_fn=self.on_click_load_stage, style={"color":0xFF00FFFF}) 

                    self._update_event_stream = omni.kit.app.get_app().get_update_event_stream()                
                    self._pop_event_stream_sub_id = self._update_event_stream.create_subscription_to_pop(self.tick_update_ui, name="tickupdate_ui")

                    self._stage_flash_time_remaining = LOAD_STAGE_FLASH_PERIOD
                    self._stage_flash_on = True
                    with ui.HStack():
                        self._start_button = ui.Button(BUTTON_TEXT_START_TEST, clicked_fn=self.on_click_start, visible=False) 
                        
                        self._next_round_button = ui.Button("Skip to next Round", clicked_fn=self.on_click_skip_round, visible=False) 
                        self._next_round_button.enabled = False

                ################################################################

                ui.Separator(height=10)
                self._label_test_results_report = ui.Label(f"-- waiting for test results --", height=40, alignment=ui.Alignment.CENTER_TOP, style={"font_size": 22}) 
                ui.Separator(height=10)
                
                ################################################################
 

                ui.Label(f"wheel_friction_prim_path", height=25)   
                self._wheel_friction_prim_path_value_model = ui.SimpleStringModel()
                ui.StringField(model=self._wheel_friction_prim_path_value_model, height=25)

     
                ui.Label(f"wheel_friction VALUE", height=25)    
                #self._wheel_friction_value_model = ui.SimpleStringModel()
                #ui.StringField(model=self._wheel_friction_value_model, height=25, enabled=True)

                self._wheel_friction_value_model = ui.SimpleFloatModel()
                ui.FloatField(model=self._wheel_friction_value_model, height=25, enabled=True)

                ui.Spacer(height=20)
  
                ui.Label(f"vehicle_prim_path", height=25)
                self._vehicle_prim_path_value_model = ui.SimpleStringModel()
                ui.StringField(model=self._vehicle_prim_path_value_model, height=25)

                ui.Spacer(height=20)

                ################## Torque #################
                with ui.VStack():     
                    with ui.HStack():
                        ui.Spacer(width=30) 
                        ui.Label("Original peakTorque from stage:", height=25,width=150, alignment=ui.Alignment.RIGHT) 
                        self._original_torque_value_model = ui.SimpleFloatModel()
                        ui.Spacer(width=10)
                        ui.FloatField(model=self._original_torque_value_model, height=25, width=130,alignment=ui.Alignment.LEFT)
                        ui.Spacer(width=10)
                        
                        self._set_torque_to_stage_button = ui.Button(
                            "Set torque to Stage", clicked_fn=self.on_click_set_torque_to_stage, style={"color":0xFF00FFFF},visible=True, height=25, width=180)
                        # ui.Spacer(width=50)
                        
                        
                    ui.Spacer(height=10)
                    ui.Separator(height=10)
                    ui.Label(f"Test Parameters", height=25,alignment=ui.Alignment.CENTER)   
                    ui.Spacer(height=20)       

                    with ui.HStack():
                        ui.Spacer(width=20)
                        with ui.VStack():
                            with ui.VStack():
                                ui.Label("Torque Steps (best if odd)", height=25) 
                                self._engine_torque_steps_model = ui.SimpleIntModel()
                                self._engine_torque_steps_model.as_int = 1    
                                self._t_field = ui.IntField(model=self._engine_torque_steps_model, height=25, width=130) 
                                
                            with ui.VStack():
                                ui.Label(f"+/- deviation %", height=25)
                                self._torque_pct_spread_value_model = ui.SimpleFloatModel()
                                self._torque_pct_spread_value_model.as_float = TORQUE_SPREAD_DEFAULT
                                self.val_spread_id = self._torque_pct_spread_value_model.subscribe_end_edit_fn(self.on_end_edit_sim_param)
                                ui.FloatField(model=self._torque_pct_spread_value_model, height=25, width=130)

                        with ui.VStack():
                            with ui.VStack():
                                ui.Label("Min Torque", height=25) 
                                self._engine_torque_min_model = ui.SimpleFloatModel()
                                ui.FloatField(model=self._engine_torque_min_model, height=25, width=130)

                            with ui.VStack():
                                ui.Label("Max Torque", height=25) 
                                self._engine_torque_max_model = ui.SimpleFloatModel()
                                ui.FloatField(model=self._engine_torque_max_model, height=25, width=130)

                        ui.Spacer(width=20)
 
                        self.val_changed_id = self._engine_torque_steps_model.subscribe_end_edit_fn(self.on_end_edit_sim_param)
        
                       
                    ui.Spacer(height=20)
                    ui.Separator(height=10)
                    ui.Label(f"Current Round Stats", height=25,alignment=ui.Alignment.CENTER)   
                    ui.Spacer(height=20)       
                    with ui.HStack():
                        with ui.VStack():
                            with ui.HStack():
                                ui.Label("Testing peakTorque", height=25, width=130)
                                ui.FloatField(model=self.sim_data.round_torque_model, height=25, width=100)

                            ui.Spacer(height=5)

                            with ui.HStack():
                                ui.Label("testing STEP:", height=25, width=130)
                                ui.IntField(model=self.sim_data.round_step_model, height=25, width=100)

                        with ui.VStack():
                            with ui.HStack():
                                ui.Label("Largest Body Impules", height=25, width=130)
                                ui.FloatField(model=self.sim_data.round_largest_body_impulse_model, height=25, width=100)

                            ui.Spacer(height=5)

                            with ui.HStack():
                                ui.Label("Largest Susp. Force", height=25, width=130) 
                                ui.FloatField(model=self.sim_data.round_largest_susp_force_model, height=25, width=100)

                            ui.Spacer(height=20)


                ###############################################################               
            
                ui.Spacer(width=30, height=20)
                ui.Separator(height=10)
                ui.Label(f"Best Test Stats", height=25,alignment=ui.Alignment.CENTER)   
                ui.Spacer(height=10)  
              
                with ui.VStack():
                    with ui.HStack():
                        with ui.VStack():
                            ui.Label("Best Torque", height=25) 
                            ui.FloatField(model=self.sim_data.best_torque_model, height=25)

                        ui.Spacer(width=10)
                        with ui.VStack():
                            ui.Label("Best Body impulse: ", height=25) 
                            
                            ui.FloatField(model=self.sim_data.best_landing_body_impulse_model, height=25)

                        ui.Spacer(width=10)
                        with ui.VStack():
                               
                            ui.Spacer(height=0, width=25)
                            self._best_torque_to_stage_button = ui.Button("Set best torque to Stage", clicked_fn=self.on_click_best_torque_to_stage, style={"color":0xFF00FFFF},visible=False,height=35, width=180)
                            ui.Spacer(height=20)
                            with ui.HStack():
                                
                                self._div_spread_cb = ui.CheckBox(value=True, width=25,alignment=ui.Alignment.RIGHT,visible=False)
                                self._div_spread_cb.model.set_value(True)
                                self._half_dev_label = ui.Label("Half Next Test Deviation", height=25,alignment=ui.Alignment.LEFT,visible=False)
                                
                    with ui.HStack():
                        with ui.VStack():
                            ui.Label("Best Susp. Force", height=25) 
                            ui.FloatField(model=self.sim_data.best_landing_susp_force_model, height=25)

                        ui.Spacer(width=10)
                        with ui.VStack():
                            ui.Label("Best Round", height=25)
                            ui.IntField(model=self.sim_data.best_round_idx_model, height=25)

                        ui.Spacer(width=10)
                        with ui.VStack():
                            pass
                        
 
                
    
    def on_click_load_stage(self):
        
        # stop sim if running, then reload
        if self._test_round:
            #if self._test_running == True:
            if self._test_running.test_running == True:
                    
                #self._test_round.finish_test()
                
                self._test_round.kill_subscriptions()
                #self._test_running = False
                self._test_round = None
                self._next_round_button.visible = False
            
        usd_context = omni.usd.get_context()
        usd_context.open_stage(MY_STAGE_NAME)#, self.load_stage)
        self.sim_data.test_stage = usd_context.get_stage()

        # tell vehicle helper to reload
        wr_inst = omni.docs.vehicle.helper.get_instance()
        if wr_inst:
            wr_inst.force_load = True
        
        #self.mute_persistent_sounds()
        self._start_button.visible = True 
        self._next_round_button.visible = False   
        self.populate_fields_from_stage()
        self.populate_default_test_params()
        self._stage_loaded = True     
        

        
        impact_large_list = [ 
            SOUND_IMPACT_LARGE_PATH_1,
            SOUND_IMPACT_LARGE_PATH_2,
            SOUND_IMPACT_LARGE_PATH_3  ]
        
        args = {
            'sim_data' : self.sim_data,
            'vehicle_prim_path' : self.sim_data.vehicle_prim_path ,
            'stage' : self.sim_data.test_stage,
            'audio_car_engine_prim_path' : SOUND_VEHICLE_ENGINE_PATH,
            'audio_tire_skid_prim_path' : SOUND_TIRE_SKID_PATH,
            'audio_impact_heavy_list' : impact_large_list,
            'audio_suspension_slam_path' : SOUND_SUSPENSION_SLAM_PATH,
            'audio_engine_rev_path' : SOUND_ENGINE_REV_PATH,
            'ui_data' : self.ui_data} 
        
        self.sim_data.vehicle_audio = VehicleAudio()
        self.sim_data.vehicle_audio.setup_audio(**args)

        #self.ui_data = UI_Data()
        # test_func = wheel.ref.test.get_instance()
        # if test_func is not None:
        #     inst = test_func()
        #     if inst is not None:
        #         inst.setup_sim()
        #         inst.start_sim()

    def on_click_start(self): 
        if self._test_round:
            #was_running = self._test_running
            was_running = self._test_round.test_running
            if was_running:
                self._test_round.finish_test()
            self._test_round = None
            #self._test_running = False
            self._next_round_button.visible = False
            self._test_round = None
            if was_running == True:
                return

        # callbacks
        #self.ui_data.end_of_round_report_fn = self.end_of_round_report_fn
        self.ui_data.test_round_event_fn = self.test_round_event_fn
        self.ui_data.test_done_report_fn = self.test_done_report_fn
        self.ui_data.set_stage_params_fn = self.set_stage_params_fn
        self.ui_data.engine_rpm_model = self._wheel_friction_value_model

        self._test_round = JumpTestRound(self.sim_data, self.ui_data)
        
        self.sim_data.sim_torque_steps = self._engine_torque_steps_model.as_int
        self.sim_data.sim_min_torque = self._engine_torque_min_model.as_float
        self.sim_data.sim_max_torque = self._engine_torque_max_model.as_float
        
        self._test_round.reset_test(self.sim_data)
        

        #stage_audio = omni.usd.audio.get_stage_audio_interface()
        self.sim_data.audio_win_bell_prim = self.sim_data.test_stage.GetPrimAtPath(SOUND_WIN_PATH)
        
        veh_prim = self.sim_data.test_stage.GetPrimAtPath(self.sim_data.vehicle_prim_path)
            
        engine_rot_speed_attr = veh_prim.GetAttribute(ENGINE_MAX_ROT_SPEED_ATTR_NAME)
        self.sim_data.engine_max_rpm = engine_rot_speed_attr.Get()
            
        self.sim_data.audio_fail_prim = self.sim_data.test_stage.GetPrimAtPath(SOUND_FAIL_PATH)

        self.sim_data.audio_start_race_prim = self.sim_data.test_stage.GetPrimAtPath(SOUND_START_RACE_PATH)
        
        #self.sim_data.audio_car_engine_prim = self.sim_data.test_stage.GetPrimAtPath(SOUND_VEHICLE_ENGINE_PATH)

        #self.sim_data.audio_tire_skid_prim = self.sim_data.test_stage.GetPrimAtPath(SOUND_TIRE_SKID_PATH)
            
        self._test_round.start_test()
        
        self._start_button.text = BUTTON_TEXT_STOP_TEST
        #self._test_running = True    
        self._next_round_button.visible = True   
        self._next_round_button.enabled = True 
        self._best_torque_to_stage_button.visible = False
        self._div_spread_cb.visible = False 
        self._half_dev_label.visible = False     

    def on_click_skip_round(self): 
        omni.timeline.get_timeline_interface().stop()
        if self._test_round is not None:
            self._test_round.end_current_round(False, "Round cancelled...")
            
            
    def on_click_best_torque_to_stage(self):
        print(f"HALF DEV: {self._div_spread_cb.model.get_value_as_bool()}")
        if self._div_spread_cb.model.get_value_as_bool():
            self._torque_pct_spread_value_model.as_float *= 0.5

        if self.sim_data.best_torque_model.as_float > 0:
            self._engine_torque_value.Set(self.sim_data.best_torque_model.as_float)
                    
    
    ######################################################################            

    def tick_update_ui(self, e: carb.events.IEvent):       
        if self._stage_loaded == True:
            self._pop_event_stream_sub_id = None
            self._stage_flash_on = False
            self._stage_button.text = "Reload Stage"
            self._stage_button.set_style({"color":0xFFBBBBBB})
        else:
            self._stage_flash_time_remaining -= e.payload["dt"]
            if self._stage_flash_time_remaining < 0:
                self._stage_flash_time_remaining = LOAD_STAGE_FLASH_PERIOD
                self._stage_flash_on = not self._stage_flash_on
                self._stage_button.set_style({"color":0xFF00FFFF} if self._stage_flash_on else {"color":0xFF555555})
            
                 
            



    def test_done_report_fn(self):
        self._label_test_results_report.text = "TEST OVER"
        self._start_button.text = BUTTON_TEXT_START_TEST
        self._next_round_button.visible = False
        self._audio = omni.usd.audio.get_stage_audio_interface()  


        report_text = ""
        # good test? (any good rounds)
        good_test = self.sim_data.best_round_idx_model.as_int > -1
        if good_test:
            self._best_torque_to_stage_button.visible = True
            self._div_spread_cb.visible = True 
            self._half_dev_label.visible = True 
            self.audio_good_test_prim = self.sim_data.test_stage.GetPrimAtPath(SOUND_GOOD_TEST_PATH)
            self._audio_good_test = self._audio.spawn_voice(self.audio_good_test_prim)
            report_text = "Success, best round #" + f"{self.sim_data.best_round_idx_model.as_int}"

        else:
            if self.sim_data.test_stage is not None:
                self.audio_bad_test_prim = self.sim_data.test_stage.GetPrimAtPath(SOUND_BAD_TEST_PATH)
                self._audio_bad_test = self._audio.spawn_voice(self.audio_bad_test_prim)
                report_text = "Test Failed..."
                
        self.test_round_event_fn(report_text, good_test)
        self.restore_default_stage_values()
                    
                
    # def end_of_round_report_fn(self):
    #     print("___________end_of_round_report___________")
        
    def test_round_event_fn(self, report_text, good=False):
        new_style: dict = {"font_size": 22}
        new_style["color"] = 0xFF00FF00 if good else 0xFF0000FF 
        
        self._label_test_results_report.set_style(new_style)  
        self._label_test_results_report.text = report_text
    

    def on_shutdown(self):
        print("[omni.docs.vehicle.jumper] MyExtension shutdown")
        if self._test_round:
            self._test_round.on_shutdown()
    

    ######################## STAGE FUNCTIONS #############################
    
    def restore_default_stage_values(self):
        self._engine_torque_value.Set(self._original_torque_value_model.as_float)
            

    def populate_fields_from_stage(self):
        
        self._wheel_friction_prim_path_value_model.as_string = WHEEL_FRICTION_PRIM_PATH
        
        wheel_attr_name = "defaultFrictionValue"
        prim_path = Sdf.Path(WHEEL_FRICTION_PRIM_PATH)
 
        self._fric_prim = self.sim_data.test_stage.GetPrimAtPath(prim_path)
        default_friction_value = self._fric_prim.GetAttribute(wheel_attr_name) 
        self._wheel_friction_value_model.as_float = round(default_friction_value.Get(),3)


        self._vehicle_prim_path_value_model.as_string = VEHICLE_PRIM_PATH
        eng_prim_path = Sdf.Path(VEHICLE_PRIM_PATH)
 
        self._engine_prim = self.sim_data.test_stage.GetPrimAtPath(eng_prim_path)
        self._engine_torque_value = self._engine_prim.GetAttribute(ENGINE_TORQUE_ATTR_NAME)
   
        self._original_torque_value_model.as_float = round(self._engine_torque_value.Get(),0)
        
    def on_click_set_torque_to_stage(self):
        eng_prim = self.sim_data.test_stage.GetPrimAtPath(VEHICLE_PRIM_PATH)
        float_val = round(self._original_torque_value_model.as_float,0) 
        eng_prim.GetAttribute(ENGINE_TORQUE_ATTR_NAME).Set(float_val)
        self.populate_fields_from_stage()
        self.populate_default_test_params()
 
    def set_new_stage_values(self):
        eng_prim = self.sim_data.test_stage.GetPrimAtPath(VEHICLE_PRIM_PATH)
        float_val = round(self.sim_data.round_torque_model.as_float,0) 
        eng_prim.GetAttribute(ENGINE_TORQUE_ATTR_NAME).Set(float_val)
 

    def set_stage_params_fn(self, **kwargs): 
        self.set_new_stage_values()
 
    def populate_default_test_params(self):
        steps = self._engine_torque_steps_model.as_int
        if steps < 1:
            return

        pct_dev_amt = self._original_torque_value_model.as_float * (self._torque_pct_spread_value_model.as_float * 0.01)
        step_size = (pct_dev_amt) / float(steps)
        self._engine_torque_steps_model.as_int = steps 

        self._engine_torque_min_model.as_float = self._original_torque_value_model.as_float - pct_dev_amt
        self._engine_torque_max_model.as_float = self._original_torque_value_model.as_float + pct_dev_amt


    # def mute_persistent_sounds(self):
    #     # Silent and looping
    #     prim = self.sim_data.test_stage.GetPrimAtPath(VEHICLE_ENGINE_SOUND_PATH)
    #     prim.GetAttribute("timeScale").Set(1.0)
    #     prim.GetAttribute("gain").Set(0.0)
    #     prim.GetAttribute("loopCount").Set(-1)

    def on_end_edit_sim_param(self, item_model):
        self.populate_default_test_params()
