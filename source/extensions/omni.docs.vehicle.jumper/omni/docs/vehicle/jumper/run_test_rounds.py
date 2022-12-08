import omni.timeline	
import carb
from pxr import PhysxSchema, UsdPhysics
from omni.physx import get_physx_simulation_interface
from omni.physx.scripts.physicsUtils import *
import omni.usd
from pxr import PhysicsSchemaTools, UsdPhysics, PhysxSchema
import omni.kit.app

from .jumper_cam import *

from omni.physx.bindings._physx import (
    VEHICLE_WHEEL_STATE_GROUND_MATERIAL,
    VEHICLE_WHEEL_STATE_SUSPENSION_FORCE
)

contact_sensitivity = 100.0
delay_after_round = 2.5

STARTING_GUN_DELAY = 5.0
STARTING_GUN_PRE_DELAY = 4.0

__all__ = ['JumpTestRound']

class JumpTestRound:

    def __init__(self, sim_data, ui_data):
        self.sim_data = sim_data
        self.ui_data = ui_data
        self._end_of_round_update_sub_id = None
        self._physxInterface = omni.physx.get_physx_interface()

        # sim started per round
        self._sim_running = False
        # the entire test, not just per round
        self.test_running = False

        timelineInterface = omni.timeline.get_timeline_interface()
        stream = timelineInterface.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self.timeline_event)
        self.headlight_prims = []

    def on_shutdown(self):
        print("TESTER on_shutdown...")
        self.kill_subscriptions()
        
    def kill_subscriptions(self):
        sub_list = ['_end_of_round_update_sub_id',
                    '_contact_report_sub',
                    '_physics_tick_update_sub_id',
                    '_timeline_subscription' ]
        
        for sub_key in sub_list:
            if sub_key in self.__dict__:
                if self.__dict__[sub_key] is not None:
                    self.__dict__[sub_key].unsubscribe()
                    self.__dict__[sub_key] = None
        
    def reset_test(self, sim_data):
        self._cur_test_step = 0
        self.sim_data.round_torque_model.as_float = self.sim_data.sim_min_torque

        self.vehicle_prim = sim_data.test_stage.GetPrimAtPath(self.sim_data.vehicle_prim_path)
        
        cam_args = {
            'camera_prim_path' : sim_data.vehicle_camera_path,
            'vehicle_prim' : self.vehicle_prim,
            'stage' : sim_data.test_stage }
            
        self.vehicle_camera = JumperCam()
        self.vehicle_camera.setup_camera(**cam_args)
        
       
        self.goal_prim = sim_data.test_stage.GetPrimAtPath(self.sim_data.end_goal_prim_path)

        self.wheel_friction_prim = sim_data.test_stage.GetPrimAtPath(self.sim_data.wheel_friction_prim_path)

        self._audio = omni.usd.audio.get_stage_audio_interface() 
        
        # self._engine_moi = self.vehicle_prim.GetAttribute("physxVehicleEngine:moi").Get()
        # self._damp_full_throttle = self.vehicle_prim.GetAttribute("physxVehicleEngine:dampingRateFullThrottle").Get()
        # self._damp_no_throttle_clutch_engaged = self.vehicle_prim.GetAttribute("physxVehicleEngine:dampingRateZeroThrottleClutchEngaged").Get()
        #self._engine_moi = 24275
        self._engine_moi = 24275
        self._damp_full_throttle = 3641
        self._damp_no_throttle_clutch_engaged = 48551
        
        self.rear_left_prim = sim_data.test_stage.GetPrimAtPath(self.sim_data.vehicle_prim_path + "/LeftWheel2References")
        self.rear_right_prim = sim_data.test_stage.GetPrimAtPath(self.sim_data.vehicle_prim_path + "/RightWheel2References")
        
        self.headlight_prims = []
        for headlight in self.sim_data.vehicle_headlights:
            light_prim = sim_data.test_stage.GetPrimAtPath(headlight)
            if light_prim:
                self.headlight_prims.append(light_prim)
        
        

                  

    def start_test(self):
        #update_stream = omni.kit.app.get_app().get_update_event_stream()
        # reset test stats ui        
        self.sim_data.best_torque_model.as_float = 0
        self.sim_data.best_landing_body_impulse_model.as_float = 0
        self.sim_data.best_landing_susp_force_model.as_float = 0
        self.sim_data.best_round_idx_model.as_int = -1
        
        self._best_round_was_soft_landing = False
        self.test_running = True

        UsdPhysics.CollisionAPI.Apply(self.goal_prim)
        PhysxSchema.PhysxTriggerAPI.Apply(self.goal_prim)
        self.triggerStateAPI = PhysxSchema.PhysxTriggerStateAPI.Apply(self.goal_prim)


        self.start_next_round()
        

    def start_next_round(self):
        if self.is_test_done():
            self.finish_test()
            return
        
        # self._start_race_sound = self._audio.spawn_voice(self.sim_data.audio_start_race_prim)
        self.vehicle_camera.car_started_moving = False
        self.vehicle_camera.set_initial_position()
        self._wait_for_go_time_remaining = STARTING_GUN_DELAY 
        self.wait_gas_flip_last_num = round(STARTING_GUN_DELAY * 3 )
        self.wait_gas_flip_b = False
        
        self._wait_to_start_countdown_sound = STARTING_GUN_PRE_DELAY

        # waiting to start
        self.set_throttle(1)
        self.set_brake0(.01)
        self.set_brake1(0)

        self.sim_data.vehicle_audio.killed_throttle = False
        self.reduced_throttle = False
      
                
        self._round_over = False 
        self.ui_data.test_round_event_fn("")           
 
        # wheels, not body
        self._wheels_touched_ramp = False 
        self._wheels_touched_dead_zone = False
        #self._largest_round_jounce = 0.0
        self._round_largest_body_impulse = 0.0
        self._round_largest_susp_force = 0.0
        self.sim_data.round_largest_susp_force_model.as_float = 0.0
        self.sim_data.round_largest_body_impulse_model.as_float = 0.0

        self._cur_test_step += 1
        if self._cur_test_step > self.sim_data.sim_torque_steps:
            self._cur_test_step = 0
        
        torque_range = self.sim_data.sim_max_torque - self.sim_data.sim_min_torque
        if self.sim_data.sim_torque_steps == 1:
            # special case: use midpoint of range (for testing original torque)
            torque_inc = torque_range / 2.0
            self.current_test_torque = self.sim_data.sim_min_torque + torque_inc
        else:
            torque_inc = torque_range / float(self.sim_data.sim_torque_steps - 1)
            self.current_test_torque = self.sim_data.sim_min_torque + (float(self._cur_test_step - 1) * torque_inc)

        # torque_inc = torque_range / float(self.sim_data.sim_torque_steps - 1)
        # self.current_test_torque = self.sim_data.sim_min_torque + (float(self._cur_test_step - 1) * torque_inc)


        self.sim_data.round_torque_model.as_float = self.current_test_torque
        self.ui_data.set_stage_params_fn(torque=self.current_test_torque)

        
        # update ui round stats
        self.sim_data.round_step_model.as_int = self._cur_test_step
        
        #	enginePrim = stage.GetPrimAtPath(engine_prim_path)
        contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(self.vehicle_prim)
        contactReportAPI.CreateThresholdAttr().Set(1000)

        self._contact_report_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)
        self._skip_first_update_event = True
        self.sim_data.vehicle_audio.start_audio() 
        omni.timeline.get_timeline_interface().play(1,1500,False)
       
 

    def set_throttle(self, throttle_amount):
        self.vehicle_prim.GetAttribute("physxVehicleController:accelerator").Set(throttle_amount)

    def set_brake0(self, brake_amount):
        self.vehicle_prim.GetAttribute("physxVehicleController:brake0").Set(brake_amount)
    def set_brake1(self, brake_amount):
        self.vehicle_prim.GetAttribute("physxVehicleController:brake1").Set(brake_amount)
 
         

 
    def is_test_done(self):
        return self._cur_test_step >= self.sim_data.sim_torque_steps
    

    def finish_test(self):
        omni.timeline.get_timeline_interface().stop()
        self.kill_subscriptions()
        self._round_over = True
        self._sim_running = False
        self.test_running = False
        #self.test_done = True
        self.ui_data.test_done_report_fn()



   
    def timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            update_stream = omni.kit.app.get_app().get_update_event_stream()
            self._physics_tick_update_sub_id = update_stream.create_subscription_to_pop(self.physics_tick_update, name="tickupdate_sim")
            self._skip_first_update_event = True            
            self.sim_data.vehicle_audio.start_audio()  

        if event.type == int(omni.timeline.TimelineEventType.STOP):
            print("timeline STOPPED")
            self._physics_tick_update_sub_id = None
       

    ########################## End of Round ###############################

    def finalize_end_of_round(self):
        self.sim_data.vehicle_audio.stop_audio()    
        self._physics_tick_update_sub_id = None    
        self.start_next_round()

    # End of Round countdown, delay to restart sim 
    def end_of_round_update(self, e: carb.events.IEvent):
 
        self._update_wait_remaining -= e.payload["dt"]

        if self._update_wait_remaining < 0.5 and self._sim_running:
            self._sim_running = False
            omni.timeline.get_timeline_interface().stop()

        if self._update_wait_remaining <= 0:
            self._end_of_round_update_sub_id = None
            self.finalize_end_of_round()
            

    def end_current_round(self, hit_goal, fail_str=""):     
        
        self._round_over = True
         
        if not hit_goal:
            self._fail_sound = self._audio.spawn_voice(self.sim_data.audio_fail_prim)
        
        self.set_throttle(0.0)
        self.sim_data.vehicle_audio.killed_throttle = True
        
        success = False
        if hit_goal:
            if self._wheels_touched_ramp:
                out_str = "Success!" # add if best round 
                success = True
            else: 
                out_str = "FAILED: missed landing ramp"  
        else:
            out_str = "FAILED: " + fail_str
    
        self.ui_data.test_round_event_fn(out_str, success)

        self._wheels_touched_ramp = False 
        self._wheels_touched_dead_zone = False

        # Process success results
        if success:
            self._win_sound = self._audio.spawn_voice(self.sim_data.audio_win_bell_prim)

            # best round critia in this order
            # best suspension force : soft landing, with least susp force
            # best body impulse : body hit something
            bestround = False
            soft_landing = self._round_largest_body_impulse == 0
            if soft_landing:
                # take first soft landing or the best soft landing (we can have smaller suspension force from hard landings)
                if not self._best_round_was_soft_landing or self._round_largest_susp_force < self.sim_data.best_landing_susp_force_model.as_float:
                    bestround = True 
                    self._best_round_was_soft_landing = True
                
            # use body impulse if others rounds aren't soft
            elif not self._best_round_was_soft_landing:
                if self.sim_data.best_landing_body_impulse_model.as_float == 0:
                    bestround = True
                elif self._round_largest_body_impulse < self.sim_data.best_landing_body_impulse_model.as_float:                    
                    bestround = True

            if bestround:
                self.sim_data.best_torque_model.as_float = self.sim_data.round_torque_model.as_float
                self.sim_data.best_landing_body_impulse_model.as_float = 0 if self._best_round_was_soft_landing else self._round_largest_body_impulse
                self.sim_data.best_landing_susp_force_model.as_float = self._round_largest_susp_force
                self.sim_data.best_round_idx_model.as_int = self.sim_data.round_step_model.as_int
                    
        # start countdown to next round
        if self._end_of_round_update_sub_id is None:
            self.set_throttle(0.0)
            self.sim_data.vehicle_audio.killed_throttle = True
            update_stream = omni.kit.app.get_app().get_update_event_stream()
            self._end_of_round_update_sub_id = update_stream.create_subscription_to_pop(self.end_of_round_update, name="EndRound")
            
            self._sim_running = True
            self._update_wait_remaining = delay_after_round
            

    ############################### Physics ###############################

    def _on_contact_report_event(self, contact_headers, contact_data):
        # dont spam 'end_current_round'
        # if self._round_over:
        #     return

        big_impulse = float(0.0)
        hit_ramp = False
        hit_roof = False
        found_goal = False
        out_of_bounds = False
        for contact_header in contact_headers:
            act0_path = str(PhysicsSchemaTools.intToSdfPath(contact_header.actor0))
            act1_path = str(PhysicsSchemaTools.intToSdfPath(contact_header.actor1))

            cur_collider = str(PhysicsSchemaTools.intToSdfPath(contact_header.collider0))
            
           
            if cur_collider == self.sim_data.vehicle_roof_prim_path:
                hit_roof = True

            if act0_path == self.sim_data.vehicle_prim_path:
                # reached end goal?
                if act1_path == self.sim_data.end_goal_prim_path:
                    #if not self._round_over:
                    found_goal = True

                if act1_path == self.sim_data.landing_ramp_prim_path:
                    hit_ramp = True
            
                contact_data_offset = contact_header.contact_data_offset
                num_contact_data = contact_header.num_contact_data
                for index in range(contact_data_offset, contact_data_offset + num_contact_data, 1):
                    cur_contact = contact_data[index]
                    cur_impulse =  cur_contact.impulse[0] * cur_contact.impulse[0]
                    cur_impulse += cur_contact.impulse[1] * cur_contact.impulse[1]
                    cur_impulse += cur_contact.impulse[2] * cur_contact.impulse[2]
                    cur_impulse = math.sqrt(cur_impulse)
                    
                    if not self._round_over:
                        cur_material = str(PhysicsSchemaTools.intToSdfPath(cur_contact.material1))
                        if(cur_material == self.sim_data.material_out_of_bounds):
                            out_of_bounds = True  
 
                    if big_impulse < cur_impulse:
                        big_impulse = cur_impulse
                    
        big_impulse = round(big_impulse,0)
        if self.sim_data.vehicle_audio is not None:
            self.sim_data.vehicle_audio.impact(big_impulse)
 
        if not self._round_over:
            if hit_roof:
                self.end_current_round(False, "Hit Roof!")
            elif out_of_bounds:
                self.end_current_round(False, "Left Safe Area")   
            elif found_goal:
                self.end_current_round(True)
 
        if hit_ramp:
            if big_impulse > self._round_largest_body_impulse:
                self._round_largest_body_impulse = big_impulse
                self.sim_data.round_largest_body_impulse_model.as_float = big_impulse

    def restore_engine_params(self):
        self.wheel_friction_prim.GetAttribute("defaultFrictionValue").Set(1)
        self.vehicle_prim.GetAttribute("physxVehicleEngine:moi").Set(self._engine_moi)
        self.vehicle_prim.GetAttribute("physxVehicleEngine:dampingRateFullThrottle").Set(self._damp_full_throttle)
        self.vehicle_prim.GetAttribute("physxVehicleEngine:dampingRateZeroThrottleClutchEngaged").Set(self._damp_no_throttle_clutch_engaged)  
        
    def pre_race_burnouts(self, dt):
           
        chk_time = round(self._wait_for_go_time_remaining )
        if chk_time < self.wait_gas_flip_last_num:
            self.wait_gas_flip_b = not self.wait_gas_flip_b
            self.wait_gas_flip_last_num = chk_time
        
        if self.wait_gas_flip_b:
            # STOP burning out
            self.vehicle_prim.GetAttribute("physxVehicleController:targetGear").Set(1)
            self.wheel_friction_prim.GetAttribute("defaultFrictionValue").Set(1)
            self.rear_left_prim.GetAttribute("physxVehicleTire:longitudinalStiffness").Set(1023209)
            self.rear_right_prim.GetAttribute("physxVehicleTire:longitudinalStiffness").Set(1023209)
            self.set_brake0(1)
            self.set_throttle(0.0)
            self.vehicle_prim.GetAttribute("physxVehicleEngine:moi").Set(24275)
            self.vehicle_prim.GetAttribute("physxVehicleEngine:dampingRateFullThrottle").Set(8496)
            self.vehicle_prim.GetAttribute("physxVehicleEngine:dampingRateZeroThrottleClutchEngaged").Set(48551)
        else:
            # START burning out
            self.vehicle_prim.GetAttribute("physxVehicleController:targetGear").Set(1)
            # self.wheel_friction_prim.GetAttribute("defaultFrictionValue").Set(.001)            
            # self.rear_left_prim.GetAttribute("physxVehicleTire:longitudinalStiffness").Set(0.01)
            # self.rear_right_prim.GetAttribute("physxVehicleTire:longitudinalStiffness").Set(0.01) 
             
            self.set_brake0(1)
            self.set_throttle(1)
            self.vehicle_prim.GetAttribute("physxVehicleEngine:moi").Set(24275)
            self.vehicle_prim.GetAttribute("physxVehicleEngine:dampingRateFullThrottle").Set(3641)
            self.vehicle_prim.GetAttribute("physxVehicleEngine:dampingRateZeroThrottleClutchEngaged").Set(48551)
              
                
        self._wait_for_go_time_remaining -= dt
     
    def reduce_throttle(self):
        self.set_throttle(0.5) 
        self.reduced_throttle = True 

    def physics_tick_update(self, e: carb.events.IEvent):

        self.vehicle_camera.update_camera(e)

        # burnouts before start 
        if self._wait_for_go_time_remaining > 0:
            
            self.pre_race_burnouts(e.payload["dt"])
            if self._wait_for_go_time_remaining <= 0:
                # GO! restore wheel frictions, throttle and brake
                self.wheel_friction_prim.GetAttribute("defaultFrictionValue").Set(1)
                self.vehicle_prim.GetAttribute("physxVehicleController:targetGear").Set(255)
                self.restore_engine_params()
                self.set_throttle(1)
                self.set_brake0(0)
                self.set_brake1(0)
                self.rear_left_prim.GetAttribute("physxVehicleTire:longitudinalStiffness").Set(1023209)
                self.rear_right_prim.GetAttribute("physxVehicleTire:longitudinalStiffness").Set(1023209)
                
                self.vehicle_camera.car_started_moving = True

        # starting sound (may differ from car's start)        
        if self._wait_to_start_countdown_sound > 0:
            self._wait_to_start_countdown_sound -= e.payload["dt"]
            if self._wait_to_start_countdown_sound <= 0:
                self._start_race_sound = self._audio.spawn_voice(self.sim_data.audio_start_race_prim)

        if self._skip_first_update_event:
            self._skip_first_update_event = False
            self.last_time = int(omni.timeline.get_timeline_interface().get_current_time()*3)
            return     
           
        if self.sim_data.vehicle_audio is not None:
            self.sim_data.vehicle_audio.update_audio(e)
            
        if self.vehicle_camera is not None:
            self.vehicle_camera.update_camera(e)    
                    
        # brighten headlights when engine rev's
        rev_amount = self.ui_data.engine_rpm_model.as_float
        for h_light in self.headlight_prims:
            light_amt = 2000000.0 + (1500000.0 * rev_amount)
            h_light.GetAttribute("intensity").Set(light_amt)
                
        # dont spam 'end_current_round'
        if self._round_over:
            return        
             
        triggerColliders = self.triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        if len(triggerColliders) > 0:
            if self.sim_data.vehicle_collision_prim_path == triggerColliders[0]:
                self.end_current_round(True)   

        if omni.timeline.get_timeline_interface().is_playing():
            out_result = False
            
            new_time = int(omni.timeline.get_timeline_interface().get_current_time()*3)
            if new_time > self.last_time:
                self.last_time = new_time
                out_result = False

            # check upside down...
            if self.vehicle_prim:
                
                local_mat = UsdGeom.Xformable(self.vehicle_prim).GetLocalTransformation()
                up = local_mat.GetRow3(1)
                upness = up[1]
                
                if out_result:
                    print(f"upness: {upness}")
                    
                if upness < -0.5:
                    self.end_current_round(False, "Upside down...")
                    
                pos = local_mat.ExtractTranslation()
                if pos[2] > 3100.0:
                    #print(f"*******************************vehicle Z: {pos[2]}")
                    if self.reduced_throttle == False:
                        pass
                        self.reduce_throttle()
              


            found_ramp = False
            found_dead_zone = False          
   
            for curWheel in self.sim_data.wheel_list:

                CurWheelPath = self.sim_data.vehicle_prim_path + curWheel
                wheelState = self._physxInterface.get_wheel_state(CurWheelPath)
                if wheelState: 
                    wheel_mat = wheelState[VEHICLE_WHEEL_STATE_GROUND_MATERIAL]
                    
                    wheel_on_ramp = wheel_mat == self.sim_data.material_safe_ramp
                    found_ramp = found_ramp or wheel_on_ramp
                    
                    # keep tracking forces even after touching ramp
                    if found_ramp:
         
                        suspension_force = wheelState[VEHICLE_WHEEL_STATE_SUSPENSION_FORCE]

                        f_mag = math.sqrt(  suspension_force[0] * suspension_force[0] + 
                                            suspension_force[1] * suspension_force[1] + 
                                            suspension_force[2] * suspension_force[2])
                        
                        if self._round_largest_susp_force < f_mag:
                            self._round_largest_susp_force = f_mag
                            self.sim_data.round_largest_susp_force_model.as_float = f_mag
 

                    if out_result:
                        print(f"wheel_mat : {wheel_mat}")                            
                    
                    found_dead_zone = found_dead_zone or wheel_mat == self.sim_data.material_out_of_bounds
                    
            self._wheels_touched_ramp = self._wheels_touched_ramp or found_ramp
            self._wheels_touched_dead_zone = self._wheels_touched_dead_zone or found_dead_zone
            
            if self._wheels_touched_dead_zone:
                self.end_current_round(False, "Left Safe Area")
