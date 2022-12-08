import omni
import omni.timeline
from omni.physx.scripts.physicsUtils import *
from omni.physx import get_physx_interface, get_physx_simulation_interface
import random

from omni.physx.bindings._physx import (
    VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED,
    VEHICLE_WHEEL_STATE_SUSPENSION_FORCE,
    VEHICLE_WHEEL_STATE_IS_ON_GROUND,
    VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP,
    VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP
)

# Engine sound params
OFF_GROUND_RAMP_UP_MAX = 0.4
OFF_GROUND_RAMP_UP_SPEED = 4.5
OFF_GROUND_RAMP_DOWN_SPEED = 11.0
MAX_CONTRIBUTION_VELOCITY = 2500.0
RPM_REV_START = 0.255

# Suspension params
SUSPENSION_SLAM_FORCE_MIN = 1500000.0
SUSPENSION_SLAM_FORCE_MAX = 4500000.0

# Chassis impacts
SMALL_IMPULSE = 20000.0
LARGE_IMPULSE = 600000.0
MIN_TIME_BETWEEN_IMPACTS = 0.5
        
class VehicleAudio():
   
    def __init__(self):
        
        self.audio_tire_skid_prim = None
        self.audio_car_engine_prim = None
        self.audio_prims = {}

        self.audio_impact_prims = {}
        self.last_impact_time = -100.0
        self.last_impact_size = -1.0
        self.audio_impact_heavy_list = None
        
        self.audio_suspension_slam_prim = None
        self.suspension_slam_sound = None
        self.last_slam_time = -10.0
        self.last_slam_size = -1.0        
        
        self.physx_interface = omni.physx.get_physx_interface()
        # sim has let off throttle due to round's end
        self.killed_throttle = False
        # when flying through the air with the throttle down, wind up engine
        self.off_ground_windup = 0.0
        
        self.impact_sounds = {}
        self.impact_sound_idx = 0
        
    def setup_audio(self, **kwargs): 
        
        self._audio = omni.usd.audio.get_stage_audio_interface() 
        expected = [    'vehicle_prim_path',
                        'audio_tire_skid_prim_path',
                        'audio_car_engine_prim_path',
                        'audio_engine_rev_path',
                        'audio_suspension_slam_path',
                        'audio_impact_heavy_list',
                        'sim_data',
                        'ui_data',
                        'stage' ] 
        failed = False
        for arg_key in expected:
            if arg_key in kwargs:
                self.__dict__[arg_key] = kwargs[arg_key]
            else:
                failed = True
                print(f"setup_audio: Missing kwarg: {arg_key}")
        
        if failed:
            print("VehicleAudio: not enough args to setup")
            return
        

        if self.stage is not None:
            self.vehicle_prim = self.stage.GetPrimAtPath(self.vehicle_prim_path)

            self.audio_prims['tire_skid'] = self.stage.GetPrimAtPath(self.audio_tire_skid_prim_path)
            
            self.audio_prims['engine'] = self.stage.GetPrimAtPath(self.audio_car_engine_prim_path)
            
            self.audio_prims['rev'] = self.stage.GetPrimAtPath(self.audio_engine_rev_path)
            
            self.audio_slam_prim = self.stage.GetPrimAtPath(self.audio_suspension_slam_path)
            
            if self.audio_impact_heavy_list is not None:
                for idx in range(len(self.audio_impact_heavy_list)):
                    impact_name = "impact_heavy_" + str(idx)
                    self.audio_impact_prims[impact_name] = self.stage.GetPrimAtPath(self.audio_impact_heavy_list[idx])

            self.stop_audio()
            
    def start_audio(self):
        self.off_ground_windup = 0.0
        self.last_impact_time = -10.0
        self.last_impact_size = -1
        self.last_slam_time = -10.0
        self.audio_slam_prim.GetAttribute("gain").Set(0.0)
        for prim in self.audio_prims.values():
            prim.GetAttribute("timeScale").Set(1.0)
            prim.GetAttribute("gain").Set(0.0)
            prim.GetAttribute("loopCount").Set(-1)
            #prim.GetAttribute("startTime").Set(-1)
        
        for prim in self.audio_impact_prims.values():
            prim.GetAttribute("timeScale").Set(1.0)
            prim.GetAttribute("gain").Set(1)
            prim.GetAttribute("loopCount").Set(0)
        
        
    def stop_audio(self):   
        self.off_ground_windup = 0.0
        self.last_impact_time = -10.0
        self.last_impact_size = -1
        self.last_slam_time = -10.0
        self.audio_slam_prim.GetAttribute("gain").Set(0.0)
        for prim in self.audio_prims.values():
            prim.GetAttribute("timeScale").Set(1.0)
            prim.GetAttribute("gain").Set(0.0)
            prim.GetAttribute("loopCount").Set(-1)
            
        for prim in self.audio_impact_prims.values():
            prim.GetAttribute("timeScale").Set(1.0)
            prim.GetAttribute("gain").Set(1)
            prim.GetAttribute("loopCount").Set(0)        
            
    def get_time(self):
        timeline = omni.timeline.get_timeline_interface()
        ct = timeline.get_current_time()
        tcps = timeline.get_time_codes_per_seconds()
        return ct #* tcps
    

         
    def impact(self, impulse_mag, hit_roof=False):
 
        if impulse_mag < SMALL_IMPULSE:
            return
        
        curtime = self.get_time()
        dt = curtime - self.last_impact_time
       
        # play impact sound if it's been at least MIN_TIME_BETWEEN_IMPACTS
        # time between impacts or if it's bigger than the last one 
        
        bigger_ratio_min = 1.5
        impact_mag_ratio = impulse_mag / self.last_impact_size
        
        #if dt > MIN_TIME_BETWEEN_IMPACTS or impulse_mag > self.last_impact_size:
        if dt > MIN_TIME_BETWEEN_IMPACTS or impact_mag_ratio > bigger_ratio_min:
            self.last_impact_time = curtime
            self.last_impact_size = impulse_mag
        else:
            return
      
        # normalize and clamp  
        mag = impulse_mag / LARGE_IMPULSE
        mag = min(2.0, max(0.4, mag))           
        #mag = 2.0
        # pick a random impact sound
        s_idx = random.randint(0, 2)
        s_idx = 2
        impact_n = "impact_heavy_" + str(s_idx)
        #print(f"impact_n: {impact_n}")
        prim = self.audio_impact_prims[impact_n]
        prim.GetAttribute("timeScale").Set(1)
        prim.GetAttribute("gain").Set(mag)
        #prim.GetAttribute("loopCount").Set(1)
        #self.impactsound = self._audio.spawn_voice(prim)
        print("##########################################")
        print(f"Playing Impact: {impact_n} mag: {mag}  imp: {impulse_mag}   dt: {dt}")
        #self._audio.spawn_voice(prim)
        self.impact_sounds[self.impact_sound_idx] = self._audio.spawn_voice(prim)
        self.impact_sound_idx += 1
        if self.impact_sound_idx > 10:
            self.impact_sound_idx = 0
 
        
    def update_audio(self, e: carb.events.IEvent):
         
        # Iterate over all the wheels for 2 things:
        # - get the slip contribution for each tire that's on the ground
        #   for the skidding sound
        # - if all of the wheels are off the ground (could just require back two) 
        #   then we increase the engine pitch: engine wind up
        all_wheels_off_ground = True
        lat_slip_total = 0.0
        long_slip_total = 0.0
        largest_slip = 0        
        for curWheel in self.sim_data.wheel_list:
            CurWheelPath = self.sim_data.vehicle_prim_path + curWheel
            wheelState = self.physx_interface.get_wheel_state(CurWheelPath)
            if wheelState: 
                    
                lateralSlip = wheelState[VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP]
                longSlip = wheelState[VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP]
                    
                if wheelState[VEHICLE_WHEEL_STATE_IS_ON_GROUND]:
                    all_wheels_off_ground = False
                    lat_slip_total += abs(lateralSlip)
                    long_slip_total += abs(longSlip)
                    if largest_slip < long_slip_total:
                        largest_slip = long_slip_total
                        
                suspension_force = wheelState[VEHICLE_WHEEL_STATE_SUSPENSION_FORCE]
                f_mag = math.sqrt(  suspension_force[0] * suspension_force[0] + 
                                    suspension_force[1] * suspension_force[1] + 
                                    suspension_force[2] * suspension_force[2]) 
                
                
                #SUSPENSION_SLAM_FORCE_MIN = 1000000.0
                #SUSPENSION_SLAM_FORCE_MAX = 2500000.0
                
                if f_mag >= SUSPENSION_SLAM_FORCE_MIN:
                    if f_mag > SUSPENSION_SLAM_FORCE_MAX:
                        f_mag = SUSPENSION_SLAM_FORCE_MAX
                    
                    f_mag /= SUSPENSION_SLAM_FORCE_MAX
                    # 2275568.19376
                    print(f"f_mag: {f_mag}")
                    curtime = self.get_time()
                    dt = curtime - self.last_slam_time
                    if dt > MIN_TIME_BETWEEN_IMPACTS or f_mag > self.last_slam_size:
                        print(f"f_mag: {f_mag}____________________")
                        self.last_slam_time = curtime
                        self.last_slam_size = f_mag
                        slam_p = self.audio_slam_prim
                        slam_p.GetAttribute("gain").Set(f_mag)
                        self._audio.spawn_voice(slam_p)
   
                        
        total_slip = lat_slip_total + long_slip_total
        #print(f"total_slip: {total_slip}")           
            
        # normalize it and clamp 
        max_slip = 8.0
        slip_amt = total_slip / max_slip
        SLIP_CLAMP = 0.8
        if slip_amt > SLIP_CLAMP:
            slip_amt =SLIP_CLAMP
        elif slip_amt < 0.5:
            # power curve fade small amounts that linger
            slip_amt *= slip_amt * slip_amt
       
        #print(f"slip_amt: {slip_amt}")
            
        largest_slip = largest_slip / 25.0
        larg_slip_min = 0.9
        larg_slip_max = 1.15
        if largest_slip < larg_slip_min:
            largest_slip = larg_slip_min
            
        if largest_slip > larg_slip_max:
            largest_slip = larg_slip_max
        #print(f"largest_slip: {largest_slip}")

        skid_p = self.audio_prims['tire_skid']          
        skid_p.GetAttribute("timeScale").Set(largest_slip)
        skid_p.GetAttribute("gain").Set(slip_amt)
            
        #if self.sim_data.audio_car_engine_prim:
        if self.audio_prims['engine'] is not None:
            drive_state = self.physx_interface.get_vehicle_drive_state(self.vehicle_prim_path)
            rpm = drive_state[VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED]
            #self.ui_data.engine_rpm_model.as_float = rpm
            rpm_amt = rpm / self.sim_data.engine_max_rpm
            
            # engine rev-up off ground
            if rpm_amt > RPM_REV_START:
                if self.audio_prims['rev'] is not None:
                    rev_range = 1.0 - RPM_REV_START
                    rev_amt = ((rpm_amt - RPM_REV_START) / rev_range) + (self.off_ground_windup / 3.0)
                    rev_scale = 1.5 + (self.off_ground_windup / 3.0)
                    #self.audio_prims['rev']
                    self.audio_prims['rev'].GetAttribute("timeScale").Set(1.0 + (rev_amt * rev_scale))
                    self.audio_prims['rev'].GetAttribute("gain").Set(rev_amt)
            

            self.ui_data.engine_rpm_model.as_float = rpm_amt
            amt = rpm_amt
            if self.killed_throttle:
                amt *= 0.5
                
            velAttr = self.vehicle_prim.GetAttribute(UsdPhysics.Tokens.physicsVelocity)
            vehicle_vel = velAttr.Get() 
            velmag = math.sqrt( vehicle_vel[0] * vehicle_vel[0] + 
                                vehicle_vel[1] * vehicle_vel[1] + 
                                vehicle_vel[2] * vehicle_vel[2])
            
            #print (f"velmag: {velmag}")
            velamt = velmag / MAX_CONTRIBUTION_VELOCITY
            #scale = velamt + (amt * 2)
            scale = velamt + (amt * 2)
            if scale < 1.0:
                scale = 1.0 
            
            # if all of the wheels are off of the ground then wind-up the engine,
            # but lerp in and out at different rates, since landing brings the
            # engine back to its "proper" rpm quickly 
            if all_wheels_off_ground and not self.killed_throttle:
                self.off_ground_windup = min(OFF_GROUND_RAMP_UP_MAX, self.off_ground_windup + OFF_GROUND_RAMP_UP_SPEED * e.payload["dt"]) 
                            
            else: # on ground, wind back down  
                if self.off_ground_windup > 0.0: # false most common, skip calcs
                    self.off_ground_windup = max(0.0, self.off_ground_windup - OFF_GROUND_RAMP_DOWN_SPEED * e.payload["dt"]   )
                     
            scale += self.off_ground_windup 
            engine_p = self.audio_prims['engine']
            engine_p.GetAttribute("timeScale").Set(scale  )
            #engine_p.GetAttribute("gain").Set(0.75) 
            engine_p.GetAttribute("gain").Set(scale / 2.5) 
            
           # print(f"scale ========== {scale}")
            

            
            