import math
import time
from geometry_msgs.msg import Twist
from .utils import marker_world

class StateMachine:
    def __init__(self, controller, compute_distances, params):
        self.ctrl          = controller
        self.calc          = compute_distances
        self.params        = params
        self.state         = 'IDLE'
        self.rotation_dir  = 1.0
        self.wx_r = self.wy_r = None
        self.wx_f = self.wy_f = None
        self.d_f = self.d_l = self.d_r = float('nan')
        self.target        = None
        self.start_pose    = None
        self.last_target   = None
        self.accum_time    = 0.0
        self.cont_accum    = 0.0
        self.ft_accum_cont = 0.0
        self.prev_time     = time.time()

    def pose_cb(self, msg):
        self.wx_r, self.wy_r = msg.point.x, msg.point.y

    def front_cb(self, msg):
        self.wx_f, self.wy_f = msg.point.x, msg.point.y

    def wp6_cb(self, msg):
        marker_world[6] = (msg.point.x, msg.point.y)

    def wp7_cb(self, msg):
        marker_world[7] = (msg.point.x, msg.point.y)

    def target_cb(self, msg):
        tid = msg.data
        if tid in marker_world and self.wx_r is not None:
            self.target      = marker_world[tid]
            self.start_pose  = (self.wx_r, self.wy_r)
            self.last_target = tid
            self.state       = 'IDLE'
            self.accum_time  = 0.0
            self.cont_accum  = 0.0
            self.ft_accum_cont = 0.0

    def target_point_cb(self, msg):
        if self.wx_r is not None:
            self.target      = (msg.point.x, msg.point.y)
            self.start_pose  = (self.wx_r, self.wy_r)
            self.last_target = None
            self.state       = 'IDLE'
            self.accum_time  = 0.0
            self.cont_accum  = 0.0

    def scan_cb(self, msg):
        self.d_f, self.d_l, self.d_r = self.calc(msg.ranges, msg.angle_increment)

    def arrived_cb(self, msg):
        if self.last_target in (31, 32):
            self.state         = 'FINAL_TURN'
            self.accum_time    = 0.0
            self.ft_accum_cont = 0.0
            yaw      = math.atan2(self.wy_f - self.wy_r, self.wx_f - self.wx_r)
            desired  = math.pi if self.last_target == 31 else 0.0
            abs_err0 = abs((desired - yaw + math.pi) % (2*math.pi) - math.pi)
            self.ft_threshold = max(abs_err0 - self.params['mini_rad'], 0.0)
        else:
            self.state       = 'IDLE'
            self.target      = None
            self.last_target = None

    def step(self, dt, pub):
        # 1) FINAL_TURN
        if self.state == 'FINAL_TURN':
            yaw      = math.atan2(self.wy_f - self.wy_r, self.wx_f - self.wx_r)
            desired  = math.pi if self.last_target == 31 else 0.0
            error    = (desired - yaw + math.pi) % (2*math.pi) - math.pi
            abs_err  = abs(error)
            if abs_err < math.radians(self.params['angle_tolerance']):
                pub(Twist()); self.state='IDLE'; self.target=None; return
            cont_thr = max(abs_err - self.params['mini_rad'], 0.0)
            if self.ft_accum_cont < cont_thr:
                t = Twist(); t.angular.z=math.copysign(self.params['fixed_omega'], error)
                self.ft_accum_cont += abs(self.params['fixed_omega']*dt)
                pub(t); return
            if self.accum_time < self.params['turn_interval']: t = Twist(); t.angular.z=math.copysign(self.params['fixed_omega'], error)
            elif self.accum_time < self.params['turn_interval']+self.params['pause_interval']: t = Twist()
            else: self.accum_time=0.0; t=Twist()
            self.accum_time += dt; pub(t); return

        # 2) pose/target 체크
        if None in (self.wx_r,self.wy_r,self.wx_f,self.wy_f) or self.target is None:
            pub(Twist()); return

        # 3) 장애물 회피
        if self.state=='DRIVING' and not math.isnan(self.d_f) and self.d_f<self.params['obs_detect_thresh']:
            self.state='CONFIRM_OBS'; self.accum_time=0.0; return
        if self.state=='CONFIRM_OBS':
            if self.d_f<self.params['obs_detect_thresh']:
                self.accum_time+=dt
                if self.accum_time>=1.0: self.state='STOP_OBS'; self.accum_time=0.0
            else: self.state='DRIVING'; self.accum_time=0.0
            pub(Twist()); return
        if self.state=='STOP_OBS':
            self.accum_time+=dt
            if self.accum_time>=self.params['pause_interval']:
                self.rotation_dir = 1.0 if self.d_l>self.d_r else -1.0
                self.state='ROTATING_OBS'; self.accum_time=0.0
            pub(Twist()); return
        if self.state=='ROTATING_OBS':
            heading=math.atan2(self.wy_f-self.wy_r,self.wx_f-self.wx_r)
            if not hasattr(self,'_rotate_obs_start'): self._rotate_obs_start=heading; self.accum_time=0.0
            delta=(heading-self._rotate_obs_start+math.pi)%(2*math.pi)-math.pi
            if abs(delta)<math.pi/2-0.05:
                t=Twist()
                if self.accum_time<self.params['turn_interval']: t.angular.z=self.rotation_dir*self.params['fixed_omega']
                self.accum_time+=dt; pub(t)
            else:
                actual=abs(math.degrees(delta))
                if abs(actual-90)<=2.0: delattr(self,'_rotate_obs_start'); self.state='DRIVE_OBS'; self.accum_time=0.0
            return
        if self.state=='DRIVE_OBS':
            side=self.d_r if self.rotation_dir>0 else self.d_l
            if not hasattr(self,'drive_obs_cleared'):
                if side>=self.params['obs_clear_thresh']: self.drive_obs_cleared=time.time()
                t=Twist(); t.linear.x=self.params['drive_speed']; pub(t); return
            if time.time()-self.drive_obs_cleared>=self.params['drive_obs_time']:
                self.state='ROTATE_SURFACE'; self.accum_time=0.0; self.rotation_dir*=-1; delattr(self,'drive_obs_cleared')
            else: t=Twist(); t.linear.x=self.params['drive_speed']; pub(t)
            return

        # 4) 기본 FSM: IDLE→ROTATING→DRIVING
        tx,ty=self.target; fv=(self.wx_f-self.wx_r,self.wy_f-self.wy_r); tv=(tx-self.wx_r,ty-self.wy_r)
        err_deg=math.degrees(math.atan2(fv[0]*tv[1]-fv[1]*tv[0], fv[0]*tv[0]+fv[1]*tv[1]))
        if self.state=='IDLE':
            if abs(err_deg)>=self.angle_tolerance:
                self.rotation_dir=math.copysign(1.0,err_deg); self.accum_time=0.0; self.state='ROTATING'; self.cont_accum=0.0
            else: self.state='DRIVING'
        if self.state=='ROTATING':
            abs_err=abs(err_deg); cont_thr=max(abs_err-self.mini_deg,0.0)
            if cont_thr>0 and self.cont_accum<cont_thr:
                t=Twist(); t.angular.z=math.copysign(self.fixed_omega,err_deg)
                self.cont_accum+=abs(self.fixed_omega*dt); pub(t); return
            if self.accum_time<self.turn_interval: t=Twist(); t.angular.z=math.copysign(self.fixed_omega,err_deg)
            self.accum_time+=dt; pub(t)
            if abs_err<self.angle_tolerance: self.state='DRIVING'; self.accum_time=0.0
            return
        if self.state=='DRIVING': t=Twist(); t.linear.x=self.drive_speed; pub(t); return
