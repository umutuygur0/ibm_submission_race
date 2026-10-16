# Bu dosya racelab/planner/build_line_submission.py tarafından üretildi.
# Kaynak iskelet: gym_torcs/torcs_jm_par.py   Parametreler: racelab/planner/runs/snap_gr3L.json

import socket
import sys
import getopt
import os
import time
import json
PI= 3.14159265359

data_size = 2**17

ophelp=  'Options:\n'
ophelp+= ' --host, -H <host>    TORCS server host. [localhost]\n'
ophelp+= ' --port, -p <port>    TORCS port. [3001]\n'
ophelp+= ' --id, -i <id>        ID for server. [SCR]\n'
ophelp+= ' --steps, -m <#>      Maximum simulation steps. 1 sec ~ 50 steps. [100000]\n'
ophelp+= ' --episodes, -e <#>   Maximum learning episodes. [1]\n'
ophelp+= ' --track, -t <track>  Your name for this track. Used for learning. [unknown]\n'
ophelp+= ' --stage, -s <#>      0=warm up, 1=qualifying, 2=race, 3=unknown. [3]\n'
ophelp+= ' --parameters, -f <#>   Input parameters of target_speed : int, steer_gain : int, centering_gain : float, brake_threshold : float, 6*[gear_speeds : int] (dont put brackets in the input tho), enable_traction_control : bool. [1]\n'
ophelp+= ' --debug, -d          Output full telemetry.\n'
ophelp+= ' --help, -h           Show this help.\n'
ophelp+= ' --version, -v        Show current version.'
usage= 'Usage: %s [ophelp [optargs]] \n' % sys.argv[0]
usage= usage + ophelp
version= "20130505-2"

#### Parameters

#############################################
# MODULAR DRIVE LOGIC WITH USER PARAMETERS  #
#############################################

import math


# ================= USER CONFIGURABLE PARAMETERS =================

######### REQUIRED FOR PRACTICE MODE - CAN CHANGE BUT DO NOT REMOVE #########
TARGET_SPEED = 100  # Target speed in km/h. Increasing this makes the car go faster but may reduce stability.
STEER_GAIN = 30     # Steering sensitivity. Higher values make the car turn more aggressively.
CENTERING_GAIN = 0.20  # How strongly the car corrects its position toward the center of the track.
BRAKE_THRESHOLD = 0.9  # Angle threshold for braking. Lower values brake earlier.
GEAR_SPEEDS = [0, 20, 40, 80, 100, 180]  # Speed thresholds for gear shifting.
ENABLE_TRACTION_CONTROL = True  # Toggle traction control system.
######### END OF REQUIRED #########

# User configurable parameters as cli input:
# python torcs_jm_par.py --parameters [json data]


# ================= HELPER FUNCTIONS =================
def calculate_steering(S):
    steer = (S['angle'] * STEER_GAIN / math.pi) - (S['trackPos'] * CENTERING_GAIN)
    return max(-1, min(1, steer))

def calculate_throttle(S, R):
    if S['speedX'] < TARGET_SPEED - (R['steer'] * 2.5):
        accel = min(1.0, R['accel'] + 0.4)
    else:
        accel = max(0.0, R['accel'] - 0.2)
    if S['speedX'] < 10:
        accel += 1 / (S['speedX'] + 0.1)
    return max(0.0, min(1.0, accel))

def apply_brakes(S):
    return 0.3 if abs(S['angle']) > BRAKE_THRESHOLD else 0.0

def shift_gears(S):
    gear = 1
    for i, speed in enumerate(GEAR_SPEEDS):
        if S['speedX'] > speed:
            gear = i + 1
    return min(gear, 6)

def traction_control(S, accel):
    if ENABLE_TRACTION_CONTROL:
        if ((S['wheelSpinVel'][2] + S['wheelSpinVel'][3]) - (S['wheelSpinVel'][0] + S['wheelSpinVel'][1])) > 2:
            accel -= 0.1
    return max(0.0, accel)

####

def clip(v,lo,hi):
    if v<lo: return lo
    elif v>hi: return hi
    else: return v

def bargraph(x,mn,mx,w,c='X'):
    '''Draws a simple asciiart bar graph. Very handy for
    visualizing what's going on with the data.
    x= Value from sensor, mn= minimum plottable value,
    mx= maximum plottable value, w= width of plot in chars,
    c= the character to plot with.'''
    if not w: return '' # No width!
    if x<mn: x= mn      # Clip to bounds.
    if x>mx: x= mx      # Clip to bounds.
    tx= mx-mn # Total real units possible to show on graph.
    if tx<=0: return 'backwards' # Stupid bounds.
    upw= tx/float(w) # X Units per output char width.
    if upw<=0: return 'what?' # Don't let this happen.
    negpu, pospu, negnonpu, posnonpu= 0,0,0,0
    if mn < 0: # Then there is a negative part to graph.
        if x < 0: # And the plot is on the negative side.
            negpu= -x + min(0,mx)
            negnonpu= -mn + x
        else: # Plot is on pos. Neg side is empty.
            negnonpu= -mn + min(0,mx) # But still show some empty neg.
    if mx > 0: # There is a positive part to the graph
        if x > 0: # And the plot is on the positive side.
            pospu= x - max(0,mn)
            posnonpu= mx - x
        else: # Plot is on neg. Pos side is empty.
            posnonpu= mx - max(0,mn) # But still show some empty pos.
    nnc= int(negnonpu/upw)*'-'
    npc= int(negpu/upw)*c
    ppc= int(pospu/upw)*c
    pnc= int(posnonpu/upw)*'_'
    return '[%s]' % (nnc+npc+ppc+pnc)

class Client():
    def __init__(self,H=None,p=None,i=None,e=None,t=None,s=None,d=None,vision=False):
        self.vision = vision

        self.host= 'localhost'
        self.port= 3001
        self.sid= 'SCR'
        self.maxEpisodes=1 # "Maximum number of learning episodes to perform"
        self.trackname= 'unknown'
        self.stage= 3 # 0=Warm-up, 1=Qualifying 2=Race, 3=unknown <Default=3>
        self.debug= False
        self.maxSteps= 100000  # 50steps/second
        self.parse_the_command_line()
        if H: self.host= H
        if p: self.port= p
        if i: self.sid= i
        if e: self.maxEpisodes= e
        if t: self.trackname= t
        if s: self.stage= s
        if d: self.debug= d
        self.S= ServerState()
        self.R= DriverAction()
        self.setup_connection()

    def setup_connection(self):
        try:
            self.so= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as emsg:
            print('Error: Could not create socket...')
            sys.exit(-1)
        self.so.settimeout(1)

        n_fail = 5
        while True:
            a= "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"

            initmsg='%s(init %s)' % (self.sid,a)

            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error as emsg:
                sys.exit(-1)
            sockdata= str()
            try:
                sockdata,addr= self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print("Waiting for server on %d............" % self.port)
                print("Count Down : " + str(n_fail))
                if n_fail < 0:
                    print("relaunch torcs")
                    os.system('pkill torcs')
                    time.sleep(1.0)
                    if self.vision is False:
                        os.system('torcs -nofuel -nodamage -nolaptime &')
                    else:
                        os.system('torcs -nofuel -nodamage -nolaptime -vision &')

                    time.sleep(1.0)
                    os.system('sh autostart.sh')
                    n_fail = 5
                n_fail -= 1

            identify = '***identified***'
            if identify in sockdata:
                print("Client connected on %d.............." % self.port)
                break

    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(sys.argv[1:], 'H:p:i:m:e:t:s:f:dhv',
                       ['host=','port=','id=','steps=',
                        'episodes=','track=','stage=', "parameters=",
                        'debug','help','version'])
        except getopt.error as why:
            print('getopt error: %s\n%s' % (why, usage))
            sys.exit(-1)
        try:
            for opt in opts:
                if opt[0] == '-h' or opt[0] == '--help':
                    print(usage)
                    sys.exit(0)
                if opt[0] == '-d' or opt[0] == '--debug':
                    self.debug= True
                if opt[0] == '-H' or opt[0] == '--host':
                    self.host= opt[1]
                if opt[0] == '-i' or opt[0] == '--id':
                    self.sid= opt[1]
                if opt[0] == '-t' or opt[0] == '--track':
                    self.trackname= opt[1]
                if opt[0] == '-s' or opt[0] == '--stage':
                    self.stage= int(opt[1])
                ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
                if opt[0] == '-p' or opt[0] == '--port':
                    self.port= int(opt[1])
                ######## END OF REQUIRED #########
                if opt[0] == '-e' or opt[0] == '--episodes':
                    self.maxEpisodes= int(opt[1])
                if opt[0] == '-m' or opt[0] == '--steps':
                    self.maxSteps= int(opt[1])
                ######### REQUIRED FOR PRACTICE MODE - CAN CHANGE BUT DO NOT REMOVE #########
                if opt[0] == '-f' or opt[0] == '--parameters':
                    global TARGET_SPEED, STEER_GAIN, CENTERING_GAIN, BRAKE_THRESHOLD, GEAR_SPEEDS, ENABLE_TRACTION_CONTROL
                    json_object=json.loads(opt[1])
                    TARGET_SPEED = int(json_object['target_speed'])
                    STEER_GAIN = int(json_object['steer_gain'])
                    CENTERING_GAIN = float(json_object['centering_gain'])
                    BRAKE_THRESHOLD = float(json_object['brake_threshold'])
                    GEAR_SPEEDS = [int(x) for x in json_object['gear_thresholds']]
                    ENABLE_TRACTION_CONTROL = json_object['traction_control']
                    print(f"Running with parameters:\ntarget speed={TARGET_SPEED}\nsteer gain={STEER_GAIN}\ncentering gain={CENTERING_GAIN}\nbrake threshold={BRAKE_THRESHOLD}\ngear speeds={GEAR_SPEEDS}\ntraction control enabled={ENABLE_TRACTION_CONTROL}")
                ########## END OF REQUIRED #########
                if opt[0] == '-v' or opt[0] == '--version':
                    print('%s %s' % (sys.argv[0], version))
                    sys.exit(0)
        except ValueError as why:
            print('Bad parameter \'%s\' for option %s: %s\n%s' % (
                                       opt[1], opt[0], why, usage))
            sys.exit(-1)
        if len(args) > 0:
            print('Superflous input? %s\n%s' % (', '.join(args), usage))
            sys.exit(-1)

    ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
    def get_servers_input(self):
        '''Server's input is stored in a ServerState object'''
        if not self.so: return
        sockdata= str()

        while True:
            try:
                sockdata,addr= self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print('.', end=' ')
            if '***identified***' in sockdata:
                print("Client connected on %d.............." % self.port)
                continue
            elif '***shutdown***' in sockdata:
                print((("Server has stopped the race on %d. "+
                        "You were in %d place.") %
                        (self.port,self.S.d['racePos'])))
                self.shutdown()
                return
            elif '***restart***' in sockdata:
                print("Server has restarted the race on %d." % self.port)
                self.shutdown()
                return
            elif not sockdata: # Empty?
                continue       # Try again.
            else:
                self.S.parse_server_str(sockdata)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H") # Clear for steady output.
                    print(self.S)
                break # Can now return from this function.
    ########## END OF REQUIRED #########

    def respond_to_server(self):
        if not self.so: return
        try:
            message = repr(self.R)
            self.so.sendto(message.encode(), (self.host, self.port))
        except socket.error as emsg:
            print("Error sending to server: %s Message %s" % (emsg[1],str(emsg[0])))
            sys.exit(-1)
        if self.debug: print(self.R.fancyout())

    def shutdown(self):
        if not self.so: return
        print(("Race terminated or %d steps elapsed. Shutting down %d."
               % (self.maxSteps,self.port)))
        self.so.close()
        self.so = None

class ServerState():
    '''What the server is reporting right now.'''
    def __init__(self):
        self.servstr= str()
        self.d= dict()

    def parse_server_str(self, server_string):
        '''Parse the server string.'''
        self.servstr= server_string.strip()[:-1]
        sslisted= self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w= i.split(' ')
            self.d[w[0]]= destringify(w[1:])

    def __repr__(self):
        return self.fancyout()
        out= str()
        for k in sorted(self.d):
            strout= str(self.d[k])
            if type(self.d[k]) is list:
                strlist= [str(i) for i in self.d[k]]
                strout= ', '.join(strlist)
            out+= "%s: %s\n" % (k,strout)
        return out

    def fancyout(self):
        '''Specialty output for useful ServerState monitoring.'''
        out= str()
        sensors= [ # Select the ones you want in the order you want them.
        'stucktimer',
        'fuel',
        'distRaced',
        'distFromStart',
        'opponents',
        'wheelSpinVel',
        'z',
        'speedZ',
        'speedY',
        'speedX',
        'targetSpeed',
        'rpm',
        'skid',
        'slip',
        'track',
        'trackPos',
        'angle',
        ]

        for k in sensors:
            if type(self.d.get(k)) is list: # Handle list type data.
                if k == 'track': # Nice display for track sensors.
                    strout= str()
                    raw_tsens= ['%.1f'%x for x in self.d['track']]
                    strout+= ' '.join(raw_tsens[:9])+'_'+raw_tsens[9]+'_'+' '.join(raw_tsens[10:])
                elif k == 'opponents': # Nice display for opponent sensors.
                    strout= str()
                    for osensor in self.d['opponents']:
                        if   osensor >190: oc= '_'
                        elif osensor > 90: oc= '.'
                        elif osensor > 39: oc= chr(int(osensor/2)+97-19)
                        elif osensor > 13: oc= chr(int(osensor)+65-13)
                        elif osensor >  3: oc= chr(int(osensor)+48-3)
                        else: oc= '?'
                        strout+= oc
                    strout= ' -> '+strout[:18] + ' ' + strout[18:]+' <-'
                else:
                    strlist= [str(i) for i in self.d[k]]
                    strout= ', '.join(strlist)
            else: # Not a list type of value.
                if k == 'gear': # This is redundant now since it's part of RPM.
                    gs= '_._._._._._._._._'
                    p= int(self.d['gear']) * 2 + 2  # Position
                    l= '%d'%self.d['gear'] # Label
                    if l=='-1': l= 'R'
                    if l=='0':  l= 'N'
                    strout= gs[:p]+ '(%s)'%l + gs[p+3:]
                elif k == 'damage':
                    strout= '%6.0f %s' % (self.d[k], bargraph(self.d[k],0,10000,50,'~'))
                elif k == 'fuel':
                    strout= '%6.0f %s' % (self.d[k], bargraph(self.d[k],0,100,50,'f'))
                elif k == 'speedX':
                    cx= 'X'
                    if self.d[k]<0: cx= 'R'
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k],-30,300,50,cx))
                elif k == 'speedY': # This gets reversed for display to make sense.
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k]*-1,-25,25,50,'Y'))
                elif k == 'speedZ':
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k],-13,13,50,'Z'))
                elif k == 'z':
                    strout= '%6.3f %s' % (self.d[k], bargraph(self.d[k],.3,.5,50,'z'))
                elif k == 'trackPos': # This gets reversed for display to make sense.
                    cx='<'
                    if self.d[k]<0: cx= '>'
                    strout= '%6.3f %s' % (self.d[k], bargraph(self.d[k]*-1,-1,1,50,cx))
                elif k == 'stucktimer':
                    if self.d[k]:
                        strout= '%3d %s' % (self.d[k], bargraph(self.d[k],0,300,50,"'"))
                    else: strout= 'Not stuck!'
                elif k == 'rpm':
                    g= self.d['gear']
                    if g < 0:
                        g= 'R'
                    else:
                        g= '%1d'% g
                    strout= bargraph(self.d[k],0,10000,50,g)
                elif k == 'angle':
                    asyms= [
                          "  !  ", ".|'  ", "./'  ", "_.-  ", ".--  ", "..-  ",
                          "---  ", ".__  ", "-._  ", "'-.  ", "'\.  ", "'|.  ",
                          "  |  ", "  .|'", "  ./'", "  .-'", "  _.-", "  __.",
                          "  ---", "  --.", "  -._", "  -..", "  '\.", "  '|."  ]
                    rad= self.d[k]
                    deg= int(rad*180/PI)
                    symno= int(.5+ (rad+PI) / (PI/12) )
                    symno= symno % (len(asyms)-1)
                    strout= '%5.2f %3d (%s)' % (rad,deg,asyms[symno])
                elif k == 'skid': # A sensible interpretation of wheel spin.
                    frontwheelradpersec= self.d['wheelSpinVel'][0]
                    skid= 0
                    if frontwheelradpersec:
                        skid= .5555555555*self.d['speedX']/frontwheelradpersec - .66124
                    strout= bargraph(skid,-.05,.4,50,'*')
                elif k == 'slip': # A sensible interpretation of wheel spin.
                    frontwheelradpersec= self.d['wheelSpinVel'][0]
                    slip= 0
                    if frontwheelradpersec:
                        slip= ((self.d['wheelSpinVel'][2]+self.d['wheelSpinVel'][3]) -
                              (self.d['wheelSpinVel'][0]+self.d['wheelSpinVel'][1]))
                    strout= bargraph(slip,-5,150,50,'@')
                else:
                    strout= str(self.d[k])
            out+= "%s: %s\n" % (k,strout)
        return out

class DriverAction():
    '''What the driver is intending to do (i.e. send to the server).
    Composes something like this for the server:
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus 0)(meta 0) or
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus -90 -45 0 45 90)(meta 0)'''
    def __init__(self):
       self.actionstr= str()
       self.d= { 'accel':0.2,
                   'brake':0,
                  'clutch':0,
                    'gear':1,
                   'steer':0,
                   'focus':[-90,-45,0,45,90],
                    'meta':0
                    }

    def clip_to_limits(self):
        """There pretty much is never a reason to send the server
        something like (steer 9483.323). This comes up all the time
        and it's probably just more sensible to always clip it than to
        worry about when to. The "clip" command is still a snakeoil
        utility function, but it should be used only for non standard
        things or non obvious limits (limit the steering to the left,
        for example). For normal limits, simply don't worry about it."""
        self.d['steer']= clip(self.d['steer'], -1, 1)
        self.d['brake']= clip(self.d['brake'], 0, 1)
        self.d['accel']= clip(self.d['accel'], 0, 1)
        self.d['clutch']= clip(self.d['clutch'], 0, 1)
        if self.d['gear'] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d['gear']= 0
        if self.d['meta'] not in [0,1]:
            self.d['meta']= 0
        if type(self.d['focus']) is not list or min(self.d['focus'])<-180 or max(self.d['focus'])>180:
            self.d['focus']= 0

    def __repr__(self):
        self.clip_to_limits()
        out= str()
        for k in self.d:
            out+= '('+k+' '
            v= self.d[k]
            if not type(v) is list:
                out+= '%.3f' % v
            else:
                out+= ' '.join([str(x) for x in v])
            out+= ')'
        return out
        return out+'\n'

    def fancyout(self):
        '''Specialty output for useful monitoring of bot's effectors.'''
        out= str()
        od= self.d.copy()
        od.pop('gear','') # Not interesting.
        od.pop('meta','') # Not interesting.
        od.pop('focus','') # Not interesting. Yet.
        for k in sorted(od):
            if k == 'clutch' or k == 'brake' or k == 'accel':
                strout=''
                strout= '%6.3f %s' % (od[k], bargraph(od[k],0,1,50,k[0].upper()))
            elif k == 'steer': # Reverse the graph to make sense.
                strout= '%6.3f %s' % (od[k], bargraph(od[k]*-1,-1,1,50,'S'))
            else:
                strout= str(od[k])
            out+= "%s: %s\n" % (k,strout)
        return out

def destringify(s):
    '''makes a string into a value or a list of strings into a list of
    values (if possible)'''
    if not s: return s
    if type(s) is str:
        try:
            return float(s)
        except ValueError:
            print("Could not find a value in %s" % s)
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]

def drive_example(c):
    '''This is only an example. It will get around the track but the
    correct thing to do is write your own `drive()` function.'''
    S,R= c.S.d,c.R.d
    target_speed=160

    R['steer']= S['angle']*25 / PI
    R['steer']-= S['trackPos']*.25

    R['accel'] = max(0.0, min(1.0, R['accel']))
    

    if S['speedX'] < target_speed - (R['steer']*2.5):
        R['accel']+= .4
    else:
        R['accel']-= .2
    if S['speedX']<10:
       R['accel']+= 1/(S['speedX']+.1)

    if ((S['wheelSpinVel'][2]+S['wheelSpinVel'][3]) -
       (S['wheelSpinVel'][0]+S['wheelSpinVel'][1]) > 2):
       R['accel']-= 0.1



    R['gear']=1
    if S['speedX']>60:
        R['gear']=2
    if S['speedX']>100:
        R['gear']=3
    if S['speedX']>140:
        R['gear']=4
    if S['speedX']>190:
        R['gear']=5
    if S['speedX']>220:
        R['gear']=6
    return


# =====================================================================
# YARIŞ ÇİZGİSİ SÜRÜCÜSÜ (racelab/planner/build_line_submission.py ile üretildi)
# Plan: pistin geometrisinden önceden hesaplanmış yarış çizgisi + hız profili (2 m aralık).
# Canlı: konum (distFromStart, trackPos, angle) → direksiyon; hız hedefi → gaz/fren; ABS, patinaj, vites kuraldır.
# Parametreler: racelab/planner/runs/snap_gr3L.json
# =====================================================================
import base64 as _b64
import math as _math
import zlib as _zlib
from array import array as _array

_LP = {'offset': 0.0, 'ctrl': 'st', 'st_kh': 1.36583, 'st_ke': 1.298866, 'st_kff': 1.995787, 'st_kd': 0.065858, 'st_ah': 5.94596, 'st_aff': 0.949604, 'st_kus': 0.000515, 'st_lp': 1.0, 'dt_real': False, 'ld0': 8.9137, 'ld_v': 0.23176, 'k_steer': 2.54163, 'k_ff': 1.0, 'look_t': 0.068171, 'band': 4.116862, 'k_brake': 0.02984, 'tcs': 8.061355, 'upshift': 19517.138089, 'max_gear': 6, 'tcs_lam': 0.0, 'tcs_k': 5.0, 'pred': 0.0, 'st_course': 0.055873, 'st_csign': 1.0, 'st_slipmax': 0.0, 'g2_v': 44.097307, 'dn_off': 2000.0, 'launch': 0, 'launch_up_v': 30.0, 'st_lowv': 0.0, 'st_lowv_min': 0.15, 'cl_max': 0.66, 'cl_top': 18155.190224, 'cl_band': 1500, 'cl_shift': 0.16029, 'tcs_lv': 160, 'tcs_l': 100}
_LN = 1804
_LL = 3607.966206329104
_DS = 2.0
_WB = 2.95
_SLOCK = _math.radians(21.0)
_WHEEL_R = [0.3179, 0.3179, 0.3307, 0.3307]
_GEAR_RATIOS = [3.9, 2.9, 2.3, 1.87, 1.68, 1.54, 1.46]
_PLAN_B64 = (
    "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"
    "K0SNDrl9z3PFV8mgRVpcV9ydyr2c6e2KO44N2ZPHXCGz56uQ4W5XZC5YdaJA0BUCrxQWz/3tAh0dTT6rbhesE40sKX1ArL+JKwZL"
    "cl1wo6l7xIFN+EI3c4FyT6KuFBXPr0/4xqUX53rKuWCmqzqiWsAFDhXp1UKTzjhmGpfg3+UMlZEoszpiXcvE+4hsynYGtzovNoxJ"
    "+LeKRFabO+FvS0MSpXUJX1dge4op64zPq0PM3qwi6m9dxuQnnHAg3yyZ2+mETK0uh/f3nOCasPnGvkwnyDx1WnCC7oQx/nzzIVcn"
    "KNw8seuQthPsybTm0zucwNUM8vzK54Qbq5wXqf9whNbwKvHzrxzxsu/3tp8VjnDo6t1leMERw88a9l4h9rPPmwz3zHNxxJqhQ22W"
    "mo5QuSHvWyLtCPcY8WWLVjoikZM7ZjdKgknjoPrtlySsX7EjfWU5Ce8NAj65ZZBw0cCr+WEsCdKaxi82OJFwW3V/V4gGCZo8nhee"
    "biPBv9Fqv8RyEs6k6nREjzjgof2BkOfPHbDvqoLG9tsO6O5/WUk96wDa5lCFzhgHSFoKFu0iOSAj+3wZR80BNrV9GT2SDhD+tI2u"
    "vMwB5RveP5PotoeSk/fFjhx7lBdOesV52kN5lLHvgJw97jsmKwz9tINU9PJlqVV2SEmNew+mHcbiDld/07FD0LOWpKxVxFjI2tq4"
    "0xavBbUSJHJtsSPMdWoJ2RbUV3TfL+a2eLE3612rnC2cSnuZT/fYItRFpZKzzxYcobM/Dh2wBe1Q4USEgy3Gz7bNCTluC7cfk0v9"
    "q2yxXL95fCuPHcobzmv+NrWDy1Hfcy2FdtBacTH27FJ7KFLWuKVF2kN8NF4/5ZM9vFb4fj0m64C9q6xFPXwdME9Q04RG9KubRxc7"
    "VQ/PUob0H71neXWWw7OUcZytz/LqLIdnKeOE6//qTij9V3dCyb+6E0JXzSPqzqDz/HKs9nbG8ZGvjtVXnZHx7oNj9bAzRpl9jtUy"
    "RI76P72J/PJ/XnXB1/9zeHYs4zpbd/3v/Kuu+P6vPntcxu2/87zd/tNdnR0Pu83W3f8739t9Vu/+33Eip/43v8d/Ou9ZXvWY1XvM"
    "6j1n9Z7/ja96zt6fJ778q3th+F/d67/xVa9Zvdes3ntW742xf3VvjP+re+Pnv7rPf/Se5dVZDs9SxheT/+q++P2v7os//+q+mPpX"
    "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"
    "k+FNqXd/6VBMmD8WuTwFU2Lc1S1sKejQUt0ayJ2CIgd1lndrUtDvaZ56Z0MKTBZGxlmEU/D+yjsG65YUtAi7H4kST0HvQGZ8mkQK"
    "krUDrbWlUyCreijzwo4UsKbI/EiTS0GOzn0lJfkU/DHlekRTSEGw5eRg2q4UHAsJXH1ydwoKWxmbuwnGNi+rWrYnBV4TYfwfibFI"
    "UZC6P0GpWUp39f/nz+6fvUPoo2UtP1kT6607/ojrCbH+b4u+MyPE+8SlHla9IN7fICkrHCuZgpfDtUUs21Pg/ZIj9eTWFOw3WXCh"
    "i6agnELRySb2sZqF5BErSPh/kTKjtz4FnVv38Xwn9nu6f9+EJ28Ktud87GrmTIHYvqo1K9hTcFefeZPQyhRciJi4zM+cghPCRZe/"
    "ztOR66S+Pm+WjqBbggtKxLnypdqZlY3TcVyzWYBnjA4Z390OR4bpELjSfSL0Ix03ZCmzCcR3yaZeVw15TcfKwB5L0y46WJqU1m5q"
    "p8OoYky6tYmOn7TlW72I7ypztX5kro6OtNgbUUHVdNBnHsYNlNExE3v3p2IJHX4NjRbB1+koOLk4fSWfDjHPvDO1OXT4Z/dw16fT"
    "8cK53rr4NB3GmVan4xLpsM/nWDgcQ/xeHW6VLYugE/kleOWVQDpeap3S3k2mg71RoqbMlQ7HzyKpm+2J/eT/GQ61pMOXa3r4MfH7"
    "fb116vG8IR0lpIDb4jp08Fxi/bT/EB2vOpkj1JXpiDYhzyjtosOhYEhikwwdttVHr0xspWPk+wrHSuKepISvfuIqQEftxgf+XNx0"
    "DC8IaV4l7tMu6UNrZJjo2Pqx5H7BDHF/876ochL37jaLmZrL12TURcZZlw8QbDcNnXyTjNKDr+O3vCDigevRUo3mZPzMcr1/4lEy"
    "nj6pDyPVJOOLPft5izIiPnzeul+3KBnuTx0fS11OhuJdWYXFrGSYD7Pff3I2Ga2NvU+oicnwfTnGuotCrOf4d7wnOBndEl4p3mTi"
    "3mfNBsw7E3HlQJZGtE0ywrUF8mfMkpF9panGyTAZCVx8as2aydibuE5o6wFinDWbFbA7GaPsHaiVToZS0+rqmS3JWHDafU92YzI6"
    "1neUWvEmIyBkHwd1FeHfpKnyxcUkBBYkc1cScbFqRX/5k29JuCevtq/jUxK65WcuvehNgsyBQ71tL5Lwp7Bq4uGzJJwaGd90m5GE"
    "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"
    "mff+7nL9XTk/Lgu9fVmv7As2jku5VDtz5z9fStb5dAu1PjBaf39uvi3hZN6/0i3lUuX+kX8ZX823byljl/E/Vf9y9S41z/8YZMrO"
    "/2PuPDLv0XeJZ+oWUbpQfqqDwnMCzipRFJYl8crULCLVxIWKrVg9fxxWTSkRWUQDk74sGgoHHCRiYBGDZg6kqlDo/O3wJ8IiZr37"
    "cUGLwt3z+/cfU5UNB6Uo1DPZsJOfQtINszu2FH6L8/1KvYh3X/DS6WH9/J/aZn55MvAvvppPt5TuZt6VEYvIXljx5PDc9wBLGDff"
    "7qUMnC9noUv1/49L1ec8396lXKq826lbQjn+4nL9eDafbqHL5UuYT7fQh7skra//xWrB0eG/efHUXcsErL9LSR+gtm/unLGULIRj"
    "2ZZ/ccVDDgeRv3iwU+3A2r/42Llk7edF9LqIEmT/opLsxUr2v2j6vC776V9cc/1f8f5S7n/8Msn837Bx/jyxlEvlW2eFX7PpPyiv"
    "RdSe139xufz/br+3B2y5NLc/L/Sfjtd/13+339W7/5VuKbdqh/+cex4t5T8tf7l+LTePA+bTLTQwd13W3PcTf/QYblKzplAr/fuG"
    "WxQmdt1N0Z873zGZ3Jp7Xv8xwrBt82sKX24Qc/ahkNHTcDSQwj4ZfY1CCkvE9Gnnvle9cCzn4B7s7z9GrHYh+VGYbut54iOFfvd4"
    "Xw1SyMGzmoYJK+ePaXuff57b3/54tDfSY+5cpq3r7M5LoZXfNilGCm3K1YzoKWScFqvWprBj/F/xxB+9sz0L5+IZmzeu9TQUWnA7"
    "FZ6i0O11Cf8mCr9XBXxgpPCqn479OgrFttEYS1MoePz0wP86pxY3HRCh0H77rvNz/f+jrP8er7nvR/7IKJizVotC0wiT8HIs3R/v"
    "jj/tT8UM4dgRX7aIKTWWDx3/4lL5/t384/Pju5Tl8+mWUnQ+7lvKf1rucu1abjz+p+3575a7nEuVu9z9EQ+Vv3xhEf/d9ixV33Lj"
    "3zmfbqHL5Vuun18IVUGVi7hcvv/0+vin4/Rovv1L+d+tp1ZMqbiYwv9Uf/7p/F1ufaSteM3OOfc8WGCuUPwWRGFDb2FOwSK+eKa0"
    "unERr8qfw3+mkGEzf9vc+9GFbujD7yn/i0LRjc5mWP6lXC6/hIWyKukvdu3+u8vlH939d+Ut/q7tfDsXanz6EsNpCg2aqN4+X8Rr"
    "7vCYhsLBzjUGNxexDs67hP0DZw55+uyc+95pgb8EiiLrsc+XUn3+/LJQofnvRZdypqJBMWiu3iUsWPXyudrce/gFNv4mdAf9xckf"
    "q4M1/mI8+vhB+C/uiAhmSsPqX2g3P8k46y+uTdsJClj+pSR2lazj/AdK4neL5/5Fh7bS4rl4ZylJy3i/M34o5N/w6fw+stDYFCVz"
    "oUUM9FcqWfEXmUdPi7yf+/3BAtV+8X6bew+ylBp7v9GLL6IfbbZbx1/MXcda77SIJYxd0np/saUv/Yj5XxR5HR5Dt4hxm6rZ5r7P"
    "W8rWe1P3Jefe2yywbP7zf2rcfDn/xWXas1R5pfP5lzJuOZeor24Jq6tcr5lR+NhWYvfcvHi5247mIoXqIEWzm8Ln5aKBLyn8Mn+O"
    "+uMGJslrhnPxgNdgMplCgeeSUkUU2pRkPJ+7738UZSONi8zFsfOum0/3x4XlLaXlAluC/pX/j8vVs7CdC8s/Mt/eMGvqQBMKqTcf"
    "I7BRKJSf64EozDooUDv3vk4jXCb1FoVFp7eXXKEwuGy8Q4LCisQX2nO/T5qMOcp4k0LB/TOvcylE8/vbHxlbn0zNndM+eylVv/1v"
    "qCi7wVqRws1et4WdKXzkmb0Xh7neIrKbisLy6SzvuefLHwXYMkPm4vgvxzfDfQrl203CrlOY0u2Xe5VCpnaVPmasvEif0TtD2N8L"
    "9eaWf0CeOxcv4QmiZo7RX6Q9mZDZPfdebQkd59MtdLl8VHFb1dfPvQ+Yd/uBX4Zz1xcqHGV9cnIROx1tXrUt4i3c9z2vKXRcdZXv"
    "4SIOP/Nbe23uvcO8tcHpXdpzcfW8oY53i+beh1/BabF8pVDA2eG8G4XxSXEjEpjXQ96yGVNIU1CW+IVCR/Wpurl6hW/1+flQuK/1"
    "qckeCv3duy5e+TdkbivlalhEqiNO3zoX0Sk8nvbHIs44ru2Zm1cLvUG4mzM3/guV9r/zee49/UKTC395s1G4fUP4GpZF/BF+m3Pu"
    "fdJCT46Pdc4uYsJTDY+574MWWu9A1TXXnj8WRWeHze0fC33loeY6PvceZYFrtdUfzX1ftdBQZwUlvkUsk/Bxmfu93kIvoUydb4tY"
    "H6h9KG4RyV1PJEsX8UPChR+Zi+jiHOWWs4gecqFXqyj0TaK53/UXddf3qc3tsws9JB8p9/IvLpXvcaV41Nx5YqFLpf/jsJ2HwaVF"
    "NJcp/sD5byjiG20zFxcvlGMZifP5l9JyGetffJzdsoiNXsyic7/HWMo6nsdVFn9xqXxUvP/6fCm3zPdroVPz+ZdyqfH74z9tz3lC"
    "0wc5bDz+aHm1/sOGRbw0/3uihX6T5XKcW68LVbReXT8XVyz06SEWF61FJDC4psy97/jjYCL3Hs+53xEukDD+cu+pRVz9cqJh7nuN"
    "hT693ewy9z3yH3fJm8bOrc+FSplrrniziOLlKTxz++1COfjSsrT+IvW/ac7hVX5z3yst5XL1T6z8u0vlY1xGx4DpC65/kfG/Wc5S"
    "Ljcuy+V3n2/HUi43LsuNu+r8/V+o+L/pB/fxhrnfuS3lcvk3L+Ny/Y6fr2ehy9XrkxZ+cW69L+XtJcyxoRG5tYg0ElZPB/+i9Hy9"
    "Cw18umrV1Nx7uwWeOej0OnIRtzy7wTXzF5kZ1RyiFlFUL2Dlt0Wk+l//xP6LXwwUbqr9P2Duf8j/r7V3qXH+4+quRvlVf5EcFB45"
    "F48tJe1LqQufFnG5chXjcs/NpfunhqrutIz9iyrz6ZZyuX79p9r1n86/a779S/nvjvs/NWK+/Uu5XP7l7hf1/Lxbyn+3/cvNp6X6"
    "tVR6KeXmo0fmfv9J5fa1hcI//5/Q7kj+DfHaGNQvH+up9TEW2WR0nB73f4eiDglsVnB9j1ZFvMssPxKHAi+4OqkZxCO7749TNLck"
    "oLj3ak5qnInodB/PT3J3ImrjiNpQmvkBrT3ua3fI9yO6zxirqL03CbHPvo/WXpOMzL5kB3RWJaMwM2LFY58UpNI28GyXQioSaxcr"
    "5q9NRbYGxxxWeX5C9ZG/15XLpKHwzNmgopo05K1+9nyZejp6c/DSNcLjdOQdV9BC152OEtyiqIXhM2raoxj89N5npI+73ezW8Bkp"
    "yHxJPCGdgTL5lVHMlQxUo/kp5WdRBjp6dGjq9tpMxK/ta3vaLhMlCWzasT8jEwn3Ngrocn9Bz7fMrh6z/oJ2fE2hE4z/giLXw0sX"
    "uiw0uL96BW5nFnrpIpjk9iYLnTw4O3RxLAu58sp+/q2bjRSk5I9+eZyNrp6MnKZrz0YXDk6QJuRzUMmeHka3GzkoosbopFxlDpJr"
    "Iu5qEc5FakOyb33O5iJe/M6wkfRcNJOcOHCGIw/dtc68WbsvD+X3f3x1ODoPfYpzWE+czUOWw9IfDE2+osiXN6WVgr4iRQG/16t7"
    "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"
    "Cvk5Bjye2eabuAzhUaa69cPYwWT8fFLMdcj4v4ruYyTTI+Ztm8YM4lGwbWVIL1IfWTs0V/bQIfVl+k7GDuSxradg8yBtHhEGDjMd"
    "SB1N3vFj8lItHjuvmXbd059Himh2Zlk/HidbY7SUide7fJ9g25fHm/7HN6SR+lt16ZCvhNjn9W6v2D48/IcfOy8hRr4UVcZp8Nhy"
    "s3GLhKhfyo7bo87jqbXq+f7E/Q8LIrJ5Hr4Leoe7EI3Gn1b0Jn7QmnbnKqlv+4FVFruILtHNeUHEi5FDW12JP2wzT9gT072iJ9sQ"
    "gwp3hNgRdb48P+JNjF25zzuMuM9314H9xJnRA0aWEE9kxGl0IcfXPNTr3Tji3Kk5fTYSPx4ynPuKmP2HWDiTXJeu+YJDb4jXLSzf"
    "RpLrlkyusbUg7YmrtcoWknY+unNFtQtpv9U8/e5KpD8mqcwYJCf91CyO1plE+q01LeXKDtKPs3fue9hI+tVpaW7MXNLfmt6Fez6Q"
    "/k9/qojaQMYlrsiudSgZL+dJc1fUkPELLxQFJpBxPdLudNODjPeG2+U3dEgd5OwOif9B6mLZsMb+b0idrDteMuI1qZ+Bs33dO0hd"
    "FfOXzxqRekuNOlUdRepwebcr/d+T+nw8QCvLndTxjZnS5fdJfX8+uO3ATHJf1A42H/59Mo/3+spWJ+3J/dpZ+vsyRx7Tzsz87DSV"
    "h3fq86H203kEJF6bOcuV1EEwn7jRg7S726c7+V48Jnp029jdh/8/v/9LQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0ND"
    "Q0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0ND86+XZkvq/+voPxyj9yy+R6VSqVQqlfq3c/3L0r35CUVUKpX6lzX+99RRqVQq9Z/cvL4b3jvl"
    "1VKpVCqV+i9h5qNZIY59S6hUKpVKpf4F6b6CSqX+V0m/X0e/v0j9+0o/P6X+T/gfP69bT6VSqf+J9PcaqFQqlUql/s95q+LPNFKp"
    "VCqVSqX+7aSfY1Gp1H+U9HMrKpVKpVKpVCqV+s/m49tfneuO3KRSqVQqlfoX7F19eMnY3tVUKpX6D5d+PkM/T6JSqdT/W7o+UqlU"
    "KpVKpVKpfx/p7xP+/210+vNf5vlKpVKpVCqV+rdTacgm+ePbH6hUKvUvK/j3PzRTqVQq9Z/cmOLPejNjqFQqlUr917BYoeusfPgX"
    "lUqlUqnUv6Dcb0rrDucmKpVK/Yd7cfKf37H6TP1f5vMk0ZHCK9+o/+T+x/9j2EKl/rdKfw6ISqX+Z/750y1fnd9SqVQqlUr9b/Df"
    "ACiGnwc="
)


def _plan_load():
    raw = _zlib.decompress(_b64.b64decode("".join(_PLAN_B64)))
    a = _array("d")
    a.frombytes(raw)
    cols = {}
    names = ['Cx', 'Cy', 'theta', 'Nx', 'Ny', 'Px', 'Py', 'n', 'k', 'v', 'w', 'theta_p', 'ilc', 'kh_s', 'ke_s', 'ff_s']
    for i, nm in enumerate(names):
        cols[nm] = list(a[i * _LN:(i + 1) * _LN])
    return cols


_PL = _plan_load()


def _lp_interp(key, s):
    x = (s % _LL) / _DS
    i = int(x) % _LN
    f = x - int(x)
    j = (i + 1) % _LN
    arr = _PL[key]
    return arr[i] * (1 - f) + arr[j] * f


def _lp_theta(key, s):
    x = (s % _LL) / _DS
    i = int(x) % _LN
    f = x - int(x)
    j = (i + 1) % _LN
    a, b = _PL[key][i], _PL[key][j]
    d = (b - a + _math.pi) % (2 * _math.pi) - _math.pi
    return a + d * f


def _lp_gear(gear, rpm, vx):
    up = _LP["upshift"]
    gear = int(gear)
    if gear < 1 or vx < 5:
        g = 1
    elif gear < 7 and rpm > up:
        g = gear + 1
    elif gear > 1 and rpm < up * _GEAR_RATIOS[gear - 1] / _GEAR_RATIOS[gear - 2] - _LP["dn_off"]:
        g = gear - 1
    else:
        g = gear
    if _LP["g2_v"] and vx > _LP["g2_v"]:
        g = max(g, 2)
    return min(g, int(_LP["max_gear"]))


_LP_STATE = {"prev_heading": None, "prev_steer": 0.0, "since_up": 99, "clutch": 0.0}


def _lp_wheel_rpm(S, g):
    w = S['wheelSpinVel']
    if g < 1:
        return 0.0
    return (w[2] + w[3]) / 2 * _GEAR_RATIOS[g - 1] * 4.5 * 60 / (2 * _math.pi)


def line_drive(c):
    S, R = c.S.d, c.R.d
    p = _LP
    s = S['distFromStart'] + p["offset"]
    v = max(S['speedX'] / 3.6, 0.0)
    half_w = _lp_interp("w", s) / 2
    cx, cy = _lp_interp("Cx", s), _lp_interp("Cy", s)
    nx, ny = _lp_interp("Nx", s), _lp_interp("Ny", s)
    carx, cary = cx + S['trackPos'] * half_w * nx, cy + S['trackPos'] * half_w * ny
    heading = _lp_theta("theta", s) - S['angle']
    yaw_rate = 0.0
    t_now = S.get('curLapTime', 0.0)
    dt = 0.02
    if p["dt_real"] and _LP_STATE.get("prev_t") is not None and 0.0 < t_now - _LP_STATE["prev_t"] < 0.1:
        dt = t_now - _LP_STATE["prev_t"]
    _LP_STATE["prev_t"] = t_now
    if _LP_STATE["prev_heading"] is not None:
        yaw_rate = ((heading - _LP_STATE["prev_heading"] + _math.pi) % (2 * _math.pi) - _math.pi) / dt
    _LP_STATE["prev_heading"] = heading
    if p["ctrl"] == "st":
        n_car = S['trackPos'] * half_w
        if p["pred"]:  # gecikme telafisi: durumu pred saniye ileri tahmin et
            s = s + v * p["pred"]
            n_car = n_car + v * _math.sin(-S['angle']) * p["pred"]
            heading = heading + yaw_rate * p["pred"]
        e = _lp_interp("n", s) - n_car
        th = _lp_theta("theta_p", s + p["st_ah"])
        beta = _math.atan2(p["st_csign"] * S.get('speedY', 0.0), max(S['speedX'], 5.0))  # kayma açısı
        beta = max(-p["st_slipmax"], min(p["st_slipmax"], beta))
        dh = (th - (heading + p["st_course"] * beta) + _math.pi) % (2 * _math.pi) - _math.pi
        kp = _lp_interp("k", s + p["st_aff"])
        fkh, fke, fff = _lp_interp("kh_s", s), _lp_interp("ke_s", s), _lp_interp("ff_s", s)
        delta = (p["st_kh"] * fkh * dh + _math.atan2(p["st_ke"] * fke * e, v + 5.0)
                 + p["st_kff"] * fff * _math.atan(_WB * kp) - p["st_kd"] * (yaw_rate - v * kp) * 0.1
                 + p["st_kus"] * v * v * kp + _lp_interp("ilc", s))
    else:
        ld = p["ld0"] + p["ld_v"] * v
        tx, ty = _lp_interp("Px", s + ld), _lp_interp("Py", s + ld)
        dx, dy = tx - carx, ty - cary
        lx = _math.cos(heading) * dx + _math.sin(heading) * dy
        ly = -_math.sin(heading) * dx + _math.cos(heading) * dy
        alpha = _math.atan2(ly, max(lx, 0.1))
        delta = _math.atan(2 * _WB * _math.sin(alpha) / max(_math.hypot(lx, ly), 1.0)) * p["k_steer"]
        k_ahead = _lp_interp("k", s + 0.5 * ld)
        delta += (p["k_ff"] - 1.0) * _math.atan(_WB * k_ahead)
    steer = max(-1.0, min(1.0, delta / _SLOCK))
    if p["ctrl"] == "st":
        steer = _LP_STATE["prev_steer"] + p["st_lp"] * (steer - _LP_STATE["prev_steer"])
        _LP_STATE["prev_steer"] = steer
    vt = min(_lp_interp("v", s), _lp_interp("v", s + v * p["look_t"]))
    err = v - vt
    if err > p["band"]:
        accel, brake = 0.0, min(1.0, p["k_brake"] * (err - p["band"]) + 0.3)
    else:
        accel, brake = (1.0 if err < -0.5 else max(0.0, min(1.0, 0.5 - err))), 0.0
    w = S['wheelSpinVel']
    spin = (w[2] * _WHEEL_R[2] + w[3] * _WHEEL_R[3]) - (w[0] * _WHEEL_R[0] + w[1] * _WHEEL_R[1])
    if p["tcs_lam"] > 0:  # kayma oranı tabanlı patinaj kontrolü
        vr = (w[2] * _WHEEL_R[2] + w[3] * _WHEEL_R[3]) / 2
        vf = (w[0] * _WHEEL_R[0] + w[1] * _WHEEL_R[1]) / 2
        lam = (vr - vf) / max(vf, 3.0)
        if lam > p["tcs_lam"]:
            accel = max(0.0, accel * (1.0 - p["tcs_k"] * (lam - p["tcs_lam"])))
    else:  # patinaj kontrolü (eşik); kalkışta daha yüksek eşik
        th = p["tcs"]
        if p["tcs_lv"] and S.get('distRaced', 1e9) < 500 and S['speedX'] < p["tcs_lv"]:
            th = p["tcs_l"]
        if spin > th:
            accel = max(0.0, accel - min(accel, (spin - th) * 0.1))
    if brake > 0 and S['speedX'] >= 10:  # ABS
        slip = S['speedX'] / 3.6 - sum(x * r for x, r in zip(w, _WHEEL_R)) / 4.0
        if slip > 2.0:
            brake = brake - min(brake, (slip - 2.0) * 0.1)
    if S['speedX'] < 10 and accel > 0:
        accel = max(accel, 0.5)
    rpm_eff = S['rpm']
    if _LP_STATE["clutch"] > 0 and int(S['gear']) >= 1:  # debriyaj kayarken tahrik tekerleği devri
        rpm_eff = _lp_wheel_rpm(S, int(S['gear']))
    gear = _lp_gear(S['gear'], rpm_eff, S['speedX'])
    clutch = 0.0
    if p["cl_max"] > 0 and accel > 0 and brake == 0 and gear >= 1:  # debriyaj kaydırma (pedal ≤ 2/3 tam tork)
        clutch = p["cl_max"] * min(1.0, max(0.0, (p["cl_top"] - _lp_wheel_rpm(S, gear)) / max(p["cl_band"], 1.0)))
        if _LP_STATE["since_up"] < p["cl_shift"]:
            clutch = max(clutch, p["cl_max"])
    _LP_STATE["since_up"] = 0 if gear > int(S['gear']) >= 1 else _LP_STATE["since_up"] + 1
    _LP_STATE["clutch"] = clutch
    if p["launch"]:  # geri sayımda boşta tam gaz, direksiyon düz; kalkışta erken 2. vites
        if S.get('curLapTime', 0.0) < 0:
            gear, accel, brake, steer = 0, 1.0, 0.0, 0.0
        elif S.get('distRaced', 1e9) < 300 and S['speedX'] > p["launch_up_v"]:
            gear = max(gear, 2)
    if p["st_lowv"] > 0 and v < p["st_lowv"]:  # düşük hızda direksiyon sınırı
        steer *= max(v / p["st_lowv"], p["st_lowv_min"])
    R['steer'], R['accel'], R['brake'] = steer, accel, brake
    R['gear'] = gear
    R['clutch'] = clutch


def drain_latest(c):
    """Sokette birikmiş sensör mesajı varsa en yenisine geç (gecikme birikmesini önler)."""
    if not c.so:
        return
    latest = None
    c.so.setblocking(False)
    try:
        while True:
            data, _ = c.so.recvfrom(data_size)
            latest = data.decode('utf-8')
            if '***' in latest:
                break
    except (BlockingIOError, OSError):
        pass
    if c.so:
        c.so.settimeout(1)
    if latest is None or '***identified***' in latest:
        return
    if '***shutdown***' in latest or '***restart***' in latest:
        print("Server has stopped the race on %d." % c.port)
        c.shutdown()
        return
    c.S.parse_server_str(latest)

# ================= MAIN DRIVE FUNCTION =================
def drive_modular(c):
    # Yarış çizgisi sürücüsü (yukarıdaki bölüm). Kural tabanlı eski mantık ve
    # --parameters değerleri bu sürümde sürüşte kullanılmaz (sözleşme için korunur).
    line_drive(c)
    return

# ================= MAIN LOOP =================
if __name__ == "__main__":
    #### DO NOT HARCODE THE PORT
    C = Client()
    for step in range(C.maxSteps, 0, -1):
        C.get_servers_input()
        drain_latest(C)
        if not C.so:
            break
        drive_modular(C)
        C.respond_to_server()
    C.shutdown()
