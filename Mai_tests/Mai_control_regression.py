#!/usr/bin/env python3
"""Run the actual control functions on the host with mocked hardware I/O.

This validates control decisions and PWM commands, not vehicle dynamics or ADS.
Temporary C sources/executables are removed on exit.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def read_source(name):
    data = (ROOT / name).read_bytes()
    try:
        return data.decode("utf-8-sig")
    except UnicodeDecodeError:
        return data.decode("gb18030")


def function(source, name):
    match = re.search(r"^(?:void|int32|int16)\s+" + name + r"\([^\n]*\)\s*\n\{", source, re.M)
    assert match, name
    # These source functions have their closing brace at column zero.
    end = source.index("\n}", match.end()) + 2
    return source[match.start():end] + "\n"


CONTROL = read_source("code/control.c")
PID_SOURCE = read_source("code/PID.c")
UI_HEADER = read_source("code/Lu_UI.h")

PRELUDE = r"""
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32;
typedef int8_t int8; typedef int16_t int16; typedef int32_t int32;
typedef struct {int32 LastError; float KP,KI,KD;} PID;
typedef struct {float Kp,Ki,Kd; int32 Pe,Ie,De,e0,e1,e2,Sum,DefaultSum,Speed;} MotorPid;
PID Point_PID; MotorPid RMPID,LMPID;
uint8 Point=60,Point_Mid=60,Stop_1,Flag_stop,Flag_turn,steer_settle=1;
uint8 steer_mode,cone_ok,cone_cnt,cone_near,mk_state,mk_dia_seen,mk_cw_seen,mk_cw_near;
uint8 Distance_ON,Distance_Test_ON,end_stopped,Line_Point;
uint16 Left_Line[61],Right_Line[61],Width_Real[61],Mid_Line[61],Pwm_Min=847,Pwm_Max=987,end_stop_cm,steer_guard_bits;
uint8 Fictitious_Midline[61]; int8 Line_Count=4; uint8 Result_Far_Row=59;
int16 cone_x=-1,lane_k=75,Steer_Mid=917,Steer_Trim=16,slalom_dir=-1;
uint8 cone_group_cnt; char Point_Error;
int32 SD5_Duty,steer_target,steer_raw_target,LMotorSpeed,RMotorSpeed,LSpeedPWM,RSpeedPWM,iError;
int16 Rcount,Lcount,Rrealspeed,Lrealspeed,Fictitious_Beeline_Num;
double Distance,Distance_Test,Run_Odo; float Mid_Ka,Mid_Kb;
volatile uint32 isr_tick_5ms,vision_processed_frames;
#define MT9V03X_W 120
#define COL 120
#define ABS(x) ((x)<0?-(x):(x))
#define MotorMax 3000
#define MotorMin (-3000)
#define ENCODER1_TIM 1
#define ENCODER2_TIM 2
#define MOTOR1_A 3
#define MOTOR2_A 4
#define MOTOR1_B 1
#define MOTOR2_B 2
#define DISTANCE(x) ((double)(x)*0.0288)
int32 motor_pwm[3],servo_pwm;
int16 mock_encoder[3];
void gpio_set_level(int pin,int value){(void)pin;(void)value;}
void pwm_set_duty(int pin,int32 value){motor_pwm[pin]=value;}
#define SteerSet(x) (servo_pwm=(x))
int16 encoder_get_count(int which){return mock_encoder[which];}
void encoder_clear_count(int which){mock_encoder[which]=0;}
int32 range_protect(int32 x,int32 lo,int32 hi){return x<lo?lo:(x>hi?hi:x);}
void Least_Sqaure_Method(float*,float*,uint8,uint8);
int16 Fictitious_Beeline(float,float);
"""

CONSTANTS = [
    "STEER_NO_LINE_LINECOUNT", "STEER_MIN_VALID_ROWS", "STEER_ONE_LANE_MAX",
    "STEER_LANE_W_DEF", "STEER_STRAIGHT_RANGE", "STEER_TARGET_STEP", "STEER_START_TICKS",
    "CONE_CONFIRM_FRAMES", "STEER_LANE_FOLLOW", "STEER_FORCE_CENTER",
    "CONE_LANE_CM", "CONE_HOLD_MAXTICKS", "CONE_CLEAR_TICKS", "CONE_LANE_HALF",
    "CONE_TRIG_ROW", "CONE_MAX_GROUPS", "MOTOR_TEST_MODE", "MOTOR_TEST_PWM",
    "MOTOR_PWM_MAX", "MOTOR_RAMP_TICKS", "MOTOR_TEST_TARGET_SPEED", "SPEED_PERCENT",
    "MK_DIA_SPEED_PCT", "MK_GAP_CM", "MK_DIA_CONFIRM", "MK_CW_CONFIRM",
    "MK_STOP_ROW", "MK_STOP_TICKS", "MK_PARK_DUTY", "CONE_ACT_ENABLE",
]


def macro(name):
    match = re.search(r"^#define\s+" + name + r"\s+(\([^\n]*?\)|\d+)", CONTROL + "\n" + UI_HEADER, re.M)
    assert match, name
    return "#define " + name + " " + match.group(1) + "\n"


TESTS = r"""
static void check(int ok,const char *why){if(!ok){fprintf(stderr,"FAIL: %s (mode=%d raw=%d target=%d duty=%d MK=%d ES=%d)\n",why,steer_mode,steer_raw_target,steer_target,SD5_Duty,mk_state,end_stopped);exit(1);}}
static void lane(int l,int r){for(int i=0;i<61;i++){Left_Line[i]=l;Right_Line[i]=r;Width_Real[i]=r-l;Mid_Line[i]=(l+r)/2;} Point=60;Result_Far_Row=59;}
static void start(void){Stop_1=0;S3010_Control();Stop_1=1;for(int i=0;i<STEER_START_TICKS;i++){S3010_Control();check(steer_mode==1 && servo_pwm==933 && steer_target==60,"startup is centered for every tick");}}
int main(int argc,char **argv){check(argc==2,"scenario required");
 if(!strcmp(argv[1],"straight")){
  lane(48,117);Point=81;start();
  S3010_Control();check(steer_raw_target==69 && steer_target==61,"logged left-only boundary no longer adds an extra lane");
  for(int i=0;i<40;i++){S3010_Control();check(steer_target>=50 && steer_target<=70 && servo_pwm>847 && servo_pwm<987,"straight correction stays bounded");}
  lane(1,97);Point=48;for(int i=0;i<40;i++)S3010_Control();
  lane(1,117);S3010_Control();check(steer_mode==4 && servo_pwm==933,"both missing edges return to neutral, not old turn");
  lane(38,82);Point=2;S3010_Control();check(steer_mode==4 && servo_pwm==933,"invalid fitted point returns to neutral");
 }
 else if(!strcmp(argv[1],"cone")){
  lane(20,100);cone_ok=1;cone_cnt=2;cone_near=50;cone_x=80;start();
  vision_processed_frames=1;for(int i=0;i<100;i++){S3010_Control();check(steer_mode!=3,"one cone frame cannot trigger a maneuver");}
  vision_processed_frames=2;S3010_Control();check(steer_mode!=3,"two frames do not confirm a cone");
  vision_processed_frames=3;S3010_Control();check(steer_mode==3 && steer_target==42,"confirmed cone remains able to maneuver");
 }
 else if(!strcmp(argv[1],"crosswalk")){
  lane(38,82);start();for(int i=0;i<1200;i++)Control_Speed();check(LMotorSpeed>0,"normal drive remains available");
  mk_cw_seen=1;mk_cw_near=50;for(int i=0;i<MK_CW_CONFIRM+1;i++)Control_Speed();
  check(mk_state==2,"confirmed crosswalk triggers stop state");
#if MK_ACT_ENABLE && MK_FINAL_STOP
  check(end_stopped && LMotorSpeed==0 && RMotorSpeed==0,"crosswalk immediately latches stop");
  mk_cw_seen=0;mk_cw_near=0;
  for(int i=0;i<6000;i++){Control_Speed();check(end_stopped && LMotorSpeed==0 && RMotorSpeed==0,"no timed restart for 30 seconds");}
  RMPID.Sum=2000;LMPID.Sum=1800;RMPID.e0=LMPID.e0=50;motor_pwm[1]=motor_pwm[2]=2000;
  mock_encoder[1]=20;mock_encoder[2]=-20;Motor_Control();
  check(motor_pwm[1]==0 && motor_pwm[2]==0 && RMPID.Sum==0 && LMPID.Sum==0,"final stop clears PID history and actual PWM outputs");
  S3010_Control();check(steer_mode==5 && servo_pwm==933,"parked car also centers the servo");
#else
  check(!end_stopped,"timed/log-only option does not latch final stop");
  for(int i=0;i<MK_STOP_TICKS;i++)Control_Speed();
  check(mk_state!=2 && LMotorSpeed>0,"5 second restart is retained when override disabled");
#endif
 }
 else if(!strcmp(argv[1],"fit")){
  float a,b;lane(38,82);
  for(int lc=55;lc<=59;lc++){Line_Count=lc;Point_Control();check(Point==0 && isfinite(Mid_Ka),"short fit publishes invalid without NaN");}
  for(int end=56;end<=61;end++){Least_Sqaure_Method(&a,&b,58,end);check(isfinite(a)&&isfinite(b),"fit boundary stays finite");}
  Line_Count=4;Point_Control();check(Point==60,"valid straight fit is unchanged");
 }
 else if(!strcmp(argv[1],"distance")){
  lane(38,82);start();end_stop_cm=100;Run_Odo=99;
  for(int i=0;i<1200;i++){Control_Speed();}check(!end_stopped && LMotorSpeed>0,"distance stop not early");
  Run_Odo=100;Control_Speed();check(end_stopped && LMotorSpeed==0,"distance threshold triggers");
  for(int i=0;i<1200;i++){Control_Speed();check(LMotorSpeed==0 && RMotorSpeed==0,"distance stop remains latched");}
 }
 else if(!strcmp(argv[1],"replay")){
  FILE *f=fopen("Mai_trace.txt","r");check(f!=NULL,"replay input exists");int l,r,w,p,fr,lc,cn,near,cx,n=0;
  lane(38,82);start();
  while(fscanf(f,"%d %d %d %d %d %d %d %d %d",&l,&r,&w,&p,&fr,&lc,&cn,&near,&cx)==9){
   lane(l,r);Width_Real[22]=w;Point=p;Result_Far_Row=fr;Line_Count=lc;
   cone_cnt=cn;cone_near=near;cone_x=cx;cone_ok=cn>=2;vision_processed_frames++;
   S3010_Control();if(steer_mode!=3)check(steer_target>=50 && steer_target<=70,"logged non-cone frames cannot command a lane change");n++;
  }fclose(f);printf("Replay frames: %d\n",n);
 }
 else check(0,"unknown scenario");
 printf("PASS %s (final=%d actions=%d)\n",argv[1],MK_FINAL_STOP,MK_ACT_ENABLE);return 0;}
"""


def main():
    definitions = "".join(macro(name) for name in CONSTANTS)
    functions = "".join(function(PID_SOURCE, n) for n in ["PlacePID_Control", "RMotorPid_Col", "LMotorPid_Col"])
    functions += "".join(function(CONTROL, n) for n in ["S3010_Control", "Motor_Control", "Point_Control", "Control_Speed", "Least_Sqaure_Method", "Fictitious_Beeline"])
    with tempfile.TemporaryDirectory(prefix="Mai_control_regression_") as directory:
        work = Path(directory)
        source = work / "Mai_control.c"
        source.write_text(PRELUDE + definitions + functions + TESTS)
        log = ROOT.parent / "tools/Mai_log_20261027_194946.txt"
        replay = []
        if log.is_file():
            for line in log.read_text(errors="replace").splitlines():
                v = dict(re.findall(r"(\w+)=([^\s]+)", line))
                if "T" not in v or not v.get("st", "").startswith("0/"):
                    continue
                cn, near, cx = v["CN"].split("/")
                replay.append(" ".join([v[k] for k in ["L", "R", "W", "P", "FR", "LC"]] + [cn, near, cx]))
        (work / "Mai_trace.txt").write_text("\n".join(replay))
        for final, actions in [(1, 1), (0, 1), (1, 0)]:
            executable = work / f"Mai_control_{final}_{actions}"
            subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror=format", "-Werror=implicit-function-declaration", f"-DMK_FINAL_STOP={final}", f"-DMK_ACT_ENABLE={actions}", str(source), "-lm", "-o", str(executable)], check=True)
            cases = ["crosswalk"]
            if final and actions:
                cases += ["straight", "cone", "fit", "distance"]
                if replay:
                    cases.append("replay")
                else:
                    print("SKIP log replay: original local telemetry is not available", flush=True)
            for case in cases:
                subprocess.run([str(executable), case], cwd=work, check=True)


if __name__ == "__main__":
    main()
