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
    match = re.search(r"^(?:static\s+)?(?:void|int32|int16|uint16|uint32|uint8)\s+" + name + r"\([^\n]*\)\s*\n\{", source, re.M)
    assert match, name
    # These source functions have their closing brace at column zero.
    end = source.index("\n}", match.end()) + 2
    return source[match.start():end] + "\n"


CONTROL = read_source("code/control.c")
PID_SOURCE = read_source("code/PID.c")
UI_HEADER = read_source("code/Lu_UI.h")
IMAGE_SOURCE = read_source("code/Img_analyse.c")
SYSTEM_HEADER = read_source("user/System.h")

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
uint8 cone_px[48],cone_py[48],cone_pts,diedai_img_avg=80,image[70*120];
uint16 cone_wh[5];
uint8 cone_group_ok,cone_group_support,cone_group_span_x,cone_group_span_y,cone_group_error;
int16 cone_x=-1,lane_k=75,Steer_Mid=917,Steer_Trim=16,slalom_dir=-1;
uint8 cone_group_cnt; char Point_Error;
int32 SD5_Duty,steer_target,steer_raw_target,LMotorSpeed,RMotorSpeed,LSpeedPWM,RSpeedPWM,iError;
int16 Rcount,Lcount,Rrealspeed,Lrealspeed,Fictitious_Beeline_Num;
double Distance,Distance_Test,Run_Odo; float Mid_Ka,Mid_Kb;
volatile uint32 isr_tick_5ms,vision_processed_frames,tof_sample_tick;
uint16 vision_age_ticks,track_bad_ticks,mk_stop_elapsed_ms,mk_clear_remaining_mm,steer_lane_left,steer_lane_right;
uint8 steer_look_row,mk_cw_score,mk_dia_score,mk_dia_near;
uint8 drive_inhibit,mk_finish_reason,mk_near_white,tof_has_sample;
uint16 tof_sample_mm;double run_distance_cm;
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
    "MK_DIA_SPEED_PCT", "MK_DIA_CONFIRM", "MK_CW_CONFIRM", "MK_CW_NEAR_CONFIRM",
    "MK_BAND_L", "MK_BAND_R", "MK_ROW_MIN", "MK_ROW_MAX", "MK_CW_RUN_MIN", "MK_CW_MIN_LANE_W", "MK_CW_SPAN_PCT", "MK_THIN_MIN", "MK_THIN_MAX", "MK_DIA_MIN_PAIRS", "MK_DIA_ROW_MAX", "MK_CW_MIN_ROWS", "MK_DIA_MIN_ROWS", "MK_DIA_MAX_ROWS",
    "MK_STOP_ROW", "MK_STOP_TICKS", "MK_PARK_DUTY", "CONE_ACT_ENABLE",
    "CONE_ROW_MIN", "CONE_ROW_MAX", "CONE_COL_MIN", "CONE_COL_MAX",
    "CONE_W_MIN", "CONE_W_MAX", "CONE_LINK_X", "CONE_NEAR_ROW",
    "CONE_MIN_ROWS", "CONE_MAX_BLOBS", "CONE_MAX_PTS",
    "VISION_STALE_TICKS", "TRACK_LOSS_TICKS", "TOF_FRESH_TICKS", "MK_CROSS_SPEED",
    "MK_CLEAR_CM", "MK_CROSS_LIMIT_CM", "MK_STILL_COUNTS", "All_Line", "Ratio", "Length", "Multiple",
]


def macro(name):
    match = re.search(r"^#define\s+" + name + r"\s+(\([^\n]*?\)|-?\d+(?:\.\d+)?)", CONTROL + "\n" + UI_HEADER + "\n" + IMAGE_SOURCE + "\n" + SYSTEM_HEADER, re.M)
    assert match, name
    return "#ifndef " + name + "\n#define " + name + " " + match.group(1) + "\n#endif\n"


TESTS = r"""
static void check(int ok,const char *why){if(!ok){fprintf(stderr,"FAIL: %s (mode=%d raw=%d target=%d duty=%d MK=%d ES=%d)\n",why,steer_mode,steer_raw_target,steer_target,SD5_Duty,mk_state,end_stopped);exit(1);}}
static void lane(int l,int r){for(int i=0;i<61;i++){Left_Line[i]=l;Right_Line[i]=r;Width_Real[i]=r-l;Mid_Line[i]=(l+r)/2;} Point=60;Result_Far_Row=59;}
static void start(void){vision_processed_frames=1;Stop_1=0;S3010_Control();Stop_1=1;for(int i=0;i<STEER_START_TICKS;i++){vision_processed_frames++;S3010_Control();check(steer_mode==1 && servo_pwm==933 && steer_target==60,"startup is centered for every tick");}}
static void band(int x,int y){for(int i=0;i<8;i++)image[y*COL+x+i]=255;}
int main(int argc,char **argv){check(argc==2,"scenario required");
 if(!strcmp(argv[1],"straight")){
  lane(48,117);Point=81;start();
  S3010_Control();check(steer_raw_target==69 && steer_target==61,"logged left-only boundary no longer adds an extra lane");
  for(int i=0;i<40;i++){S3010_Control();check(steer_target>=50 && steer_target<=70 && servo_pwm>847 && servo_pwm<987,"straight correction stays bounded");}
  lane(1,97);Point=48;for(int i=0;i<40;i++)S3010_Control();
  lane(1,117);S3010_Control();check(steer_mode==4 && servo_pwm==933,"both missing edges return to neutral, not old turn");
  lane(38,82);Point=2;Line_Count=59;S3010_Control();check(steer_mode==4 && servo_pwm==933,"invalid fitted point returns to neutral");
 }
 else if(!strcmp(argv[1],"cone")){
  lane(20,100);cone_ok=1;cone_cnt=2;cone_near=50;cone_x=80;start();
  vision_processed_frames=1;for(int i=0;i<100;i++){S3010_Control();check(steer_mode!=3,"one cone frame cannot trigger a maneuver");}
  vision_processed_frames=2;S3010_Control();check(steer_mode!=3,"two frames do not confirm a cone");
  vision_processed_frames=3;S3010_Control();
#if CONE_ACT_ENABLE
  check(steer_mode==3 && steer_target==42,"confirmed cone remains able to maneuver when enabled");
#else
  for(int i=0;i<1000;i++){
   vision_processed_frames++;S3010_Control();
   check(steer_mode!=3 && lane_k==75 && steer_target>=50 && steer_target<=70,"disabled cone action cannot steer or change lanes even with persistent false detections");
  }
#endif
 }
 else if(!strcmp(argv[1],"crosswalk")){
  lane(38,82);start();for(int i=0;i<1200;i++){Control_Speed();}check(LMotorSpeed>0,"normal drive remains available");
  mk_cw_seen=1;mk_cw_near=50;for(int i=0;i<MK_CW_CONFIRM;i++){vision_processed_frames++;Control_Speed();}
  check(mk_state==2 && !end_stopped,"crosswalk first enters temporary stop");
#if MK_ACT_ENABLE
  check(drive_inhibit && LMotorSpeed==0,"temporary stop immediately inhibits output");
  Lrealspeed=Rrealspeed=5;for(int i=0;i<200;i++)Control_Speed();
  check(mk_state==2 && mk_stop_elapsed_ms==0,"moving vehicle cannot consume the five stationary seconds");
  Lrealspeed=Rrealspeed=0;for(int i=0;i<MK_STOP_TICKS-1;i++)Control_Speed();
  check(mk_state==2 && mk_stop_elapsed_ms==4995,"cannot leave before a full five stationary seconds");
  Control_Speed();check(mk_state==3 && !end_stopped && mk_stop_elapsed_ms==5000,"five seconds release low speed crossing");
  run_distance_cm=10;mk_near_white=1;vision_processed_frames++;Control_Speed();
  run_distance_cm=59.9;mk_near_white=0;vision_processed_frames++;Control_Speed();
  check(mk_state==3 && !end_stopped,"49.9 cm after last stripe is not full clearance");
  run_distance_cm=60;Control_Speed();check(mk_state==4 && mk_finish_reason==1,"last stripe plus 50 cm completes crossing");
#if MK_FINAL_STOP
  check(end_stopped && LMotorSpeed==0 && RMotorSpeed==0,"completed crossing latches final stop");
  for(int i=0;i<6000;i++){Control_Speed();check(end_stopped && LMotorSpeed==0,"final parking cannot restart");}
  RMPID.Sum=LMPID.Sum=2000;Motor_Control();check(motor_pwm[1]==0 && motor_pwm[2]==0 && RMPID.Sum==0 && LMPID.Sum==0,"parked PWM and PID history are zero");
  S3010_Control();check(steer_mode==5 && servo_pwm==933,"final parking centers servo");
#else
  for(int i=0;i<1200;i++){Control_Speed();}check(!end_stopped && LMotorSpeed>0,"optional continue mode keeps driving after crossing");
#endif
#else
  for(int i=0;i<1200;i++){Control_Speed();}check(!end_stopped && LMotorSpeed>0,"log-only mode does not park");
#endif
 }
 else if(!strcmp(argv[1],"diamond_hold")){
  lane(38,82);start();for(int i=0;i<1200;i++)Control_Speed();
  mk_dia_seen=1;for(int i=0;i<MK_DIA_CONFIRM;i++){vision_processed_frames++;Control_Speed();}
  check(mk_state==1,"first diamond starts slowdown");
  mk_dia_seen=0;Distance=10000;run_distance_cm=400;
  for(int i=0;i<2000;i++)Control_Speed();
  check(mk_state==1 && !drive_inhibit && LMotorSpeed==12,"unmarked gap retains half speed without stopping");
  mk_dia_seen=1;for(int i=0;i<100;i++)Control_Speed();
  check(mk_state==1 && LMotorSpeed==12,"second diamond retains same low speed");
 }
 else if(!strcmp(argv[1],"corridor")){
  lane(20,96);Point=56;start();S3010_Control();check(steer_raw_target==58 && steer_target<=60,"76 pixel current lane is centered, not shifted to 77");
 }
 else if(!strcmp(argv[1],"near_lane")){
  lane(28,102);start();
  for(int lc=23;lc<=48;lc++){
   Line_Count=lc;Left_Line[22]=Right_Line[22]=0;
   Left_Line[lc+2]=28;Right_Line[lc+2]=102;Width_Real[lc+2]=74;
   for(int t=0;t<60;t++){vision_processed_frames++;S3010_Control();Control_Speed();}
   check(steer_look_row==lc+2 && steer_mode==2 && !drive_inhibit,"fresh near lane avoids false line-loss stop");
  }
  lane(1,117);Line_Count=31;
  Left_Line[45]=Left_Line[46]=20;Right_Line[45]=Right_Line[46]=96;
  for(int t=0;t<60;t++){vision_processed_frames++;S3010_Control();Control_Speed();}
  check(steer_look_row==45 && steer_mode==2 && !drive_inhibit,"two fresh nearer rows recover a clipped marker or cone row");
  lane(1,117);Line_Count=51;
  Left_Line[53]=Left_Line[54]=20;Right_Line[53]=Right_Line[54]=96;
  for(int t=0;t<60;t++){vision_processed_frames++;S3010_Control();Control_Speed();}
  check(steer_look_row==53 && steer_mode==2 && !drive_inhibit,"short far scan can still use a visible near corridor");
  Line_Count=59;for(int t=0;t<TRACK_LOSS_TICKS;t++){vision_processed_frames++;S3010_Control();}
  Control_Speed();check(steer_mode==4 && drive_inhibit,"truly insufficient scan still stops");
 }
 else if(!strcmp(argv[1],"vision")){
  lane(38,82);start();vision_processed_frames=0;
  for(int i=0;i<1200;i++){Control_Speed();}check(drive_inhibit && LMotorSpeed==0,"no processed image cannot accelerate");
  motor_pwm[1]=motor_pwm[2]=2000;Motor_Control();check(motor_pwm[1]==0 && motor_pwm[2]==0,"missing camera inhibits actual motor output");
  vision_processed_frames=1000;vision_age_ticks=0;track_bad_ticks=0;Control_Speed();check(LMotorSpeed==0,"camera recovery restarts ramp from zero");
  for(int i=0;i<1200;i++){Control_Speed();}check(LMotorSpeed>0,"valid camera can recover");
  vision_age_ticks=VISION_STALE_TICKS;Control_Speed();check(drive_inhibit && LMotorSpeed==0,"stalled processing stops vehicle");
  vision_age_ticks=0;Line_Count=59;for(int i=0;i<TRACK_LOSS_TICKS;i++){vision_processed_frames++;S3010_Control();}
  Control_Speed();check(drive_inhibit && LMotorSpeed==0,"persistent invalid lane stops vehicle");
 }
 else if(!strcmp(argv[1],"tof_odo")){
  lane(38,82);start();mock_encoder[1]=100;mock_encoder[2]=-100;Motor_Control();
  check(fabs(run_distance_cm-100.0*Length*Multiple/(All_Line*Ratio))<1e-6,"physical odometry uses wheel geometry, not legacy O units");
  check(Tof_Get_Distance()==8192,"no ToF sample is unknown");
  tof_has_sample=1;tof_sample_mm=250;tof_sample_tick=100;isr_tick_5ms=104;
  check(Tof_Get_Distance()==250 && Tof_Get_Age_Ms()==20,"new ToF value is held between polling rounds");
  isr_tick_5ms=141;check(Tof_Get_Distance()==8192,"stale ToF is marked unknown");
  tof_sample_tick=0xFFFFFFF0U;isr_tick_5ms=5;check(Tof_Get_Distance()==250,"ToF freshness handles timer wrap");
 }
 else if(!strcmp(argv[1],"cross_limit")){
  lane(38,82);start();mk_state=3;run_distance_cm=MK_CROSS_LIMIT_CM;Control_Speed();
  check(mk_state==4 && mk_finish_reason==2,"unverifiable crossing has a bounded distance safeguard");
 }
 else if(!strcmp(argv[1],"near_band")){
  memset(image,0,sizeof(image));check(!Mai_Near_White_Scan(image),"black road is not a stripe");
  for(int y=45;y<=58;y++){band(48,y);}check(!Mai_Near_White_Scan(image),"vertical marking is not a transverse stripe");
  memset(image,0,sizeof(image));for(int x=38;x<78;x++)image[50*COL+x]=255;
  check(Mai_Near_White_Scan(image),"near transverse band updates clearance anchor");
 }
 else if(!strcmp(argv[1],"marker_image")){
  lane(20,100);Line_Count=31;
  for(int y=8;y<20;y++){image[y*COL+32]=image[y*COL+33]=255;image[y*COL+76]=image[y*COL+77]=255;}
  Marker_Scan(image);check(!mk_dia_seen && !mk_cw_seen,"stale far lane rows cannot supply diamond evidence");
  memset(image,0,sizeof(image));Line_Count=4;
  for(int y=8;y<=58;y++){image[y*COL+42]=image[y*COL+43]=255;image[y*COL+77]=image[y*COL+78]=255;}
  Marker_Scan(image);check(!mk_dia_seen,"two parallel thin lines inside the window are not a diamond");
  memset(image,0,sizeof(image));
  for(int y=44;y<=55;y++)for(int x=30;x<=90;x++)image[y*COL+x]=255;
  Marker_Scan(image);check(!mk_cw_seen,"one tall solid blob is not a crosswalk");
  memset(image,0,sizeof(image));
  for(int y=40;y<=43;y++)for(int x=24;x<=96;x++)image[y*COL+x]=255;
  for(int y=51;y<=56;y++)for(int x=24;x<=96;x++)image[y*COL+x]=255;
  Marker_Scan(image);check(mk_cw_seen && mk_cw_near==56,"separated transverse stripes still identify a crosswalk");
  Line_Count=51;Marker_Scan(image);check(mk_cw_seen,"fresh near stripe plus raw striped pattern survives a short lane scan");
  memset(image,0,sizeof(image));Line_Count=4;
  for(int y=25;y<37;y++){int x=39+(y-25)/2;image[y*COL+x]=image[y*COL+x+1]=255;image[y*COL+120-x]=image[y*COL+119-x]=255;}
  Marker_Scan(image);check(mk_dia_seen && !mk_cw_seen,"hollow sloping marker requests slowdown only");
  Line_Count=40;Marker_Scan(image);check(mk_dia_seen,"current diamond pixels remain usable above lane extraction stop");
 }
 else if(!strcmp(argv[1],"marker_frame")){
  lane(38,82);start();mk_cw_seen=1;mk_cw_near=50;
  for(int i=0;i<1000;i++)Control_Speed();
  check(mk_state==0,"one frozen crosswalk frame cannot confirm a stop");
  mk_cw_near=30;for(int i=0;i<MK_CW_CONFIRM+2;i++){vision_processed_frames++;Control_Speed();}
  mk_cw_near=50;vision_processed_frames++;Control_Speed();
  check(mk_state==0,"far crosswalk history cannot promote a single near false detection");
 }
 else if(!strcmp(argv[1],"perception")){
  lane(1,117);start();band(70,54);band(74,56);
  for(int i=0;i<100;i++){
   Cone_Scan(image);vision_processed_frames++;S3010_Control();
   check(cone_cnt==2 && !cone_ok && steer_mode!=3 && lane_k==75,"repeated raw two-blob false detections never enter a maneuver");
  }
  memset(image,0,sizeof(image));band(26,25);band(51,40);band(76,55);
  for(int i=0;i<CONE_CONFIRM_FRAMES;i++){Cone_Scan(image);vision_processed_frames++;S3010_Control();}
  check(cone_ok && steer_mode==3 && lane_k==0,"validated diagonal group can still trigger automatic avoidance");
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
 printf("PASS %s (final=%d actions=%d cone=%d)\n",argv[1],MK_FINAL_STOP,MK_ACT_ENABLE,CONE_ACT_ENABLE);return 0;}
"""


def main():
    definitions = "".join(macro(name) for name in CONSTANTS)
    functions = '#include "Mai_cone_geometry.h"\n' + function(IMAGE_SOURCE, "Mai_Near_White_Scan") + function(IMAGE_SOURCE, "Cone_Scan") + function(IMAGE_SOURCE, "Mai_Diamond_Scan") + function(IMAGE_SOURCE, "Marker_Scan")
    functions += "".join(function(PID_SOURCE, n) for n in ["PlacePID_Control", "RMotorPid_Col", "LMotorPid_Col"])
    functions += "".join(function(CONTROL, n) for n in ["Mai_Lane_Row_Usable", "S3010_Control", "Motor_Control", "Point_Control", "Control_Speed", "Least_Sqaure_Method", "Fictitious_Beeline", "Tof_Get_Distance", "Tof_Get_Age_Ms"])
    with tempfile.TemporaryDirectory(prefix="Mai_control_regression_") as directory:
        work = Path(directory)
        source = work / "Mai_control.c"
        source.write_text(PRELUDE + definitions + functions + TESTS)
        log = ROOT.parent / "tools/Mai_log_20261027_194946.txt"
        replay = []
        if log.is_file():
            for line in log.read_text(errors="replace").splitlines():
                # V23/V24 reused R for stop_run later in the same line.
                # Keep the first R, which is the actual right lane boundary.
                v = {}
                for key, value in re.findall(r"(\w+)=([^\s]+)", line):
                    v.setdefault(key, value)
                if "T" not in v or not v.get("st", "").startswith("0/"):
                    continue
                cn, near, cx = v["CN"].split("/")
                replay.append(" ".join([v[k] for k in ["L", "R", "W", "P", "FR", "LC"]] + [cn, near, cx]))
        (work / "Mai_trace.txt").write_text("\n".join(replay))
        for final, actions, cone in [(1, 1, 1), (1, 1, 0), (0, 1, 1), (1, 0, 1)]:
            executable = work / f"Mai_control_{final}_{actions}_{cone}"
            subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror=format", "-Werror=implicit-function-declaration", "-I", str(ROOT / "code"), f"-DMK_FINAL_STOP={final}", f"-DMK_ACT_ENABLE={actions}", f"-DCONE_ACT_ENABLE={cone}", str(source), "-lm", "-o", str(executable)], check=True)
            cases = ["crosswalk"]
            if final and actions:
                cases += ["straight", "cone", "fit", "distance", "corridor", "diamond_hold", "near_lane", "vision", "tof_odo", "cross_limit", "near_band", "marker_image", "marker_frame"]
                if cone:
                    cases.append("perception")
                if replay:
                    cases.append("replay")
                else:
                    print("SKIP log replay: original local telemetry is not available", flush=True)
            for case in cases:
                subprocess.run([str(executable), case], cwd=work, check=True)


if __name__ == "__main__":
    main()
