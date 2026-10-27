#!/usr/bin/env python3
"""Exercise the real Cone_Scan on synthetic binary images; no hardware required."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "code/Img_analyse.c").read_text(encoding="utf-8-sig")


def main():
    names = ["CONE_ROW_MIN", "CONE_ROW_MAX", "CONE_COL_MIN", "CONE_COL_MAX",
             "CONE_W_MIN", "CONE_W_MAX", "CONE_LINK_X", "CONE_NEAR_ROW",
             "CONE_MIN_ROWS", "CONE_MAX_BLOBS", "CONE_MAX_PTS"]
    definitions = []
    for name in names:
        m = re.search(r"^#define\s+" + name + r"\s+\((\d+)\)", SOURCE, re.M)
        assert m, name
        definitions.append(f"#define {name} ({m.group(1)})\n")
    start = SOURCE.index("void Cone_Scan(")
    end = SOURCE.index("\n}", start) + 2
    scan = SOURCE[start:end]
    prelude = r"""
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t uint8;typedef uint16_t uint16;typedef int16_t int16;typedef int32_t int32;
#define COL 120
uint16 Left_Line[61],Right_Line[61];
uint8 diedai_img_avg=80;
volatile uint8 cone_cnt,cone_near,cone_ok,cone_pts,cone_group_ok;
volatile uint8 cone_group_support,cone_group_span_x,cone_group_span_y,cone_group_error;
volatile int16 cone_x;
volatile uint16 cone_wh[5];
uint8 cone_px[48],cone_py[48],image[70*120];
"""
    tests = r"""
static void reset(void){memset(image,0,sizeof(image));diedai_img_avg=80;for(int i=0;i<61;i++){Left_Line[i]=1;Right_Line[i]=117;}}
static void band(int x,int y,int width){for(int i=0;i<width;i++)image[y*COL+x+i]=255;}
static void expect(int valid,const char *why){Cone_Scan(image);if(cone_ok!=valid || cone_group_ok!=valid){fprintf(stderr,"FAIL %s: CN=%d CG=%d GP=%d/%d/%d/%d\n",why,cone_cnt,cone_group_ok,cone_group_support,cone_group_span_x,cone_group_span_y,cone_group_error);exit(1);}printf("PASS %s: CN=%d CG=%d GP=%d/%d/%d/%d\n",why,cone_cnt,cone_group_ok,cone_group_support,cone_group_span_x,cone_group_span_y,cone_group_error);}
int main(void){
 reset();expect(0,"empty road");
 reset();band(70,54,8);band(74,56,8);expect(0,"two nearby fragments of one feature");
 reset();band(26,25,8);band(76,55,8);expect(0,"two separated white blobs are insufficient");
 reset();for(int y=14;y<=56;y++)band(48,y,8);expect(0,"continuous lane stripe");
 reset();for(int y=18;y<=54;y+=9)band(48,y,8);expect(0,"vertical dashed lane stripe");
 reset();band(25,45,8);band(55,45,8);band(85,45,8);expect(0,"horizontal white fragments");
 reset();for(int y=20;y<=23;y++)band(45,y,30);for(int y=35;y<=38;y++)band(45,y,30);for(int y=50;y<=53;y++)band(45,y,30);expect(0,"repeated horizontal crosswalk bands");
 reset();band(16,20,8);band(76,28,8);band(31,55,8);expect(0,"non-collinear blobs");
 reset();band(26,25,8);band(51,40,8);band(76,55,8);expect(1,"three distributed diagonal candidates");
 reset();for(int i=0;i<5;i++)band(27+i*13,18+i*9,6);expect(1,"five distributed diagonal candidates");
 diedai_img_avg=200;expect(0,"overexposure clears previous candidate group");
 reset();diedai_img_avg=5;band(26,25,8);band(51,40,8);band(76,55,8);expect(1,"dark background keeps diagonal evidence");
 return 0;}
"""
    with tempfile.TemporaryDirectory(prefix="Mai_cone_detection_") as directory:
        p = Path(directory) / "Mai_cones.c"
        executable = Path(directory) / "Mai_cones"
        p.write_text(prelude + "".join(definitions) + '#include "Mai_cone_geometry.h"\n' + scan + tests)
        subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror=implicit-function-declaration",
                        "-Werror=format", "-I", str(ROOT / "code"), str(p), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
