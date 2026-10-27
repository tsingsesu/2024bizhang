#!/usr/bin/env python3
"""Exercise the actual camera threshold on synthetic grayscale road images."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
data = (ROOT / "libraries/zf_device/zf_device_mt9v03x.c").read_bytes()
source = data.decode("gb18030")
start = source.index("uint8 Diedai(")
end = source.index("\n}", start) + 2
prelude = r"""
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t uint8; typedef uint16_t uint16;
typedef uint32_t uint32; typedef int32_t int32;
uint8 diedai_img_min,diedai_img_max,diedai_img_avg,diedai_threshold;
uint32 diedai_img_hi128,diedai_img_hi192;
uint8 zyuzhi=95,Differ_light,image[120*70];
"""
tests = r"""
static void road(int dark,int white){
 memset(image,dark,sizeof(image));
 for(int y=0;y<70;y++)for(int dx=0;dx<4;dx++){
  image[y*120+20+dx]=white;image[y*120+96+dx]=white;
 }
}
static void check(int ok,const char *why,int t){
 if(!ok){fprintf(stderr,"FAIL %s: threshold=%d\n",why,t);exit(1);}
 printf("PASS %s: threshold=%d\n",why,t);
}
int main(void){
 road(30,100);int base=Diedai(image,120,70);
 check(base>30 && base<100,"ordinary white lines remain visible",base);
 for(int i=0;i<25;i++)image[32*120+55+i%5+(i/5)*120]=255;
 int glare=Diedai(image,120,70);
 check(glare>30 && glare<100,"small saturated spot cannot erase dimmer white lines",glare);
 check(abs(glare-base)<=5,"local spot does not move the global threshold sharply",glare);
 road(10,60);image[30*120+60]=255;int dim=Diedai(image,120,70);
 check(dim>10 && dim<60,"dim road with one bright pixel retains lane paint",dim);
 road(80,180);int bright=Diedai(image,120,70);
 check(bright>80 && bright<180,"brighter ambient light retains lane separation",bright);
 road(0,255);int full=Diedai(image,120,70);
 check(full>0 && full<255,"high contrast paint is retained",full);
 memset(image,0,sizeof(image));int black=Diedai(image,120,70);
 check(black>=12 && black<=240,"uniform dark frame stays finite and bounded",black);
 memset(image,255,sizeof(image));int saturated=Diedai(image,120,70);
 check(saturated>=12 && saturated<=240,"uniform bright frame stays finite and bounded",saturated);
 return 0;
}
"""


def main():
    with tempfile.TemporaryDirectory(prefix="Mai_threshold_") as directory:
        path = Path(directory) / "Mai_threshold.c"
        executable = Path(directory) / "Mai_threshold"
        path.write_text(prelude + source[start:end] + tests)
        subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror=format",
                        str(path), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
