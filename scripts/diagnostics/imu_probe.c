// Raw USB probe for the D435i IMU, bypassing librealsense. Powers the accelerometer (100 Hz)
// and gyroscope (200 Hz) on through HID feature reports on interface 5, reads interrupt
// endpoint 0x86 and prints how many 38-byte reports arrive per 0.5 s, then powers them off.
// A healthy IMU gives ~400 reports per window; a hung one gives 0 even though power-on succeeds.
//
// build: gcc -O1 -o imu_probe imu_probe.c -lusb-1.0      (needs libusb-1.0-0-dev)
// run:   ./imu_probe [seconds]      (no librealsense program may hold the camera at the time)
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#pragma pack(push,1)
struct fr { uint8_t id, conn, state, power, minrep; uint16_t report, sens; };
#pragma pack(pop)
static double now(){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec/1e9; }
static int feat(libusb_device_handle*h,int id,int power,int ms){
  struct fr f; memset(&f,0,sizeof f); int v=(3<<8)+id;
  int r=libusb_control_transfer(h,0xa1,1,v,5,(uint8_t*)&f,sizeof f,1000);
  if(r<0) return r;
  f.power=power; if(ms) f.report=ms;
  r=libusb_control_transfer(h,0x21,9,v,5,(uint8_t*)&f,sizeof f,1000);
  r=libusb_control_transfer(h,0xa1,1,v,5,(uint8_t*)&f,sizeof f,1000);
  return f.power==power?0:-1;
}
int main(int argc,char**argv){
  double secs=argc>1?atof(argv[1]):6;
  libusb_context*c; libusb_init(&c);
  libusb_device_handle*h=libusb_open_device_with_vid_pid(c,0x8086,0x0b3a);
  if(!h){puts("open failed");return 1;}
  if(libusb_kernel_driver_active(h,5)==1) libusb_detach_kernel_driver(h,5);
  if(libusb_claim_interface(h,5)){puts("claim failed");return 1;}
  printf("power on: accel=%d gyro=%d\n",feat(h,1,2,10),feat(h,2,2,5));
  uint8_t b[64]; double t0=now(), w=t0; int n=0,total=0;
  setvbuf(stdout,NULL,_IOLBF,0);
  while(now()-t0<secs){
    int got=0; int r=libusb_interrupt_transfer(h,0x86,b,64,&got,20);
    if(r==0&&got==38){n++;total++;}
    else if(r!=0&&r!=LIBUSB_ERROR_TIMEOUT){printf("read err %s\n",libusb_error_name(r));break;}
    if(now()-w>=0.5){printf("  imu t=%.1fs reports in last 0.5s: %d\n",now()-t0,n);n=0;w=now();}
  }
  printf("total reports=%d\n",total);
  feat(h,1,6,0); feat(h,2,6,0);
  libusb_release_interface(h,5); libusb_attach_kernel_driver(h,5);
  libusb_close(h); libusb_exit(c); return 0;
}
