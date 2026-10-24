// Replays a controller capture (psvr2xr-selftest --capture-controllers) through sense_track.c,
// printing each frame's poses in the head and drawing r000.ppm... into out dir: the camera
// pictures with the dots found (yellow) and the fitted LEDs (cyan left, magenta right).
//
//   cc -O2 -I src tests/sense_track_replay.c src/sense_track.c -lm -o build/sense_track_replay
//   build/sense_track_replay ~/Desktop/"PS VR2 Controller Capture" /tmp/replay [capture]
//
// The cameras start from the built-in ones, or the capture's controller_cameras.json with
// "capture"; CAMS="f cx cy yaw pitch roll" x2 overrides them, THR the dot threshold.
#include "sense_track.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define W 1024
#define H 1016
#define PB (W*H/2)
static void qrot(const float q[4], const float v[3], float o[3]) {
    float t[3] = {2 * (q[1] * v[2] - q[2] * v[1]), 2 * (q[2] * v[0] - q[0] * v[2]), 2 * (q[0] * v[1] - q[1] * v[0])};
    float r[3] = {v[0] + q[3] * t[0] + (q[1] * t[2] - q[2] * t[1]), v[1] + q[3] * t[1] + (q[2] * t[0] - q[0] * t[2]),
                  v[2] + q[3] * t[2] + (q[0] * t[1] - q[1] * t[0])};
    memcpy(o, r, sizeof r);
}
static void qconj(const float q[4], float o[4]) { o[0]=-q[0]; o[1]=-q[1]; o[2]=-q[2]; o[3]=q[3]; }
static void rotm(const float ax[3], float a, float R[9]) {
    float c=cosf(a), s=sinf(a), v=1-c, x=ax[0], y=ax[1], z=ax[2];
    float M[9]={c+x*x*v, x*y*v-z*s, x*z*v+y*s, y*x*v+z*s, c+y*y*v, y*z*v-x*s, z*x*v-y*s, z*y*v+x*s, c+z*z*v};
    memcpy(R,M,sizeof M);
}
static void mm(const float A[9], const float B[9], float O[9]) {
    float r[9];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) r[i*3+j]=A[i*3]*B[j]+A[i*3+1]*B[3+j]+A[i*3+2]*B[6+j];
    memcpy(O,r,sizeof r);
}
static void camrot(const strack_camera *c, float R[9]) {
    float Ry[9], Rx[9], Rz[9], C[9]={1,0,0,0,-1,0,0,0,-1}, t[9], y[3]={0,1,0}, x[3]={1,0,0}, z[3]={0,0,1};
    rotm(y, c->yaw_deg*M_PI/180, Ry); rotm(x, c->pitch_deg*M_PI/180, Rx); rotm(z, c->roll_deg*M_PI/180, Rz);
    mm(Ry,Rx,t); mm(t,Rz,t); mm(t,C,R);
}
static unsigned char img[H][2*W][3];
static void dot(int cam, float u, float v, int r, unsigned char cr, unsigned char cg, unsigned char cb) {
    for (int dy=-r; dy<=r; dy++) for (int dx=-r; dx<=r; dx++) {
        if (abs(dx)!=r && abs(dy)!=r) continue;
        int x=(int)u+dx, y=(int)v+dy; if (x<0||y<0||x>=W||y>=H) continue;
        img[y][cam*W+x][0]=cr; img[y][cam*W+x][1]=cg; img[y][cam*W+x][2]=cb;
    }
}
static void bc4(const unsigned char *b, unsigned char *px) {
    int a0=b[0], a1=b[1], p[8]; p[0]=a0; p[1]=a1;
    if (a0>a1) for (int i=1;i<7;i++) p[i+1]=((7-i)*a0+i*a1)/7; else { for (int i=1;i<5;i++) p[i+1]=((5-i)*a0+i*a1)/5; p[6]=0; p[7]=255; }
    unsigned long long bits=0; for (int i=0;i<6;i++) bits|=(unsigned long long)b[2+i]<<(8*i);
    for (int i=0;i<16;i++) px[i]=p[(bits>>(3*i))&7];
}
static float jnum(const char *s, const char *key) { const char *p=strstr(s,key); return p? strtof(strchr(p,':')+1,NULL):0; }
static int jarr(const char *s, const char *key, float *out, int n) {
    const char *p=strstr(s,key); if (!p) return 0; p=strchr(p,'['); if (!p) return 0; p++;
    for (int i=0;i<n;i++) { char *e; out[i]=strtof(p,&e); p=e; while (*p==','||*p==' ') p++; }
    return 1;
}
int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s <capture dir> <out dir> [capture]\n", argv[0]); return 1; }
    char path[1024], line[1<<16];
    setvbuf(stdout, NULL, _IONBF, 0);
    strack_camera cams[2]; strack_default_cameras(cams);
    if (argc>3 && !strcmp(argv[3],"capture")) {
        snprintf(path,sizeof path,"%s/controller_cameras.json",argv[1]);
        FILE *f=fopen(path,"r"); static char js[8192]; size_t n=fread(js,1,sizeof js-1,f); js[n]=0; fclose(f);
        char *objs[2]={js, js};
        char *mid=strstr(js+1,"},"); if (mid) { *mid=0; objs[1]=mid+2; }
        for (int c=0;c<2;c++) {
            const char *o=objs[c];
            cams[c].f=jnum(o,"\"f\""); cams[c].cx=jnum(o,"\"cx\""); cams[c].cy=jnum(o,"\"cy\"");
            cams[c].yaw_deg=jnum(o,"yaw_deg"); cams[c].pitch_deg=jnum(o,"pitch_deg"); cams[c].roll_deg=jnum(o,"roll_deg");
            jarr(o,"\"k\"",cams[c].k,4); jarr(o,"position",cams[c].position,3);
        }
    }
    const char *ov=getenv("CAMS");   // f,cx,cy,yaw,pitch,roll for both: "f cx cy y p r f cx cy y p r"
    if (ov) { float v[12]; sscanf(ov,"%f %f %f %f %f %f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7],&v[8],&v[9],&v[10],&v[11]);
        for (int c=0;c<2;c++){cams[c].f=v[c*6];cams[c].cx=v[c*6+1];cams[c].cy=v[c*6+2];cams[c].yaw_deg=v[c*6+3];cams[c].pitch_deg=v[c*6+4];cams[c].roll_deg=v[c*6+5];} }
    for (int c=0;c<2;c++) printf("cam %d: f %.1f c %.1f %.1f yaw %.1f pitch %.1f roll %.1f\n",c,cams[c].f,cams[c].cx,cams[c].cy,cams[c].yaw_deg,cams[c].pitch_deg,cams[c].roll_deg);
    strack_params prm; strack_default_params(&prm);
    if (getenv("THR")) prm.threshold=atof(getenv("THR"));
    prm.calibrate = getenv("CAL") != NULL;   // off, as in the headset
    strack *t=strack_create(cams,&prm);
    snprintf(path,sizeof path,"%s/frames.jsonl",argv[1]);
    FILE *fl=fopen(path,"r");
    unsigned char *buf=malloc(256+2*PB);
    int frame=0;
    while (fgets(line,sizeof line,fl)) {
        char name[64]; sscanf(strstr(line,"\"file\": \"")+9,"%63[^\"]",name);
        snprintf(path,sizeof path,"%s/%s",argv[1],name);
        FILE *fb=fopen(path,"rb"); fread(buf,1,256+2*PB,fb); fclose(fb);
        strack_input in={0};
        in.time_ns=(long long)jnum(line,"time_ns");
        char *hd=strstr(line,"\"head\""); jarr(hd,"\"q\"",in.head.q,4); jarr(hd,"\"p\"",in.head.p,3);
        char *lt=strstr(line,"\"left\""), *rt=strstr(line,"\"right\""), *dl=strstr(line,"\"dots_left\"");
        char *seg[2]={lt,rt};
        for (int h=0;h<2;h++) {
            char tmp[2048]; char *end = h==0? rt : dl; size_t n=end-seg[h]; if(n>=sizeof tmp)n=sizeof tmp-1; memcpy(tmp,seg[h],n); tmp[n]=0;
            in.prior_valid[h]=strstr(tmp,"\"imu\": true")!=NULL;
            if (in.prior_valid[h]) jarr(tmp,"prior_q",in.prior_q[h],4);
        }
        for (int c=0;c<2;c++) strack_detect_bc4(t,c,buf+256+c*PB,256);
        strack_result r[2]; strack_solve(t,&in,r);
        strack_camera cc[2]; strack_get_cameras(t,cc);
        printf("%s", name);
        // picture
        for (int c=0;c<2;c++) for (int by=0;by<H/4;by++) for (int bx=0;bx<W/4;bx++) {
            unsigned char px[16]; bc4(buf+256+c*PB+(by*256+bx)*8,px);
            for (int i=0;i<16;i++) { unsigned char g=px[i]; img[by*4+i/4][c*W+bx*4+i%4][0]=g; img[by*4+i/4][c*W+bx*4+i%4][1]=g; img[by*4+i/4][c*W+bx*4+i%4][2]=g; }
        }
        for (int c=0;c<2;c++) { int n; const strack_blob *b=strack_blobs(t,c,&n); for (int i=0;i<n;i++) dot(c,b[i].x,b[i].y,6,255,255,0); }
        float hqc[4]; qconj(in.head.q,hqc);
        for (int h=0;h<2;h++) {
            if (!r[h].valid) { printf(" | %s -", h?"R":"L"); continue; }
            float ph[3]; float d[3]={r[h].pose.p[0]-in.head.p[0],r[h].pose.p[1]-in.head.p[1],r[h].pose.p[2]-in.head.p[2]};
            qrot(hqc,d,ph);
            printf(" | %s head(%.2f %.2f %.2f) leds %d rms %.2f yaw %.0f%s", h?"R":"L", ph[0],ph[1],ph[2], r[h].leds, r[h].rms_px, r[h].yaw_error*57.3f, r[h].found?" found":"");
            int pred=0;
            for (int c=0;c<2;c++) {
                float Rc[9]; camrot(&cc[c],Rc);
                for (int i=0;i<STRACK_LEDS;i++) {
                    float lp[3], nrm[3], w[3], hh[3], pc[3]; strack_led(h,i,lp,nrm);
                    qrot(r[h].pose.q,lp,w); for(int k=0;k<3;k++) w[k]+=r[h].pose.p[k]-in.head.p[k];
                    qrot(hqc,w,hh); for(int k=0;k<3;k++) hh[k]-=cc[c].position[k];
                    for (int k=0;k<3;k++) pc[k]=Rc[k]*hh[0]+Rc[3+k]*hh[1]+Rc[6+k]*hh[2];
                    float nw[3], nh[3], nc[3]; qrot(r[h].pose.q,nrm,nw); qrot(hqc,nw,nh);
                    for (int k=0;k<3;k++) nc[k]=Rc[k]*nh[0]+Rc[3+k]*nh[1]+Rc[6+k]*nh[2];
                    float dd=sqrtf(pc[0]*pc[0]+pc[1]*pc[1]+pc[2]*pc[2]); float facing=-(nc[0]*pc[0]+nc[1]*pc[1]+nc[2]*pc[2])/dd;
                    float u,v; if (strack_project(&cc[c],pc,&u,&v) && facing>0.5f) pred++;
                    if (strack_project(&cc[c],pc,&u,&v) && facing>0.2f) dot(c,u,v,3,h?255:0,h?0:255,h?255:255);
                }
            }
            printf(" pred %d", pred);
        }
        float rms; int cal=strack_calibration(t,&rms);
        printf(" | cal %d %.2f\n", cal, rms);
        snprintf(path,sizeof path,"%s/r%03d.ppm",argv[2],frame++);
        FILE *o=fopen(path,"wb"); fprintf(o,"P6 %d %d 255\n",2*W,H); fwrite(img,1,sizeof img,o); fclose(o);
    }
    strack_camera cc[2]; strack_get_cameras(t,cc);
    for (int c=0;c<2;c++) printf("final cam %d: f %.1f c %.1f %.1f yaw %.1f pitch %.1f roll %.1f\n",c,cc[c].f,cc[c].cx,cc[c].cy,cc[c].yaw_deg,cc[c].pitch_deg,cc[c].roll_deg);
}
