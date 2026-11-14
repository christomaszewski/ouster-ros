#include "../src/experimental_gpu_abi.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
// CPU-only ownership/lifecycle oracle for the ROS loader; never a GPU fallback.
// Stats: creates, prepares, runs, releases, destroys, invalid transitions.
static uint64_t stats[6]{};
struct ouster_gpu_handle {
    ouster_gpu_config c;std::vector<float> d,o;std::vector<int32_t> shifts;
    uint8_t* output=nullptr;size_t bytes=0;
};
struct alignas(16) PointBytes {float x,y,z,w,intensity;uint32_t t;uint16_t reflectivity,ring,ambient,pad30;uint32_t range,pad36,pad40,pad44;};
static_assert(sizeof(PointBytes)==48);
extern "C" void ouster_gpu_test_reset(){std::fill(std::begin(stats),std::end(stats),0);}
extern "C" void ouster_gpu_test_stats(uint64_t* out){std::copy(std::begin(stats),std::end(stats),out);}
extern "C" int ouster_gpu_create(const ouster_gpu_config* c,ouster_gpu_handle** out,char*,size_t) {
    if(!c||!out||c->abi_version!=OUSTER_GPU_ABI_VERSION||c->struct_size!=sizeof(*c)||c->ownership!=OUSTER_GPU_REGISTERED_OUTPUT)return 1;
    auto* h=new ouster_gpu_handle;h->c=*c;size_t n=size_t(c->width)*c->height;
    h->d.assign(c->direction,c->direction+n*3);h->o.assign(c->offset,c->offset+n*3);h->shifts.assign(c->shifts,c->shifts+c->height);*out=h;++stats[0];return 0;
}
extern "C" int ouster_gpu_prepare(ouster_gpu_handle* h,uint8_t* out,size_t bytes,const ouster_gpu_scan*,size_t count,char*,size_t) {
    if(!h||!out||count||h->output||bytes!=size_t(h->c.crop_width)*h->c.height*48||reinterpret_cast<uintptr_t>(out)%16){++stats[5];return 1;}
    h->output=out;h->bytes=bytes;++stats[1];return 0;
}
extern "C" int ouster_gpu_run(ouster_gpu_handle* h,const ouster_gpu_scan* s,uint8_t* out,size_t bytes,int* dense,char*,size_t) {
    if(!h||!s||out!=h->output||bytes!=h->bytes||s->pixels!=size_t(h->c.width)*h->c.height||s->columns!=h->c.width){++stats[5];return 1;}
    *dense=1;for(uint32_t row=0;row<h->c.height;row++)for(uint32_t col=0;col<h->c.crop_width;col++) {
        uint32_t global=(h->c.crop_start+col)%h->c.width;int32_t shift=(h->shifts[row]+int32_t(h->c.width))%int32_t(h->c.width);int32_t source=int32_t(global)-shift;if(source<0)source+=h->c.width;
        size_t i=size_t(row)*h->c.width+source;PointBytes p{};uint32_t r=s->range[i];
        if(r<=h->c.min_range||r>=h->c.max_range) {uint32_t q=0x7fc00000;std::memcpy(&p.x,&q,4);p.y=p.z=p.x;}else {float f=float(r);p.x=std::fma(f,h->d[i*3],h->o[i*3]);p.y=std::fma(f,h->d[i*3+1],h->o[i*3+1]);p.z=std::fma(f,h->d[i*3+2],h->o[i*3+2]);}
        p.w=1;p.intensity=float(s->signal[i]);uint64_t ts=s->timestamps[source];p.t=uint32_t(ts>s->scan_ts?ts-s->scan_ts:0);p.reflectivity=s->reflectivity[i];p.ring=uint16_t(row);p.ambient=s->near_ir[i];p.range=r;
        *dense&=!(std::isnan(p.x)||std::isnan(p.y)||std::isnan(p.z));std::memcpy(out+(size_t(row)*h->c.crop_width+col)*48,&p,48);
    }++stats[2];return 0;
}
extern "C" int ouster_gpu_release(ouster_gpu_handle* h,char*,size_t) {
    if(h&&h->output){volatile uint8_t still_alive=*h->output;(void)still_alive;h->output=nullptr;++stats[3];}return 0;
}
extern "C" void ouster_gpu_destroy(ouster_gpu_handle* h) {if(h){ouster_gpu_release(h,nullptr,0);delete h;++stats[4];}}
