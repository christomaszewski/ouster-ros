#pragma once
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include "experimental_gpu_abi.h"

namespace ouster_ros {
// Isolated experiment only. No CUDA link dependency or production parameter.
// The message owner must release registration before replacement/reallocation.
class ExperimentalGpuCloud {
    void* library_ = nullptr;
    ouster_gpu_handle* handle_ = nullptr;
    decltype(&ouster_gpu_create) create_ = nullptr;
    decltype(&ouster_gpu_prepare) prepare_ = nullptr;
    decltype(&ouster_gpu_run) run_ = nullptr;
    decltype(&ouster_gpu_release) release_ = nullptr;
    decltype(&ouster_gpu_destroy) destroy_ = nullptr;
    uint8_t* registered_ = nullptr;
    size_t bytes_, pixels_, columns_;
    bool required_;
    uint64_t frames_ = 0, bindings_ = 0;
    template<class T> T symbol(const char* name) {
        dlerror();void* p=dlsym(library_,name);const char* error=dlerror();
        if(error||!p)throw std::runtime_error(std::string("GPU symbol missing: ")+name);
        return reinterpret_cast<T>(p);
    }
    void close() noexcept {
        if(handle_&&destroy_)destroy_(handle_);handle_=nullptr;
        if(library_)dlclose(library_);library_=nullptr;registered_=nullptr;
    }
public:
    static const char* library_path() {const char* p=std::getenv("OUSTER_GPU_CLOUD_LIBRARY");return p&&*p?p:nullptr;}
    static bool required() {const char* p=std::getenv("OUSTER_GPU_CLOUD_REQUIRE");return p&&std::strcmp(p,"1")==0;}
    static bool requested() {return library_path()||required();}
    ExperimentalGpuCloud(const ouster_gpu_config& config)
        :bytes_(size_t(config.crop_width)*config.height*48),pixels_(size_t(config.width)*config.height),columns_(config.width),required_(required()) {
        try {
            if(!library_path())throw std::runtime_error("OUSTER_GPU_CLOUD_LIBRARY missing");
            library_=dlopen(library_path(),RTLD_NOW|RTLD_LOCAL);
            if(!library_)throw std::runtime_error(std::string("GPU library load: ")+dlerror());
            create_=symbol<decltype(create_)>("ouster_gpu_create");prepare_=symbol<decltype(prepare_)>("ouster_gpu_prepare");
            run_=symbol<decltype(run_)>("ouster_gpu_run");release_=symbol<decltype(release_)>("ouster_gpu_release");destroy_=symbol<decltype(destroy_)>("ouster_gpu_destroy");
            char error[512]{};if(create_(&config,&handle_,error,sizeof(error)))throw std::runtime_error(error);
            std::cerr<<"EXPERIMENTAL_GPU_CLOUD_READY input=native_LEGACY_copy output=registered_host bytes="<<bytes_<<" require="<<required_<<'\n';
        } catch(const std::exception& e) {
            close();if(required_)throw;
            std::cerr<<"EXPERIMENTAL_GPU_CLOUD_CPU_FALLBACK reason="<<e.what()<<'\n';
        }
    }
    ~ExperimentalGpuCloud() {
        // Processor declares this owner after pc_msgs, so mappings are removed
        // while those message allocations still exist, even during shutdown.
        close();if(frames_)std::cerr<<"EXPERIMENTAL_GPU_CLOUD_STOP frames="<<frames_<<" bindings="<<bindings_<<'\n';
    }
    ExperimentalGpuCloud(const ExperimentalGpuCloud&)=delete;
    ExperimentalGpuCloud& operator=(const ExperimentalGpuCloud&)=delete;
    bool enabled() const {return handle_!=nullptr;}
    size_t output_bytes() const {return bytes_;}
    void release_output() {
        if(!handle_||!registered_)return;char error[512]{};
        if(release_(handle_,error,sizeof(error)))throw std::runtime_error(std::string("GPU output release: ")+error);
        registered_=nullptr;
    }
    void before_prepare(const sensor_msgs::msg::PointCloud2& message,bool replacing) {
        if(replacing||message.data.capacity()<bytes_||(registered_&&message.data.data()!=registered_))release_output();
    }
    bool compute(const ouster::sdk::core::LidarScan& scan,uint64_t scan_ts,
                 sensor_msgs::msg::PointCloud2& message,bool& dense) {
        if(!handle_)return false;
        if(message.data.size()!=bytes_)throw std::runtime_error("GPU output schema/size changed");
        if(!registered_) {
            char error[512]{};
            if(prepare_(handle_,message.data.data(),message.data.size(),nullptr,0,error,sizeof(error))) {
                if(required_)throw std::runtime_error(std::string("GPU output prepare: ")+error);
                std::cerr<<"EXPERIMENTAL_GPU_CLOUD_CPU_FALLBACK reason="<<error<<'\n';close();return false;
            }
            registered_=message.data.data();++bindings_;
        }
        if(registered_!=message.data.data())throw std::runtime_error("GPU output moved without release");
        auto range=scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
        auto signal=scan.field<uint16_t>(ouster::sdk::core::ChanField::SIGNAL);
        auto reflectivity=scan.field<uint8_t>(ouster::sdk::core::ChanField::REFLECTIVITY);
        auto near_ir=scan.field<uint16_t>(ouster::sdk::core::ChanField::NEAR_IR);
        const auto& timestamps=scan.timestamp();
        if(size_t(range.size())!=pixels_||size_t(signal.size())!=pixels_||size_t(reflectivity.size())!=pixels_||size_t(near_ir.size())!=pixels_||size_t(timestamps.size())!=columns_)
            throw std::runtime_error("GPU native scan dimensions changed");
        ouster_gpu_scan input{range.data(),signal.data(),reflectivity.data(),near_ir.data(),timestamps.data(),scan_ts,pixels_,columns_};
        char error[512]{};int result=0;
        if(run_(handle_,&input,message.data.data(),message.data.size(),&result,error,sizeof(error)))
            throw std::runtime_error(std::string("GPU execution failed; stopping experimental path: ")+error);
        dense=result!=0;if(++frames_==1)std::cerr<<"EXPERIMENTAL_GPU_CLOUD_ACTIVE synchronous=1 native_input_copy=1 output_copy=0\n";
        return true;
    }
};
}
