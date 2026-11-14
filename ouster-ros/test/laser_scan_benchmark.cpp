// Actual full-image-copy baseline versus selected-ring converter, no publication.
#include "ouster_ros/os_ros.h"
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
namespace ouster_ros {
sensor_msgs::msg::LaserScan frozen_lidar_scan_to_laser_scan_msg(
    const ouster::sdk::core::LidarScan&, const rclcpp::Time&, const std::string&,
    ouster::sdk::core::LidarMode, uint16_t, const std::vector<int>&, int);
}
namespace core = ouster::sdk::core;
template<class T> std::vector<T> read(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary|std::ios::ate);
    if (!file) throw std::runtime_error("Cannot read "+path.string());
    const auto bytes=file.tellg();
    if (bytes%sizeof(T)) throw std::runtime_error("Invalid fixture size");
    std::vector<T> data(size_t(bytes)/sizeof(T));file.seekg(0);
    file.read(reinterpret_cast<char*>(data.data()), bytes);
    if(!file) throw std::runtime_error("Truncated fixture");return data;
}
int main(int argc, char**argv) try {
    if(argc<2) throw std::runtime_error("usage: benchmark FIXTURES [FRAMES=400] [ROUNDS=8] [RING=64]");
    const size_t frames=argc>2?std::stoul(argv[2]):400,rounds=argc>3?std::stoul(argv[3]):8;
    const auto ring=static_cast<uint16_t>(argc>4?std::stoul(argv[4]):64);
    if(!frames||!rounds||ring>=128)throw std::runtime_error("Invalid benchmark arguments");
    std::vector<std::filesystem::path> paths;
    for(const auto& entry:std::filesystem::directory_iterator(argv[1]))if(entry.is_directory())paths.push_back(entry.path());
    std::sort(paths.begin(),paths.end());
    std::vector<core::LidarScan> scans;std::vector<int> shifts;
    for(const auto& path:paths){
        auto range=read<uint32_t>(path/"range.u32"),signal=read<uint32_t>(path/"signal.u32");
        auto row_shifts=read<int32_t>(path/"shifts.i32");
        if(range.size()!=128*2048||signal.size()!=range.size()||row_shifts.size()!=128)throw std::runtime_error("Fixture dimensions");
        shifts.assign(row_shifts.begin(),row_shifts.end());scans.emplace_back(2048,128,core::UDPProfileLidar::LEGACY);
        std::copy(range.begin(),range.end(),scans.back().field<uint32_t>(core::ChanField::RANGE).data());
        std::copy(signal.begin(),signal.end(),scans.back().field<uint16_t>(core::ChanField::SIGNAL).data());
    }
    if(scans.empty())throw std::runtime_error("No scans");
    using Fn=decltype(&ouster_ros::lidar_scan_to_laser_scan_msg);
    const Fn functions[]={ouster_ros::frozen_lidar_scan_to_laser_scan_msg,ouster_ros::lidar_scan_to_laser_scan_msg};
    for(const auto& scan:scans){
        const auto old=functions[0](scan,rclcpp::Time(0),"frame",core::LidarMode::_2048x10,ring,shifts,0);
        const auto now=functions[1](scan,rclcpp::Time(0),"frame",core::LidarMode::_2048x10,ring,shifts,0);
        if(old!=now)throw std::runtime_error("LaserScan output mismatch");
    }
    double checksum=0;
    std::cout<<"round,order,implementation,frames,ring,cpu_ms,wall_ms,checksum\n"<<std::setprecision(12);
    for(size_t round=0;round<rounds;++round)for(size_t order=0;order<2;++order){
        const size_t implementation=(round+order)%2;const auto wall=std::chrono::steady_clock::now();const auto cpu=std::clock();
        for(size_t i=0;i<frames;++i){const auto message=functions[implementation](scans[i%scans.size()],rclcpp::Time(i),"frame",core::LidarMode::_2048x10,ring,shifts,0);checksum+=message.ranges[i%2048]+message.intensities[(i*13)%2048];}
        const auto cpu_ms=1000.0*(std::clock()-cpu)/CLOCKS_PER_SEC;
        const auto wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-wall).count();
        std::cout<<round<<','<<order<<','<<(implementation?"candidate":"frozen")<<','<<frames<<','<<ring<<','<<cpu_ms<<','<<wall_ms<<','<<checksum<<'\n';
    }
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
