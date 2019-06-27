#ifndef NAVIGATION_DEVICE_H
#define NAVIGATION_DEVICE_H

#include <mutex>
namespace navigation{

    class NavigationDevice {

    public:
        NavigationDevice() : new_data_ready_(false) { };
        void SetNewDataReady();
        bool IsNewDataReady();
        void SetNewDataUsed();

    private:
        bool new_data_ready_;
        std::mutex data_ready_mutex_;
    };

    inline void NavigationDevice::SetNewDataReady() {
        std::lock_guard<std::mutex> guard(data_ready_mutex_);
        new_data_ready_ = true;
    };

    inline bool NavigationDevice::IsNewDataReady() {
        std::lock_guard<std::mutex> guard(data_ready_mutex_);
        return new_data_ready_;
    };

    inline void NavigationDevice::SetNewDataUsed() {
        std::lock_guard<std::mutex> guard(data_ready_mutex_);
        new_data_ready_ = false;
        };
}

#endif