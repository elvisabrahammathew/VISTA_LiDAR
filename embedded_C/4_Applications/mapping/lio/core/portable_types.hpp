#pragma once
#include <Eigen/Core>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

// Private ikd-Tree point layout, not a replacement or public imitation of PCL.
namespace vista::lio_port {
struct PointXYZ { float x{}, y{}, z{}; };
struct PointXYZI : PointXYZ { float intensity{}; };
struct PointXYZINormal : PointXYZI { float normal_x{}, normal_y{}, normal_z{}, curvature{}; };

// The generated copy renames POSIX calls to these private C++17 equivalents.
// Indirection permits the upstream node's value assignments without copying mutexes.
using mutex = std::shared_ptr<std::mutex>;
using thread = std::shared_ptr<std::thread>;
inline int mutex_init(mutex* m, const void*) { *m=std::make_shared<std::mutex>(); return 0; }
inline int mutex_destroy(mutex* m) { m->reset(); return 0; }
inline int mutex_lock(mutex* m) { (*m)->lock(); return 0; }
inline int mutex_unlock(mutex* m) { (*m)->unlock(); return 0; }
inline int mutex_trylock(mutex* m) { return (*m)->try_lock() ? 0 : 1; }
inline int thread_create(thread* t, const void*, void* (*fn)(void*), void* arg) {
    *t=std::make_shared<std::thread>([fn,arg]{ fn(arg); }); return 0;
}
inline int thread_join(thread& t, void**) { if(t && t->joinable()) t->join(); return 0; }
inline void sleep_us(unsigned int us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }
inline double wall_time() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace vista::lio_port
