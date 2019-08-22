#include <vector>
#include <boost/array.hpp>

#define NUM_CPU_STATES 10
#define S_USER 0
#define S_NICE 1
#define S_SYSTEM 2
#define S_IDLE 3
#define S_IOWAIT 4
#define S_IRQ 5
#define S_SOFTIRQ 6
#define S_STEAL 7
#define S_GUEST 8
#define S_GUEST_NICE 9

typedef struct CPUData
{
    std::string cpu;
    size_t times[NUM_CPU_STATES];
} CPUData;

class SystemMonitor {
public:
    SystemMonitor();
    virtual ~SystemMonitor();

    boost::array<uint8_t, 8> getCpuCores();
    boost::array<uint8_t, 10> getCpuCombined();


private:
   	std::vector<CPUData> prev_cpu_times_;
    boost::array<uint8_t, 8> cpu_cores_;
    boost::array<uint8_t, 10> cpu_combined_;

    void readCpuUsage();
    std::vector<CPUData> GetCpuTimes();
    size_t GetIdleTime(const CPUData & e);
    size_t GetActiveTime(const CPUData & e);

};