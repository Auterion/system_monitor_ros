#include <boost/array.hpp>
#include <vector>

#define NUM_CPU_STATES 10

enum CpuStates
{
    USER,
    NICE,
    SYSTEM,
    IDLE,
    IOWAIT,
    IRQ,
    SOFTIRQ,
    STEAL,
    GUEST,
    GUEST_NICE
};

typedef struct CpuData
{
    std::string cpu;
    size_t times[NUM_CPU_STATES];
} CpuData;

class SystemMonitor {
public:
    SystemMonitor();
    virtual ~SystemMonitor();

    uint32_t getUpTime();
    boost::array<uint8_t, 8> getCpuCores();
    boost::array<uint8_t, 10> getCpuCombined();
    int8_t getBoardTemperature();


private:
   	std::vector<CpuData> prev_cpu_times_;
    boost::array<uint8_t, 8> cpu_cores_;
    boost::array<uint8_t, 10> cpu_combined_;
    int8_t board_temp_;
    uint32_t up_time_;

    void readUpTime();
    void readCpuUsage();
    void readBoardTemperature();
    std::vector<CpuData> GetCpuTimes();
    size_t GetIdleTime(const CpuData & e);
    size_t GetActiveTime(const CpuData & e);

};