#include <array>
#include <atomic>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#define NUM_CPU_STATES 10
#define HISTOGRAM_ARRAY_SIZE 10

enum CpuStates { USER, NICE, SYSTEM, IDLE, IOWAIT, IRQ, SOFTIRQ, STEAL, GUEST, GUEST_NICE };

typedef struct CpuData {
  std::string cpu;
  size_t times[NUM_CPU_STATES];
} CpuData;

class SystemMonitor {
 public:
  SystemMonitor();
  virtual ~SystemMonitor();

  uint32_t getUpTime();
  std::array<uint8_t, 8> getCpuCores();
  std::array<uint8_t, 10> getCpuCombined();
  int8_t getBoardTemperature();
  uint32_t getRamUsage();
  uint32_t getRamTotal();

 private:
  std::vector<CpuData> prev_cpu_times_;
  std::array<uint8_t, 8> cpu_cores_;
  std::deque<uint8_t> cpu_combined_;
  int8_t board_temp_;
  uint32_t up_time_;

  std::atomic_bool hist_stop_{false};
  std::thread cpu_combined_hist_th_;
  std::mutex hist_mtx_;

  void readUpTime();
  void readCpuUsage();
  void readBoardTemperature();
  std::vector<CpuData> GetCpuTimes();
  size_t GetIdleTime(const CpuData& e);
  size_t GetActiveTime(const CpuData& e);

  /**
   * @brief Following the spec in
   * https://mavlink.io/en/messages/common.html#ONBOARD_COMPUTER_STATUS
   * i.e, builds an histogram of combined CPU usage of the last 10 slices of 100ms.
   * This allows that, even if the status is published at rates lower than 10hz,
   * one can still analyse the previous ten CPU usage values in the last second
   */
  void SetCPUCombinedHist();
};
