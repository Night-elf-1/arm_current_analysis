#include <iostream>
#include <chrono>
#include <thread>

// 定义定时器频率（10Hz 即 100 毫秒间隔）
constexpr int TIMER_INTERVAL_MS = 100;

void timerLoop() {
    // 获取当前时间点
    auto next_trigger_time = std::chrono::steady_clock::now();
    
    while (true) {
        // 更新下一次触发的时间点
        next_trigger_time += std::chrono::milliseconds(TIMER_INTERVAL_MS);

        // 运行你想执行的任务
        std::cout << "任务执行中..." << std::endl;

        // 等待直到下一次触发时间
        std::this_thread::sleep_until(next_trigger_time);
    }
}

int main() {
    std::thread timer_thread(timerLoop);  // 在单独线程中启动定时器
    timer_thread.join();  // 等待线程完成（示例中永远不会完成）

    return 0;
}
