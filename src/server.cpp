#include <bits/stdc++.h>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sched.h>
#include <iomanip>
#include <sys/syscall.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "../include/kuka_rsi_hw_interface/udp_server.h"
#include "../include/kuka_rsi_hw_interface/AxesMapper.h"
#include "../include/kuka_rsi_hw_interface/kuka_hardware_interface.h"
#include "../include/kuka_rsi_hw_interface/KukaAxes.h"

#define RUNTIME_OVERRUN_TIMER 60 * 60 // seconds


std::queue<std::string> state_queue;
std::mutex state_mutex;
std::condition_variable state_cv;

std::queue<std::string> command_queue;
std::mutex command_mutex;
std::condition_variable command_cv;

// Function to set thread priority and policy
void set_realtime_priority(int priority)
{
    struct sched_param schedParam;
    schedParam.sched_priority = priority;

    // Set the thread to real-time FIFO scheduling
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedParam) != 0)
    {
        perror("Failed to set real-time priority");
        exit(EXIT_FAILURE);
    }
}

volatile sig_atomic_t overrun_counter_robot = 0;

void sigxcpu_handler(int signum) {
    // Use thread-local storage or other mechanisms to identify the thread
    // For example, using pthread_self()
    overrun_counter_robot++;
}

void setup_signal_handler() {
    struct sigaction sa;
    sa.sa_handler = sigxcpu_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGXCPU, &sa, NULL) != 0) {
        perror("Failed to set SIGXCPU handler");
        exit(EXIT_FAILURE);
    }
}



struct sched_attr
{
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;
    int32_t sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};

void set_realtime_deadline(unsigned long runtime, unsigned long deadline, unsigned long period)
{
    struct sched_attr attr;
    int ret;

    // Zero out the structure
    memset(&attr, 0, sizeof(attr));

    // Set the scheduling policy to SCHED_DEADLINE
    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = runtime;
    attr.sched_deadline = deadline;
    attr.sched_period = period;

    // Enable Overrun detection
    attr.sched_flags |= SCHED_FLAG_DL_OVERRUN;

    // Use syscall to set the scheduling policy and parameters
    ret = syscall(SYS_sched_setattr, gettid(), &attr, 0);
    if (ret != 0)
    {
        perror("Failed to set SCHED_DEADLINE");
        exit(EXIT_FAILURE);
    }
}

void set_CPU(size_t cpu1)
{
    cpu_set_t mask;
    CPU_ZERO(&mask);

    CPU_SET(cpu1, &mask);

    // pid = 0 means "calling process"
    if (sched_setaffinity(0, sizeof(mask), &mask) == -1)
    {
        std::cerr << "Error setting CPU affinity: "
                  << std::strerror(errno) << std::endl;
    }
}

std::string get_current_time()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_time), "%d.%m.%Y %H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
    return oss.str();
}

bool write_noCorrection(UDPServer *socketServer, unsigned long long ipoc)
{

    kuka_rsi_hw_interface::KukaAxes axes = kuka_rsi_hw_interface::KukaAxes();

    for (int i = 0; i < kuka_rsi_hw_interface::KukaAxes::MAX_INTERNAL_AXES; ++i)
    {
        axes.getInternalAxes()[i] = 0;
    }
    for (int i = 0; i < kuka_rsi_hw_interface::KukaAxes::MAX_EXTERNAL_AXES; ++i)
    {
        axes.getExternalAxes()[i] = 0;
    }

    kuka_rsi_hw_interface::RSICommand rsi_command = kuka_rsi_hw_interface::RSICommand(axes, ipoc);

    std::string out_buffer = rsi_command.xml_doc;

    socketServer->send(out_buffer);

    return true;
}

bool write_Correction(UDPServer *socketServer, unsigned long long ipoc, std::string buffer)
{
    kuka_rsi_hw_interface::KukaAxes axes = kuka_rsi_hw_interface::KukaAxes();

    for (int i = 0; i < kuka_rsi_hw_interface::KukaAxes::MAX_INTERNAL_AXES; ++i)
    {
        axes.getInternalAxes()[i] = 0;
    }
    for (int i = 0; i < kuka_rsi_hw_interface::KukaAxes::MAX_EXTERNAL_AXES; ++i)
    {
        axes.getExternalAxes()[i] = 0;
    }

    kuka_rsi_hw_interface::RSICommand rsi_command = kuka_rsi_hw_interface::RSICommand(axes, ipoc);

    std::string out_buffer = rsi_command.xml_doc;

    socketServer->send(out_buffer);

    return true;
}

bool check_state(std::string str) {
    if (str.size() < 11) return false;

    return (str.compare(str.size() - 7, 7, "</Rob>\n") == 0 && str.compare(0, 4, "<Rob") == 0);
}

void robot()
{

    set_CPU(0);

    auto start_prog = std::chrono::system_clock::now();

    unsigned long long ipoc;
    ssize_t bytes;
    std::string in_buffer;
    std::string buffer = "";

    std::string ip = "172.29.3.25";
    int port = 6008;

    UDPServer *socketServer = new UDPServer(ip, port);

    int runtime =   200 * 1000;
    int deadline =  200 * 1000;
    int period =    200 * 1000; 

    set_realtime_deadline(runtime, deadline, period);

    auto start_rtt = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point end_rtt;

    double max_loop = 0.0;
    int counter = 0;

    while (true)
    {
        bytes = socketServer->recv_nonblocking(in_buffer);

        if (bytes > 0)
        {
            buffer = buffer + in_buffer;
            while (!check_state(buffer)) {
                bytes = socketServer->recv_nonblocking(in_buffer);
                if (bytes > 0) {
                    buffer = buffer + in_buffer;
                }
                else if (bytes == 0) {
                    std::cerr << "Robot Connection closed by peer!" << std::endl;
                    return;
                }
            }
        }
        else if (bytes == 0)
        {
            std::cerr << "Robot Connection closed by peer!" << std::endl;
            return;
        }

        if (check_state(buffer))
        {
            auto rsi_state = kuka_rsi_hw_interface::RSIState(buffer);
            ipoc = rsi_state.ipoc;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                if (state_queue.size() > 0)
                    state_queue.pop();
                state_queue.push(buffer);
            }
            state_cv.notify_one();

            buffer = "";

            if (command_queue.size() > 0)
            {
                {
                    std::lock_guard<std::mutex> lock(command_mutex);
                    buffer = command_queue.front();
                    command_queue.pop();
                }
                command_cv.notify_one();

                write_Correction(socketServer, ipoc, buffer);

                buffer = "";

            }
            else
            {
                write_noCorrection(socketServer, ipoc);
            }

        }

        sched_yield();
    }
}

void overrun_thread() {
    while (true) {
        std::cout << overrun_counter_robot << "," << get_current_time() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(RUNTIME_OVERRUN_TIMER));
    }
}


int main() {

    setup_signal_handler();

    std::thread t1(robot);
    std::thread t2(overrun_thread);

    t1.join();
    t2.join();

    return 0;
}