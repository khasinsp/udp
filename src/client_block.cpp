#include <bits/stdc++.h>
#include <linux/sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/net_tstamp.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/sockios.h>
// #include "../include/kuka_rsi_hw_interface/AxesMapper.h"
// #include "../include/kuka_rsi_hw_interface/rsi_command.h"
// #include "../include/kuka_rsi_hw_interface/tcp_server.h"


#define SERVER_IP "172.29.3.25" // The IP address of the server (VM)
#define PORT 6008
#define BUFFER_SIZE 2048
#define RT_THRESHOLD 500
#define PRINT_THRESHOLD 1000
#define PERIOD 1000

std::vector<unsigned long long> hist(1000, 0);
std::vector<unsigned long long> hw_hist(1000, 0);

bool log_rt;


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

void sigxcpu_handler(int signum) {
    // Use thread-local storage or other mechanisms to identify the thread
    // For example, using pthread_self()
    std::cerr << "Runtime overrun detected: Task exceeded allocated runtime" << std::endl;
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

struct sched_attr {
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

void set_CPU(size_t cpu)
{
    cpu_set_t mask;
    CPU_ZERO(&mask);

    CPU_SET(cpu, &mask);

    // pid = 0 means "calling process"
    if (sched_setaffinity(0, sizeof(mask), &mask) == -1)
    {
        std::cerr << "Error setting CPU affinity: "
                  << std::strerror(errno) << std::endl;
    }
}

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

struct sockaddr_in serv_addr, local_addr;
socklen_t serv_len = sizeof(serv_addr);

void setup_connection(int &sock) {


    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cout << "Error opening socket" << std::endl;
    }

    struct ifreq ifr;
    struct hwtstamp_config hwcfg;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, "ens64", IFNAMSIZ);
    ifr.ifr_data = (char*)&hwcfg;
    memset(&hwcfg, 0, sizeof(hwcfg));
    hwcfg.flags = 0;
    hwcfg.tx_type = HWTSTAMP_TX_ON;
    hwcfg.rx_filter = HWTSTAMP_FILTER_ALL;
    if(ioctl(sock, SIOCSHWTSTAMP, &ifr) < 0) {
        perror("SIOCSHWTSTAMP");
        close(sock);
        return;
    }

    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval, sizeof(int));
    
    int flags = SOF_TIMESTAMPING_TX_HARDWARE | 
                SOF_TIMESTAMPING_RX_HARDWARE | 
                SOF_TIMESTAMPING_RAW_HARDWARE;
    if (setsockopt(sock, SOL_SOCKET, SO_TIMESTAMPING, &flags, sizeof(flags)) < 0) {
        perror("SO_TIMESTAMPING");
        close(sock);
        return;
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = 0;
    if (inet_pton(AF_INET, "172.29.3.28", &local_addr.sin_addr) <= 0) {
        perror("Invalid Local Address");
        close(sock);
        return;
    }

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        perror("bind failed");
        return;
    }

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0)
    {
        perror("Invalid address/Address not supported");
        return;
    }
}


void hist_to_csv(std::ofstream* csv_ptr, std::vector<unsigned long long> *vec_ptr) {

    set_CPU(1);
    set_realtime_priority(99);
    while (true) {
        for (int i = 0; i < (*vec_ptr).size(); i++) {
            (*csv_ptr) << (*vec_ptr)[i];
            if (i < (*vec_ptr).size() - 1) (*csv_ptr) << ",";
        }
        (*csv_ptr) << "\n";
        (*csv_ptr).flush();

        std::this_thread::sleep_for(std::chrono::seconds(30));
    }

}


void add_to_hist(std::vector<unsigned long long> &vec, double t, double res, std::string type) {
    int idx = t / res;
    if (idx >= 0 && idx < vec.size()) vec[idx]++;

    if (type == "HW") {
        if (t > RT_THRESHOLD) std::cout << "HW," << "RTT," << get_current_time() << "," << t << std::endl;
        if (t > PRINT_THRESHOLD) std::cout << "HW," << "OUT," << get_current_time() << "," << t << std::endl;
    }
    if (type == "NO_HW") {
        if (t > RT_THRESHOLD) std::cout << "NO_HW," << "RTT," << get_current_time() << "," << t << std::endl;
        if (t > PRINT_THRESHOLD) std::cout << "NO_HW," << "OUT," << get_current_time() << "," << t << std::endl;
    }
}


bool check_command(std::string str) {
    if (str.size() < 10) return false;

    return (str.compare(str.size() - 6, 6, "</Sen>") == 0 && str.compare(0, 4, "<Sen") == 0);
}

unsigned long long delta_hw(timespec tx_timespec, timespec rx_timespec) {
    return (rx_timespec.tv_sec - tx_timespec.tv_sec) * 1e9 + (rx_timespec.tv_nsec - tx_timespec.tv_nsec);
}

void main_loop() {

    set_CPU(0);

    std::chrono::high_resolution_clock::time_point rt_ts_start;
    std::chrono::high_resolution_clock::time_point rt_ts_end;
    double rt;

    std::string buffer = "";

    const char *msg = R"(<Rob Type="KUKA">
	<Dat TaskType="b">
		<ComStatus>continuous</ComStatus>
		<RIst X="93.9531" Y="-347.9674" Z="1844.9703" A="-126.9460" B="24.9226" C="-28.2871"/>
		<RSol X="93.9542" Y="-347.9674" Z="1844.9702" A="-126.9459" B="24.9224" C="-28.2874"/>
		<AIPos A1="74.8478" A2="-128.7977" A3="75.8236" A4="102.2260" A5="0.0987" A6="-56.5347"/>
		<ASPos A1="74.8478" A2="-128.7977" A3="75.8236" A4="102.2260" A5="0.0983" A6="-56.5347"/>
		<EIPos E1="226966.9326" E2="0.0000" E3="0.0000" E4="0.0000" E5="0.0000" E6="0.0000"/>
		<ESPos E1="226966.9375" E2="0.0000" E3="0.0000" E4="0.0000" E5="0.0000" E6="0.0000"/>
		<MACur A1="0.1143" A2="-1.5825" A3="-0.5625" A4="0.0793" A5="-0.0178" A6="0.0437"/>
		<MECur E1="-0.0005" E2="0.0000" E3="0.0000" E4="0.0000" E5="0.0000" E6="0.0000"/>
		<IPOC>680309708</IPOC>
		<BMode>3</BMode>
		<IPOStat>65</IPOStat>
		<Tech x="1" p6="0.0000" p7="0.0000" p8="0.0000" p6x1="0.0000" p7x1="0.0000" p8x1="0.0000" p6x2="0.0000" p7x2="0.0000" p8x2="0.0000" p6x3="0.0000" p7x3="0.0000" p8x3="0.0000"/>
		<RGH X="0" Y="0" Z="0" A="0" B="0" C="0" T="00000"/>
		<DiI>0</DiI>
		<Tick>0000000000000000</Tick>
		<RWMode>C</RWMode>
	</Dat>
</Rob>
)";

    char in_buffer[BUFFER_SIZE];

    int sock = 0;
    setup_connection(sock);

    set_realtime_priority(99);

    while (true) {

        auto start = std::chrono::high_resolution_clock::now();

        struct msghdr send_msg;
        std::memset(&send_msg, 0, sizeof(send_msg));
        struct iovec send_iov;
        send_iov.iov_base = (void*)msg;
        send_iov.iov_len = std::strlen(msg);
        send_msg.msg_iov = &send_iov;
        send_msg.msg_iovlen = 1;
        send_msg.msg_name = &serv_addr;
        send_msg.msg_namelen = sizeof(serv_addr);


        std::vector<std::chrono::high_resolution_clock::time_point> before_recv;
        std::vector<std::chrono::high_resolution_clock::time_point> after_recv;

        ssize_t nsent = sendmsg(sock, &send_msg, 0);
        if (nsent < 0) {
            perror("Error sending data");
            close(sock);
            return;
        }

        rt_ts_start = std::chrono::high_resolution_clock::now();

        char ctrl_buf[512];
        std::memset(ctrl_buf, 0, sizeof(ctrl_buf));
        char dummy_buf[1]; // Platzhalter
        struct iovec dummy_iov;
        dummy_iov.iov_base = dummy_buf;
        dummy_iov.iov_len = sizeof(dummy_buf);
        struct msghdr err_msg;
        std::memset(&err_msg, 0, sizeof(err_msg));
        err_msg.msg_control = ctrl_buf;
        err_msg.msg_controllen = sizeof(ctrl_buf);
        err_msg.msg_iov = &dummy_iov;
        err_msg.msg_iovlen = 1;

        struct pollfd pfd;
        pfd.fd = sock;
        // std::cout << "Waiting until poll returns" << std::endl;
        int poll_ret = poll(&pfd, 1, -1);
        if (poll_ret < 0) {
            perror("poll");
            return;
        }

        if (pfd.revents & POLLERR) {
            ssize_t ret = recvmsg(sock, &err_msg, MSG_ERRQUEUE);
            if (ret < 0) {
                perror("recvmsg(ERRQUEUE)");
                close(sock);
                return;
                
            }
        }

        timespec tx_timestamp;
        bool found_tx_timestamp = false;
        for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&err_msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&err_msg, cmsg)) {
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPING) {
                // Die übergebenen Daten enthalten ein Array von 3 timespec-Strukturen:
                // [0]: Software-Timestamp, [1]: Hardware-Timestamp, [2]: Raw-Hardware-Timestamp.
                struct timespec *ts = (struct timespec *) CMSG_DATA(cmsg);
                tx_timestamp = ts[2]; // Hardware-Timestamp verwenden
                found_tx_timestamp = true;
                break;
            }
        }

        if (!found_tx_timestamp) {
            std::cerr << "TX-Timestamp nicht gefunden" << std::endl;
            close(sock);
            return;
        }

        char recv_buf[BUFFER_SIZE];
        char ctrl_buf_rx[512];
        std::memset(&recv_buf, 0, sizeof(recv_buf));
        std::memset(&ctrl_buf_rx, 0, sizeof(ctrl_buf_rx));
        struct msghdr recv_msg;
        std::memset(&recv_msg, 0, sizeof(recv_msg));
        struct iovec recv_iov;
        recv_iov.iov_base = recv_buf;
        recv_iov.iov_len = sizeof(recv_buf);
        recv_msg.msg_iov = &recv_iov;
        recv_msg.msg_iovlen = 1;
        recv_msg.msg_control = ctrl_buf_rx;
        recv_msg.msg_controllen = sizeof(ctrl_buf_rx);

        ssize_t nrecv = recvmsg(sock, &recv_msg, 0);
        if (nrecv < 0) {
            perror("recvmsg");
            close(sock);
            return;
        }

        if (!check_command(std::string(recv_buf))) {
            std::cerr << "Command not complete" << std::endl;
            std::cout << std::string(recv_buf) << std::endl;
            close(sock);
            return;
        }

        rt_ts_end = std::chrono::high_resolution_clock::now();

        timespec rx_timestamp;
        bool found_rx_timestamp = false;
        for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&recv_msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&recv_msg, cmsg)) {
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPING) {
                struct timespec *ts = (struct timespec *) CMSG_DATA(cmsg);
                rx_timestamp = ts[2]; // Hardware-Timestamp verwenden
                found_rx_timestamp = true;
                break;
            }
        }
        if (!found_rx_timestamp) {
            std::cerr << "RX-Timestamp nicht gefunden" << std::endl;
            close(sock);
            return;
        }

        double delta = delta_hw(tx_timestamp, rx_timestamp) / 1000.0;
        add_to_hist(hw_hist, delta, 1.0, "HW");
        

        rt = std::chrono::duration_cast<std::chrono::microseconds>(rt_ts_end - rt_ts_start).count();
        add_to_hist(hist, rt, 1.0, "NO_HW");

        auto end = std::chrono::high_resolution_clock::now();
        auto t_rem = PERIOD - std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        if (t_rem < 0) {
            std::cout << "Period Overrun: " << 1000 - t_rem << std::endl;
        }
        else {
            std::this_thread::sleep_for(std::chrono::microseconds(t_rem));
        }
    }

    close(sock);
    return;
}




int main() {

    // RTT
    std::ofstream hist_csv("/home/urc/UDP_Client/response_times/PCI_tests/blocking/02_04_1/hist.csv");
    if (!hist_csv.is_open()) {
        std::cerr << "Histogram CSV could not be opened" << std::endl;
    }
    for (int i = 0; i < hist.size(); i++) {
        hist_csv << i;
        if (i < hist.size() - 1) hist_csv << ",";
    }
    hist_csv << "\n";
    hist_csv.flush();

    // RTT
    std::ofstream hw_hist_csv("/home/urc/UDP_Client/response_times/PCI_tests/blocking/02_04_1/hw_hist.csv");
    if (!hw_hist_csv.is_open()) {
        std::cerr << "Histogram CSV could not be opened" << std::endl;
    }
    for (int i = 0; i < hw_hist.size(); i++) {
        hw_hist_csv << i;
        if (i < hw_hist.size() - 1) hw_hist_csv << ",";
    }
    hw_hist_csv << "\n";
    hw_hist_csv.flush();


    std::thread main_(main_loop);
    std::thread hist_1(hist_to_csv, &hist_csv, &hist);
    std::thread hist_2(hist_to_csv, &hw_hist_csv, &hw_hist);


    main_.join();
    hist_1.join();
    hist_2.join();

    return 0;

}
