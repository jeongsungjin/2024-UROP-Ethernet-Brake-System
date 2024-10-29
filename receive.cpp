#include <iostream>
#include <cstring>     // memset
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>    // close
#include <fcntl.h>     // fcntl
#include <chrono>      // high_resolution_clock
#include <iomanip>     // std::setw
#include <errno.h>     // errno
#include <time.h>      // clock_nanosleep
#include <pigpio.h>
#include <string>

#define PORT 8080
#define PACKET_SIZE 8
#define LOOP_INTERVAL_MS 5  // 루프 주기 (5ms)
#define PACKET_BUFFER_SIZE 32

struct Packet {
    uint32_t sequence;
    char pedal_val[PACKET_BUFFER_SIZE];
};

int main() {
    int sock1, sock2;
    struct sockaddr_in addr1, addr2;
    const int ESC_PIN = 18;
    int rc_speed = 1500;
    char type;
    char value;

    if (gpioInitialise() < 0) {
        std::cerr << "pigpio 초기화 실패 ㅋㅋ" << std::endl;
        return 1;
    }

    // 소켓 생성
    if ((sock1 = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "sock1 생성 실패" << std::endl;
        return -1;
    }
    if ((sock2 = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "sock2 생성 실패" << std::endl;
        close(sock1);
        return -1;
    }

    // sock1 바인드 (192.168.1.200:8080)
    memset(&addr1, 0, sizeof(addr1));
    addr1.sin_family = AF_INET;
    addr1.sin_port = htons(PORT);
    inet_pton(AF_INET, "192.168.1.200", &addr1.sin_addr);

    if (bind(sock1, (struct sockaddr *)&addr1, sizeof(addr1)) < 0) {
        std::cerr << "sock1 바인드 실패" << std::endl;
        close(sock1);
        close(sock2);
        return -1;
    }

    // sock2 바인드 (192.168.1.201:8080)
    memset(&addr2, 0, sizeof(addr2));
    addr2.sin_family = AF_INET;
    addr2.sin_port = htons(PORT);
    inet_pton(AF_INET, "192.168.1.201", &addr2.sin_addr);

    if (bind(sock2, (struct sockaddr *)&addr2, sizeof(addr2)) < 0) {
        std::cerr << "sock2 바인드 실패" << std::endl;
        close(sock1);
        close(sock2);
        return -1;
    }

    // 소켓을 논블로킹 모드로 설정
    int flags1 = fcntl(sock1, F_GETFL, 0);
    fcntl(sock1, F_SETFL, flags1 | O_NONBLOCK);

    int flags2 = fcntl(sock2, F_GETFL, 0);
    fcntl(sock2, F_SETFL, flags2 | O_NONBLOCK);

    uint32_t total_sequences = 0;
    auto total_start_time = std::chrono::high_resolution_clock::now();
    auto last_packet_time = std::chrono::high_resolution_clock::now();

    // **마지막으로 "z,0" 패킷을 출력한 시간 기록**
    auto last_z0_print_time = std::chrono::steady_clock::now();

    while (true) {
        auto loop_start = std::chrono::high_resolution_clock::now();

        bool packet_received = false;
        Packet packet;

        // 더 빠르게 도착한 소켓에서 패킷 수신
        ssize_t len1 = recvfrom(sock1, &packet, sizeof(packet), 0, NULL, NULL);
        ssize_t len2 = recvfrom(sock2, &packet, sizeof(packet), 0, NULL, NULL);

        if (len1 > 0 || len2 > 0) {
            auto packet_time = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(packet_time - last_packet_time).count();

            type = packet.pedal_val[0];
            value = packet.pedal_val[2];

            // **"z,0" 패킷 처리**
            if (packet.pedal_val[0] == 'z' && packet.pedal_val[1] == ',' && packet.pedal_val[2] == '0') {
                // 1초마다 한 번만 출력
                auto now = std::chrono::steady_clock::now();
                double seconds_since_last_z0 = std::chrono::duration<double>(now - last_z0_print_time).count();
                if (seconds_since_last_z0 >= 1.0) {
                    std::cout << "z,0 패킷 수신됨: " << packet.pedal_val << std::endl;
                    last_z0_print_time = now;
                }
            } else {
                // "a,..." 또는 "b,..." 패킷은 바로 처리
                std::cout << "수신된 패킷 시퀀스 번호: " << packet.sequence << " 수신된 정보: " << packet.pedal_val << " | 걸린 시간: " << elapsed_ms << " ms" << std::endl;

                if(type == 'a') {
                    rc_speed = rc_speed + (int)(value - '0');
                }
                else if(type == 'b') {
                    rc_speed = rc_speed - 50*(int)(value - '0');
                }
                if(rc_speed > 1800)
                    rc_speed = 1800;
                else if(rc_speed <1400)
                    rc_speed = 1400;
                if(rc_speed <= 1800 && rc_speed >= 1400) {
                    gpioServo(ESC_PIN, rc_speed);
                    std::cout << "rc_speed: " << rc_speed << std::endl;
                }
            }

            last_packet_time = packet_time;
            packet_received = true;

            // 남은 패킷 버리기
            char buffer[PACKET_SIZE];
            while (recvfrom(sock1, buffer, sizeof(buffer), 0, NULL, NULL) > 0) {}
            while (recvfrom(sock2, buffer, sizeof(buffer), 0, NULL, NULL) > 0) {}
        } else {
            if (errno != EWOULDBLOCK && errno != EAGAIN) {
                std::cerr << "패킷 수신 오류" << std::endl;
            }
            // 데이터가 전혀 수신되지 않은 경우에는 별도 처리를 하지 않음
        }

        if (packet_received) {
            total_sequences++;
        }

        // 1000개의 시퀀스마다 총 걸린 시간 출력
        if (total_sequences > 0 && total_sequences % 1000 == 0) {
            auto now = std::chrono::high_resolution_clock::now();
            double total_elapsed_ms = std::chrono::duration<double, std::milli>(now - total_start_time).count();
            std::cout << "1000개의 시퀀스 처리 완료 | 걸린 시간: " << total_elapsed_ms << " ms" << std::endl;

            // 총 시간 초기화
            total_start_time = now;
        }

        // 루프 주기 제어 (5ms) - clock_nanosleep 사용
        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = LOOP_INTERVAL_MS * 1000000;  // 5ms를 나노초로 변환

        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
    }
    gpioTerminate();
    close(sock1);
    close(sock2);

    return 0;
}



