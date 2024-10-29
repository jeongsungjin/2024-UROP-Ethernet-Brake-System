#include <iostream>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <libusb-1.0/libusb.h>
#include <mutex>
#include <condition_variable>

#define PORT 8080
#define PACKET_SIZE 16 // 최대로 사용할 패킷 사이즈
#define LOOP_INTERVAL_MS 5 // 루프 주기 (5ms)
#define VENDOR_ID 0x3416
#define PRODUCT_ID 0x1018
#define ENDPOINT_IN 0x82  // EP 2 IN (0x80 + endpoint number)
#define PACKET_BUFFER_SIZE 32 // 패킷 버퍼 크기

struct Packet {
    uint32_t sequence;
    char data[PACKET_BUFFER_SIZE]; // 데이터 크기 수정
};

// 공유 변수와 뮤텍스, 조건 변수 선언
std::vector<unsigned char> shared_data(PACKET_SIZE);
std::mutex data_mutex;
std::condition_variable data_cv;
bool data_available = false;
bool running = true;

void usb_read_thread(libusb_device_handle* deviceHandle) {
    while (running) {
        std::vector<unsigned char> data(PACKET_SIZE);
        int actual_length;
        int result;

        // USB 데이터 읽기 (타임아웃을 적절히 설정)
        result = libusb_interrupt_transfer(deviceHandle, ENDPOINT_IN, data.data(), PACKET_SIZE, &actual_length, 10);

        if (result == LIBUSB_SUCCESS) {
            // 뮤텍스를 사용하여 공유 데이터에 접근
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                shared_data = data;
                data_available = true;
            }
            data_cv.notify_one(); // 메인 루프에 데이터가 도착했음을 알림
        } else if (result == LIBUSB_ERROR_TIMEOUT) {
            // 타임아웃 발생 시 아무 것도 하지 않음
            continue;
        } else {
            std::cerr << "USB 읽기 오류: " << libusb_error_name(result) << std::endl;
            break;
        }

        // 너무 빠른 반복을 방지하기 위해 잠시 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main() {
    int sock;
    struct sockaddr_in serv_addr1, serv_addr2;

    // 소켓 생성
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "소켓 생성 실패" << std::endl;
        return -1;
    }

    // 첫 번째 수신부 주소 설정
    memset(&serv_addr1, 0, sizeof(serv_addr1));
    serv_addr1.sin_family = AF_INET;
    serv_addr1.sin_port = htons(PORT);
    inet_pton(AF_INET, "192.168.1.200", &serv_addr1.sin_addr);

    // 두 번째 수신부 주소 설정
    memset(&serv_addr2, 0, sizeof(serv_addr2));
    serv_addr2.sin_family = AF_INET;
    serv_addr2.sin_port = htons(PORT);
    inet_pton(AF_INET, "192.168.1.201", &serv_addr2.sin_addr);

    // libusb 초기화
    libusb_context *context = nullptr;
    libusb_device_handle *deviceHandle = nullptr;
    int result;
    result = libusb_init(&context);
    deviceHandle = libusb_open_device_with_vid_pid(context, VENDOR_ID, PRODUCT_ID);

    // 장치 활성화
    if (libusb_kernel_driver_active(deviceHandle, 0) == 1) {
        libusb_detach_kernel_driver(deviceHandle, 0);
    }
    result = libusb_claim_interface(deviceHandle, 0);
    uint32_t sequence = 0;

    // USB 읽기 스레드 시작
    std::thread usb_thread(usb_read_thread, deviceHandle);

    while (true) {
        std::unique_lock<std::mutex> lock(data_mutex);
        if (data_cv.wait_for(lock, std::chrono::milliseconds(LOOP_INTERVAL_MS), []{ return data_available; })) {
            // 데이터가 도착한 경우
            data_available = false;
            std::vector<unsigned char> data = shared_data;
            lock.unlock();

            Packet packet;
            // 악셀과 브레이크 동작 구분하여 데이터 설정
            if (data[2] == 0xff) { // 악셀 동작
                if ((int)data[1] != 255 && (int)data[1] != 15 && (int)data[1] != 0) {
                    snprintf(packet.data, sizeof(packet.data), "a,%d", abs(14 - (int)data[1]));
                    std::cout << "악셀 동작: " << packet.data << std::endl;

                    // 패킷 전송
                    packet.sequence = sequence++;
                    sendto(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&serv_addr1, sizeof(serv_addr1));
                    sendto(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&serv_addr2, sizeof(serv_addr2));
                }
            } else if (data[0] == 0xff) { // 브레이크 동작
                if ((int)data[3] != 255 && (int)data[3] != 15 && (int)data[3] != 0) {
                    snprintf(packet.data, sizeof(packet.data), "b,%d", abs(14 - (int)data[3]));
                    std::cout << "브레이크 동작: " << packet.data << std::endl;

                    // 패킷 전송
                    packet.sequence = sequence++;
                    sendto(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&serv_addr1, sizeof(serv_addr1));
                    sendto(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&serv_addr2, sizeof(serv_addr2));
                }
            }
        } else {
            // 데이터가 없을 경우
            lock.unlock();
            Packet packet;
            snprintf(packet.data, sizeof(packet.data), "z,%d", 0);
            //std::cout << "데이터 없음: " << packet.data << std::endl;

            packet.sequence = sequence++;
            sendto(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&serv_addr1, sizeof(serv_addr1));
            sendto(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&serv_addr2, sizeof(serv_addr2));
        }

        // 루프 주기 유지
        //std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_INTERVAL_MS));
    }

    // 프로그램 종료 시
    running = false;
    usb_thread.join();

    // 자원 해제
    libusb_release_interface(deviceHandle, 0);
    libusb_close(deviceHandle);
    libusb_exit(context);
    close(sock);

    return 0;
}


송신부 ㅋㅋ