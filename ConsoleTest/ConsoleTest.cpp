#include <iostream>
#include <string>
#include <limits>
#include <chrono>
#include <cstring>              // [MOD] 为 std::memcpy
#include "NokovSDKClient.h"

// [MOD] 仅保留 Windows 所需的头文件与库
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

using namespace std::chrono;

// [MOD] 配置：PLC 的 IP 与端口（根据你的 S7-1200 TCON/TSEND 设置修改）
static const char* PLC_IP = "192.168.1.10";
static const int   PLC_PORT = 2000;

// [MOD] 简单的全局 socket（学习用途，不做复杂并发/重连）
static SOCKET g_sock = INVALID_SOCKET;
static bool   g_sock_ok = false;

// [修正] 避免与 WinSock2.h 中的 htonf 冲突，重命名为 my_htonf
static inline uint32_t my_htonf(float f)
{
    uint32_t u;
    static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32-bit");
    std::memcpy(&u, &f, sizeof(float));
    u = htonl(u);
    return u;
}

// [MOD] 将 6 个 float（cx,cy,cz,vx,vy,vz）打包并通过 TCP 发送（无复杂错误处理）
static inline void SendSixFloats(float cx, float cy, float cz, float vx, float vy, float vz)
{
    if (!g_sock_ok) return;

    uint32_t buf[6];
    buf[0] = my_htonf(cx);
    buf[1] = my_htonf(cy);
    buf[2] = my_htonf(cz);
    buf[3] = my_htonf(vx);
    buf[4] = my_htonf(vy);
    buf[5] = my_htonf(vz);

    send(g_sock, reinterpret_cast<const char*>(buf), sizeof(buf), 0);
}

void DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    // 取 11/12/13 号标记点（1-based），数组下标分别为 10/11/12
    float x11 = data->MocapData[0].Markers[10][0];
    float y11 = data->MocapData[0].Markers[10][1];
    float z11 = data->MocapData[0].Markers[10][2];

    float x12 = data->MocapData[0].Markers[11][0];
    float y12 = data->MocapData[0].Markers[11][1];
    float z12 = data->MocapData[0].Markers[11][2];

    float x13 = data->MocapData[0].Markers[12][0];
    float y13 = data->MocapData[0].Markers[12][1];
    float z13 = data->MocapData[0].Markers[12][2];

    // 中心点位置
    float cx = (x11 + x12 + x13) / 3.0f;
    float cy = (y11 + y12 + y13) / 3.0f;
    float cz = (z11 + z12 + z13) / 3.0f;

    // 速度（相邻回调差分，间隔尽量小）
    static bool has_last = false;
    static float last_cx = 0.0f, last_cy = 0.0f, last_cz = 0.0f;
    static steady_clock::time_point last_sample_tp = steady_clock::now();

    steady_clock::time_point now_tp = steady_clock::now();
    float dt = duration_cast<duration<float>>(now_tp - last_sample_tp).count(); // 秒

    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    if (has_last && dt > 0.0f) {
        vx = (cx - last_cx) / dt;
        vy = (cy - last_cy) / dt;
        vz = (cz - last_cz) / dt;
    }
    // 更新“上一次”样本（用于下次差分）
    last_cx = cx; last_cy = cy; last_cz = cz;
    last_sample_tp = now_tp;
    has_last = true;

    // 50 ms 打印一次（只改打印频率，不改变数据更新频率）
    static steady_clock::time_point last_print_tp = steady_clock::now();
    if (duration_cast<milliseconds>(now_tp - last_print_tp).count() >= 50) {
        std::cout << "Center Pos: (" << cx << ", " << cy << ", " << cz
            << ")  Vel: (" << vx << ", " << vy << ", " << vz << ")\n";
        last_print_tp = now_tp;

        // [MOD] 在同一节拍（每 50ms）把 6 个浮点数通过 TCP 发送到 S7-1200
        SendSixFloats(cx, cy, cz, vx, vy, vz);
    }
}

int main()
{
    // [MOD] 初始化 Winsock（简化版，无复杂错误处理）
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);

    g_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_sock != INVALID_SOCKET) {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(PLC_PORT);
        InetPtonA(AF_INET, PLC_IP, &addr.sin_addr);

        // 学习用途：忽略连接失败重试，仅做一次尝试
        if (connect(g_sock, (sockaddr*)&addr, sizeof(addr)) == 0) {
            g_sock_ok = true;
            std::cout << "[TCP] Connected to PLC " << PLC_IP << ":" << PLC_PORT << "\n";
        }
        else {
            std::cout << "[TCP] Connect failed, no network sending.\n";
        }
    }
    else {
        std::cout << "[TCP] Socket create failed, no network sending.\n";
    }

    NokovSDKClient* theClient = new NokovSDKClient();
    theClient->Initialize(const_cast<char*>("192.168.1.201"));
    theClient->SetDataCallback(DataHandler, theClient);

    std::cout << "Press ENTER to quit...\n";
    std::cin.clear();
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    std::string dummy;
    std::getline(std::cin, dummy);

    theClient->Uninitialize();

    // [MOD] 关闭 TCP（简化处理）
    if (g_sock != INVALID_SOCKET) {
        closesocket(g_sock);
        g_sock = INVALID_SOCKET;
        g_sock_ok = false;
    }
    WSACleanup();
    return 0;
}
