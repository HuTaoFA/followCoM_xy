// MocapToPlcApp.cpp
#include <iostream>
#include <string>
#include <limits>
#include <chrono>
#include <cstring>   // std::memcpy
#include <optional>

// --- 第三方 SDK 头 ---
#include "NokovSDKClient.h"

// --- Windows sockets ---
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

using namespace std;
using namespace std::chrono;

// ---------- 工具函数 ----------
static inline uint32_t my_htonf(float f) {
    uint32_t u;
    static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32-bit");
    std::memcpy(&u, &f, sizeof(float));
    return htonl(u);
}

// ---------- 1) WinSock 资源守卫 ----------
class WsaGuard {
public:
    WsaGuard() {
        WSADATA wsa{};
        ok_ = (WSAStartup(MAKEWORD(2, 2), &wsa) == 0);
        if (!ok_) std::cerr << "[WSA] Startup failed\n";
    }
    ~WsaGuard() {
        if (ok_) WSACleanup();
    }
    bool ok() const { return ok_; }
private:
    bool ok_{ false };
};

// ---------- 2) TCP 发送器（负责连接 & 发送 6 个 float） ----------
class TcpSender {
public:
    TcpSender(string ip, int port) : ip_(std::move(ip)), port_(port) {}

    bool connect_once() {
        if (sock_ != INVALID_SOCKET) return true;
        sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock_ == INVALID_SOCKET) {
            std::cerr << "[TCP] socket() failed\n";
            return false;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<u_short>(port_));
        if (InetPtonA(AF_INET, ip_.c_str(), &addr.sin_addr) != 1) {
            std::cerr << "[TCP] bad IP: " << ip_ << "\n";
            closesocket(sock_); sock_ = INVALID_SOCKET;
            return false;
        }
        if (::connect(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
            connected_ = true;
            std::cout << "[TCP] Connected to PLC " << ip_ << ":" << port_ << "\n";
        }
        else {
            std::cerr << "[TCP] Connect failed (no network sending)\n";
            closesocket(sock_); sock_ = INVALID_SOCKET;
        }
        return connected_;
    }

    bool send_six(float cx, float cy, float cz, float vx, float vy, float vz) {
        if (!connected_) return false;
        uint32_t buf[6]{
            my_htonf(cx), my_htonf(cy), my_htonf(cz),
            my_htonf(vx), my_htonf(vy), my_htonf(vz)
        };
        int sent = ::send(sock_, reinterpret_cast<const char*>(buf), sizeof(buf), 0);
        return (sent == sizeof(buf));
    }

    ~TcpSender() {
        if (sock_ != INVALID_SOCKET) {
            closesocket(sock_);
            sock_ = INVALID_SOCKET;
        }
    }

private:
    string ip_;
    int    port_;
    SOCKET sock_{ INVALID_SOCKET };
    bool   connected_{ false };
};

// ---------- 3) 运动学处理（中心点 + 速度） ----------
struct SixDoFSample {
    float cx{}, cy{}, cz{};
    float vx{}, vy{}, vz{};
};

class MotionProcessor {
public:
    // 输入三个点坐标，返回样本（首帧速度为0）
    SixDoFSample update(const float p11[3], const float p12[3], const float p13[3]) {
        float cx = (p11[0] + p12[0] + p13[0]) / 3.0f;
        float cy = (p11[1] + p12[1] + p13[1]) / 3.0f;
        float cz = (p11[2] + p12[2] + p13[2]) / 3.0f;

        auto now = steady_clock::now();
        float dt = has_last_ ? duration_cast<duration<float>>(now - last_tp_).count() : 0.0f;

        SixDoFSample s{};
        s.cx = cx; s.cy = cy; s.cz = cz;
        if (has_last_ && dt > 0.f) {
            s.vx = (cx - last_cx_) / dt;
            s.vy = (cy - last_cy_) / dt;
            s.vz = (cz - last_cz_) / dt;
        }
        else {
            s.vx = s.vy = s.vz = 0.0f;
        }

        last_cx_ = cx; last_cy_ = cy; last_cz_ = cz;
        last_tp_ = now; has_last_ = true;
        return s;
    }

private:
    bool has_last_{ false };
    float last_cx_{}, last_cy_{}, last_cz_{};
    steady_clock::time_point last_tp_{ steady_clock::now() };
};

// ---------- 4) 应用（承接 SDK 回调、做节流、打印 & 发送） ----------
class MocapToPlcApp {
public:
    MocapToPlcApp(string sdk_ip, string plc_ip, int plc_port, milliseconds print_interval = 50ms)
        : sdk_ip_(std::move(sdk_ip)),
        wsa_(),
        sender_(std::move(plc_ip), plc_port),
        print_interval_(print_interval) {
    }

    bool initialize() {
        if (!wsa_.ok()) return false;

        sender_.connect_once();

        client_ = std::make_unique<NokovSDKClient>();
        if (!client_) return false;

        client_->Initialize(const_cast<char*>(sdk_ip_.c_str()));
        client_->SetDataCallback(&MocapToPlcApp::DataHandlerThunk, this);
        return true;
    }

    void run_until_enter() {
        std::cout << "Press ENTER to quit...\n";
        std::cin.clear();
        std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
        std::string dummy;
        std::getline(std::cin, dummy);
    }

    void shutdown() {
        if (client_) {
            client_->Uninitialize();
            client_.reset();
        }
        // TcpSender & WsaGuard 由析构自动清理
    }

private:
    // SDK 回调静态桥，pUserData 即 this
    static void DataHandlerThunk(sFrameOfMocapData* data, void* pUserData) {
        auto* self = static_cast<MocapToPlcApp*>(pUserData);
        if (self) self->on_data(data);
    }

    void on_data(sFrameOfMocapData* data) {
        // 防御：假设至少 1 个刚体、且有足够的 markers
        if (!data || data->nMarkerSets <= 0) return;
        const auto& body = data->MocapData[0];
        // 需要 markers[10], [11], [12]
        const float* m11 = body.Markers[10];
        const float* m12 = body.Markers[11];
        const float* m13 = body.Markers[12];

        SixDoFSample s = motion_.update(m11, m12, m13);

        auto now = steady_clock::now();
        if (now - last_print_tp_ >= print_interval_) {
            std::cout << "Center Pos: (" << s.cx << ", " << s.cy << ", " << s.cz
                << ")  Vel: (" << s.vx << ", " << s.vy << ", " << s.vz << ")\n";
            last_print_tp_ = now;

            (void)sender_.send_six(s.cx, s.cy, s.cz, s.vx, s.vy, s.vz);
        }
    }

private:
    string sdk_ip_;
    WsaGuard wsa_;
    TcpSender sender_;
    unique_ptr<NokovSDKClient> client_;
    MotionProcessor motion_;
    milliseconds print_interval_{ 50 };
    steady_clock::time_point last_print_tp_{ steady_clock::now() };
};

// ---------- 5) main：组装 ----------
int main() {
    // 可换成配置/命令行
    const string SDK_IP = "192.168.1.201";
    const string PLC_IP = "192.168.1.10";
    const int    PLC_PORT = 2000;

    MocapToPlcApp app(SDK_IP, PLC_IP, PLC_PORT, 50ms);
    if (!app.initialize()) {
        std::cerr << "Initialization failed.\n";
        return 1;
    }
    app.run_until_enter();
    app.shutdown();
    return 0;
}
