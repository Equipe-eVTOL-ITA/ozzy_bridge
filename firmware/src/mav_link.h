// =============================================================================
// mav_link.h — o lado MAVLink da ponte. Não sabe que ROS 2 existe.
// =============================================================================
//
// Esta classe faz duas coisas e nada mais:
//   1. consome bytes da UART2 e mantém um cache do estado do veículo;
//   2. empacota comandos para a Pixhawk.
//
// Tudo aqui está em NED/FRD e nas unidades do MAVLink. A conversão para ROS 2
// acontece em ros_link.cpp, usando frames.h. Manter a separação é o que impede
// a conversão de vazar para sete lugares.
// =============================================================================
#pragma once

#include <Arduino.h>
#include <common/mavlink.h>

class MavLink {
public:
    // STATUSTEXT do MAVLink tem 50 caracteres, sem '\0' garantido.
    static constexpr size_t STATUSTEXT_BUF = 52;

    // Cache do que a Pixhawk contou. Tudo em NED/FRD, unidades do MAVLink
    // já normalizadas para SI (o MAVLink manda centímetros, milivolts e
    // graus*1e7; nada disso sobrevive a este struct).
    struct State {
        // --- identidade do autopiloto, aprendida do primeiro HEARTBEAT ------
        uint8_t  target_sysid  = 1;
        uint8_t  target_compid = MAV_COMP_ID_AUTOPILOT1;
        bool     seen_autopilot = false;

        // --- HEARTBEAT ------------------------------------------------------
        bool     armed       = false;
        uint32_t custom_mode = 0;
        uint32_t hb_ms       = 0;   // millis() do último HEARTBEAT

        // --- ATTITUDE (rad, rad/s, FRD) -------------------------------------
        float roll = 0, pitch = 0, yaw = 0;
        float rollspeed = 0, pitchspeed = 0, yawspeed = 0;
        uint32_t att_ms = 0;

        // --- LOCAL_POSITION_NED (m, m/s, NED) -------------------------------
        float x = 0, y = 0, z = 0;
        float vx = 0, vy = 0, vz = 0;
        uint32_t lpos_ms = 0;

        // --- GPS -------------------------------------------------------------
        double  lat = 0, lon = 0;   // graus
        // Altitude MSL, que é o que o GLOBAL_POSITION_INT manda. O
        // sensor_msgs/NavSatFix pede altitude sobre o elipsoide WGS84 — a
        // diferença é a ondulação do geoide, dezenas de metros no Brasil.
        // Publicamos MSL mesmo assim, porque é o dado que existe; quem usar
        // este campo para algo que dependa do datum precisa saber disso.
        float   alt = 0;            // m, MSL
        uint8_t fix_type = 0;       // 0-1 sem fix, 2 = 2D, 3 = 3D
        uint8_t satellites = 0;
        float   eph = 0;            // HDOP
        uint32_t gps_ms = 0;

        // --- SYS_STATUS ------------------------------------------------------
        float voltage = 0;          // V
        float current = 0;          // A  (negativo = desconhecido)
        int8_t battery_pct = -1;    // %  (-1 = desconhecido)
        uint32_t sys_ms = 0;

        // --- saúde do link ---------------------------------------------------
        uint32_t rx_count     = 0;  // mensagens válidas desde o boot
        uint32_t parse_errors = 0;
        uint32_t last_any_ms  = 0;
    };

    void begin();

    // Consome tudo que estiver na UART. Chame todo loop: o buffer da UART2 tem
    // 256 bytes e, a 115200 com os streams de docs/ARDUPILOT.md, ele enche em
    // ~22 ms. Um loop que demore mais que isso perde mensagens, e o sintoma é
    // `mav_parse_err` subindo sem motivo aparente.
    void poll();

    const State& state() const { return state_; }

    // Mensagens/s recebidas, medida em janela de 1 s.
    float rxHz() const { return rx_hz_; }

    // STATUSTEXT pendente. Devolve false se não há nada novo. O buffer guarda
    // UMA mensagem: se duas chegarem entre duas chamadas, a segunda vence.
    // É deliberado — a alternativa é uma fila que enche calada.
    bool takeStatusText(char* out, size_t cap);

    // --- comandos --------------------------------------------------------------
    void sendHeartbeat();
    void sendArm(bool arm);
    void sendSetMode(uint32_t custom_mode);
    void sendTakeoff(float altitude_m);

    // Velocidade de mundo em NED + taxa de guinada NED. Posição ignorada.
    void sendVelocityNed(float vx, float vy, float vz, float yaw_rate);

    // Posição de mundo em NED + guinada NED. Velocidade ignorada.
    void sendPositionNed(float x, float y, float z, float yaw);

    // Nome do modo do ArduCopter a partir do custom_mode, e o caminho inverso.
    static const char* modeName(uint32_t custom_mode, char* scratch, size_t cap);
    static bool modeNumber(const char* name, uint32_t* out);

private:
    void handle_(const mavlink_message_t& msg);
    void send_(mavlink_message_t& msg);
    void sendCommandLong_(uint16_t command, float p1, float p2, float p3,
                          float p4, float p5, float p6, float p7);

    HardwareSerial* uart_ = nullptr;
    State state_;

    // janela de 1 s para rxHz()
    uint32_t rx_window_start_ = 0;
    uint32_t rx_window_count_ = 0;
    float    rx_hz_ = 0.0f;

    char statustext_[STATUSTEXT_BUF] = {0};
    bool statustext_pending_ = false;
};
