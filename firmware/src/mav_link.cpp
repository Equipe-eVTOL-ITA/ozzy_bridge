#include "mav_link.h"
#include "config.h"

#include <string.h>
#include <strings.h>   // strcasecmp

// -----------------------------------------------------------------------------
// Máscaras de SET_POSITION_TARGET_LOCAL_NED
// -----------------------------------------------------------------------------
// Cada bit em 1 diz "ignore este campo". A ordem dos bits é
// x y z | vx vy vz | ax ay az | force | yaw | yaw_rate.
//
// Estes dois valores são os canônicos da documentação do ArduPilot. Estão
// escritos em binário de propósito: em hexadecimal ninguém confere, e um bit
// errado aqui não dá erro — dá um veículo que ignora o comando em silêncio,
// ou pior, que obedece o campo errado.
static const uint16_t MASK_VEL_YAWRATE = 0b0000011111000111;  // usa vx,vy,vz e yaw_rate
static const uint16_t MASK_POS_YAW     = 0b0000101111111000;  // usa x,y,z e yaw

// -----------------------------------------------------------------------------
// Modos do ArduCopter. Plane e Rover usam OUTRA tabela.
// -----------------------------------------------------------------------------
struct ModeEntry { uint32_t num; const char* name; };
static const ModeEntry kModes[] = {
    { 0,  "STABILIZE"   }, { 1,  "ACRO"     }, { 2,  "ALT_HOLD"  },
    { 3,  "AUTO"        }, { 4,  "GUIDED"   }, { 5,  "LOITER"    },
    { 6,  "RTL"         }, { 7,  "CIRCLE"   }, { 9,  "LAND"      },
    { 11, "DRIFT"       }, { 13, "SPORT"    }, { 14, "FLIP"      },
    { 15, "AUTOTUNE"    }, { 16, "POSHOLD"  }, { 17, "BRAKE"     },
    { 18, "THROW"       }, { 20, "GUIDED_NOGPS" }, { 21, "SMART_RTL" },
};
static const size_t kNumModes = sizeof(kModes) / sizeof(kModes[0]);

const char* MavLink::modeName(uint32_t custom_mode, char* scratch, size_t cap) {
    for (size_t i = 0; i < kNumModes; ++i) {
        if (kModes[i].num == custom_mode) return kModes[i].name;
    }
    // Modo que não conhecemos sai como MODE_<n>, nunca vazio: uma string vazia
    // em /ozzy/mode seria indistinguível de "ainda não recebi HEARTBEAT".
    snprintf(scratch, cap, "MODE_%lu", (unsigned long)custom_mode);
    return scratch;
}

bool MavLink::modeNumber(const char* name, uint32_t* out) {
    for (size_t i = 0; i < kNumModes; ++i) {
        if (strcasecmp(kModes[i].name, name) == 0) { *out = kModes[i].num; return true; }
    }
    return false;
}

// -----------------------------------------------------------------------------

void MavLink::begin() {
    uart_ = &Serial2;
    uart_->begin(MAV_BAUD, SERIAL_8N1, MAV_PIN_RX, MAV_PIN_TX);
    rx_window_start_ = millis();
}

void MavLink::poll() {
    mavlink_message_t msg;
    mavlink_status_t  status;

    while (uart_ && uart_->available()) {
        const uint8_t c = (uint8_t)uart_->read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            state_.rx_count++;
            state_.last_any_ms = millis();
            rx_window_count_++;
            handle_(msg);
        }
    }

    // parse_error é cumulativo dentro do próprio status do canal.
    state_.parse_errors = mavlink_get_channel_status(MAVLINK_COMM_0)->parse_error;

    const uint32_t now = millis();
    if (now - rx_window_start_ >= 1000) {
        rx_hz_ = rx_window_count_ * 1000.0f / (float)(now - rx_window_start_);
        rx_window_count_ = 0;
        rx_window_start_ = now;
    }
}

void MavLink::handle_(const mavlink_message_t& msg) {
    const uint32_t now = millis();

    switch (msg.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);

        // Ignora batimentos que não sejam do autopiloto — a própria GCS e
        // outros componentes também mandam HEARTBEAT no mesmo barramento.
        if (hb.type == MAV_TYPE_GCS) break;

        if (!state_.seen_autopilot) {
            state_.target_sysid   = msg.sysid;
            state_.target_compid  = msg.compid;
            state_.seen_autopilot = true;
        }
        state_.armed       = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        state_.custom_mode = hb.custom_mode;
        state_.hb_ms       = now;
        break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t a;
        mavlink_msg_attitude_decode(&msg, &a);
        state_.roll = a.roll;   state_.pitch = a.pitch;  state_.yaw = a.yaw;
        state_.rollspeed = a.rollspeed;
        state_.pitchspeed = a.pitchspeed;
        state_.yawspeed = a.yawspeed;
        state_.att_ms = now;
        break;
    }

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t p;
        mavlink_msg_local_position_ned_decode(&msg, &p);
        state_.x = p.x;   state_.y = p.y;   state_.z = p.z;
        state_.vx = p.vx; state_.vy = p.vy; state_.vz = p.vz;
        state_.lpos_ms = now;
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t g;
        mavlink_msg_global_position_int_decode(&msg, &g);
        state_.lat = g.lat / 1e7;          // graus*1e7 -> graus
        state_.lon = g.lon / 1e7;
        state_.alt = g.alt / 1000.0f;      // mm -> m
        state_.gps_ms = now;
        break;
    }

    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t g;
        mavlink_msg_gps_raw_int_decode(&msg, &g);
        state_.fix_type   = g.fix_type;
        state_.satellites = g.satellites_visible;
        state_.eph        = (g.eph == UINT16_MAX) ? -1.0f : g.eph / 100.0f;
        break;
    }

    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t s;
        mavlink_msg_sys_status_decode(&msg, &s);
        state_.voltage     = s.voltage_battery / 1000.0f;              // mV -> V
        state_.current     = (s.current_battery < 0) ? -1.0f
                                                     : s.current_battery / 100.0f;  // cA -> A
        state_.battery_pct = s.battery_remaining;                      // já em %, -1 = desconhecido
        state_.sys_ms      = now;
        break;
    }

    case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t t;
        mavlink_msg_statustext_decode(&msg, &t);
        // O campo do MAVLink NÃO garante terminador quando ocupa os 50 bytes.
        memcpy(statustext_, t.text, 50);
        statustext_[50] = '\0';
        statustext_pending_ = true;
        break;
    }

    default:
        break;
    }
}

bool MavLink::takeStatusText(char* out, size_t cap) {
    if (!statustext_pending_) return false;
    strncpy(out, statustext_, cap - 1);
    out[cap - 1] = '\0';
    statustext_pending_ = false;
    return true;
}

// -----------------------------------------------------------------------------
// Envio
// -----------------------------------------------------------------------------

void MavLink::send_(mavlink_message_t& msg) {
    if (!uart_) return;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_->write(buf, len);
}

void MavLink::sendHeartbeat() {
    mavlink_message_t msg;
    // Nos declaramos GCS com autopiloto INVALID: é o que uma estação de solo
    // faz, e é esse batimento que alimenta o FS_GCS_ENABLE do ArduPilot. Se
    // ele parar, o ArduPilot dispara o failsafe dele. Veja docs/ARDUPILOT.md.
    mavlink_msg_heartbeat_pack(MAV_MY_SYSID, MAV_MY_COMPID, &msg,
                               MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                               MAV_STATE_ACTIVE);
    send_(msg);
}

void MavLink::sendCommandLong_(uint16_t command, float p1, float p2, float p3,
                               float p4, float p5, float p6, float p7) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(MAV_MY_SYSID, MAV_MY_COMPID, &msg,
                                  state_.target_sysid, state_.target_compid,
                                  command, /*confirmation=*/0,
                                  p1, p2, p3, p4, p5, p6, p7);
    send_(msg);
}

void MavLink::sendArm(bool arm) {
    // param2 = 0. O valor mágico 21196 força armar pulando checagens, e não
    // aparece neste repositório de propósito: quando o veículo não arma, a
    // resposta está no /ozzy/statustext.
    sendCommandLong_(MAV_CMD_COMPONENT_ARM_DISARM, arm ? 1.0f : 0.0f,
                     0, 0, 0, 0, 0, 0);
}

void MavLink::sendSetMode(uint32_t custom_mode) {
    sendCommandLong_(MAV_CMD_DO_SET_MODE,
                     (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                     (float)custom_mode, 0, 0, 0, 0, 0);
}

void MavLink::sendTakeoff(float altitude_m) {
    sendCommandLong_(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m);
}

void MavLink::sendVelocityNed(float vx, float vy, float vz, float yaw_rate) {
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        MAV_MY_SYSID, MAV_MY_COMPID, &msg,
        millis(), state_.target_sysid, state_.target_compid,
        MAV_FRAME_LOCAL_NED, MASK_VEL_YAWRATE,
        0, 0, 0,              // posição — ignorada pela máscara
        vx, vy, vz,
        0, 0, 0,              // aceleração — ignorada
        0, yaw_rate);
    send_(msg);
}

void MavLink::sendPositionNed(float x, float y, float z, float yaw) {
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        MAV_MY_SYSID, MAV_MY_COMPID, &msg,
        millis(), state_.target_sysid, state_.target_compid,
        MAV_FRAME_LOCAL_NED, MASK_POS_YAW,
        x, y, z,
        0, 0, 0,              // velocidade — ignorada
        0, 0, 0,              // aceleração — ignorada
        yaw, 0);
    send_(msg);
}
