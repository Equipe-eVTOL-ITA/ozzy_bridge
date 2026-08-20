#include "ros_link.h"
#include "config.h"
#include "frames.h"

#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <std_srvs/srv/set_bool.h>

#include <math.h>
#include <string.h>

// Falha na criação de uma entidade é fatal e acontece no boot — que é a hora
// certa de descobrir que o orçamento de entidades do micro-ROS estourou (veja
// docs/CONTRATO.md). Falha na publicação é rotina num link de rádio e não pode
// derrubar nada.
#define RCCHECK(fn)     { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = (fn); (void)rc; }

namespace {

// =============================================================================
// Estado
// =============================================================================
MavLink* mav = nullptr;

rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rclc_executor_t executor;

rcl_publisher_t pub_pose, pub_twist, pub_batt, pub_gps;
rcl_publisher_t pub_armed, pub_mode, pub_stat, pub_diag;

rcl_subscription_t sub_cmdvel, sub_setpoint, sub_setmode, sub_takeoff;
rcl_service_t      srv_arm;
rcl_timer_t        timer_fast, timer_slow;

geometry_msgs__msg__PoseStamped        msg_pose;
geometry_msgs__msg__TwistStamped       msg_twist;
sensor_msgs__msg__BatteryState         msg_batt;
sensor_msgs__msg__NavSatFix            msg_gps;
std_msgs__msg__Bool                    msg_armed;
std_msgs__msg__String                  msg_mode;
std_msgs__msg__String                  msg_stat;
diagnostic_msgs__msg__DiagnosticStatus msg_diag;

geometry_msgs__msg__TwistStamped in_cmdvel;
geometry_msgs__msg__PoseStamped  in_setpoint;
std_msgs__msg__String            in_setmode;
std_msgs__msg__Float32           in_takeoff;

std_srvs__srv__SetBool_Request  arm_req;
std_srvs__srv__SetBool_Response arm_res;

// --- buffers de string -------------------------------------------------------
// micro-ROS aloca tudo estaticamente: toda string, de entrada ou de saída,
// precisa apontar para memória nossa antes do primeiro uso. String com
// data == NULL chegando no serializador é a forma mais rápida de reiniciar o
// ESP32 sem entender por quê.
char buf_mode[STR_MODE_CAP];
char buf_stat[STR_STATUSTEXT_CAP];
char buf_in_mode[STR_MODE_CAP];
char buf_arm_res[STR_STATUSTEXT_CAP];
char buf_diag_msg[STR_STATUSTEXT_CAP];

// frame_id das mensagens que CHEGAM. Precisa de capacidade de verdade, não de
// um buffer simbólico: o micro-ROS desserializa a string dentro da memória que
// nós demos, e se ela não couber, a mensagem INTEIRA é descartada. Um teleop
// publicando `frame_id: "map"` contra um buffer de 1 byte produz um
// /ozzy/cmd_vel que existe, tem publicador e assinante casados, e não move o
// drone.
char buf_in_frame_cmdvel[STR_MODE_CAP];
char buf_in_frame_setpoint[STR_MODE_CAP];

char frame_map[]       = "map";
char frame_base_link[] = "base_link";
char diag_name[]       = "ozzy_bridge";
char diag_hwid[]       = "esp32";
char empty_str[]       = "";

diagnostic_msgs__msg__KeyValue diag_kv[DIAG_NUM_VALUES];
char diag_keys[DIAG_NUM_VALUES][DIAG_KEY_CAP];
char diag_vals[DIAG_NUM_VALUES][DIAG_VAL_CAP];

// --- último comando vindo do ROS 2, já em NED --------------------------------
struct Command {
    enum Kind { NONE, VELOCITY, POSITION } kind = NONE;
    float a = 0, b = 0, c = 0, d = 0;   // vx,vy,vz,yaw_rate  ou  x,y,z,yaw
    uint32_t stamp_ms = 0;
};
Command cmd;
bool cmd_timed_out = true;   // começa expirado: nada a retransmitir no boot

// --- máquina de conexão ------------------------------------------------------
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;

bool     time_synced       = false;
uint32_t last_sync_try_ms  = 0;
uint32_t last_ping_ms      = 0;
uint32_t last_setpoint_ms  = 0;
uint32_t diag_tick         = 0;

// =============================================================================
// Utilidades
// =============================================================================

void bindString(rosidl_runtime_c__String* s, char* storage, size_t capacity) {
    s->data     = storage;
    s->capacity = capacity;
    s->size     = strlen(storage);
}

void setString(rosidl_runtime_c__String* s, const char* text) {
    if (!s->data || s->capacity == 0) return;
    strncpy(s->data, text, s->capacity - 1);
    s->data[s->capacity - 1] = '\0';
    s->size = strlen(s->data);
}

// Carimbo de tempo. O ESP32 nasce em 1970; sem a sincronização com o agente,
// todo header sairia com um tempo que quebra tf2 e rosbag. Quando a
// sincronização ainda não pegou, publicamos com o relógio de boot e marcamos
// WARN no /ozzy/diagnostics — dado com carimbo ruim e anunciado é melhor que
// silêncio. Veja docs/CONTRATO.md.
void stamp(builtin_interfaces__msg__Time* t) {
    if (time_synced) {
        const int64_t ns = rmw_uros_epoch_nanos();
        t->sec     = (int32_t)(ns / 1000000000LL);
        t->nanosec = (uint32_t)(ns % 1000000000LL);
    } else {
        const uint32_t ms = millis();
        t->sec     = (int32_t)(ms / 1000);
        t->nanosec = (uint32_t)((ms % 1000) * 1000000UL);
    }
}

// =============================================================================
// Timers — publicação
// =============================================================================

void timerFast(rcl_timer_t*, int64_t) {
    if (!mav) return;
    const MavLink::State& s = mav->state();

    // --- /ozzy/pose ---------------------------------------------------------
    const frames::Vec3 p = frames::nedToEnu(s.x, s.y, s.z);
    const frames::Quat q = frames::attitudeFrdToFlu(s.roll, s.pitch, s.yaw);

    stamp(&msg_pose.header.stamp);
    msg_pose.pose.position.x = p.x;
    msg_pose.pose.position.y = p.y;
    msg_pose.pose.position.z = p.z;
    msg_pose.pose.orientation.w = q.w;
    msg_pose.pose.orientation.x = q.x;
    msg_pose.pose.orientation.y = q.y;
    msg_pose.pose.orientation.z = q.z;
    RCSOFTCHECK(rcl_publish(&pub_pose, &msg_pose, NULL));

    // --- /ozzy/twist --------------------------------------------------------
    // linear: velocidade de MUNDO em ENU. angular: taxas de CORPO em FLU.
    // A mistura é deliberada e está documentada em docs/CONTRATO.md — é o que
    // os dados do MAVLink permitem, e é o que o mavros faz no tópico
    // equivalente.
    const frames::Vec3 v = frames::nedToEnu(s.vx, s.vy, s.vz);
    const frames::Vec3 w = frames::bodyRatesFrdToFlu(s.rollspeed, s.pitchspeed, s.yawspeed);

    stamp(&msg_twist.header.stamp);
    msg_twist.twist.linear.x  = v.x;
    msg_twist.twist.linear.y  = v.y;
    msg_twist.twist.linear.z  = v.z;
    msg_twist.twist.angular.x = w.x;
    msg_twist.twist.angular.y = w.y;
    msg_twist.twist.angular.z = w.z;
    RCSOFTCHECK(rcl_publish(&pub_twist, &msg_twist, NULL));
}

void fillDiagnostics(const MavLink::State& s) {
    const uint32_t now = millis();
    const uint32_t hb_age = (s.hb_ms == 0) ? UINT32_MAX : now - s.hb_ms;
    const int32_t  rssi   = WiFi.RSSI();
    const uint32_t heap   = ESP.getFreeHeap();

    snprintf(diag_vals[0], DIAG_VAL_CAP, "%.1f", mav->rxHz());
    snprintf(diag_vals[1], DIAG_VAL_CAP, "%lu", (unsigned long)s.parse_errors);
    snprintf(diag_vals[2], DIAG_VAL_CAP, "%ld",
             (hb_age == UINT32_MAX) ? -1L : (long)hb_age);
    snprintf(diag_vals[3], DIAG_VAL_CAP, "%ld",
             (cmd.kind == Command::NONE) ? -1L : (long)(now - cmd.stamp_ms));
    snprintf(diag_vals[4], DIAG_VAL_CAP, "%ld", (long)rssi);
    snprintf(diag_vals[5], DIAG_VAL_CAP, "%lu", (unsigned long)heap);
    for (size_t i = 0; i < DIAG_NUM_VALUES; ++i) {
        diag_kv[i].value.size = strlen(diag_vals[i]);
    }

    // A Pixhawk muda é ERROR: sem ela, /ozzy/pose está publicando o último
    // valor conhecido, e nada no tópico denuncia isso.
    if (hb_age > MAV_SILENCE_MS) {
        msg_diag.level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
        setString(&msg_diag.message, "Pixhawk muda");
    } else if (!time_synced) {
        msg_diag.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
        setString(&msg_diag.message, "relogio nao sincronizado");
    } else if (rssi < WARN_RSSI_DBM) {
        msg_diag.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
        setString(&msg_diag.message, "wifi fraco");
    } else if (heap < WARN_FREE_HEAP) {
        msg_diag.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
        setString(&msg_diag.message, "heap baixo");
    } else {
        msg_diag.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
        setString(&msg_diag.message, "ok");
    }
}

void timerSlow(rcl_timer_t*, int64_t) {
    if (!mav) return;
    const MavLink::State& s = mav->state();

    // --- /ozzy/armed --------------------------------------------------------
    msg_armed.data = s.armed;
    RCSOFTCHECK(rcl_publish(&pub_armed, &msg_armed, NULL));

    // --- /ozzy/mode ---------------------------------------------------------
    char scratch[STR_MODE_CAP];
    setString(&msg_mode.data, MavLink::modeName(s.custom_mode, scratch, sizeof(scratch)));
    RCSOFTCHECK(rcl_publish(&pub_mode, &msg_mode, NULL));

    // --- /ozzy/battery ------------------------------------------------------
    stamp(&msg_batt.header.stamp);
    msg_batt.voltage = s.voltage;
    // Convenção do sensor_msgs/BatteryState: corrente NEGATIVA ao descarregar.
    // O MAVLink manda o consumo como positivo. Trocar o sinal aqui é o que
    // impede um gráfico de bateria com a corrente ao contrário.
    msg_batt.current    = (s.current < 0) ? NAN : -s.current;
    msg_batt.percentage = (s.battery_pct < 0) ? NAN : s.battery_pct / 100.0f;
    msg_batt.present    = (s.sys_ms != 0);
    RCSOFTCHECK(rcl_publish(&pub_batt, &msg_batt, NULL));

    // --- /ozzy/gps ----------------------------------------------------------
    stamp(&msg_gps.header.stamp);
    msg_gps.latitude  = s.lat;
    msg_gps.longitude = s.lon;
    msg_gps.altitude  = s.alt;
    msg_gps.status.status = (s.fix_type >= 2)
        ? sensor_msgs__msg__NavSatStatus__STATUS_FIX
        : sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
    if (s.eph > 0) {
        const double var = (double)s.eph * (double)s.eph;
        msg_gps.position_covariance[0] = var;
        msg_gps.position_covariance[4] = var;
        msg_gps.position_covariance[8] = var * 4.0;   // vertical é sempre pior
        msg_gps.position_covariance_type =
            sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
        msg_gps.position_covariance_type =
            sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
    }
    RCSOFTCHECK(rcl_publish(&pub_gps, &msg_gps, NULL));

    // --- /ozzy/diagnostics, a cada duas voltas (1 Hz) -----------------------
    if (++diag_tick % 2 == 0) {
        fillDiagnostics(s);
        RCSOFTCHECK(rcl_publish(&pub_diag, &msg_diag, NULL));
    }
}

// =============================================================================
// Callbacks de assinatura
// =============================================================================

void onCmdVel(const void* msgin) {
    const auto* m = (const geometry_msgs__msg__TwistStamped*)msgin;

    const frames::Vec3 v = frames::enuToNed((float)m->twist.linear.x,
                                            (float)m->twist.linear.y,
                                            (float)m->twist.linear.z);
    cmd.kind     = Command::VELOCITY;
    cmd.a = v.x;  cmd.b = v.y;  cmd.c = v.z;
    cmd.d = frames::yawRateEnuToNed((float)m->twist.angular.z);
    cmd.stamp_ms = millis();
    cmd_timed_out = false;

    mav->sendVelocityNed(cmd.a, cmd.b, cmd.c, cmd.d);
    last_setpoint_ms = cmd.stamp_ms;
}

void onSetpoint(const void* msgin) {
    const auto* m = (const geometry_msgs__msg__PoseStamped*)msgin;

    const frames::Vec3 p = frames::enuToNed((float)m->pose.position.x,
                                            (float)m->pose.position.y,
                                            (float)m->pose.position.z);
    const frames::Quat q = { (float)m->pose.orientation.w, (float)m->pose.orientation.x,
                             (float)m->pose.orientation.y, (float)m->pose.orientation.z };

    cmd.kind     = Command::POSITION;
    cmd.a = p.x;  cmd.b = p.y;  cmd.c = p.z;
    cmd.d = frames::yawEnuToNed(frames::yawFromQuat(q));
    cmd.stamp_ms = millis();
    cmd_timed_out = false;

    mav->sendPositionNed(cmd.a, cmd.b, cmd.c, cmd.d);
    last_setpoint_ms = cmd.stamp_ms;
}

void onSetMode(const void* msgin) {
    const auto* m = (const std_msgs__msg__String*)msgin;
    if (!m->data.data || m->data.size == 0) return;

    uint32_t num = 0;
    if (MavLink::modeNumber(m->data.data, &num)) {
        mav->sendSetMode(num);
    } else {
        // Nome errado não pode falhar em silêncio: quem pediu está esperando
        // /ozzy/mode mudar, e ficaria esperando para sempre.
        char warn[STR_STATUSTEXT_CAP];
        snprintf(warn, sizeof(warn), "modo desconhecido: %s", m->data.data);
        ros_link::publishStatusText(warn);
    }
}

void onTakeoff(const void* msgin) {
    const auto* m = (const std_msgs__msg__Float32*)msgin;
    mav->sendTakeoff(m->data);
}

void onArm(const void* req, void* res) {
    const auto* rq = (const std_srvs__srv__SetBool_Request*)req;
    auto*       rs = (std_srvs__srv__SetBool_Response*)res;

    mav->sendArm(rq->data);

    // `success` significa "comando enviado à Pixhawk", não "o veículo armou".
    // Quem responde a segunda pergunta é /ozzy/armed, e o chamador TEM que
    // esperar por ele. Devolver `true` como se fosse confirmação seria mentir
    // numa interface onde a mentira custa uma hélice.
    rs->success = true;
    setString(&rs->message, rq->data ? "arm enviado; confirme em /ozzy/armed"
                                     : "disarm enviado; confirme em /ozzy/armed");
}

// =============================================================================
// Ciclo de vida das entidades
// =============================================================================

void initMessages() {
    // Headers
    bindString(&msg_pose.header.frame_id,  frame_map,       sizeof(frame_map));
    bindString(&msg_twist.header.frame_id, frame_map,       sizeof(frame_map));
    bindString(&msg_batt.header.frame_id,  frame_base_link, sizeof(frame_base_link));
    bindString(&msg_gps.header.frame_id,   frame_base_link, sizeof(frame_base_link));

    // Strings de saída
    buf_mode[0] = '\0';  bindString(&msg_mode.data, buf_mode, STR_MODE_CAP);
    buf_stat[0] = '\0';  bindString(&msg_stat.data, buf_stat, STR_STATUSTEXT_CAP);

    // Strings de ENTRADA. Sem isto o micro-ROS recebe uma String sem lugar
    // onde escrever.
    buf_in_mode[0] = '\0';
    bindString(&in_setmode.data, buf_in_mode, STR_MODE_CAP);
    buf_in_frame_cmdvel[0] = '\0';
    buf_in_frame_setpoint[0] = '\0';
    bindString(&in_cmdvel.header.frame_id,   buf_in_frame_cmdvel,   STR_MODE_CAP);
    bindString(&in_setpoint.header.frame_id, buf_in_frame_setpoint, STR_MODE_CAP);

    // Resposta do serviço
    buf_arm_res[0] = '\0';
    bindString(&arm_res.message, buf_arm_res, STR_STATUSTEXT_CAP);

    // BatteryState tem duas strings que não usamos — e que não podem ficar em
    // NULL na hora de serializar.
    bindString(&msg_batt.location,      empty_str, sizeof(empty_str));
    bindString(&msg_batt.serial_number, empty_str, sizeof(empty_str));
    msg_batt.power_supply_status     = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    msg_batt.power_supply_health     = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
    msg_batt.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
    msg_batt.charge = NAN; msg_batt.capacity = NAN; msg_batt.design_capacity = NAN;
    msg_batt.temperature = NAN;

    msg_gps.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

    // DiagnosticStatus: a sequência de KeyValue é estática, com as chaves
    // gravadas uma vez só. Só os valores mudam a cada publicação.
    static const char* kKeys[DIAG_NUM_VALUES] = {
        "mav_rx_hz", "mav_parse_err", "mav_last_hb_ms",
        "ros_cmd_age_ms", "wifi_rssi", "free_heap",
    };
    for (size_t i = 0; i < DIAG_NUM_VALUES; ++i) {
        strncpy(diag_keys[i], kKeys[i], DIAG_KEY_CAP - 1);
        diag_keys[i][DIAG_KEY_CAP - 1] = '\0';
        diag_vals[i][0] = '\0';
        bindString(&diag_kv[i].key,   diag_keys[i], DIAG_KEY_CAP);
        bindString(&diag_kv[i].value, diag_vals[i], DIAG_VAL_CAP);
    }
    msg_diag.values.data     = diag_kv;
    msg_diag.values.size     = DIAG_NUM_VALUES;
    msg_diag.values.capacity = DIAG_NUM_VALUES;
    bindString(&msg_diag.name,        diag_name, sizeof(diag_name));
    bindString(&msg_diag.hardware_id, diag_hwid, sizeof(diag_hwid));
    buf_diag_msg[0] = '\0';
    bindString(&msg_diag.message, buf_diag_msg, sizeof(buf_diag_msg));
}

bool createEntities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "ozzy_bridge", "", &support));

    // QoS: telemetria de alta taxa vai BEST_EFFORT. Num link de rádio, QoS
    // confiável transforma um pacote perdido em retransmissão, e a
    // retransmissão atrasa o dado SEGUINTE — que já é mais novo e mais útil.
    // Comandos e eventos vão RELIABLE: perder um "arm" não é aceitável.
    RCCHECK(rclc_publisher_init_best_effort(&pub_pose, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped), "/ozzy/pose"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_twist, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "/ozzy/twist"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_batt, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "/ozzy/battery"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_gps, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/ozzy/gps"));

    RCCHECK(rclc_publisher_init_default(&pub_armed, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/ozzy/armed"));
    RCCHECK(rclc_publisher_init_default(&pub_mode, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/ozzy/mode"));
    RCCHECK(rclc_publisher_init_default(&pub_stat, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/ozzy/statustext"));
    RCCHECK(rclc_publisher_init_default(&pub_diag, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus), "/ozzy/diagnostics"));

    RCCHECK(rclc_subscription_init_best_effort(&sub_cmdvel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "/ozzy/cmd_vel"));
    RCCHECK(rclc_subscription_init_best_effort(&sub_setpoint, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped), "/ozzy/setpoint"));
    RCCHECK(rclc_subscription_init_default(&sub_setmode, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/ozzy/set_mode"));
    RCCHECK(rclc_subscription_init_default(&sub_takeoff, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/ozzy/takeoff"));

    RCCHECK(rclc_service_init_default(&srv_arm, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "/ozzy/arm"));

    RCCHECK(rclc_timer_init_default(&timer_fast, &support, RCL_MS_TO_NS(PUB_FAST_MS), timerFast));
    RCCHECK(rclc_timer_init_default(&timer_slow, &support, RCL_MS_TO_NS(PUB_SLOW_MS), timerSlow));

    // 7 handles: 4 assinaturas + 2 timers + 1 serviço. Este número tem que
    // bater com o que é adicionado abaixo — um handle a mais e o
    // rclc_executor_add_* devolve erro no boot.
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmdvel, &in_cmdvel,
                                           onCmdVel, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_setpoint, &in_setpoint,
                                           onSetpoint, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_setmode, &in_setmode,
                                           onSetMode, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_takeoff, &in_takeoff,
                                           onTakeoff, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_fast));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_slow));
    RCCHECK(rclc_executor_add_service(&executor, &srv_arm, &arm_req, &arm_res, onArm));

    return true;
}

void destroyEntities() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&pub_pose,  &node);
    rcl_publisher_fini(&pub_twist, &node);
    rcl_publisher_fini(&pub_batt,  &node);
    rcl_publisher_fini(&pub_gps,   &node);
    rcl_publisher_fini(&pub_armed, &node);
    rcl_publisher_fini(&pub_mode,  &node);
    rcl_publisher_fini(&pub_stat,  &node);
    rcl_publisher_fini(&pub_diag,  &node);

    rcl_subscription_fini(&sub_cmdvel,   &node);
    rcl_subscription_fini(&sub_setpoint, &node);
    rcl_subscription_fini(&sub_setmode,  &node);
    rcl_subscription_fini(&sub_takeoff,  &node);

    rcl_service_fini(&srv_arm, &node);
    rcl_timer_fini(&timer_fast);
    rcl_timer_fini(&timer_slow);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    time_synced = false;
}

// =============================================================================
// Retransmissão de setpoint e failsafe de nível 1
// =============================================================================
//
// O GUIDED do ArduPilot mantém a última velocidade comandada até receber
// outra. Duas consequências, e as duas estão aqui:
//
//   1. É preciso REPETIR o setpoint, senão o ArduPilot pode considerar o
//      comando velho e recusar continuar.
//   2. É preciso PARAR quando o ROS 2 emudece — senão um cabo de rede que cai
//      vira um drone que atravessa a arena na última velocidade recebida.
void streamSetpoint() {
    const uint32_t now = millis();
    if (now - last_setpoint_ms < SETPOINT_TX_MS) return;
    last_setpoint_ms = now;

    if (cmd.kind == Command::NONE) return;

    if (now - cmd.stamp_ms > ROS_CMD_TIMEOUT_MS) {
        if (!cmd_timed_out) {
            // Uma vez só, e depois silêncio: manda parar e devolve o veículo
            // ao que o GUIDED faz sem comando (ficar onde está).
            mav->sendVelocityNed(0, 0, 0, 0);
            cmd_timed_out = true;
            cmd.kind = Command::NONE;
        }
        return;
    }

    if (cmd.kind == Command::VELOCITY) mav->sendVelocityNed(cmd.a, cmd.b, cmd.c, cmd.d);
    else                               mav->sendPositionNed(cmd.a, cmd.b, cmd.c, cmd.d);
}

}  // namespace

// =============================================================================
// Interface pública
// =============================================================================

namespace ros_link {

void begin(MavLink* m) {
    mav = m;

    IPAddress agent_ip;
    agent_ip.fromString(OZZY_AGENT_IP);
    set_microros_wifi_transports((char*)OZZY_WIFI_SSID, (char*)OZZY_WIFI_PASS,
                                 agent_ip, OZZY_AGENT_PORT);
    initMessages();
}

bool connected() { return agent_state == AGENT_CONNECTED; }

void publishStatusText(const char* text) {
    if (agent_state != AGENT_CONNECTED) return;
    setString(&msg_stat.data, text);
    RCSOFTCHECK(rcl_publish(&pub_stat, &msg_stat, NULL));
}

void spin() {
    const uint32_t now = millis();

    switch (agent_state) {

    case WAITING_AGENT:
        // Ping barato e raro: procurar o agente a cada loop entope o WiFi e
        // não acelera nada.
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) agent_state = AGENT_AVAILABLE;
        break;

    case AGENT_AVAILABLE:
        agent_state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
        if (agent_state == WAITING_AGENT) destroyEntities();
        break;

    case AGENT_CONNECTED:
        // Pingar a cada loop custaria mais que toda a telemetria junta. Uma
        // vez por segundo detecta a queda dentro do FS_GCS_TIMEOUT do
        // ArduPilot, que é o prazo que realmente importa.
        if (now - last_ping_ms > 1000) {
            last_ping_ms = now;
            if (rmw_uros_ping_agent(200, 3) != RMW_RET_OK) {
                agent_state = AGENT_DISCONNECTED;
                break;
            }
        }
        if (!time_synced && now - last_sync_try_ms > TIME_SYNC_RETRY_MS) {
            last_sync_try_ms = now;
            if (rmw_uros_sync_session(1000) == RMW_RET_OK) time_synced = true;
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        break;

    case AGENT_DISCONNECTED:
        destroyEntities();
        agent_state = WAITING_AGENT;
        break;
    }

    // Fora do switch de propósito: o failsafe tem que rodar mesmo — e
    // sobretudo — quando o agente caiu.
    streamSetpoint();
}

}  // namespace ros_link
