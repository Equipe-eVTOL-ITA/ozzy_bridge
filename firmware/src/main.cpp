// =============================================================================
// ozzy_bridge — ponte ESP32 entre ROS 2 (micro-ROS) e ArduPilot (MAVLink).
// =============================================================================
//
// O contrato de tópicos está em docs/CONTRATO.md.
// A ligação física está em docs/HARDWARE.md.
// Os parâmetros do ArduPilot estão em docs/ARDUPILOT.md.
//
// Este arquivo é curto de propósito: ele só liga as duas metades e mantém o
// batimento. Toda a lógica de MAVLink está em mav_link.cpp e toda a de ROS 2
// em ros_link.cpp, e nenhuma das duas inclui a outra.
// =============================================================================

#include <Arduino.h>

#include "config.h"
#include "mav_link.h"
#include "ros_link.h"

static MavLink  mav;
static uint32_t last_heartbeat_ms = 0;

void setup() {
    Serial.begin(DBG_BAUD);
    delay(200);
    Serial.println();
    Serial.println("ozzy_bridge — ESP32 <-> Pixhawk 2.4.8 (ArduPilot)");
    Serial.printf("MAVLink: UART%d @ %d, RX=GPIO%d TX=GPIO%d\n",
                  MAV_UART_NUM, MAV_BAUD, MAV_PIN_RX, MAV_PIN_TX);
    Serial.printf("Agente:  %s:%d\n", OZZY_AGENT_IP, OZZY_AGENT_PORT);

    mav.begin();
    ros_link::begin(&mav);
}

void loop() {
    // NENHUM delay() neste loop, e isso não é descuido.
    //
    // O buffer da UART2 tem 256 bytes. A 115200 baud, com os streams pedidos em
    // docs/ARDUPILOT.md, ele enche em ~22 ms. Um `delay(50)` inocente aqui
    // descarta mensagens da Pixhawk, e o sintoma é `mav_parse_err` subindo no
    // /ozzy/diagnostics — um erro de PARSING causado por um atraso de leitura,
    // que é a última coisa em que alguém pensa.
    mav.poll();

    const uint32_t now = millis();

    // Batimento de GCS. É ele que alimenta o FS_GCS_ENABLE do ArduPilot: se
    // este loop travar, o ArduPilot dispara o failsafe dele sozinho. Por isso
    // ele vem antes de qualquer coisa que possa demorar.
    if (now - last_heartbeat_ms >= MAV_HEARTBEAT_MS) {
        last_heartbeat_ms = now;
        mav.sendHeartbeat();
    }

    // STATUSTEXT sai nos dois lugares: no tópico, para a máquina de estados, e
    // no console USB, para quem está na bancada com a placa na mão. É onde o
    // ArduPilot explica por que não armou.
    char text[MavLink::STATUSTEXT_BUF];
    if (mav.takeStatusText(text, sizeof(text))) {
        Serial.printf("[AP] %s\n", text);
        ros_link::publishStatusText(text);
    }

    // Reconexão, executor e retransmissão de setpoint. Roda conectado ou não —
    // o failsafe de nível 1 mora aqui dentro.
    ros_link::spin();
}
