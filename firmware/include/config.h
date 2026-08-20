// =============================================================================
// config.h — todos os números que alguém pode querer mudar, num lugar só.
// =============================================================================
//
// Credenciais de WiFi NÃO moram aqui. Elas ficam em `secrets.h`, que é
// ignorado pelo git. Copie `secrets.h.example` e preencha.
// =============================================================================
#pragma once

#include "secrets.h"

// -----------------------------------------------------------------------------
// UART para a Pixhawk
// -----------------------------------------------------------------------------
// UART2 do ESP32, separada da UART0 que é usada pela gravação USB e pelo
// monitor serial. É por isso que dá para depurar com o cabo do MAVLink ligado.
//
// GPIO16/17 são os pinos default da UART2 num ESP32-WROOM. Num WROVER eles são
// da PSRAM e NÃO servem — veja docs/HARDWARE.md. A UART2 passa pela matriz de
// GPIO, então trocar por outro par aqui é suficiente.
#define MAV_UART_NUM   2
#define MAV_PIN_RX     16   // recebe o TX da TELEM1 (pino 2)
#define MAV_PIN_TX     17   // vai para o RX da TELEM1 (pino 3)
#define MAV_BAUD       115200   // casa com SERIAL1_BAUD=115 no ArduPilot

// Identidade deste nó na rede MAVLink. 255 é a convenção de estação de solo, e
// é o default de SYSID_MYGCS no ArduPilot — mudar aqui sem mudar lá faz o
// ArduPilot DESCARTAR nossos comandos em silêncio. Veja docs/ARDUPILOT.md.
#define MAV_MY_SYSID   255
#define MAV_MY_COMPID  190   // MAV_COMP_ID_MISSIONPLANNER

// -----------------------------------------------------------------------------
// Console de depuração (UART0, o cabo USB)
// -----------------------------------------------------------------------------
#define DBG_BAUD       115200

// -----------------------------------------------------------------------------
// Cadências, em milissegundos
// -----------------------------------------------------------------------------
#define PUB_FAST_MS            50   // 20 Hz — /ozzy/pose e /ozzy/twist
#define PUB_SLOW_MS           500   // 2 Hz  — bateria, gps, armed, mode
#define MAV_HEARTBEAT_MS     1000   // 1 Hz  — alimenta o FS_GCS do ArduPilot
#define SETPOINT_TX_MS        100   // 10 Hz — retransmissão do último setpoint

// Silêncio do ROS 2 que faz a ponte soltar o controle. O GUIDED do ArduPilot
// mantém a última velocidade comandada até receber outra: sem este limite, um
// cabo de rede que cai vira um drone que atravessa a arena. Veja a seção de
// failsafe em docs/CONTRATO.md.
#define ROS_CMD_TIMEOUT_MS    500

// Silêncio da Pixhawk que marca ERROR no /ozzy/diagnostics.
#define MAV_SILENCE_MS       3000

// Repetição da sincronização de relógio com o agente, quando ela falha.
#define TIME_SYNC_RETRY_MS   5000

// -----------------------------------------------------------------------------
// Limiares de diagnóstico
// -----------------------------------------------------------------------------
#define WARN_RSSI_DBM         (-80)
#define WARN_FREE_HEAP        20000

// -----------------------------------------------------------------------------
// Buffers de string
// -----------------------------------------------------------------------------
// micro-ROS aloca tudo estaticamente: toda string publicada precisa de um
// buffer de capacidade fixa, dado na inicialização. Estourar a capacidade não
// causa crash, causa truncamento — que é o comportamento que queremos aqui.
#define STR_MODE_CAP           24
#define STR_STATUSTEXT_CAP     64   // STATUSTEXT do MAVLink tem 50 chars + '\0'
#define DIAG_KEY_CAP           20
#define DIAG_VAL_CAP           20
#define DIAG_NUM_VALUES         6
