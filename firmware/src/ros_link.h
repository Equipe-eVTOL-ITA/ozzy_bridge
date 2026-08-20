// =============================================================================
// ros_link.h — o lado ROS 2 da ponte. Fala ENU/FLU e SI, e mais nada.
// =============================================================================
//
// Namespace em vez de classe, de propósito: os callbacks do rclc não recebem
// ponteiro de contexto, então o estado teria que virar um ponteiro global para
// a instância de qualquer jeito. Com todo o estado em `static` de arquivo no
// .cpp, ele é privado de verdade — nada fora de ros_link.cpp consegue tocá-lo —
// e o cabeçalho não arrasta os headers do micro-ROS para dentro do main.cpp.
//
// O contrato de tópicos que este arquivo implementa está em docs/CONTRATO.md.
// Os dois têm que mudar juntos.
// =============================================================================
#pragma once

#include "mav_link.h"

namespace ros_link {

// Sobe o transporte WiFi. Não conecta no agente: quem faz isso é spin(), que
// tolera o agente ainda não estar de pé.
void begin(MavLink* mav);

// Máquina de reconexão + executor + retransmissão de setpoint. Chame todo loop.
//
// Sobrevive ao agente cair e voltar: destrói as entidades, volta para
// WAITING_AGENT e reconstrói tudo quando o ping responder. É o comportamento
// que importa em bancada, onde o agente é reiniciado o tempo todo.
void spin();

bool connected();

// Publica em /ozzy/statustext. Sem efeito se o agente não estiver conectado.
void publishStatusText(const char* text);

}  // namespace ros_link
