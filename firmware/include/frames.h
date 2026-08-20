// =============================================================================
// frames.h — a fronteira NED/FRD <-> ENU/FLU. O único arquivo que a conhece.
// =============================================================================
//
// O contrato do workspace diz que nada acima do `drone_lib` toca NED/FRD. Aqui
// o `ozzy_bridge` faz o papel do `drone_lib`: o MAVLink é NED/FRD dos dois
// lados do fio, e tudo que sai em tópico ROS 2 já saiu convertido.
//
// Misturar frames é o bug mais caro de software de drone, e o mais difícil de
// enxergar: o veículo voa, só que para o lado errado. A defesa é ter UM lugar
// com a conversão, com os nomes dizendo de onde para onde, e nenhuma
// multiplicação de matriz escrita à mão em outro arquivo.
//
//   NED  = north-east-down   (mundo, MAVLink)
//   FRD  = front-right-down  (corpo, MAVLink)
//   ENU  = east-north-up     (mundo, ROS 2)
//   FLU  = front-left-up     (corpo, ROS 2)
// =============================================================================
#pragma once

#include <math.h>

namespace frames {

struct Vec3 { float x, y, z; };
struct Quat { float w, x, y, z; };

// --- Mundo: NED <-> ENU ------------------------------------------------------
//
// Vale para posição E para velocidade linear: as duas são vetores no mesmo
// frame de mundo, e a troca é a mesma.
//
//   east = north_ned? NÃO. east = o segundo eixo do NED.
//   ENU.x (east)  = NED.y
//   ENU.y (north) = NED.x
//   ENU.z (up)    = -NED.z
//
// A transformação é sua própria inversa, mas as duas funções existem com nomes
// diferentes de propósito: no ponto de uso, `enuToNed` num comando e `nedToEnu`
// numa telemetria dizem o sentido, e o leitor não precisa lembrar da involução.
inline Vec3 nedToEnu(float n, float e, float d) { return Vec3{e, n, -d}; }
inline Vec3 enuToNed(float x, float y, float z) { return Vec3{y, x, -z}; }

// --- Corpo: taxas angulares FRD -> FLU ---------------------------------------
//
// Rolagem mantém o sinal (o eixo X aponta para frente nos dois); arfagem e
// guinada invertem, porque Y e Z invertem.
inline Vec3 bodyRatesFrdToFlu(float p, float q, float r) { return Vec3{p, -q, -r}; }

// --- Atitude -----------------------------------------------------------------
//
// Roll/pitch/yaw do ATTITUDE do MAVLink estão em NED/FRD. Em ENU/FLU:
//
//   roll  igual
//   pitch invertido
//   yaw   pi/2 - yaw     (o zero do NED é o norte; o do ENU é o leste)
//
// A forma longa dessa conversão é q_ENU = q_(NED->ENU) * q * q_(FRD->FLU), com
// duas rotações de 180 graus. As três linhas abaixo são o mesmo resultado, e
// são conferíveis a olho — que é o que importa num arquivo como este.
inline float yawNedToEnu(float yaw_ned) {
    float y = (float)M_PI_2 - yaw_ned;
    while (y >  (float)M_PI) y -= 2.0f * (float)M_PI;
    while (y < -(float)M_PI) y += 2.0f * (float)M_PI;
    return y;
}

inline float yawEnuToNed(float yaw_enu) { return yawNedToEnu(yaw_enu); }  // involução

// Taxa de guinada acompanha o sinal da guinada: yaw_enu = pi/2 - yaw_ned, logo
// a derivada troca de sinal.
inline float yawRateNedToEnu(float r_ned) { return -r_ned; }
inline float yawRateEnuToNed(float r_enu) { return -r_enu; }

// Quaternion (w,x,y,z) a partir de roll/pitch/yaw, convenção ZYX intrínseca —
// a mesma que o ROS 2 usa.
inline Quat quatFromRpy(float roll, float pitch, float yaw) {
    const float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);
    return Quat{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    };
}

// Atalho: atitude do MAVLink (NED/FRD) direto para o quaternion do ROS 2.
inline Quat attitudeFrdToFlu(float roll, float pitch, float yaw) {
    return quatFromRpy(roll, -pitch, yawNedToEnu(yaw));
}

// Guinada de um quaternion ROS 2, para mandar de volta como setpoint.
inline float yawFromQuat(const Quat& q) {
    return atan2f(2.0f * (q.w * q.z + q.x * q.y),
                  1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

}  // namespace frames
