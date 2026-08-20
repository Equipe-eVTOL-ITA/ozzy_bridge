#!/usr/bin/env bash
# =============================================================================
# agent.sh — sobe o Micro XRCE-DDS Agent para o ESP32 do ozzy_bridge.
# =============================================================================
#
#   ./scripts/agent.sh          # UDP na porta 8888
#   ./scripts/agent.sh 6        # com verbosidade 6 (mostra cada mensagem XRCE)
#
# É o mesmo binário e a mesma porta do templates/scripts/agent.sh do workspace.
# A diferença é que aqui o cliente NUNCA é serial: o ESP32 fala UDP pelo WiFi,
# então a variante `serial --dev /dev/ttyTHS1` daquele template não se aplica.
#
# Deixe rodando num terminal e ligue o ESP32 depois. Em verbosidade 6, a
# primeira linha nova ao ligar a placa prova que ela chegou até aqui — o que
# separa "problema de rede" de "problema de ROS 2" em dois segundos.
# =============================================================================
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_root="$(cd "$script_dir/../../.." && pwd)"

source "$ws_root/scripts/ros_env.sh"

export PATH="$PATH:/usr/local/bin"

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
    echo "ERRO: MicroXRCEAgent não encontrado." >&2
    echo "      Veja docs/SETUP.md do evtol-dev, seção 5. O doctor.sh confere isso." >&2
    exit 1
fi

verbose="${1:-4}"

echo "Micro XRCE-DDS Agent — UDP porta 8888, verbosidade $verbose"
echo "Aguardando o ESP32. O IP desta máquina, para o secrets.h:"
hostname -I | tr ' ' '\n' | grep -v '^$' | sed 's/^/    /'
echo

exec MicroXRCEAgent udp4 -p 8888 -v "$verbose"
