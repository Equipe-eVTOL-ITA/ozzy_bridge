"""Clona o c_library_v2 num commit fixo e o põe no include path.

Roda automaticamente antes de todo build (`extra_scripts = pre:` no
platformio.ini). Não há passo manual, e não há como compilar contra uma versão
do MAVLink diferente da declarada aqui.

O c_library_v2 é código GERADO: não tem tags, e o master recebe commits quase
todo dia. Pinar um hash é a única forma de o firmware de hoje e o de daqui a
seis meses falarem o mesmo dialeto.
"""

import os
import subprocess
import sys

Import("env")  # noqa: F821  — injetado pelo SCons do PlatformIO

REPO = "https://github.com/mavlink/c_library_v2.git"

# master de 2026-08-20. Para atualizar: troque o hash, compile os dois
# ambientes e confira que `/ozzy/diagnostics` mostra `mav_parse_err: 0` numa
# Pixhawk real antes de fazer o commit.
COMMIT = "ab34796526f9ee149ac0efe5443d462ccefa20e6"

DEST = os.path.join(env.subst("$PROJECT_DIR"), "vendor", "mavlink")  # noqa: F821


def run(args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def current_commit():
    try:
        out = subprocess.run(
            ["git", "-C", DEST, "rev-parse", "HEAD"],
            check=True, capture_output=True, text=True,
        )
        return out.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def ensure():
    if current_commit() == COMMIT:
        return

    if not os.path.isdir(os.path.join(DEST, ".git")):
        os.makedirs(os.path.dirname(DEST), exist_ok=True)
        print("[mavlink] clonando %s" % REPO)
        # Sem --depth: precisamos alcançar um commit específico, e um clone
        # raso do master não o contém se ele não for a ponta.
        run(["git", "clone", REPO, DEST])

    print("[mavlink] checkout %s" % COMMIT[:12])
    try:
        run(["git", "-C", DEST, "checkout", "--quiet", COMMIT])
    except subprocess.CalledProcessError:
        run(["git", "-C", DEST, "fetch", "--quiet", "origin"])
        run(["git", "-C", DEST, "checkout", "--quiet", COMMIT])


try:
    ensure()
except (subprocess.CalledProcessError, OSError) as exc:
    print("[mavlink] ERRO ao obter o c_library_v2: %s" % exc)
    print("[mavlink] confira a rede e se o `git` está instalado.")
    sys.exit(1)

# `#include <common/mavlink.h>` resolve a partir da raiz; os includes internos
# do dialeto são relativos ao próprio arquivo.
env.Append(CPPPATH=[DEST])  # noqa: F821
