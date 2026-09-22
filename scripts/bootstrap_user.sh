#!/usr/bin/env bash
set -eo pipefail
if [[ "$EUID" != 0 ]]; then echo 'Run as root inside WSL.' >&2; exit 1; fi
SIM_USER=${1:-drone}
if [[ ! "$SIM_USER" =~ ^[a-z][a-z0-9_-]{0,30}$ ]]; then echo 'Invalid Linux username.' >&2; exit 1; fi
if ! id "$SIM_USER" >/dev/null 2>&1; then
  useradd --create-home --shell /bin/bash "$SIM_USER"
fi
usermod -aG sudo "$SIM_USER"
SIM_USER="$SIM_USER" python3 - <<'PY'
import configparser, os
from pathlib import Path
p = Path('/etc/wsl.conf')
c = configparser.ConfigParser()
if p.exists():
    c.read(p)
if not c.has_section('user'):
    c.add_section('user')
c.set('user', 'default', os.environ['SIM_USER'])
with p.open('w') as f:
    c.write(f)
PY
echo "Linux user ready: $SIM_USER. No password has been set. Use passwd later if needed."
