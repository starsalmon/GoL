# PlatformIO pre-script for env:freenove_fnk0104_ota.
# Reads OTA_PASSWORD from include/secrets.h and passes it to espota.
# Never prints the password.
Import("env")

from pathlib import Path
import re
import sys

secrets = Path(env["PROJECT_DIR"]) / "include" / "secrets.h"
if not secrets.exists():
    sys.stderr.write(
        "include/secrets.h missing; copy include/secrets.example.h and fill it in.\n"
    )
    env.Exit(1)

text = secrets.read_text()
match = re.search(r'#define\s+OTA_PASSWORD\s+"([^"]*)"', text)
if not match:
    sys.stderr.write("OTA_PASSWORD not found in include/secrets.h\n")
    env.Exit(1)

password = match.group(1)
if not password:
    sys.stderr.write("OTA_PASSWORD in include/secrets.h is empty\n")
    env.Exit(1)

env.Append(UPLOADERFLAGS=["--auth=" + password, "--port=3232", "--timeout=30"])
