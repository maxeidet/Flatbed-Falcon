#!/usr/bin/env bash
set -e

# NO_VNC=1: skip the virtual display + VNC layer (start-stellar.sh --native-gui draws on
# the computer's own display instead) and go straight to the normal entrypoint.
if [ -n "${NO_VNC}" ]; then
  exec /usr/local/bin/ros2-entrypoint.sh "$@"
fi

# Starta virtuell display (1920x1080 matchar ett vanligt browserfönster bättre än 1280x720,
# så noVNC behöver skala mindre för att fylla ytan)
Xvfb "${DISPLAY}" -screen 0 1920x1080x24 &
sleep 1

# Starta fönsterhanterare mot den virtuella displayen
fluxbox &

# Dela displayen över VNC (native klient, port 5900). macOS Skärmdelning hanterar
# "None"-autentisering opålitligt (hänger på "Ansluter..."), så vi sätter ett lösenord.
# No hardcoded password: the repo is shared and Stellar machines are multi-user, so a
# fixed password would let other people take over the session. start-dev.sh passes a
# random one in; for a manual `docker run` we generate one and print it here.
if [ -z "${VNC_PASSWORD}" ]; then
  VNC_PASSWORD="$(LC_ALL=C tr -dc 'A-Za-z0-9' </dev/urandom | head -c 8)"
  echo "VNC password: ${VNC_PASSWORD}"
fi
# -passwdfile instead of -passwd so the password is not visible in `ps`.
(umask 077; printf '%s\n' "${VNC_PASSWORD}" > /tmp/vncpass)
x11vnc -display "${DISPLAY}" -forever -shared -passwdfile /tmp/vncpass -rfbport "${VNC_PORT}" -bg -o /var/log/x11vnc.log

# Dela samma VNC-server via webbläsaren (noVNC, port 6080)
websockify --web=/usr/share/novnc "${NOVNC_PORT}" "localhost:${VNC_PORT}" &

# Kör vidare in i basimagens vanliga entrypoint (samma som utan VNC-lagret),
# t.ex. MicroXRCEAgent eller px4-gazebo.
exec /usr/local/bin/ros2-entrypoint.sh "$@"
