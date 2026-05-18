# URSim e-Series with External Control URCap (optional overlay).
# Based on Multi-Actor-Interface-Library/docker/ursim

ARG VERSION=latest

FROM universalrobots/ursim_e-series:${VERSION}

RUN apt-get update && apt-get install -y --no-install-recommends curl \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ursim/.urcaps \
    && LATEST_RELEASE_URL=$(curl -s https://api.github.com/repos/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/latest \
        | grep "browser_download_url.*\\.jar" | head -1 | awk -F '"' '{print $4}') \
    && echo "Downloading External Control URCap from: $LATEST_RELEASE_URL" \
    && curl -fsSL -o /ursim/.urcaps/external_control.jar "$LATEST_RELEASE_URL" \
    && echo "External Control URCap installed"

EXPOSE 5900 6080 30001 30002 30003 30004 29999 50002
