FROM python:3.12-slim-bookworm

ARG DEBIAN_FRONTEND=noninteractive
ARG SDCC_VERSION=4.2.0+dfsg-1
ARG CPPCHECK_VERSION=2.10-2
ARG RUFF_VERSION=0.16.7
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cppcheck=${CPPCHECK_VERSION} \
        git \
        make \
        sdcc=${SDCC_VERSION} \
    && python -m pip install --no-cache-dir ruff==${RUFF_VERSION} \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME}

WORKDIR /workspace
USER ${USERNAME}

CMD ["make", "help"]
