# syntax=docker/dockerfile:1
#
# Toolchain image for dairlib-c3plus-oim (sampling-based C3+ / OIM benchmark).
#
# Carries ONLY the build toolchain and the solver distributions. The repository
# is bind-mounted at run time and Bazel's output tree lives in a named Docker
# volume, so nothing a build produces is written to the host.
#
# Use docker/shell.sh -- it builds this image on first use and drops you in.
#
FROM ubuntu:noble

# Keep in sync with MODULE.bazel (DRAKE_VERSION) and .bazeliskrc.
ARG DRAKE_VERSION=1.51.1
ARG DEBIAN_FRONTEND=noninteractive

# Match the host user so files written into the bind-mounted repository keep
# host ownership instead of turning up root-owned.
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# --- base tooling -----------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl git gnupg lsb-release sudo unzip wget zip \
        g++ pkg-config zlib1g-dev libopenblas-dev openjdk-17-jdk \
        iproute2 iputils-ping less nano procps python3-venv \
        ffmpeg graphviz libegl1 libgl1 libgl1-mesa-dri libosmesa6 fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

# --- Drake build prerequisites (also installs bazelisk and clang) ------------
# install_prereqs is the authoritative dependency list for the pinned Drake.
RUN wget -q "https://github.com/RobotLocomotion/drake/archive/v${DRAKE_VERSION}.tar.gz" \
    && tar -xzf "v${DRAKE_VERSION}.tar.gz" "drake-${DRAKE_VERSION}/setup/" \
    && "./drake-${DRAKE_VERSION}/setup/install_prereqs" \
         --developer -y --with-bazel --with-clang \
    && rm -rf "v${DRAKE_VERSION}.tar.gz" "drake-${DRAKE_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# --- procman (for the per-scenario .pmd process manifests) -------------------
# libbot2 is not packaged for noble; build only the bot2-procman target.
RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake build-essential liblcm-dev python3-lcm python3-pip \
        python3-setuptools libglu1-mesa-dev libgtk-3-dev \
    && git clone --depth 1 --branch drake \
         https://github.com/RobotLocomotion/libbot2.git /tmp/libbot2 \
    && cmake -S /tmp/libbot2 -B /tmp/libbot2-build \
         -DCMAKE_INSTALL_PREFIX=/opt/libbot2 \
    && cmake --build /tmp/libbot2-build -j"$(nproc)" --target bot2-procman \
    && cmake --install /tmp/libbot2-build \
    && rm -rf /tmp/libbot2 /tmp/libbot2-build /var/lib/apt/lists/*
ENV PATH="/opt/libbot2/bin:${PATH}"
RUN dot -c && echo 'digraph { check -> svg }' | dot -Tsvg >/dev/null

# --- SNOPT -------------------------------------------------------------------
# Consumed as a tarball path by Drake's snopt repository rule.
COPY snopt7.6.tar.gz /opt/snopt/snopt7.6.tar.gz
ENV SNOPT_PATH=/opt/snopt/snopt7.6.tar.gz

# --- Gurobi ------------------------------------------------------------------
# .bazelrc sets --define=WITH_GUROBI=ON, so the headers/libs must be present to
# LINK. A licence is only consulted at run time, and only if a configuration
# selects the MIQP projection; every scenario in this repository uses
# projection_type: 'C3+' (OSQP), so no licence is required for them.
COPY gurobi10.0.3_linux64.tar.gz /tmp/gurobi.tar.gz
RUN mkdir -p /opt/gurobi \
    && tar xzf /tmp/gurobi.tar.gz -C /opt/gurobi/ \
    && rm /tmp/gurobi.tar.gz
ENV GUROBI_HOME=/opt/gurobi/gurobi1003/linux64
ENV JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64

# --- Python tooling ----------------------------------------------------------
# Mesh/sampling generation (examples/sampling_c3/sampling_generation) plus the
# analysis and rendering scripts under scripts/.
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv --system-site-packages /opt/push-anything-venv \
    && /opt/push-anything-venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
    && /opt/push-anything-venv/bin/pip check
ENV PATH="/opt/push-anything-venv/bin:${PATH}"
ENV PYTHON=/opt/push-anything-venv/bin/python3
ENV MPLCONFIGDIR=/tmp/matplotlib

# --- unprivileged user matching the host ------------------------------------
# ubuntu:noble already ships a uid-1000 "ubuntu" account; rename it when the
# requested uid collides, otherwise create a fresh account.
RUN set -e; test "${USER_UID}" -ne 0; test "${USER_GID}" -ne 0; \
    if ! getent group "${USER_GID}" >/dev/null; then groupadd -g "${USER_GID}" dairlib; fi; \
    if getent passwd "${USER_UID}" >/dev/null; then \
        old_user="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
        [ "${old_user}"  != "dairlib" ] && usermod -l dairlib -d /home/dairlib -m "${old_user}"; \
        usermod -g "${USER_GID}" -s /bin/bash dairlib; \
    else \
        useradd -m -u "${USER_UID}" -g "${USER_GID}" -s /bin/bash dairlib; \
    fi; \
    echo "dairlib ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dairlib; \
    chmod 0440 /etc/sudoers.d/dairlib

# --- container-local Bazel settings -----------------------------------------
# Deliberately NOT in the repository's own .bazelrc: these are properties of
# the container, and the project .bazelrc is shared with host builds.
COPY <<'BAZELRC' /home/dairlib/.bazelrc
# Keep the whole output tree (external repos, action cache, compiled objects)
# in the container-local cache volume rather than under the bind-mounted
# workspace or the host's ~/.cache.
startup --output_user_root=/home/dairlib/.cache/bazel

# Cap the Bazel server heap. Without this the JVM sizes itself from the HOST's
# memory, not the cgroup limit, and competes with the compile jobs.
startup --host_jvm_args=-Xmx2g

# Job count is FIXED, not derived from HOST_CPUS. Docker's --cpus is a CFS
# quota, not a cpuset, so nproc/HOST_CPUS inside the container still report
# every host core -- `--jobs=HOST_CPUS*0.5` resolved to 16 here and the
# resulting Drake/VTK compiles were OOM-killed against the memory limit.
# Drake's heaviest translation units peak near 2 GB each, so keep
#   jobs * 2 GB + 2 GB (JVM)  <  the container memory limit
build --jobs=8
build --local_ram_resources=14000

build --verbose_failures
BAZELRC

# --- entrypoint --------------------------------------------------------------
# Makes LCM's multicast transport work inside the container's own network
# namespace. Every process of a run lives in this one container, so routing the
# multicast range over loopback both makes delivery reliable and guarantees no
# LCM traffic reaches the host network or any other container.
COPY <<'ENTRY' /usr/local/bin/entrypoint.sh
#!/usr/bin/env bash
set -euo pipefail

# Needs NET_ADMIN, which shell.sh grants. The capability is scoped to this
# container's network namespace and cannot affect the host.
sudo ip link set lo multicast on 2>/dev/null || true
if ! ip route show | grep -q '^224.0.0.0/4'; then
    sudo ip route add 224.0.0.0/4 dev lo 2>/dev/null || true
fi

exec "$@"
ENTRY

RUN mkdir -p /home/dairlib/.cache/bazel \
    && chmod 0755 /usr/local/bin/entrypoint.sh \
    && chown -R "${USER_UID}:${USER_GID}" /home/dairlib

USER dairlib
WORKDIR /home/dairlib/dairlib

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
