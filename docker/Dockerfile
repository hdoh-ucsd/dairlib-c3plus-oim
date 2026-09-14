# syntax=docker/dockerfile:1
#
# Toolchain image for dairlib-c3plus-oim (sampling-based C3+ / OIM benchmark).
#
# Carries ONLY the build toolchain and the solver distributions. The repository
# is bind-mounted at run time and Bazel's output tree lives in a named Docker
# volume. Bazel workspace symlinks and experiment outputs still appear on the
# bind-mounted checkout.
#
# Use docker/shell.sh -- it builds this image on first use and drops you in.
#
FROM ubuntu:noble@sha256:224a1869083a311ef3f13648a154ba79832fbef6364d31493642ca03082da254

# Keep in sync with MODULE.bazel (DRAKE_VERSION) and .bazeliskrc.
ARG DRAKE_VERSION=1.51.1
ARG DRAKE_SHA256=40cb4d0e3d378ba67c21aca8cc1396399774b333b12d0030f5fbe4ddff1dc786
ARG LIBBOT2_COMMIT=f007e6e902f237782b32d739325b2c0290095866
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
    && echo "${DRAKE_SHA256}  v${DRAKE_VERSION}.tar.gz" | sha256sum -c - \
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
    && git init /tmp/libbot2 \
    && git -C /tmp/libbot2 remote add origin https://github.com/RobotLocomotion/libbot2.git \
    && git -C /tmp/libbot2 fetch --depth 1 origin "${LIBBOT2_COMMIT}" \
    && git -C /tmp/libbot2 checkout --detach FETCH_HEAD \
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
# experiment support in c3plus/ and public commands in tools/.
COPY requirements.txt /tmp/requirements.txt
# Keep the runtime separate from Drake's system build/lint dependencies.
# Expose only apt's self-contained LCM binding. Importing all system packages
# mixed Ubuntu's mpl_toolkits namespace with pip's newer Matplotlib and broke
# 3-D plotting; it also made pip check report unrelated system dependencies.
RUN python3 -m venv /opt/push-anything-venv \
    && test -f /usr/lib/python3/dist-packages/lcm/__init__.py \
    && /opt/push-anything-venv/bin/python3 -c \
       "import sysconfig; from pathlib import Path; \
        Path(sysconfig.get_path('purelib'), 'lcm').symlink_to('/usr/lib/python3/dist-packages/lcm', target_is_directory=True)" \
    && /opt/push-anything-venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
    && /opt/push-anything-venv/bin/pip check \
    && /opt/push-anything-venv/bin/python3 -c \
       "import numpy, scipy, matplotlib, PIL, trimesh, yaml, lcm; \
        from pydrake.multibody.plant import MultibodyPlant; \
        from pydrake.geometry import RenderEngineVtkParams; \
        from mpl_toolkits.mplot3d import Axes3D; \
        from pydrake.common import GetDrakePath; \
        print('python deps OK: numpy', numpy.__version__, '| drake', GetDrakePath())"
ENV PATH="/opt/push-anything-venv/bin:${PATH}"
ENV PYTHON=/opt/push-anything-venv/bin/python3
ENV MPLCONFIGDIR=/tmp/matplotlib
ENV HOME=/home/dairlib

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
LABEL org.dairlib.uid="${USER_UID}" org.dairlib.gid="${USER_GID}"

# Runtime settings are container-local; see shell.sh --help for overrides.
COPY entrypoint.sh /usr/local/bin/entrypoint.sh

RUN mkdir -p /home/dairlib/.cache/bazel \
    && chmod 0755 /usr/local/bin/entrypoint.sh \
    && chown -R "${USER_UID}:${USER_GID}" /home/dairlib

USER dairlib
WORKDIR /home/dairlib/dairlib

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
