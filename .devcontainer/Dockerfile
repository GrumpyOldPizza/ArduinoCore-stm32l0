# See here for image contents: https://github.com/microsoft/vscode-dev-containers/tree/v0.245.0/containers/ubuntu/.devcontainer/base.Dockerfile

# [Choice] Ubuntu version (use ubuntu-22.04 or ubuntu-18.04 on local arm64/Apple Silicon): ubuntu-22.04, ubuntu-20.04, ubuntu-18.04
ARG VARIANT="jammy"
FROM mcr.microsoft.com/vscode/devcontainers/base:0-${VARIANT}

# [Optional] Uncomment this section to install additional OS packages.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
     && apt-get -y install --no-install-recommends build-essential ccache


# Download and install ARM toolchain
ARG TOOLCHAIN="7-2017q4-major"
RUN echo "Installing ARM GCC ${TOOLCHAIN}"
ENV ARM_TOOLCHAIN_VERSION=${TOOLCHAIN}
RUN curl -Lo gcc-arm-none-eabi-${ARM_TOOLCHAIN_VERSION}.tar.bz2 \
	"https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2"
RUN mkdir /opt/gcc-arm-none-eabi-${ARM_TOOLCHAIN_VERSION} && \
	tar xf gcc-arm-none-eabi-${ARM_TOOLCHAIN_VERSION}.tar.bz2 --strip-components=1 -C /opt/gcc-arm-none-eabi-${ARM_TOOLCHAIN_VERSION}
ENV PATH="${PATH}:/opt/gcc-arm-none-eabi-${ARM_TOOLCHAIN_VERSION}/bin"
