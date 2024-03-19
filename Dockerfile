FROM ros:rolling-perception-jammy

MAINTAINER Bey Hao Yun <beyhy94@gmail.com>

# Add user
RUN adduser --quiet --disabled-password user

# Install GUI Display utilties
RUN apt-get update && \
    apt-get install -y xauth xxd x11-xserver-utils && \
    rm -rf /var/lib/apt/lists/*

USER user

WORKDIR /home/user
