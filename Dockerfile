FROM ubuntu:22.04 AS builder

LABEL maintainer="Erik Valle <valle.erik23@gmail.com>"
LABEL description="Dockerfile for Elekta test"

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libopencv-dev \
    libpcl-dev \
    libboost-all-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY . .
RUN cmake -S . -B build && cmake --build build -j
ENTRYPOINT ["./build/sphere_clustering"]
