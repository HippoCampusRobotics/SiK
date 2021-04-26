FROM ubuntu:18.04

RUN apt-get update

RUN apt-get install -y sdcc build-essential python && rm -rf /var/lib/apt/lists/*

WORKDIR "/src"

ENTRYPOINT [ "/usr/bin/make", "install" ]
