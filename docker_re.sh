#!/bin/bash

docker stop $(docker ps -q --filter ancestor=marcobright2023/mtuav-competition:standalone)

docker run -id -p 8888:8888 -p 50051:50051 -v ./log:/mt-log marcobright2023/mtuav-competition:standalone start
