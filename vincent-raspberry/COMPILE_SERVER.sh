 #!/bin/bash

g++ tls-vincent-server.cpp ./tls/tls_server_lib.cpp ./tls/tls_pthread.cpp ./serial/serial.cpp ./serial/serialize.cpp ./lidar/lidar_custom.cpp ./gnuplot/gnuplot_i.c -lssl -lcrypto -pthread  ./lidar/librplidar_sdk.a  -lpthread -lm -O0 -o tls-vincent-server
