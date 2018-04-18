#!/bin/bash

gcc tls-vincent-client.cpp tls_client_lib.cpp tls_pthread.cpp -lssl -lcrypto -pthread -fpermissive -o tls-vincent-client