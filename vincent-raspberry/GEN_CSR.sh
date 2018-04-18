#!/bin/bash

openssl genrsa -out vincent.key 2048
openssl req -new -key vincent.key -out vincent.csr < details.txt
