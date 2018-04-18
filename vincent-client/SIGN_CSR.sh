#!/bin/bash

openssl x509 -req -in vincent.csr -CA signing.pem -CAkey signing.key -CAcreateserial -out vincent.crt -days 500 -sha256
