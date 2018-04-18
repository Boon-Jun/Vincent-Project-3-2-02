#ifndef __TLS_SERVER_LIB__
#define __TLS_SERVER_LIB__

#include <openssl/ssl.h>
#include <openssl/err.h>

// Initialize OpenSSL by loading all error strings and algorithms.
void init_openssl();

// Clean up OpenSSL
void cleanup_openssl();

// Create an OpenSSL Context
SSL_CTX *create_context();

// Configure the OpenSSL Context.
// ctx - Context createde in create_context
// cert_name - Filename of your certificate (.crt) file
// pkey_name - Filename of your private key (.key) file
void configure_context(SSL_CTX *ctx, const char *cert_name, const char *pkey_name);

// Creates a new SSL session, attaches the client file
// descriptor to it, initiates the connection and
// returns the SSL session.
// ctx = SSL context
// fd = File descriptor for client connection.
//

SSL *connectSSL(SSL_CTX *ctx, int fd);
#endif
