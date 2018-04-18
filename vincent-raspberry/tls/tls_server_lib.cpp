#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "tls_server_lib.h"

// Initialize OpenSSL by loading all error strings and algorithms.
void init_openssl()
{
	SSL_library_init();
	SSL_load_error_strings();
	OpenSSL_add_all_algorithms();
}

// Clean up OpenSSL
void cleanup_openssl()
{
	EVP_cleanup();
}

// Create an OpenSSL Context

SSL_CTX *create_context()
{
	// Create an SSL context
	const SSL_METHOD *method;
	SSL_CTX *ctx;

	// To maintain compatibility we will continue to
	// use SSLv23, but will restrict the use
	// of SSLv2 and SSLv3 which are known to have
	// security flaws.
	method = SSLv23_server_method();

	ctx = SSL_CTX_new(method);

	if(!ctx)
	{
		perror("Unable to create SSL context: ");
		ERR_print_errors_fp(stderr);
		exit(-1);
	}

	// Here is where we restrict the use of SSLv2 and SSLv3
	SSL_CTX_set_options(ctx, SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3);

	return ctx;
}

// Configure the OpenSSL Context.
// ctx - Context createde in create_context
// cert_name - Filename of your certificate (.crt) file
// pkey_name - Filename of your private key (.key) file
void configure_context(SSL_CTX *ctx, const char *cert_name, const char *pkey_name)
{
	// Use elliptic curve diffie-helman to exchange keys
	SSL_CTX_set_ecdh_auto(ctx, 1);

	// Load our server certificate
	if(SSL_CTX_use_certificate_file(ctx, cert_name, SSL_FILETYPE_PEM) <= 0)
	{
		ERR_print_errors_fp(stderr);
		exit(-1);
	}

	// Load our server private key
	if(SSL_CTX_use_PrivateKey_file(ctx, pkey_name, SSL_FILETYPE_PEM) <= 0)
	{
		ERR_print_errors_fp(stderr);
		exit(-1);
	}
}

// Creates a new SSL session, attaches the client file
// descriptor to it, initiates the connection and
// returns the SSL session.
// ctx = SSL context
// fd = File descriptor for client connection.
//

SSL *connectSSL(SSL_CTX *ctx, int fd)
{
	SSL *ssl = SSL_new(ctx);
	SSL_set_fd(ssl, fd);
	if(SSL_accept(ssl) <= 0)
	{
		ERR_print_errors_fp(stderr);
		SSL_free(ssl);
		return NULL;
	}

	return ssl;
}
