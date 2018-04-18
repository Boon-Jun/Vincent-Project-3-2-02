#include <string.h>
#include "tls_client_lib.h"

// Initialize OpenSSL by loading all error strings and algorithms.
void init_openssl()
{
	SSL_library_init();
	ERR_load_crypto_strings();
	SSL_load_error_strings();
	OpenSSL_add_all_algorithms();
}

// Clean up OpenSSL
void cleanup_openssl()
{
	EVP_cleanup();
}

// Create an OpenSSL Context
// CACertName is the filename of the CA's certificate
SSL_CTX *create_context(const char *CACertName)
{
	// Create an SSL context
	const SSL_METHOD *method;
	SSL_CTX *ctx;

	// To maintain compatibility we will continue to
	// use SSLv23, but will restrict the use
	// of SSLv2 and SSLv3 which are known to have
	// security flaws.
	method = SSLv23_client_method();

	ctx = SSL_CTX_new(method);

	if(!ctx)
	{
		perror("Unable to create SSL context: ");
		ERR_print_errors_fp(stderr);
		exit(-1);
	}

	// Turn on verification of certificate
	SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, verify_callback);

	// Set verification depth
	SSL_CTX_set_verify_depth(ctx, 4);

	// We load in the CA certificate to verify the server's certificate
	SSL_CTX_load_verify_locations(ctx, CACertName, NULL);

	// Here is where we restrict the use of SSLv2 and SSLv3
	SSL_CTX_set_options(ctx, SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3);

	return ctx;
}

// Enables host verification
long setHostVerification(SSL *ssl, const char *hostname)
{
	X509_VERIFY_PARAM *param = SSL_get0_param(ssl);
	X509_VERIFY_PARAM_set_hostflags(param, X509_CHECK_FLAG_NO_PARTIAL_WILDCARDS);

	long res = X509_VERIFY_PARAM_set1_host(param, hostname, strlen(hostname));
	return res;
}

// Creates a new SSL session, attaches the client file
// descriptor to it, initiates the connection and
// returns the SSL session.
// ctx = SSL context
// fd = File descriptor for client connection.
// hostName = Common name for the server. Set to NULL to disable host verification.
//

SSL *connectSSL(SSL_CTX *ctx, int fd, const char *hostName)
{
	SSL *ssl = SSL_new(ctx);

	// Set host verification if hostName is not NULL
	if(hostName != NULL)
		setHostVerification(ssl, hostName);

	SSL_set_fd(ssl, fd);
	if(SSL_connect(ssl) != 1)
	{
		ERR_print_errors_fp(stderr);
		SSL_free(ssl);
		return NULL;
	}

	return ssl;
}

//
// This function prints out the server's certificate details of the SSL session.
// ssl = SSL session

void printCertificate(SSL *ssl)
{
	// NEW: Getting certificates
	X509 *cert=NULL;
	X509_NAME *certname = NULL;

	cert = SSL_get_peer_certificate(ssl);

	if(cert == NULL)
	{
		printf( "Cannot get peer certificate\n");
		return;
	}

	certname = X509_NAME_new();
	certname = X509_get_subject_name(cert);
	X509_NAME_print_ex_fp(stdout, certname, 0, 0);
	printf( "\n\n");
}

// This function verifies the certificate. Returns TRUE if 
// the certificate is valid
int verifyCertificate(SSL *ssl)
{
	long res = SSL_get_verify_result(ssl);
	return (res == X509_V_OK);
}
