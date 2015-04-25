/**
 * @file GslErrorHandler.h
 * @author Daniel Keyes
 * @brief Overrides default GSL error behavior using exceptions
 */

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_errno.h>

#include <exception>

/**
 * This class is used to override default GSL error behavior. By default, GSL
 * calls abort() when an error occurs (which is bad, since that crashes the
 * program). When this class is instantiated, the GSL error function is replaced
 * by the function handleWithException, which throws an exception rather than
 * aborting the entire program.
 *
 * This behavior is implemented as a class in order to provide automatic
 * cleanup, which occurs when the object is garbage collected, as well as
 * exception safety (because C++ guarantees destructors will still be called).
 */
class GslErrorHandler {
public:
	GslErrorHandler();
	void enable();
	void disable();
	~GslErrorHandler();
private:
	gsl_error_handler_t* oldHandler;
	bool enabled;
};


/**
 * Function handle for the replacement GSL error handler
 */
void handleWithException (const char * reason,
              const char * file,
              int line,
              int gsl_errno);

/**
 * Exception class thrown when GSL encounters an error.
 */
class GslException : public std::exception {
public:
	GslException(const char * reason, const char * file, int line,
			int gsl_errno): reason(reason), file(file), line(line),
			gsl_errno(gsl_errno) {};
//	virtual const char* what() const throw(){
//		return string("GSL exception occurred: \"") + string(reason) + string("\" in file \"") +
//				string(file) + string("\", line ") + line + string(", gsl_errno ") + gsl_errno;
//	}
private:
	const char * reason;
	const char * file;
	int line;
	int gsl_errno;
};
