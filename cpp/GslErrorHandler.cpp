
/**
 * @file GslErrorHandler.cpp
 * @author Daniel Keyes
 * @brief Overrides default GSL error behavior using exceptions
 */

#include <GslErrorHandler.h>

GslErrorHandler::GslErrorHandler():oldHandler(0), enabled(false){}
void GslErrorHandler::enable(){
	if(!enabled){
		oldHandler = gsl_set_error_handler (&handleWithException);
		enabled=true;
	}
}

void GslErrorHandler::disable(){
	if(enabled){
		gsl_set_error_handler (oldHandler);
		enabled=false;
	}
}
GslErrorHandler::~GslErrorHandler(){
	disable();
}


void handleWithException (const char * reason,
              const char * file,
              int line,
              int gsl_errno){
	throw GslException(reason, file, line, gsl_errno);
}

