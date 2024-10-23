#ifndef TLX493D_LOGGER_H
#define TLX493D_LOGGER_H


// std includes
#include <stdarg.h>


#ifdef __cplusplus

extern "C" {

#endif


typedef struct TLx493D_t  TLx493D_t;

/**
 * @brief The function `printRegisters` prints out all the internal registers of the
 * passed sensor object.
 * 
 * @param[in,out] sensor A pointer to a TLx493D_t structure, which represents the TLx493D sensor.
 */
void printRegisters(TLx493D_t *sensor);

/**
 * @brief The function `printDouble` prints out a value in the double format. 
 * 
 * @param[in] d Value which will be printed to the serial monitor. 
 */
void printDouble(double d);

/**
 * @brief The function `logMessage` is used print a formatted, user defined message
 * to the serial output.
 * 
 * @param[in] prefix User defined prefix for a given string which is printed before the actual
 * message. For example "INFO : ", see also at the `info` function below.
 * @param[in] format Actual string, which should be printed to the serial output - without the prefix.
 * @param[in] vaList Variable arguments list, which is initialized by `va_start`. `va_list` is a special
 * type defined in the `stdarg.h`.
 */
void logMessage(const char *prefix, const char *format, va_list vaList);

/**
 * @brief The function `print` is used to print out a formatted string - without prefix - to the serial output.
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
 */
void print(const char *format, ...);

/**
 * @brief The function `info` is used to print out an info message to the user.
 * It uses the prefix "INFO : " to directly indicate the origin of the message.
 * It also allows to pass a variable number of arguments to the function (...).
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
 */
void info(const char *format, ...);

/**
 * @brief The function `warn` is used to print out a warning message to the user.
 * It uses the prefix "WARNING : " to directly indicate the origin of the message. 
 * It also allows to pass a variable number of arguments to the function (...).
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
 */
void warn(const char *format, ...);

/**
 * @brief The function `error` is used to print out an error message to the user.
 * It uses the prefix "ERROR : " to directly indicate the origin of the message. 
 * It also allows to pass a variable number of arguments to the function (...).
 * 
 * @param[in] format Actual string, which should be printed to the serial output.   
 */
void error(const char *format, ...);

/**
 * @brief The function `flush` is used to send a new line character to ther serial output.
 * 
 */
void flush();


#ifdef __cplusplus

}

#endif


#endif // TLX493D_LOGGER_H
