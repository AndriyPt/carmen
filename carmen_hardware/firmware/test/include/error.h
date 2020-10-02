#ifndef ERROR_H_
#define ERROR_H_

#include <stdexcept>

#define SOFTWARE_ERROR(X) (X) ? (throw std::logic_error("Software Assert")) : 0 

#define HARDWARE_ERROR(X) (X) ? (throw std::logic_error("Software Assert")) : 0

#define SOFTWARE_ASSERT(X) (X) ? 0 : (throw std::logic_error("Software Assert"))

#define HARDWARE_ASSERT(X) (X) ? 0 : (throw std::logic_error("Software Assert"))

#define CHECK_ERROR(X, Y) (X) ? (throw std::logic_error("Software Assert")) : 0

#endif /* ERROR_H_ */
