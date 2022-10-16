#ifndef BALLTZE_HARDWARE__VISIBILITY_CONTROL_H_
#define BALLTZE_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DYNAMIXEL __attribute__((dllexport))
#define BALLTZE_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define DYNAMIXEL __declspec(dllexport)
#define BALLTZE_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef BALLTZE_HARDWARE_BUILDING_DLL
#define BALLTZE_HARDWARE_PUBLIC DYNAMIXEL
#else
#define BALLTZE_HARDWARE_PUBLIC BALLTZE_HARDWARE_IMPORT
#endif
#define BALLTZE_HARDWARE_PUBLIC_TYPE BALLTZE_HARDWARE_PUBLIC
#define BALLTZE_HARDWARE_LOCAL
#else
#define DYNAMIXEL __attribute__((visibility("default")))
#define BALLTZE_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define BALLTZE_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define BALLTZE_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define BALLTZE_HARDWARE_PUBLIC
#define BALLTZE_HARDWARE_LOCAL
#endif
#define BALLTZE_HARDWARE_PUBLIC_TYPE
#endif

#endif  // BALLTZE_HARDWARE__VISIBILITY_CONTROL_H_