#ifndef BALLTZE_DYNAMIXEL_SYSTEM__VISIBILITY_CONTROL_H_
#define BALLTZE_DYNAMIXEL_SYSTEM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DYNAMIXEL __attribute__((dllexport))
#define BALLTZE_DYNAMIXEL_SYSTEM_IMPORT __attribute__((dllimport))
#else
#define DYNAMIXEL __declspec(dllexport)
#define BALLTZE_DYNAMIXEL_SYSTEM_IMPORT __declspec(dllimport)
#endif
#ifdef BALLTZE_DYNAMIXEL_SYSTEM_BUILDING_DLL
#define BALLTZE_DYNAMIXEL_SYSTEM_PUBLIC DYNAMIXEL
#else
#define BALLTZE_DYNAMIXEL_SYSTEM_PUBLIC BALLTZE_DYNAMIXEL_SYSTEM_IMPORT
#endif
#define BALLTZE_DYNAMIXEL_SYSTEM_PUBLIC_TYPE BALLTZE_DYNAMIXEL_SYSTEM_PUBLIC
#define BALLTZE_DYNAMIXEL_SYSTEM_LOCAL
#else
#define DYNAMIXEL __attribute__((visibility("default")))
#define BALLTZE_DYNAMIXEL_SYSTEM_IMPORT
#if __GNUC__ >= 4
#define BALLTZE_DYNAMIXEL_SYSTEM_PUBLIC __attribute__((visibility("default")))
#define BALLTZE_DYNAMIXEL_SYSTEM_LOCAL __attribute__((visibility("hidden")))
#else
#define BALLTZE_DYNAMIXEL_SYSTEM_PUBLIC
#define BALLTZE_DYNAMIXEL_SYSTEM_LOCAL
#endif
#define BALLTZE_DYNAMIXEL_SYSTEM_PUBLIC_TYPE
#endif

#endif  // BALLTZE_DYNAMIXEL_SYSTEM__VISIBILITY_CONTROL_H_