#ifndef BALLTZE_SIMULATION__VISIBILITY_CONTROL_H_
#define BALLTZE_SIMULATION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DYNAMIXEL __attribute__((dllexport))
#define BALLTZE_SIMULATION_IMPORT __attribute__((dllimport))
#else
#define DYNAMIXEL __declspec(dllexport)
#define BALLTZE_SIMULATION_IMPORT __declspec(dllimport)
#endif
#ifdef BALLTZE_SIMULATION_BUILDING_DLL
#define BALLTZE_SIMULATION_PUBLIC DYNAMIXEL
#else
#define BALLTZE_SIMULATION_PUBLIC BALLTZE_SIMULATION_IMPORT
#endif
#define BALLTZE_SIMULATION_PUBLIC_TYPE BALLTZE_SIMULATION_PUBLIC
#define BALLTZE_SIMULATION_LOCAL
#else
#define DYNAMIXEL __attribute__((visibility("default")))
#define BALLTZE_SIMULATION_IMPORT
#if __GNUC__ >= 4
#define BALLTZE_SIMULATION_PUBLIC __attribute__((visibility("default")))
#define BALLTZE_SIMULATION_LOCAL __attribute__((visibility("hidden")))
#else
#define BALLTZE_SIMULATION_PUBLIC
#define BALLTZE_SIMULATION_LOCAL
#endif
#define BALLTZE_SIMULATION_PUBLIC_TYPE
#endif

#endif  // BALLTZE_SIMULATION__VISIBILITY_CONTROL_H_