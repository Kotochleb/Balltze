#ifndef DYNAMIXEL_AX12A_SYSTEM__VISIBILITY_CONTROL_H_
#define DYNAMIXEL_AX12A_SYSTEM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DYNAMIXEL __attribute__((dllexport))
#define DYNAMIXEL_AX12A_SYSTEM_IMPORT __attribute__((dllimport))
#else
#define DYNAMIXEL __declspec(dllexport)
#define DYNAMIXEL_AX12A_SYSTEM_IMPORT __declspec(dllimport)
#endif
#ifdef DYNAMIXEL_AX12A_SYSTEM_BUILDING_DLL
#define DYNAMIXEL_AX12A_SYSTEM_PUBLIC DYNAMIXEL
#else
#define DYNAMIXEL_AX12A_SYSTEM_PUBLIC DYNAMIXEL_AX12A_SYSTEM_IMPORT
#endif
#define DYNAMIXEL_AX12A_SYSTEM_PUBLIC_TYPE DYNAMIXEL_AX12A_SYSTEM_PUBLIC
#define DYNAMIXEL_AX12A_SYSTEM_LOCAL
#else
#define DYNAMIXEL __attribute__((visibility("default")))
#define DYNAMIXEL_AX12A_SYSTEM_IMPORT
#if __GNUC__ >= 4
#define DYNAMIXEL_AX12A_SYSTEM_PUBLIC __attribute__((visibility("default")))
#define DYNAMIXEL_AX12A_SYSTEM_LOCAL __attribute__((visibility("hidden")))
#else
#define DYNAMIXEL_AX12A_SYSTEM_PUBLIC
#define DYNAMIXEL_AX12A_SYSTEM_LOCAL
#endif
#define DYNAMIXEL_AX12A_SYSTEM_PUBLIC_TYPE
#endif

#endif  // DYNAMIXEL_AX12A_SYSTEM__VISIBILITY_CONTROL_H_