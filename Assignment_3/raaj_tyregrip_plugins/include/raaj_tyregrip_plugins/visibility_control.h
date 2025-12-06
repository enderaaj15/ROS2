#ifndef RAAJ_TYREGRIP_PLUGINS__VISIBILITY_CONTROL_H_
#define RAAJ_TYREGRIP_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RAAJ_TYREGRIP_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define RAAJ_TYREGRIP_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define RAAJ_TYREGRIP_PLUGINS_EXPORT __declspec(dllexport)
    #define RAAJ_TYREGRIP_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef RAAJ_TYREGRIP_PLUGINS_BUILDING_LIBRARY
    #define RAAJ_TYREGRIP_PLUGINS_PUBLIC RAAJ_TYREGRIP_PLUGINS_EXPORT
  #else
    #define RAAJ_TYREGRIP_PLUGINS_PUBLIC RAAJ_TYREGRIP_PLUGINS_IMPORT
  #endif
  #define RAAJ_TYREGRIP_PLUGINS_PUBLIC_TYPE RAAJ_TYREGRIP_PLUGINS_PUBLIC
  #define RAAJ_TYREGRIP_PLUGINS_LOCAL
#else
  #define RAAJ_TYREGRIP_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define RAAJ_TYREGRIP_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define RAAJ_TYREGRIP_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define RAAJ_TYREGRIP_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RAAJ_TYREGRIP_PLUGINS_PUBLIC
    #define RAAJ_TYREGRIP_PLUGINS_LOCAL
  #endif
  #define RAAJ_TYREGRIP_PLUGINS_PUBLIC_TYPE
#endif

#endif  // RAAJ_TYREGRIP_PLUGINS__VISIBILITY_CONTROL_H_
