#ifndef DENTAQT_CONTROLLERS_VISIBILITY_CONTROL_H
#define DENTAQT_CONTROLLERS_VISIBILITY_CONTROL_H

#if defined _WIN32 || defined __CYGWIN__
#  ifdef __GNUC__
#    define DENTAQT_CONTROLLERS_EXPORT __attribute__((dllexport))
#    define DENTAQT_CONTROLLERS_IMPORT __attribute__((dllimport))
#  else
#    define DENTAQT_CONTROLLERS_EXPORT __declspec(dllexport)
#    define DENTAQT_CONTROLLERS_IMPORT __declspec(dllimport)
#  endif
#  ifdef DENTAQT_CONTROLLERS_BUILDING_DLL
#    define DENTAQT_CONTROLLERS_PUBLIC DENTAQT_CONTROLLERS_EXPORT
#  else
#    define DENTAQT_CONTROLLERS_PUBLIC DENTAQT_CONTROLLERS_IMPORT
#  endif
#  define DENTAQT_CONTROLLERS_PUBLIC_TYPE DENTAQT_CONTROLLERS_PUBLIC
#  define DENTAQT_CONTROLLERS_LOCAL
#else
#  define DENTAQT_CONTROLLERS_EXPORT __attribute__((visibility("default")))
#  define DENTAQT_CONTROLLERS_IMPORT
#  if __GNUC__ >= 4
#    define DENTAQT_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#    define DENTAQT_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#  else
#    define DENTAQT_CONTROLLERS_PUBLIC
#    define DENTAQT_CONTROLLERS_LOCAL
#  endif
#  define DENTAQT_CONTROLLERS_PUBLIC_TYPE
#endif

#endif // DENTAQT_CONTROLLERS_VISIBILITY_CONTROL_H
