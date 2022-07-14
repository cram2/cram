(in-package #:png)

(flag "-I/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.7.sdk/usr/X11/include")
(flag "/usr/include")
(include "png.h")

(defwrapper* "get_png_libpng_ver_string" :string () "return PNG_LIBPNG_VER_STRING;")
