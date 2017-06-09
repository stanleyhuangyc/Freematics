
This directory is intended for the project specific (private) libraries.
PlatformIO will compile them to static libraries and link to executable file.

The source code of each library should be placed in separate directory, like
"lib/private_lib/[here are source files]".

For example, see how we can reference FreematicsPlus library from the sketch.

|--lib
|  |--FreematicsPlus
|  |  |- FreematicsPlus.cpp
|  |  |- FreematicsPlus.h
|  |  |- ...
|  |- readme.txt --> THIS FILE
|- platformio.ini
|--telelogger
   |- telelogger.ino

PlatformIO will find your libraries automatically, configure preprocessor's
include paths and build them.

More information about PlatformIO Library Dependency Finder
- http://docs.platformio.org/page/librarymanager/ldf.html
