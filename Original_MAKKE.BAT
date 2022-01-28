@set GCC_EXEC_PREFIX=c:/gcc850/lib/gcc-lib/
@set COMPILER_PATH=c:/gcc850/bin/
@set LIBRARY_PATH=c:/gcc850/lib/
@set GCC_COLLECT=c:/gcc850/lib/
@set C_INCLUDE_PATH=c:/gcc850/include; c:/gcc850/include/c++;c:/gcc850/include/sys;
@set CPLUS_INCLUDE_PATH=c:/gcc850/include; c:/gcc850/include/sys; c:/gcc850/include/c++/4.3.1; c:/gcc850/include/g++/std;c:/gcc850/include/v850-elf
@set TMPDIR=c:/gcc850
@set PATH=c:\gcc850\bin;c:\gcc850\lib;c:\gcc850\source\M3300_CPU;%PATH%
make.exe -s -f SG2.mak
