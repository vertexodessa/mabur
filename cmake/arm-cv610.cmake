# Cross toolchain for Hi3516CV610: OpenIPC musl ARMv7 soft-float ABI, DYNAMIC.
# The MPP libraries are soft-float musl shared objects and the sensor plugin is
# dlopen'd, so neither a hard-float nor a static binary can load them.
# softfp keeps that ABI and still enables NEON, which gf256 needs: with
# -mfloat-abi=soft __ARM_NEON__ is undefined and the FEC runs on scalar tables.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CV610_TOOLCHAIN_BIN "$ENV{CV610_TOOLCHAIN_BIN}")
set(CMAKE_C_COMPILER   ${CV610_TOOLCHAIN_BIN}/arm-openipc-linux-musleabi-gcc)
set(CMAKE_CXX_COMPILER ${CV610_TOOLCHAIN_BIN}/arm-openipc-linux-musleabi-g++)
set(CMAKE_C_FLAGS_INIT   "-Os -mfpu=neon-vfpv4 -mfloat-abi=softfp")
set(CMAKE_CXX_FLAGS_INIT "-Os -mfpu=neon-vfpv4 -mfloat-abi=softfp")
set(CMAKE_FIND_ROOT_PATH "$ENV{MABUR_CV610_STAGING}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
