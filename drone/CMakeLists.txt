add_library(mabur_drone_core STATIC
  src/config.cpp
  src/debug_http.cpp
  src/frame_source.cpp
  src/frame_pipeline.cpp
  src/radio_tx.cpp
  src/rc_agent.cpp
  src/msp_serial.cpp
  src/telemetry.cpp
  src/vtx_recorder.cpp
  src/cal_apply.cpp
  src/config_apply.cpp
  src/cal_sweep.cpp
  src/cal_link.cpp
  src/live_power_table.cpp
  src/mav_params.cpp
  src/mav_component.cpp
  vendor/venc_frame_ring.c
  # venc_cfg.c is dependency-free C99 (string.h only), so it compiles on
  # host too — config.cpp's parse_venc() needs venc_cfg_defaults() at
  # host-test time, not just on the ARM cross-build. Excluded from venc_core's glob below to avoid compiling it
  # twice into the same link. camera_modes.c likewise: config.cpp checks
  # venc.size against the sensor mode its fps selects.
  venc/venc_cfg.c
  venc/camera_modes.c)
target_include_directories(mabur_drone_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/vendor
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
target_link_libraries(mabur_drone_core PUBLIC mabur_common rt devourer)
# Unconditional (not just ARM cross-builds): a later task's config header
# includes a drone/venc header on host builds too. drone/vendor is already
# on mabur_drone_core above.
target_include_directories(mabur_drone_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/venc)

add_executable(maburd src/main.cpp)

# Same rule as mabur_common (common/CMakeLists.txt): the producer loop lives
# in main.cpp and the pipeline in mabur_drone_core, and both must not fall
# back to Buildroot's -Os. venc_core below is the deliberate exception.
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(mabur_drone_core PRIVATE -O3)
  target_compile_options(maburd PRIVATE -O3)
endif()
target_include_directories(maburd PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(maburd PRIVATE mabur_drone_core mabur_common devourer pthread)

# Build stamp: the boot line names the mabur and devourer commits this binary
# was built from, so a log says which code runs on a drone. Taken at
# configure time; tools/build-arm.sh reconfigures on every run, so its
# binaries always carry the current heads.
# Submodules do not make the stamp dirty: no build reads third_party/devourer
# (DEVOURER_DIR is the sibling checkout, stamped on its own).
function(mabur_git_stamp dir out)
  execute_process(COMMAND git -C ${dir} -c diff.ignoreSubmodules=all describe --always --dirty --abbrev=8
                  OUTPUT_VARIABLE s OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(s STREQUAL "")
    set(s "unknown")
  endif()
  set(${out} ${s} PARENT_SCOPE)
endfunction()
mabur_git_stamp(${CMAKE_SOURCE_DIR} MABUR_GIT_STAMP)
mabur_git_stamp(${DEVOURER_DIR} DEVOURER_GIT_STAMP)
target_compile_definitions(maburd PRIVATE MABUR_GIT_STAMP="${MABUR_GIT_STAMP}"
                                          DEVOURER_GIT_STAMP="${DEVOURER_GIT_STAMP}")

# Venc core: waybeam_venc star6e encoder folded in (spec
# docs/superpowers/specs/2026-08-28-venc-foldin-design.md). ARM-only: it
# drives SigmaStar MI via dlopen and has no host-runnable pipeline.
if(CMAKE_CROSSCOMPILING)
  # CONFIGURE_DEPENDS: without it the glob is evaluated once at configure
  # time, so adding or deleting a drone/venc/*.c between builds silently
  # keeps the stale file list until someone re-runs cmake by hand — on a
  # cross-build that surfaces as a link error (or worse, a quietly missing
  # translation unit) far from its cause.
  file(GLOB VENC_CORE_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/venc/*.c)
  # venc_cfg.c and camera_modes.c are already sources of mabur_drone_core
  # (unconditionally, for host builds) — don't compile them a second time
  # into venc_core.
  list(REMOVE_ITEM VENC_CORE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/venc/venc_cfg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/venc/camera_modes.c)
  # One backend behind the venc_core.h facade, picked at build time.
  # star6e dlopens the SigmaStar MI libraries; cv610 links the HiSilicon MPP
  # libraries, which are not part of this repository (docs/hi3516cv610.md).
  set(MABUR_VENC_BACKEND "star6e" CACHE STRING "Encoder backend: star6e or cv610")
  set_property(CACHE MABUR_VENC_BACKEND PROPERTY STRINGS star6e cv610)
  set(MABUR_CV610_SDK_INC "" CACHE PATH "OpenHisilicon checkout (MPP headers)")
  set(MABUR_CV610_SDK_LIB "" CACHE PATH "Directory holding the MPP .so files")
  if(MABUR_VENC_BACKEND STREQUAL "cv610")
    if(NOT MABUR_CV610_SDK_INC OR NOT MABUR_CV610_SDK_LIB)
      message(FATAL_ERROR "the cv610 backend needs MABUR_CV610_SDK_INC and MABUR_CV610_SDK_LIB")
    endif()
    set(VENC_CORE_SRC
      ${CMAKE_CURRENT_SOURCE_DIR}/venc/venc_core_cv610.c
      ${CMAKE_CURRENT_SOURCE_DIR}/venc/cv610_pipeline.c
      ${CMAKE_CURRENT_SOURCE_DIR}/venc/idr_rate_limit.c
      ${CMAKE_CURRENT_SOURCE_DIR}/venc/timing.c)
  elseif(MABUR_VENC_BACKEND STREQUAL "star6e")
    list(FILTER VENC_CORE_SRC EXCLUDE REGEX "cv610[^/]*\\.c$")
  else()
    message(FATAL_ERROR "unknown MABUR_VENC_BACKEND ${MABUR_VENC_BACKEND}")
  endif()
  # NOTE: waybeam links these translation units as a flat .o list; here they
  # become an archive, and an archive member is pulled in only to satisfy an
  # UNDEFINED symbol. Any "registers itself / overrides a weak default"
  # pattern silently loses its object. venc_jpeg.c's weak backend stubs were
  # exactly that (star6e_jpeg.c.o never linked -> snapshot ENOSYS); they were
  # deleted rather than papered over with --whole-archive, so a missing
  # backend is now a link error. Keep new files free of that pattern.
  add_library(venc_core STATIC ${VENC_CORE_SRC})
  target_include_directories(venc_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/venc ${CMAKE_CURRENT_SOURCE_DIR}/vendor)
  if(MABUR_VENC_BACKEND STREQUAL "cv610")
    target_compile_definitions(venc_core PRIVATE PLATFORM_CV610 _GNU_SOURCE)
    # PUBLIC through mabur_drone_core: the config loader and main.cpp compile
    # against the build's sensor table and its module loading.
    target_compile_definitions(mabur_drone_core PUBLIC MABUR_VENC_CV610)
    set(CV610_INC ${MABUR_CV610_SDK_INC}/kernel/include/hi3516cv6xx)
    set(CV610_ISP_INC ${MABUR_CV610_SDK_INC}/libraries/isp/include/hi3516cv6xx)
    target_include_directories(venc_core PRIVATE
      ${CV610_INC} ${CV610_INC}/exp_inc
      ${CV610_ISP_INC} ${CV610_ISP_INC}/3a ${CV610_ISP_INC}/ext_inc)
    target_link_directories(venc_core PUBLIC ${MABUR_CV610_SDK_LIB})
    # --no-as-needed: the sensor plugin is dlopen'd and resolves its ISP
    # symbols from the libraries already loaded, so none of them may be
    # dropped for looking unused. --allow-shlib-undefined: the MPP libraries
    # reference each other and the toolchain links them unversioned.
    target_link_libraries(venc_core PUBLIC
      -Wl,--allow-shlib-undefined -Wl,--no-as-needed
      ss_mpi ss_mpi_isp ss_mpi_ae ss_mpi_awb ot_mpi_isp ss_mpi_sysmem
      ss_mpi_sysbind acs bnr calcflicker dehaze drc extend_stats ir_auto ldci
      securec ot_osal -Wl,--as-needed)
  else()
    target_compile_definitions(venc_core PRIVATE
      PLATFORM_STAR6E HAVE_BACKEND_STAR6E=1 _GNU_SOURCE)
  endif()
  # -Os, overriding the Release -O3, to match the optimisation level waybeam
  # ships these exact translation units at. Not cosmetic: the VIF->VPE
  # "layout type 2, bindmode 4 not sync err" / "ISP channel [0] have NOT been
  # created" failure class on this SoC is documented upstream
  # (waybeam documentation/VIF_VPE_SYNC_ERROR_POSTMORTEM.md) as
  # binary-layout-dependent UB from MI struct handling, where "any binary
  # layout change -- new variables, different optimization -- can trigger the
  # latent bug". The MI structs the pipeline, the rate-control verbs and
  # the 3A threads pass by pointer are pinned with a _Static_assert at the
  # size the library copies, in star6e.h, star6e_cus3a.c, star6e_awb.c,
  # pipeline_common.c and star6e_pipeline.c (MI_VENC_Stream_t is not:
  # GetStream fills it field by field). Three were short and are fixed:
  # i6_vif_dev (48 of 52; the stack word past it broke warm restarts
  # whenever it was non-zero), MI_VENC_RcParam_t (52 of 56) and the CUS3A
  # AE status (48 of 65, written past on the 3A thread's stack). A new
  # failure of this class means checking a struct the asserts do not cover
  # yet. But running the SDK-facing code at an optimisation level nobody has flown it
  # at buys nothing. The encoder path is I/O-bound; waybeam does 1080p60 at
  # -Os on this chip.
  target_compile_options(venc_core PRIVATE -Os)
  target_link_libraries(venc_core PUBLIC dl pthread rt m)
  # maburd calls the venc_core.h facade directly on ARM. MABUR_HAVE_VENC
  # guards those call sites so the host build (which compiles none of
  # drone/venc) still builds and tests.
  target_link_libraries(mabur_drone_core PUBLIC venc_core)
  # Declared cycle, on purpose: venc_core's star6e_output.c calls
  # venc_frame_ring_{create,destroy,...}, which mabur_drone_core compiles
  # from drone/vendor. CMake repeats archives in a static-library cycle, so
  # the link does not depend on frame_source.cpp happening to pull that
  # object in first.
  target_link_libraries(venc_core PUBLIC mabur_drone_core)
  target_compile_definitions(mabur_drone_core PUBLIC MABUR_HAVE_VENC)
endif()
