# OpenSees: isteğe bağlı GitHub kaynak klonu + Berkeley önceden derlenmiş Windows paketi + POST_BUILD kopyası.

option(PORTAL_DOWNLOAD_OPENSEES_SOURCE
       "CMake yapılandırmasında OpenSees v3.8.0 kaynak ağacını klonla (ayrıca kendiniz derlemeniz gerekir)" OFF)

set(PORTAL_OPENSEES_SOURCE_DIR "${CMAKE_SOURCE_DIR}/third_party/opensees-source"
    CACHE PATH "OpenSees kaynaklarının çıkarılacağı klasör (PORTAL_DOWNLOAD_OPENSEES_SOURCE=ON iken)")

if(PORTAL_DOWNLOAD_OPENSEES_SOURCE)
    include(FetchContent)
    FetchContent_Declare(
        opensees_upstream
        GIT_REPOSITORY https://github.com/OpenSees/OpenSees.git
        GIT_TAG v3.8.0
        GIT_SHALLOW TRUE
        SOURCE_DIR "${PORTAL_OPENSEES_SOURCE_DIR}"
    )
    FetchContent_GetProperties(opensees_upstream)
    if(NOT opensees_upstream_POPULATED)
        message(STATUS "OpenSees kaynakları indiriliyor → ${PORTAL_OPENSEES_SOURCE_DIR} ...")
        FetchContent_Populate(opensees_upstream)
    endif()
    message(STATUS "OpenSees kaynakları: ${opensees_upstream_SOURCE_DIR}")
    message(STATUS "Kaynak derleme: https://opensees.github.io/OpenSeesDocumentation/developer/build.html")
endif()

# Berkeley resmi x64 paketi (OpenSees 3.8.0) — kaynaktan derlemek yerine önerilen yol.
set(_portal_berkeley_exe "${CMAKE_SOURCE_DIR}/third_party/opensees-berkeley/OpenSees3.8.0/bin/OpenSees.exe")
set(_portal_berkeley_zip "${CMAKE_SOURCE_DIR}/third_party/cache/OpenSees3.8.0-x64.exe.zip")
set(_portal_berkeley_url "https://opensees.berkeley.edu/OpenSees/code/OpenSees3.8.0-x64.exe.zip")

if(WIN32)
    option(PORTAL_FETCH_BERKELEY_OPENSEES
           "third_party içinde OpenSees.exe yoksa Berkeley sitesinden x64 zip indir (~36 MB)" ON)
    if(PORTAL_FETCH_BERKELEY_OPENSEES AND NOT EXISTS "${_portal_berkeley_exe}")
        file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/third_party/cache")
        if(NOT EXISTS "${_portal_berkeley_zip}")
            message(STATUS "Berkeley OpenSees 3.8.0 x64 indiriliyor (bir kez)...")
            file(DOWNLOAD "${_portal_berkeley_url}" "${_portal_berkeley_zip}"
                 SHOW_PROGRESS
                 STATUS _portal_dl_status
                 TLS_VERIFY ON)
            list(GET _portal_dl_status 0 _portal_dl_rc)
            if(NOT _portal_dl_rc EQUAL 0)
                list(GET _portal_dl_status 1 _portal_dl_err)
                message(WARNING "OpenSees zip indirilemedi (${_portal_dl_err}). İnternet veya PORTAL_FETCH_BERKELEY_OPENSEES=OFF deneyin.")
            endif()
        endif()
        if(EXISTS "${_portal_berkeley_zip}" AND NOT EXISTS "${_portal_berkeley_exe}")
            file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/third_party/opensees-berkeley")
            execute_process(
                COMMAND powershell -NoProfile -ExecutionPolicy Bypass -Command
                        "Expand-Archive -LiteralPath '${_portal_berkeley_zip}' -DestinationPath '${CMAKE_SOURCE_DIR}/third_party/opensees-berkeley' -Force"
                RESULT_VARIABLE _portal_ex
                ERROR_VARIABLE _portal_ex_err)
            if(NOT _portal_ex EQUAL 0)
                message(WARNING "Zip açılamadı: ${_portal_ex_err}")
            endif()
        endif()
    endif()
endif()

set(OPENSEES_EXE_TO_BUNDLE "" CACHE FILEPATH
    "İsteğe bağlı: OpenSees.exe tam yolu. Boş bırakılırsa third_party Berkeley sürümü kullanılır (varsa).")

set(_portal_bundle_src "")
if(NOT OPENSEES_EXE_TO_BUNDLE STREQUAL "")
    set(_portal_bundle_src "${OPENSEES_EXE_TO_BUNDLE}")
elseif(EXISTS "${_portal_berkeley_exe}")
    set(_portal_bundle_src "${_portal_berkeley_exe}")
endif()

if(_portal_bundle_src)
    if(NOT EXISTS "${_portal_bundle_src}")
        message(WARNING "OpenSees paket dosyası yok: ${_portal_bundle_src}")
    else()
        message(STATUS "OpenSees POST_BUILD kaynağı: ${_portal_bundle_src}")
        if(WIN32)
            set(_portal_opensees_dest_name "OpenSees.exe")
        else()
            set(_portal_opensees_dest_name "OpenSees")
        endif()
        get_filename_component(_portal_bundle_bindir "${_portal_bundle_src}" DIRECTORY)
        get_filename_component(_portal_bundle_root "${_portal_bundle_bindir}" DIRECTORY)
        set(_portal_bundle_lib "${_portal_bundle_root}/lib")
        set(_portal_iomp5 "${_portal_bundle_bindir}/libiomp5md.dll")
        # Berkeley: bin/OpenSees.exe + bin/libiomp5md.dll + lib/tcl8.6/... — Tcl olmadan "wipe" tanınmaz.
        if(WIN32 AND EXISTS "${_portal_bundle_lib}" AND IS_DIRECTORY "${_portal_bundle_lib}")
            if(EXISTS "${_portal_iomp5}")
                add_custom_command(
                    TARGET PortalSolver
                    POST_BUILD
                    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                            "${_portal_bundle_src}"
                            "$<TARGET_FILE_DIR:PortalSolver>/${_portal_opensees_dest_name}"
                    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                            "${_portal_iomp5}"
                            "$<TARGET_FILE_DIR:PortalSolver>/libiomp5md.dll"
                    COMMAND "${CMAKE_COMMAND}" -E copy_directory
                            "${_portal_bundle_lib}"
                            "$<TARGET_FILE_DIR:PortalSolver>/lib"
                    COMMENT "OpenSees.exe, libiomp5md.dll ve lib/ (Tcl) PortalSolver çıktısına kopyalanıyor"
                )
            else()
                add_custom_command(
                    TARGET PortalSolver
                    POST_BUILD
                    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                            "${_portal_bundle_src}"
                            "$<TARGET_FILE_DIR:PortalSolver>/${_portal_opensees_dest_name}"
                    COMMAND "${CMAKE_COMMAND}" -E copy_directory
                            "${_portal_bundle_lib}"
                            "$<TARGET_FILE_DIR:PortalSolver>/lib"
                    COMMENT "OpenSees.exe ve lib/ (Tcl) PortalSolver çıktısına kopyalanıyor"
                )
            endif()
        elseif(WIN32 AND EXISTS "${_portal_iomp5}")
            add_custom_command(
                TARGET PortalSolver
                POST_BUILD
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                        "${_portal_bundle_src}"
                        "$<TARGET_FILE_DIR:PortalSolver>/${_portal_opensees_dest_name}"
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                        "${_portal_iomp5}"
                        "$<TARGET_FILE_DIR:PortalSolver>/libiomp5md.dll"
                COMMENT "OpenSees.exe ve libiomp5md.dll (lib/tcl yok — Tcl kurulumu gerekebilir)"
            )
        else()
            add_custom_command(
                TARGET PortalSolver
                POST_BUILD
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                        "${_portal_bundle_src}"
                        "$<TARGET_FILE_DIR:PortalSolver>/${_portal_opensees_dest_name}"
                COMMENT "OpenSees yürütülebilir dosyasını PortalSolver çıktı klasörüne kopyala"
            )
        endif()
    endif()
else()
    message(STATUS "OpenSees POST_BUILD yok — third_party/opensees-berkeley yok ve OPENSEES_EXE_TO_BUNDLE boş.")
endif()
