# Ceres Solver - A fast non-linear least squares minimizer
# Copyright 2015 Google Inc. All rights reserved.
# http://ceres-solver.org/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Google Inc. nor the names of its contributors may be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: arnaudgelas@gmail.com (Arnaud Gelas)
#         alexs.mac@gmail.com (Alex Stewart)

if (COMMAND cmake_policy)
  # Ignore empty elements in LIST() commands.
  cmake_policy(SET CMP0007 OLD)
endif (COMMAND cmake_policy)

if (NOT EXISTS "/home/s/carto_ws/build_isolated/ceres-solver/devel/install_manifest.txt")
  message(FATAL_ERROR "Cannot find install manifest: "
                      "\"/home/s/carto_ws/build_isolated/ceres-solver/devel/install_manifest.txt\"")
endif (NOT EXISTS "/home/s/carto_ws/build_isolated/ceres-solver/devel/install_manifest.txt")

file(READ "/home/s/carto_ws/build_isolated/ceres-solver/devel/install_manifest.txt" INSTALL_MANIFEST)
string(REGEX REPLACE "\n" ";" INSTALL_MANIFEST "${INSTALL_MANIFEST}")
list(REVERSE INSTALL_MANIFEST)

foreach (INSTALLED_FILE ${INSTALL_MANIFEST})
  # Save the root ceres include install directory, e.g. /usr/local/include/ceres
  # so that we can remove it at the end.
  if (NOT CERES_INCLUDE_INSTALL_ROOT)
    get_filename_component(FILE_NAME ${INSTALLED_FILE} NAME)
    if (FILE_NAME STREQUAL ceres.h)
      # Ensure that the directory is nested as we expect, as we are going to
      # remove it, and we do not want to remove files pertaining to anyone else.
      get_filename_component(PARENT_DIR ${INSTALLED_FILE} PATH)
      get_filename_component(PARENT_DIR_NAME ${PARENT_DIR} NAME)
      if (PARENT_DIR_NAME STREQUAL ceres AND IS_DIRECTORY ${PARENT_DIR})
        set(CERES_INCLUDE_INSTALL_ROOT ${PARENT_DIR})
      endif (PARENT_DIR_NAME STREQUAL ceres AND IS_DIRECTORY ${PARENT_DIR})
    endif (FILE_NAME STREQUAL ceres.h)
  endif (NOT CERES_INCLUDE_INSTALL_ROOT)

  message(STATUS "Uninstalling \"$ENV{DESTDIR}${INSTALLED_FILE}\"")
  if (EXISTS "$ENV{DESTDIR}${INSTALLED_FILE}")
    execute_process(COMMAND /usr/bin/cmake
                    -E remove "$ENV{DESTDIR}${INSTALLED_FILE}"
                    OUTPUT_VARIABLE RM_OUT
                    RESULT_VARIABLE RM_RETVAL)
    if (NOT ${RM_RETVAL} EQUAL 0)
      message(FATAL_ERROR
              "Problem when removing \"$ENV{DESTDIR}${INSTALLED_FILE}\"")
    endif (NOT ${RM_RETVAL} EQUAL 0)
  else (EXISTS "$ENV{DESTDIR}${INSTALLED_FILE}")
    message(STATUS "File \"$ENV{DESTDIR}${INSTALLED_FILE}\" does not exist.")
  endif (EXISTS "$ENV{DESTDIR}${INSTALLED_FILE}")
endforeach(INSTALLED_FILE)

# Removing Ceres include install directory.
if (CERES_INCLUDE_INSTALL_ROOT AND
    EXISTS ${CERES_INCLUDE_INSTALL_ROOT})
  message(STATUS "Removing Ceres include install directory: "
                 "\"$ENV{DESTDIR}${CERES_INCLUDE_INSTALL_ROOT}\"")
  execute_process(COMMAND /usr/bin/cmake
                  -E remove_directory
                  "$ENV{DESTDIR}${CERES_INCLUDE_INSTALL_ROOT}"
                  OUTPUT_VARIABLE RM_OUT
                  RESULT_VARIABLE RM_RETVAL)
  if (NOT ${RM_RETVAL} EQUAL 0)
    message(FATAL_ERROR
      "Failed to remove: \"$ENV{DESTDIR}${CERES_INCLUDE_INSTALL_ROOT\"")
  endif (NOT ${RM_RETVAL} EQUAL 0)
else (CERES_INCLUDE_INSTALL_ROOT AND
    EXISTS ${CERES_INCLUDE_INSTALL_ROOT})
  message(FATAL_ERROR "Failed to find Ceres installed include directory "
                      "(e.g. /usr/local/include/ceres), candidate: "
                      "\"$ENV{DESTDIR}${CERES_INCLUDE_INSTALL_ROOT}\"")
endif (CERES_INCLUDE_INSTALL_ROOT AND
  EXISTS ${CERES_INCLUDE_INSTALL_ROOT})
