##======================================================================
#
# PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
# Please see our website at <http://pixhawk.ethz.ch>
# 
#
# Original Authors:
#   @author Reto Grieder <www.orxonox.net>
#   @author Fabian Landau <www.orxonox.net>
# Contributing Authors (in alphabetical order):
#  
# Todo:
#
#
# (c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>
# 
# This file is part of the PIXHAWK project
# 
#     PIXHAWK is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
# 
#     PIXHAWK is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.
# 
##========================================================================

FUNCTION(HANDLE_LIBRARY_TYPES _name)
  # Additional libraries can be added as additional arguments
  IF(${_name}_LIBRARY_DEBUG AND ${_name}_LIBRARY_OPTIMIZED)
    SET(${_name}_LIBRARY
      optimized ${${_name}_LIBRARY_OPTIMIZED} ${ARGN}
      debug     ${${_name}_LIBRARY_DEBUG}     ${ARGN}
      PARENT_SCOPE
    )
  ELSE()
    SET(${_name}_LIBRARY
      ${${_name}_LIBRARY_OPTIMIZED} ${ARGN}
      PARENT_SCOPE
     )
  ENDIF()
ENDFUNCTION(HANDLE_LIBRARY_TYPES)
