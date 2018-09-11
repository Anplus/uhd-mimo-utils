#
# Copyright 2010-2013 Ettus Research LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

########################################################################
# example applications
########################################################################
SET(util_runtime_sources
    mimo_rx_to_file.cpp
    mimo_tx_from_file.cpp
)

#for each source: build an executable and install
FOREACH(util_source ${util_runtime_sources})
    GET_FILENAME_COMPONENT(util_name ${util_source} NAME_WE)
    ADD_EXECUTABLE(${util_name} ${util_source})
    TARGET_LINK_LIBRARIES(${util_name} ${UHD_LIBRARIES} ${Boost_LIBRARIES})
    INSTALL(TARGETS ${util_name} DESTINATION bin COMPONENT utilities)
ENDFOREACH(util_source)