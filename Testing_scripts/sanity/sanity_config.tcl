################################################################################
##
## CAN with Flexible Data-Rate IP Core 
##
## Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
##
## This program is free software; you can redistribute it and/or
## modify it under the terms of the GNU General Public License
## as published by the Free Software Foundation; either version 2
## of the License, or (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## The CAN protocol is developed by Robert Bosch GmbH and     
## protected by patents. Anybody who wants to implement this    
## IP core on silicon has to obtain a CAN protocol license
## from Bosch.
##  
## Description:
## 			Sanity test configuration file
################################################################################

quietly set SAN_CFG [ list  [ list star 10 10 10 10 0.0 0.0 \
									10 10 10 10 \
									0 5 10 15 \
									70.0 5.0 300000.0 100000.0 \
									4 1 8 8 8 3 3 1 5 2 \
									"1Mb/10Mb 20 m Star" 5
							] \
							[ list star 10 10 10 10 0.0 0.0 \
									10 10 10 10 \
									0 5 10 15 \
									70.0 5.0 300000.0 100000.0 \
									8 2 9 5 5 4 9 5 5 4 \
									"500kBit/2Mb 20 m Star" 5
							] \
							[ list bus 10 10 10 0 0.0 0.0 \
									10 10 10 10 \
									0 5 10 15 \
									70.0 5.0 300000.0 100000.0 \
									4 1 8 8 8 3 0 3 3 3 \
									"1Mb/14,2Mb 40 m Bus" 5
							] \
							[ list star 15 15 15 15 0.0 0.0 \
									20 20 20 20 \
									20 40 60 100 \
									70.0 5.0 300000.0 100000.0 \
									10 2 3 3 3 2 3 3 3 2 \
									"1Mb/5Mb 30 m Star" 5
							] \
							[ list bus 13 13 13 0.0 0.0 0.0 \
									20 20 20 20 \
									20 40 60 100 \
									70.0 5.0 500000.0 100000.0 \
									10 2 3 3 3 2 3 3 3 2 \
									"1Mb/5Mb 40 m Bus" 5
							] \
							[ list bus 13 13 13 0.0 0.0 0.0 \
									20 20 20 20 \
									10 15 25 30 \
									75.0 5.0 500000.0 200000.0 \
									10 5 3 3 3 2 3 3 3 2 \
									"1Mb/2Mb 40 m Bus" 5
							]
			]

