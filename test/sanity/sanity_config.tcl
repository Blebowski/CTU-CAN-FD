################################################################################
## 
## CAN with Flexible Data-Rate IP Core 
## 
## Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
## 
## Project advisor: Jiri Novak <jnovak@fel.cvut.cz>
## Department of Measurement         (http://meas.fel.cvut.cz/)
## Faculty of Electrical Engineering (http://www.fel.cvut.cz)
## Czech Technical University        (http://www.cvut.cz/)
## 
## Permission is hereby granted, free of charge, to any person obtaining a copy 
## of this VHDL component and associated documentation files (the "Component"), 
## to deal in the Component without restriction, including without limitation 
## the rights to use, copy, modify, merge, publish, distribute, sublicense, 
## and/or sell copies of the Component, and to permit persons to whom the 
## Component is furnished to do so, subject to the following conditions:
## 
## The above copyright notice and this permission notice shall be included in 
## all copies or substantial portions of the Component.
## 
## THE COMPONENT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
## AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
## FROM, OUT OF OR IN CONNECTION WITH THE COMPONENT OR THE USE OR OTHER DEALINGS 
## IN THE COMPONENT.
## 
## The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
## Anybody who wants to implement this IP core on silicon has to obtain a CAN 
## protocol license from Bosch.
## 
################################################################################

################################################################################
## Description:
## 			Sanity test configuration file
################################################################################

quietly set SAN_CFG [ list  [ list star 10 10 10 10 0.0 0.0 \
									10 10 10 10 \
									0 5 10 15 \
									70.0 5.0 300000.0 100000.0 \
									4 1 8 8 8 3 3 1 5 2 \
									"1Mb/10Mb 20 m Star" 5
							] 
							##[ list star 10 10 10 10 0.0 0.0 \
									10 10 10 10 \
									0 5 10 15 \
									70.0 5.0 300000.0 100000.0 \
									8 2 9 5 5 4 9 5 5 4 \
									"500kBit/2Mb 20 m Star" 5
							##] \
							##[ list bus 10 10 10 0 0.0 0.0 \
									10 10 10 10 \
									0 5 10 15 \
									70.0 5.0 300000.0 100000.0 \
									4 1 8 8 8 3 0 3 3 3 \
									"1Mb/14,2Mb 40 m Bus" 5
							##] \
							##[ list star 15 15 15 15 0.0 0.0 \
									20 20 20 20 \
									20 40 60 100 \
									70.0 5.0 300000.0 100000.0 \
									10 2 3 3 3 2 3 3 3 2 \
									"1Mb/5Mb 30 m Star" 5
							##] \
							##[ list bus 13 13 13 0.0 0.0 0.0 \
									20 20 20 20 \
									20 40 60 100 \
									70.0 5.0 500000.0 100000.0 \
									10 2 3 3 3 2 3 3 3 2 \
									"1Mb/5Mb 40 m Bus" 5
							##] \
							##[ list bus 13 13 13 0.0 0.0 0.0 \
									20 20 20 20 \
									10 15 25 30 \
									75.0 5.0 500000.0 200000.0 \
									10 5 3 3 3 2 3 3 3 2 \
									"1Mb/2Mb 40 m Bus" 5
							##]
			]

