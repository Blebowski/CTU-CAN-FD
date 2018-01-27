################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##
##   Class for language declaration of variable, constant, etc...
##
##	Revision history:
##		25.01.2018	First implementation
##
################################################################################

from abc import ABCMeta, abstractmethod

class LanDeclaration(metaclass=ABCMeta):

	specifier = None
	type = None
	bitWidth = 0
	bitIndex = None
	value = None
	name = None
	aligLen = 50
	gap = 0
	comment = None
	intType = None
	# IntType so far has: initialized, enum, bitfield
	
	def __init__(self, name, value, type=None, bitWidth=None, specifier=None,
					alignLen=50, gap=0, bitIndex=None, intType=None, comment=None):
		self.name = name
		self.value = value
		self.alignLen = alignLen
		self.gap = gap
		
		if (comment != None):
			self.comment = comment
		if (type != None):
			self.type = type
		if (bitWidth != None):
			self.bitWidth = bitWidth
		if (specifier != None):
			self.specifier = specifier
		if (bitIndex != None):
			self.bitIndex = bitIndex
		if (intType != None):
			self.intType = intType
	