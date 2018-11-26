################################################################################                                                     
## 
##   CAN with Flexible Data-Rate IP Core 
##   
##   Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
##
##   Class for component declaration and instantantion in VHDL.
##
##	Revision history:
##		06.10.2018	First implementation
##
################################################################################

from abc import ABCMeta, abstractmethod

class VhdlCompDeclaration(LanDeclaration):
	
    # VHDL component declaration inherits following attributes from
    # LanDeclaration:
    #   type = name of the entity, architecture or component 
    #   value = name of instance in case of component instatiation.
	#   comment = VHDL comment before the component instantiation
    #   intType = "component", "entity" or "architecture"
	
    # When true, component instance will be created, When false component 
    # declaration will be created.
    instance = True

    # List of "LanDeclarations" with signals which are ports of entity or
    # component. For architecture it contains list of internal signals.
    ports = None
    
    # List of "LanDeclarations" with constants which are generics of entity.
    generics = None
	
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
	
