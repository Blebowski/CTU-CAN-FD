# Register map generation

Register map can be updated by Python script like so:

python3.6 update_reg_map.py --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --updVHDLPackage True --updHeaderFile True --updLyxDocs True --updRTLRegMap True

Arguments have following meaning:

    xactSpec        Relative path to IP-XACT specification with register map description
    updVHDLPackage  If VHDL register map package with constants definition should be generated (y/n)
    updHeaderFile   If C Header file for register map should be generated (y/n)
    updLyxDocs      If Lyx documentation should be generated (y/n)
    updRTLRegMap    If VHDL RTL codes of register map should be implemented.

