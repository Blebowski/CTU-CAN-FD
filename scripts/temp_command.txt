# To generate register map package with register addresses and bit offset:
py gen_vhdl_package.py --licPath ../LICENSE --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --fieldMap CAN_FD_8bit_regs --addrMap CAN_FD_32bit_regs --wordWidth 32 --outFile ../src/Libraries/CAN_FD_register_map.vhd --packName CAN_FD_register_map


# To generate frame format related constants
py gen_vhdl_package.py --licPath ../LICENSE --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --fieldMap CAN_FD_frame_format --wordWidth 32 --outFile ../src/Libraries/CAN_FD_frame_format.vhd --packName CAN_FD_frame_format