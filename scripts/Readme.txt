# To generate register map package with register addresses and bit offset:
py gen_vhdl_package.py --licPath ../LICENSE --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --fieldMap CAN_FD_8bit_regs --addrMap CAN_FD_8bit_regs --wordWidth 32 --outFile ../src/Libraries/CAN_FD_register_map.vhd --packName CAN_FD_register_map


# To generate frame format related constants
py gen_vhdl_package.py --licPath ../LICENSE --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --fieldMap CAN_FD_frame_format --addrMap CAN_FD_frame_format --wordWidth 32 --outFile ../src/Libraries/CAN_FD_frame_format.vhd --packName CAN_FD_frame_format


# To generate C header file
py gen_c_header.py --licPath ../LICENSE --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --addrMap CAN_FD_32bit_regs --fieldMap CAN_FD_8bit_regs --wordWidth 32 --outFile ../driver/kernel_header_draft.h --headName CAN_FD_frame_format


# To generate lyx docu for register map
py gen_lyx_docu.py --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --memMap CAN_FD_8bit_regs --wordWidth 32 --lyxTemplate ../doc/core/template.lyx --outFile ../doc/core/registerMap.lyx --chaptName "Register map" --genRegions True --genFiDesc True
