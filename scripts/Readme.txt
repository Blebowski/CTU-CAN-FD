# To generate register map package with register addresses and bit offset:
python3.5 gen_vhdl_package.py --licPath ../LICENSE --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --fieldMap Regs --addrMap Regs --wordWidth 32 --outFile ../src/Libraries/CAN_FD_register_map.vhd --packName CAN_FD_register_map


# To generate frame format related constants
python3.5 gen_vhdl_package.py --licPath ../LICENSE --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --fieldMap Frame_format --addrMap Frame_format --wordWidth 32 --outFile ../src/Libraries/CAN_FD_frame_format.vhd --packName CAN_FD_frame_format


# To generate C header files (register map and frame format)
python3.5 gen_c_header.py --licPath ../lic/gpl_v2.txt --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --addrMap Regs --fieldMap Regs --wordWidth 32 --outFile ../driver/ctu_can_fd_regs.h --headName CAN_FD_frame_format
py gen_c_header.py --licPath ../lic/gpl_v2.txt --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --addrMap Frame_format --fieldMap Frame_format --wordWidth 32 --outFile ../driver/ctu_can_fd_frame.h --headName CAN_FD_frame_format


# To generate Lyx docu for register map
python3.5 gen_lyx_docu.py --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --memMap Regs --wordWidth 32 --lyxTemplate ../doc/core/template.lyx --outFile ../doc/core/registerMap.lyx --chaptName "Register map" --genRegions True --genFiDesc True


# To generate Lyx docu for CAN frame
python3.5 gen_lyx_docu.py --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --memMap Frame_format --wordWidth 32 --lyxTemplate ../doc/core/template.lyx --outFile ../doc/core/CANFrameFormat.lyx --chaptName "CAN Frame format" --genRegions False --genFiDesc True


#########################################
## To perform complete update
#########################################
python3.5 update_reg_map.py --xactSpec ../spec/CTU/ip/CAN_FD_IP_Core/2.1/CAN_FD_IP_Core.2.1.xml --updVHDL True --updHeader True --updDocs True
