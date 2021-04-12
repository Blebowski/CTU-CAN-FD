#!/bin/python3

import os, sys

def data_len_to_dlc(dlen) -> str:
    if (dlen == 0):
        return "0000"
    elif (dlen == 1):
        return "0001"
    elif (dlen == 2):
        return "0010"
    elif (dlen == 3):
        return "0011"
    elif (dlen == 4):
        return "0100"
    elif (dlen == 5):
        return "0101"
    elif (dlen == 6):
        return "0110"
    elif (dlen == 7):
        return "0111"
    elif (dlen == 8):
        return "1000"
    elif (dlen == 12):
        return "1001"
    elif (dlen == 16):
        return "1010"
    elif (dlen == 20):
        return "1011"
    elif (dlen == 24):
        return "1100"
    elif (dlen == 32):
        return "1101"
    elif (dlen == 48):
        return "1110"
    elif (dlen == 64):
        return "1111"
    else:
        print("Unsupported lenght: {}".format(dlen))

def parse_item(line_num, line, dest_file, is_last):
    """
    """
    C_MAX_REF_SEQ_LENGHT = 1024
    pos = 0;

    dest_file.write("   (\n")
    dest_file.write("    frame =>\n")
    dest_file.write("        (")
    
    if (line[pos:pos+3] != "CAN"):
        print("Invalid start of reference item: {} at line: {}".format(line[pos:pos+3], line_num))
        return
    pos += 4

    # FDF
    #print(line[pos:pos+3])
    if (line[pos:pos+3] == "2.0"):
        dest_file.write("frame_format => '0', ")
    else:
        dest_file.write("frame_format => '1', ")
    pos += 4

    # IDE
    #print(line[pos:pos+8])
    if (line[pos:pos+8] == "EXTENDED"):
        dest_file.write("ident_type => '1', ")
    else:
        dest_file.write("ident_type => '0', ")
    pos += 9

    # RTR
    #print(line[pos:pos+3])
    if (line[pos:pos+3] == "RTR"):
        dest_file.write("rtr => '1', ")
    else:
        dest_file.write("rtr => '0', ")
    pos += 4

    # BRS
    #print(line[pos:pos+3])
    if (line[pos:pos+3] == "BRS"):
        dest_file.write("brs => '1', ")
    else:
        dest_file.write("brs => '0', ")
    pos += 17

    # Data lenght
    dlen = int(line[pos:pos+2])
    dest_file.write("data_length => {}, ".format(dlen))
    pos += 7
    
    # ESI, Timestamp (default)
    dest_file.write("esi => '0',\n")
    dest_file.write('         timestamp => x"0000000000000000", ')
    dest_file.write('rwcnt => 0, ')
    dest_file.write('dlc => "{}", '.format(data_len_to_dlc(dlen)))

    # Identifier
    dest_file.write("identifier => {},\n".format(line[pos:pos+9]))
    pos += 16;
    
    # Data bytes
    dest_file.write("         data => (")
    for i in range(0, 64):
        if (i < dlen):
            dest_file.write('x"{}"'.format(line[pos:pos+2]))
            if (i != 63):
                dest_file.write(", ")
        pos += 3;     
    pos += 14
    
    if dlen < 64:
        dest_file.write('OTHERS => x"00"')

    dest_file.write(")\n")
    dest_file.write("        ),\n")
    
    # Bit sequence
    dest_file.write("    seq =>\n")
    dest_file.write("        (\n")
    
    bit_seq_list = line[pos:-1].split(" ")
    del bit_seq_list[-1]

    #print("Bit seq line: {}".format(bit_seq_list))
    #print("Lenght: {}".format(len(bit_seq_list)))
    seq_size = len(bit_seq_list) / 2

    i = 0
    while (i < len(bit_seq_list) - 1):

        if (i % 16 == 0):
            dest_file.write("           ")

        lenght = bit_seq_list[i]
        val = (int(bit_seq_list[i+1]) + 1) % 2 # Invert the value
        dest_file.write("('{}', {} ns), ".format(val, 10 * int(lenght)))

        if (i % 16 == 14):
            dest_file.write("\n")

        i += 2;

    dest_file.write(" OTHERS => ('1', 1 ns)\n")

    dest_file.write("        ),\n")
    
    # Sequence lenght
    dest_file.write("    seq_len => {}\n".format(int(len(bit_seq_list)/2)))

    if (is_last):
        dest_file.write("   )\n")
    else:
        dest_file.write("   ),\n")



def file_write_header(file, name):
    file.write("library ctu_can_fd_tb;\n")
    file.write("context ctu_can_fd_tb.ieee_context;\n")
    file.write("use ctu_can_fd_tb.reference_test_agent_pkg.all;\n")
    file.write("use ctu_can_fd_tb.feature_test_agent_pkg.all;\n")
    file.write("\n")
    file.write("package {} is\n".format(name))
    file.write("\n")    
    file.write("constant C_{} : t_reference_data_set := (\n".format(name))


def file_write_trailer(file, name):
    file.write(");\n")
    file.write("end package {};\n".format(name))
    file.write("\n");
    file.write("package body {} is\n".format(name));
    file.write("end package body;\n");


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Put two arguments: <SOURCE_FILE> <NAME>")
        sys.exit(1)

    src_file = open(sys.argv[1], 'r')
    dest_file = open("{}.vhd".format(sys.argv[2]), 'w')

    file_write_header(dest_file, sys.argv[2])

    lines = src_file.readlines()
    for line_num, line in enumerate(lines):
        #print(line_num)
        if (line_num == 999):
            parse_item(line_num, line, dest_file, True)
        else:
            parse_item(line_num, line, dest_file, False)
        

    file_write_trailer(dest_file, sys.argv[2])

    sys.exit(0)
