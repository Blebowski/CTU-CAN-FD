set_property SRC_FILE_INFO {cfile:/DOKUMENTY/Skola/CVUT-FEL/ctu_can_fd_2/synthesis/Constraints/CTU_CAN_FD_Xilinx.sdc rfile:../../../../../Constraints/CTU_CAN_FD_Xilinx.sdc id:1} [current_design]
set_property src_info {type:XDC file:1 line:48 export:INPUT save:INPUT read:READ} [current_design]
set_property ASYNC_REG true $rs_sync_chain_1
set_property src_info {type:XDC file:1 line:49 export:INPUT save:INPUT read:READ} [current_design]
set_property ASYNC_REG true $rs_sync_chain_2
set_property src_info {type:XDC file:1 line:56 export:INPUT save:INPUT read:READ} [current_design]
set_property ASYNC_REG true [get_cells *bus_sync_comp/sync_Chain_1*]
set_property src_info {type:XDC file:1 line:57 export:INPUT save:INPUT read:READ} [current_design]
set_property ASYNC_REG true [get_cells *bus_sync_comp/sync_Chain_2*]
