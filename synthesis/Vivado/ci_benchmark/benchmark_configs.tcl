###################################################################################################
## Configuration
###################################################################################################

set PART xc7z020clg400-3
set PROJECT_NAME CTU_CAN_FD_BENCHMARK
set PROJ_ROOT ../../..
set TOP can_top_level

set DESIGN_CONFIGS [list \
    [ dict create \
     name "typical_design_config" \
     generics \
        [ dict create \
         rx_buffer_size         128 \
         txt_buffer_count       4 \
         sup_filtA              "1'b1" \
         sup_filtB              "1'b0" \
         sup_filtC              "1'b0" \
         sup_range              "1'b1" \
         target_technology      1 \
         sup_traffic_ctrs       "1'b1" \
         sup_test_registers     "1'b0" \
         sup_parity             "1'b1" \
         active_timestamp_bits  32 \
         reset_buffer_rams      "1'b0" \
        ]
    ] \
]

