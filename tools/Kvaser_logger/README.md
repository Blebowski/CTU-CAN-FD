
# Kvaalt Logger

Generator / Logger for Kvaser and Quartus FPGAs.

To run this program following must be available:
1. Quartus must be installed with Signal TAP II Logic Analyzer. Logic analyzer must be in "/opt/quartus/bin/quartus_stp"
2. CAN Library from Kvaser must be installed.
3. Kvaser device with two channels MUST be connected to single CAN Bus (e.g. Kvaser II Memorator Pro)
4. Altera USB Blaster must be connected to FPGA with Signal TAP II Logic Analyzer design synthesized in.
5. CAN RX line from Kvaser must be connected to FPGA.



Program sequence:
1. Initialize 2 Kvaser Channels and Start Signal TAP II in separate deamon.
2. Generate random CAN Frame.
3. Start Signal TAP II Logic analyzer recording.
4. Send CAN Frame by Kvaser.
5. Wait until logic analyzer aquisition has finished.
6. Export data to temporary file by Signal TAP II (.tbl file)
7. Parse bit sequence from .tbl file, and add it to output file
8. Repeat "frame_count" times

Can be executed in following way:
`./build/kvalt_logger 1000 stp_session_template.stp auto_signaltap_0 "signal_set: 2018/06/18 19:22:19  #0" "trigger: 2018/06/18 19:22:19  #1" build/output_file`

