.data
.global payload_start
payload_start:
# Path is relative to PROJECT_ROOT/CMakeFiles/piccolo.dir/src/
.incbin "../../../src/payload.bin"
.global arm_payload_end
payload_end:
