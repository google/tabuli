.data
.global payload_start
payload_start:
# Path is relative to PROJECT_ROOT/CMakeFiles/piccolo-targer.dir/
.incbin "../../payload.bin"
.global payload_end
payload_end:
