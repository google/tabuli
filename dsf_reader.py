from ctypes import *


class DsdHeader(LittleEndianStructure):
    _fields_ = [('name', c_char * 4),
                ('chunk_len', c_uint32 * 2),  # 64-bit types are malfunctioning
                ('file_len', c_uint32 * 2),
                ('metadata_offset', c_uint32 * 2),
                ]


class FmtHeader(LittleEndianStructure):
    _fields_ = [('name', c_char * 4),
                ('chunk_len', c_uint32 * 2),  # 64-bit types are malfunctioning
                ('format_version', c_uint32),
                ('format_id', c_uint32),
                ('channel_type', c_uint32),
                ('channel_num', c_uint32),
                ('sampling_freq', c_uint32),
                ('bits_per_sample', c_uint32),
                ('sample_count', c_uint32 * 2),
                ('block_size_per_channel', c_uint32),
                ('reserved', c_uint32),
                ]


class DataHeader(LittleEndianStructure):
    _fields_ = [('name', c_char * 4),
                ('chunk_len', c_uint32 * 2),  # 64-bit types are malfunctioning
                ]


class Data(LittleEndianStructure):
    _fields_ = [('data', c_char * 4096),
                ]

output = bytearray()

with open('src.dsf', 'rb') as file:
    # --------------------------------------------------------------------------
    hdr = DsdHeader()
    if file.readinto(hdr) != sizeof(hdr):
        print("Failed to read file header")
        quit()
    if hdr.name.decode('utf-8') != "DSD ":
        print("Not a DSD file")
        quit()
    if [hdr.chunk_len[0], hdr.chunk_len[1]] != [28, 0]:
        print("Malformed DSD header")
        quit()
    if hdr.file_len[1] != 0 or hdr.metadata_offset[1] != 0:
        print("Sample too large")
        quit()
    print(f"File len: {hdr.file_len[0]}")
    print(f"Metadata offser: {hdr.metadata_offset[0]}")
    # --------------------------------------------------------------------------
    fmt = FmtHeader()
    if file.readinto(fmt) != sizeof(fmt):
        print("Failed to read format header")
        quit()
    if fmt.name.decode('utf-8') != "fmt ":
        print("Expected FMT chunk")
        quit()
    if [fmt.chunk_len[0], fmt.chunk_len[1]] != [52, 0]:
        print("Unexpected FMT chunk length")
        quit()
    print(f"Format version: {fmt.format_version}")
    print(f"Format id: {fmt.format_id}")
    print(f"Channel type: {fmt.channel_type}")
    if fmt.channel_type != 2:
        print("Only stereo is supported")
        quit()
    print(f"Channel num: {fmt.channel_num}")
    if fmt.channel_num != 2:
        print("Only stereo is supported")
        quit()
    print(f"Sampling frequency: {fmt.sampling_freq}")
    if fmt.sampling_freq != 2822400:
        print("Only stereo is supported")
        quit()
    print(f"Bits per sample: {fmt.bits_per_sample}")
    if fmt.bits_per_sample != 1:
        print("MSB not supported")
        quit()
    print(f"Sample count: {fmt.sample_count[0]}")
    block_len = fmt.block_size_per_channel
    print(f"Block size per channel: {fmt.block_size_per_channel}")
    print(f"Reserved: {fmt.reserved}")
    # --------------------------------------------------------------------------
    data = DataHeader()
    if file.readinto(data) != sizeof(data):
        print("Failed to read data header")
        quit()
    if data.name.decode('utf-8') != "data":
        print("Expected DATA chunk")
        quit()
    if data.chunk_len[1] != 0:
        print("Sample too large")
        quit()
    print(f"Data chunk length: {data.chunk_len[0]}")
    payload_len = data.chunk_len[0] - 12
    print(f"Payload length: {payload_len}")
    num_blocks = payload_len / block_len
    print(f"Num blocks: {num_blocks}")
    num_blocks /= 2
    print(f"Num mono-blocks: {num_blocks}")
    num_blocks = int(num_blocks)
    # --------------------------------------------------------------------------
    payload = Data()
    for i in range(num_blocks):
        if file.readinto(payload) != sizeof(payload):
            print("Failed to read payload")
            quit()
        output.extend(list(payload.data))
        # Just drop the other channel.
        if file.readinto(payload) != sizeof(payload):
            print("Failed to read payload")
            quit()

with open('dsd.raw', 'wb') as file:
    file.write(output)
