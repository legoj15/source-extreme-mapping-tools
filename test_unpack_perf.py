import time
import struct
import random

# Generate 1M bytes of fake lighting data
data = bytearray(random.getrandbits(8) for _ in range(1_000_000))
luxel_count = 100_000

print("Benchmarking struct unpacking...")

start = time.time()
sample_ofs = 0
luminances = []
lighting_slice = data[sample_ofs : sample_ofs + luxel_count * 4]
luminances = [
    (0.2126 * r + 0.7152 * g + 0.0722 * b) * (2.0 ** exp)
    for r, g, b, exp in struct.iter_unpack('BBBb', lighting_slice)
]
end = time.time()
print(f"List comprehension + iter_unpack: {end-start:.4f}s")


start = time.time()
luminances2 = []
for i in range(luxel_count):
    ofs = sample_ofs + i * 4
    r, g, b, exp = struct.unpack_from('BBBb', data, ofs)
    lum = (0.2126 * r + 0.7152 * g + 0.0722 * b) * (2.0 ** exp)
    luminances2.append(lum)
end = time.time()
print(f"For loop + unpack_from: {end-start:.4f}s")

# Test struct.unpack_from for nodes
node_count = 50000
node_data = bytearray(random.getrandbits(8) for _ in range(node_count * 32))

start = time.time()
nodes = []
for i in range(node_count):
    ofs = i * 32
    vals = struct.unpack_from('<3i3h3h2Hh2x', node_data, ofs)
    nodes.append({
        'planenum': vals[0],
        'children': (vals[1], vals[2]),
        'mins': (vals[3], vals[4], vals[5]),
        'maxs': (vals[6], vals[7], vals[8]),
        'firstface': vals[9],
        'numfaces': vals[10],
        'area': vals[11],
    })
end = time.time()
print(f"Nodes For loop + unpack_from: {end-start:.4f}s")

start = time.time()
nodes2 = [
    {
        'planenum': v[0],
        'children': (v[1], v[2]),
        'mins': (v[3], v[4], v[5]),
        'maxs': (v[6], v[7], v[8]),
        'firstface': v[9],
        'numfaces': v[10],
        'area': v[11],
    }
    for v in struct.iter_unpack('<3i3h3h2Hh2x', node_data)
]
end = time.time()
print(f"Nodes List comp + iter_unpack: {end-start:.4f}s")
