import secrets

result = [];

for load in range(129):
  bits = [1] * load + [0] * (128 - load)
  bits = bits * 128
  for j in range(len(bits)):
    a = bits[j]
    n = secrets.randbelow(len(bits) - j)
    b = bits[j + n]
    bits[j] = b
    bits[j + n] = a
  while True:
    n = secrets.randbelow(len(bits) - 128)
    sub = bits[n:n + 128]
    if sum(sub) == load:
      #print("".join([str(b) for b in result]))
      for w in range(4):
        v = 0
        for s in range(32):
          v += sub[w * 32 + s] << s
        result.append(f'0x{v:X}')
      break
print(", ".join(result))
