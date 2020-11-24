
def charArray2hexList(chars):
    out = []
    for i in range(len(chars)):
        hex_char = hex(ord(chars[i]))
        if len(hex_char) == 3:
            hex_char = "0x0" + hex_char[-1]

        out.append(hex_char)

    return out
