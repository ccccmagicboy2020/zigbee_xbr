sum = 0
key = 0xcc
byte_list = []
with open("HC-MCU-XBR.bin", "rb") as f1:
    byte_list = list(f1.read())
    for i in range(len(byte_list)):
        byte_list[i] ^= key
    print(byte_list)
    
with open("HC-MCU-XBR_ota.bin","wb") as f2:
    f2.write(bytearray(byte_list))
    for x in byte_list:
        sum += x
    print(hex(sum))


