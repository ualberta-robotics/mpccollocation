
try:
    # Python 2
    from future_builtins import filter
except ImportError:
    # Python 3
    pass

with open('source.txt', 'rb') as file_in:
    with open("commanded angles.txt", "wb") as file_out1:
            file_out1.writelines(filter(lambda line: b'commanded angles:' in line, file_in))

with open('source.txt', 'rb') as file_in:
    with open("current rotation.txt", "wb") as file_out2:
            file_out2.writelines(filter(lambda line: b'current rotation' in line, file_in))

with open('source.txt', 'rb') as file_in:
    with open("current position.txt", "wb") as file_out3:
            file_out3.writelines(filter(lambda line: b'current position' in line, file_in))


with open('source.txt', 'rb') as file_in:
    with open("commanded position.txt", "wb") as file_out4:
            file_out4.writelines(filter(lambda line: b'commanded position' in line, file_in))
