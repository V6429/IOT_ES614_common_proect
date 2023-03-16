l = [172, 0, 138, 5, 0, 0, 145, 210, 0, 228, 149, 3]

h = []

analog_value = 0

analog_adc = 0

future = []



#haha = 0

for i in l:

    h.append(i)

print(h)

ID = h[0]

digital = h[1]

for i in range(5,1,-1):

    analog_value = h[i] |  (analog_value << 8)

    print(i)

for i in range(7,5,-1):

    analog_adc = h[i] | (analog_adc << 8)

    i

for i in range(11,7,-1):

    future.append(h[i])

    i

print(ID)

print(digital)

print(analog_value)

print(analog_adc)

print(future)