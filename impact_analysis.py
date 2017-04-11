import pickle, math
import pylab as pl

# load data
class Object(object):
    pass
input = open('attention2.pkl', 'rb')
attention = pickle.load(input)
input.close()




# calculate angle triggers
values_angle = []
values_angle_trg = []
for i in range(2,len(attention.velocity)): # change in angle_degree
    value_change = abs(attention.velocity[i][2] - attention.velocity[i - 1][2])
    if value_change > 180:
        value_change = 360-value_change
    values_angle.append(value_change)
    if value_change > 30:
        values_angle_trg.append(i)
print(values_angle_trg)

# draw figure impuls
if True:
    fig = pl.figure()
    ax = fig.add_subplot(111)
    ax.bar(range(0,len(values_angle)),values_angle)

# calculate velocity triggers
values_vel_len = []
values_vel_len_trg = []
for i in range(2,len(attention.velocity)): # change in velocity length_vector
    value_change = abs(attention.velocity[i][3] - attention.velocity[i - 1][3])
    values_vel_len.append(value_change)
    if value_change > 30:
        values_vel_len_trg.append(i)
print(values_vel_len_trg)

# draw figure velocity
if True:
    fig = pl.figure()
    ax = fig.add_subplot(111)
    ax.bar(range(0,len(values_vel_len)),values_vel_len)


# find intersection of angle and velocity change steps
trg = [val for val in values_angle_trg if val in values_vel_len_trg]
print(trg)








# set the A-B map
attention.stackA = []
attention.stackB = []
for i in trg:
    print(i)
    vel_tmpA = [round(attention.velocity[i-1][0]), round(attention.velocity[i-1][1]), round(attention.velocity[i-1][2])]
    vel_tmpB = [round(attention.velocity[i][0]), round(attention.velocity[i][1]), round(attention.velocity[i][2])]
    if abs(vel_tmpA[0]) + abs(vel_tmpA[0]) < 10:
        vel_tmpA = [0, 0]
        print('abs(vel_tmpA[0]) + abs(vel_tmpA[0]) < 10:')
    if abs(vel_tmpB[0]) + abs(vel_tmpB[0]) < 10:
        vel_tmpB = [0, 0]
        print('abs(vel_tmpB[0]) + abs(vel_tmpB[0]) < 10:')

    imp_tmpA = []
    imp_tmpB = []
    for imp in attention.impulces[i - 1]:
        imp_tmpA.append([round(imp[0], 2), round(imp[1], 2)])
    for imp in attention.impulces[i]:
        imp_tmpB.append([round(imp[0], 2), round(imp[1], 2)])

    attention.stackA.append([vel_tmpA, imp_tmpA])
    attention.stackB.append([vel_tmpB, imp_tmpB])

for ii in range(0,len(attention.stackA)):
    print(attention.stackA[ii])
    print(attention.stackB[ii])
    print(' ')




if True:
    output = open('attention_db.pkl', 'wb')
    pickle.dump([attention.stackA, attention.stackB], output, pickle.HIGHEST_PROTOCOL)
    output.close()
    print('pickle.dump()')
