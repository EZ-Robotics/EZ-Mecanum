#this doesnt work rn

import math

# reference value 0.1, 0.3, 0.00001
def smoothing(path, weight_data, weight_smooth, tolerance):

    smoothed_path = path.copy()
    change = tolerance

    while change >= tolerance:
        change = 0.0

        for i in range(1, len(path)-1):

            for j in range(0, len(path[i])):
                aux = smoothed_path[i][j]

                smoothed_path[i][j] += weight_data * (path[i][j] - smoothed_path[i][j]) + weight_smooth * (
                    smoothed_path[i-1][j] + smoothed_path[i+1][j] - (2.0 * smoothed_path[i][j]))
                change += abs(aux - smoothed_path[i][j])

    return smoothed_path


def sgn(num):
    if num >= 0:
        return 1
    else:
        return -1


def findMinAngle(absTargetAngle, currentHeading):

    minAngle = absTargetAngle - currentHeading

    if minAngle > 180 or minAngle < -180:
        minAngle = -1 * sgn(minAngle) * (360 - abs(minAngle))

    return minAngle


def autoSmooth(path, maxAngle, b):
    currentMax = 0
    param = b
    new_path = path
    firstLoop = True

    counter = 0

    while (currentMax >= maxAngle or firstLoop == True):  # and counter <= 15 :wy

        param += 0.01
        firstLoop = False

        # counter += 1
        # print('this is the {} iteration'.format(counter))

        new_path = smoothing(path, 0.1, param, 0.1)
        currentMax = 0

        for i in range(1, len(new_path)-2):
            angle1 = math.atan2(
                new_path[i][1] - new_path[i-1][1], new_path[i][0] - new_path[i-1][0]) * 180/math.pi
            if angle1 < 0:
                angle1 += 360
            angle2 = math.atan2(
                new_path[i+1][1] - new_path[i][1], new_path[i+1][0] - new_path[i][0]) * 180/math.pi
            if angle2 < 0:
                angle2 += 360

            if abs(findMinAngle(angle2, angle1)) > currentMax:
                currentMax = abs(findMinAngle(angle2, angle1))

    return new_path


raw_path = [[0, 0, 50, "FAST_MOVE_FWD", 110, 75] 
   ,[-0.242536, 0.970143, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-0.485071, 1.94029, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-0.727607, 2.91043, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-0.970143, 3.88057, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-1.21268, 4.85071, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-1.45521, 5.82086, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-1.69775, 6.791, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-1.94029, 7.76114, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-2.18282, 8.73128, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-2.42536, 9.70143, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-2.66789, 10.6716, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-2.91043, 11.6417, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-3.15296, 12.6119, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-3.3955, 13.582, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-3.63803, 14.5521, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-3.88057, 15.5223, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-4.12311, 16.4924, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-4.36564, 17.4626, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-4.60818, 18.4327, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-4.85071, 19.4029, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-5.09325, 20.373, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-5.33578, 21.3431, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-5.57832, 22.3133, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-5.82086, 23.2834, 50, "LOOK_AT_TARGET_FWD", 110, 75] 
   ,[-6, 24, 50, "FAST_MOVE_FWD", 110, 75] 
   ,[-5, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[-4, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[-3, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[-2, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[-1, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[0, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[1, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[2, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[3, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[4, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[5, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[6, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[7, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[8, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[9, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[10, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[11, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[12, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[13, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[14, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[15, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[16, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[17, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[18, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[19, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[20, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[21, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[22, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[23, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[24, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[25, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[26, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[27, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[28, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[29, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[30, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[31, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[32, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[33, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[34, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[35, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[36, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[36, 24, 50, "HOLD_ANGLE", 60, 75] 
   ,[36, 23, 50, "HOLD_ANGLE", 110, 75] 
   ,[36, 22, 50, "HOLD_ANGLE", 110, 75] 
   ,[36, 21, 50, "HOLD_ANGLE", 110, 75] 
   ,[36, 20, 50, "HOLD_ANGLE", 110, 75] 
   ,[36, 19, 50, "HOLD_ANGLE", 110, 75] 
   ,[36, 18, 50, "HOLD_ANGLE", 110, 75] 
   ,[36, 18, 50, "HOLD_ANGLE", 110, 75]  
   ,[35.1056, 17.5528, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[34.2111, 17.1056, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[33.3167, 16.6584, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[32.4223, 16.2111, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[31.5279, 15.7639, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[30.6334, 15.3167, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[29.739, 14.8695, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[28.8446, 14.4223, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[27.9502, 13.9751, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[27.0557, 13.5279, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[26.1613, 13.0807, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[25.2669, 12.6334, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[24.3724, 12.1862, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[23.478, 11.739, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[22.5836, 11.2918, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[21.6892, 10.8446, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[20.7947, 10.3974, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[19.9003, 9.95016, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[19.0059, 9.50294, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[18.1115, 9.05573, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[17.217, 8.60851, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[16.3226, 8.1613, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[15.4282, 7.71409, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[14.5337, 7.26687, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[13.6393, 6.81966, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[12.7449, 6.37245, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[11.8505, 5.92523, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[10.956, 5.47802, 0, "LOOK_AT_TARGET_REV", 110, 75] 
   ,[10.0616, 5.03081, 0, "FAST_MOVE_REV", 110, 75] 
   ,[9.16718, 4.58359, 0, "FAST_MOVE_REV", 110, 75] 
   ,[8.27276, 4.13638, 0, "FAST_MOVE_REV", 110, 75] 
   ,[7.37833, 3.68916, 0, "FAST_MOVE_REV", 110, 75] 
   ,[6.4839, 3.24195, 0, "FAST_MOVE_REV", 110, 75] 
   ,[5.58948, 2.79474, 0, "FAST_MOVE_REV", 110, 75] 
   ,[4.69505, 2.34752, 0, "FAST_MOVE_REV", 110, 75] 
   ,[3.80062, 1.90031, 0, "FAST_MOVE_REV", 110, 75] 
   ,[2.90619, 1.4531, 0, "FAST_MOVE_REV", 110, 75] 
   ,[2.01177, 1.00588, 0, "FAST_MOVE_REV", 110, 75] 
   ,[1.11734, 0.55867, 0, "FAST_MOVE_REV", 110, 75] 
   ,[0.222912, 0.111456, 0, "FAST_MOVE_REV", 110, 75] 
   ,[0, 0, 0, "FAST_MOVE_REV", 110, 75] 
]




# covert to c++ 2d array format
def convert (path):
    length = 0
    print('pure_pursuit({')
    for i in range (0, len(path)):
        length += 1
        print('{{',path[i][0],',', path[i][1],', 0}, HOLD_ANGLE},')
    print('});')
    print('\n')


convert(autoSmooth(raw_path, 5, 0.0175))