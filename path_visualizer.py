from turtle import left
import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib

raw_path = [
    [1, 1],
    [3, 1],
    [3, 7],
    [1, 7],
    [1, 13],
    [3, 13],
    [3, 7],
    [1, 7],
    [1,0]
]


# also smooth path and add more points

def add_line(path):
    for i in range(0, len(path)):
        plt.plot(path[i][0], path[i][1], '.', color='red', markersize=10)

    for i in range(0, len(path)-1):
        plt.plot([path[i][0], path[i+1][0]],
                 [path[i][1], path[i+1][1]], color='b')

    plt.axis('scaled')
    # plt.show()

def add_complicated_line(path, lineStyle, lineColor, lineLabel):
    for i in range(0, len(path)):
        plt.plot(path[i][0], path[i][1], '.', color='red', markersize=10)

    for i in range(0, len(path)-1):
        if(i == 0):
            # plt.plot([path[i][0],path[i+1][0]],[path[i][1],path[i+1][1]],color='b')
            plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1]
                     [1]], lineStyle, color=lineColor, label=lineLabel)
        else:
            plt.plot([path[i][0], path[i+1][0]], [path[i][1],
                     path[i+1][1]], lineStyle, color=lineColor)

    plt.axis('scaled')

def highlight_points(points, pointColor):
    for point in points:
        plt.plot(point[0], point[1], '.', color=pointColor, markersize=10)

def draw_circle(x, y, r, circleColor):
    xs = []
    ys = []
    angles = np.arange(0, 2.2*np.pi, 0.5)

    for angle in angles:
        xs.append(r*np.cos(angle) + x)
        ys.append(r*np.sin(angle) + y)

    plt.plot(xs, ys, '-', color=circleColor)


# covert to c++ 2d array format
def convert(path):
    length = 0
    print('pure_pursuit({')
    for i in range(0, len(path)):
        length += 1
        print('{{', path[i][0], ',', path[i][1], ', 0}, HOLD_ANGLE},')
    print('});')
    print('\n')

def path_visualizer(orig_path, fig_size, field_size, segment_length, maxAngle):

    path = orig_path

    field = plt.figure()
    xscale, yscale = fig_size
    path_ax = field.add_axes([0, 0, xscale, yscale])
    # add_complicated_line(orig_path,'--','grey','original')
    add_complicated_line(path, '--', 'orange', 'smoothed')

    xMin, yMin, xMax, yMax = field_size

    # plot field
    path_ax.plot([xMin, xMax], [yMin, yMin], color='black')
    path_ax.plot([xMin, xMin], [yMin, yMax], color='black')
    path_ax.plot([xMax, xMax], [yMin, yMax], color='black')
    path_ax.plot([xMax, xMin], [yMax, yMax], color='black')

    # set grid
    xTicks = np.arange(xMin, xMax+1, 2)
    yTicks = np.arange(yMin, yMax+1, 2)

    path_ax.set_xticks(xTicks)
    path_ax.set_yticks(yTicks)
    path_ax.grid(True)

    path_ax.set_xlim(xMin-0.25, xMax+0.25)
    path_ax.set_ylim(yMin-0.25, yMax+0.25)

    # plot start and end
    path_ax.plot(path[0][0], path[0][1], '.', color='blue',
                 markersize=15, label='start')
    path_ax.plot(path[-1][0], path[-1][1], '.',
                 color='green', markersize=15, label='end')
    path_ax.legend()

    return path


path = path_visualizer(raw_path, (2, 2), (0, 0, 4, 13), 0.5, 20)
plt.show()