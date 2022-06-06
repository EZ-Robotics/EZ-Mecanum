import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib

raw_path = [[0, 0] 
   ,[0.0691775, 0.0394886] 
   ,[0.138317, 0.0789207] 
   ,[0.20738, 0.118239] 
   ,[0.276326, 0.157384] 
   ,[0.345114, 0.196294] 
   ,[0.413698, 0.234903] 
   ,[0.482032, 0.273139] 
   ,[0.550063, 0.310925] 
   ,[0.617734, 0.348177] 
   ,[0.684982, 0.3848] 
   ,[0.751737, 0.420691] 
   ,[0.817919, 0.455733] 
   ,[0.883442, 0.489796] 
   ,[0.948207, 0.522732] 
   ,[1.0121, 0.554376] 
   ,[1.075, 0.584541] 
   ,[1.13676, 0.613015] 
   ,[1.19722, 0.639558] 
   ,[1.2568, 0.66426] 
   ,[1.31649, 0.687533] 
   ,[1.37593, 0.710037] 
   ,[1.43474, 0.73242] 
   ,[1.49255, 0.755326] 
   ,[1.54895, 0.779411] 
   ,[1.60352, 0.805348] 
   ,[1.65704, 0.833596] 
   ,[1.7104, 0.864626] 
   ,[1.76351, 0.897973] 
   ,[1.81626, 0.93321] 
   ,[1.86857, 0.969944] 
   ,[1.92031, 1.0078] 
   ,[1.97136, 1.04644] 
   ,[2.02161, 1.08551] 
   ,[2.07089, 1.12469] 
   ,[2.11997, 1.16455] 
   ,[2.16965, 1.20577] 
   ,[2.21979, 1.24805] 
   ,[2.27025, 1.29111] 
   ,[2.32087, 1.33468] 
   ,[2.37152, 1.37852] 
   ,[2.42206, 1.42237] 
   ,[2.47236, 1.46597] 
   ,[2.52226, 1.50907] 
   ,[2.57163, 1.55142] 
   ,[2.62029, 1.59272] 
   ,[2.66808, 1.6327] 
   ,[2.71481, 1.67103] 
   ,[2.76029, 1.70736] 
   ,[2.80427, 1.74133] 
   ,[2.84653, 1.77252] 
   ,[2.88677, 1.80046] 
   ,[2.92469, 1.82464] 
   ,[2.95994, 1.84448] 
   ,[2.99211, 1.85933] 
   ,[3.02078, 1.86845] 
   ,[3.04543, 1.87102] 
   ,[3.06549, 1.86609] 
   ,[3.08108, 1.85337] 
   ,[3.09247, 1.83265] 
   ,[3.10018, 1.80493] 
   ,[3.10468, 1.7711] 
   ,[3.10638, 1.73195] 
   ,[3.10564, 1.68816] 
   ,[3.1028, 1.64035] 
   ,[3.09813, 1.58908] 
   ,[3.0919, 1.53484] 
   ,[3.08434, 1.47806] 
   ,[3.07566, 1.41916] 
   ,[3.06606, 1.3585] 
   ,[3.0557, 1.29641] 
   ,[3.04476, 1.23321] 
   ,[3.0334, 1.16919] 
   ,[3.02174, 1.10464] 
   ,[3.00995, 1.03981] 
   ,[3, 1] 
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


path = path_visualizer(raw_path, (2, 2), (0, 0, 12, 12), 0.5, 20)
plt.show()