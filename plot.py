
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import csv

def parse_data(filename):
    x = []
    y = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for line in reader:
            x.append(float(line[0]))
            y.append(float(line[1]))
    return x, y

def path_length(x,y):
    total_length = 0
    for i in range(1, len(x)):
        dx = x[i] - x[i - 1]
        dy = y[i] - y[i - 1]
        segment_length = math.sqrt(dx**2 + dy**2)
        total_length += segment_length
    return total_length

def update(frame, x, y, p):
    x = x[frame]
    y = y[frame]
    p.set_offsets([x, y])
    return p


data_filenames = [
    'simplegeo_data.txt',
    'simplevel_data.txt',
    'simpleacc_data.txt',
    'forestgeo_data.txt',
    'forestvel_data.txt',
    'forestacc_data.txt',
]

path_filenames = [
    'simplegeo_path.txt',
    'simplevel_path.txt',
    'simpleacc_path.txt',
    'forestgeo_path.txt',
    'forestvel_path.txt',
    'forestacc_path.txt'
]

obstacles = [
    [2.0, 2.0],
    [5.0, 2.0],
    [8.0, 2.0],
    [0.5, 5.0],
    [3.5, 5.0],
    [6.5, 5.0],
    [9.5, 5.0],
    [2.0, 8.0],
    [5.0, 8.0],
    [8.0, 8.0],
]

for i in range(len(data_filenames)):
    data_filename = data_filenames[i]
    path_filename = path_filenames[i]

    data_x, data_y = parse_data(f"results/{data_filename}")
    path_x, path_y = parse_data(f"results/{path_filename}")

    fig = plt.figure()
    ax = plt.gca()

    if 'forest' in data_filename:
        circles = [ax.add_patch(plt.Circle((x, y), 1, color='k', fill=True)) for x, y in obstacles]

    ax.add_patch(plt.Circle((9.5, 9.5), 0.5, color='g', fill=True))
    plt.scatter(data_x, data_y, marker='.')
    plt.scatter(path_x, path_y, marker='.')
    p = plt.scatter(0, 0, c = 'r')
            
    plt.savefig(f"images/{data_filename.split('_')[0]}.png")
    ani = animation.FuncAnimation(fig=fig, func=lambda f: update(f, path_x, path_y, p), frames=len(path_x), interval=100)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    ani.save(filename=f"videos/{data_filename.split('_')[0]}.mp4", writer="ffmpeg")
    plt.close(fig)