import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.colorbar as cbar
from matplotlib.colors import Normalize
from matplotlib import cm

from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator
from matplotlib.colors import ListedColormap


def plot_mean(ogm_map, figure_title, filename):
    print('Plotting map mean')

    # plot ogm using conventional black-gray-white cells
    fig, ax = plt.subplots()
    h = ogm_map.grid_size / 2

    for i in range(ogm_map.map['size']):
        m = ogm_map.map['occMap'].data[i, :]
        x = m[0] - h
        y = m[1] - h
        color = (1 - ogm_map.map['mean'][i]) * np.array([1, 1, 1])  # map cell colors
        rect = Rectangle((x, y), ogm_map.grid_size, ogm_map.grid_size, facecolor=color, edgecolor='none')  # map cells
        ax.add_patch(rect)

    ax.autoscale_view()
    ax.set_xlim(ogm_map.range_x)
    ax.set_ylim(ogm_map.range_y)
    ax.set_title(figure_title)
    plt.axis('equal')
    fig.savefig(filename)
    plt.show()
    

def plot_argmax_binary(ogm_map, figure_title, filename):
    print('Plotting map argmax')

    # plot ogm using conventional black-gray-white cells
    fig, ax = plt.subplots()
    h = ogm_map.grid_size / 2

    ogm_argmax = np.zeros_like(ogm_map.map['alpha'])
    ogm_argmax[ogm_map.map['beta'] < ogm_map.map['alpha']] = 1

    side_length = int(np.sqrt(ogm_map.map['size']))
    ogm_argmax = ogm_argmax.reshape((side_length, side_length))
    
    # Plot square grid
    ax.imshow(ogm_argmax, cmap='binary')

    ax.autoscale_view()
    ax.set_xlim(ogm_map.range_x)
    ax.set_ylim(ogm_map.range_y)
    ax.set_title(figure_title)
    plt.axis('equal')
    fig.savefig(filename)
    plt.show()


def plot_variance(ogm_map, figure_title, filename):
    print('Plotting map variance')

    # plot ogm variance
    v_max = np.max(ogm_map.map['variance'])
    x = np.arange(ogm_map.range_x[0], ogm_map.range_x[1] + ogm_map.grid_size, ogm_map.grid_size)
    y = np.arange(ogm_map.range_y[0], ogm_map.range_y[1] + ogm_map.grid_size, ogm_map.grid_size)
    Z = ogm_map.map['variance'].reshape((len(x), len(y)))

    fig, ax = plt.subplots()
    im = ax.pcolormesh(x, y, Z, cmap="jet", vmin=0, vmax=v_max)
    fig.colorbar(im, ax=ax)

    ax.autoscale_view()
    ax.set_xlim(ogm_map.range_x)
    ax.set_ylim(ogm_map.range_y)
    ax.set_title(figure_title)
    plt.axis('equal')
    fig.savefig(filename)
    plt.show()


def plot_semantic(ogm_map, figure_title, filename):
    print('Plotting map semantic')
    
    # color
    color_semantic = [[0, 0.4470, 0.7410], [0.8500, 0.3250, 0.0980], 
                      [0.9290, 0.6940, 0.1250], [0.4940, 0.1840, 0.5560], 
                      [0.4660, 0.6740, 0.1880], [0.3010, 0.7450, 0.9330], 
                      [0.6350, 0.0780, 0.1840]]

    # plot ogm with the semantic colors
    fig, ax = plt.subplots()
    h = ogm_map.grid_size / 2

    unknown = ogm_map.map['mean'][0]

    cell_class = []
    for i in range(ogm_map.map['size']):
        m = ogm_map.map['occMap'].data[i, :]
        x = m[0] - h
        y = m[1] - h
        if np.array_equal(ogm_map.map['mean'][i], unknown):
            color = 0.5 * np.array([1, 1, 1]) # unknown
            cell_class.append(0)
        else:
            semantic_class = int(np.argmax(ogm_map.map['mean'][i]))
            cell_class.append(semantic_class)
            if semantic_class == ogm_map.num_classes:
                color = np.array([1, 1, 1]) # free space
            else:
                color = color_semantic[semantic_class]  # map cell colors
        rect = Rectangle((x, y), ogm_map.grid_size, ogm_map.grid_size, facecolor=color, edgecolor='none')
        ax.add_patch(rect)
    
    ax.autoscale_view()
    ax.set_xlim(ogm_map.range_x)
    ax.set_ylim(ogm_map.range_y)
    ax.set_title(figure_title)
    plt.axis('equal')
    fig.savefig(filename)
    plt.show()