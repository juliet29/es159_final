# -*- coding: utf-8 -*-
"""extractPaths_orig.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1XtZhgr-Sh0hMM5h2YOLBYQv2fMds6q3s
"""

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import pandas as pd

# function for plottong the the contours organized by consecutive paths 
def plot_dif_contour(df, z_val):
  z_bool = df['z'] == z_val
  dz = df[z_bool].reset_index(drop=True)
  for i in range(dz.shape[0]):
    ax.plot(dz['x'][i],dz['y'][i], dz['z'][i] +1 )
    # figure out a way to always have z be in between 0 and 1

# set up the figure 
#fig_0 = plt.figure()
#ax = fig_0.gca(projection='3d')

# Make data.
# kuka iiwa workspace
# X = np.arange(-0.5, 0.1, 0.01)
# Y = np.arange(-0.5, 0.5, 0.01)
X = np.arange(-1.5, 1.5, 0.5)
Y = np.arange(-1.5, 1.5, 0.5)
X, Y = np.meshgrid(X, Y)
R = np.sqrt(X**2 + Y**2)
Z = -np.sqrt(R)
# Plot the surface.
#surf = ax.plot_surface(X, Y, Z)
#plt.show()

# Get the contours of the geometry 
C = plt.contour(X,Y,Z, levels=5)

# create raw list of all countour levels (z), and all xy points
raw_zs = C.levels
#print(C.levels)
raw_df= pd.DataFrame(C.allsegs)
# print(raw_df.head(5))

# drop dataframe rows that are nan in the first column, and their corresponding zs
mask = raw_df[0].isna()
zs = raw_zs[~mask]
df = raw_df.dropna(subset=[0])

# make sure the zs and the xys have the same shape 
assert(len(zs) == df.shape[0]), 'zs and df do not have same size!'
# print(len(zs))
# print(df.shape)

# make a nice data frame arranged by consecutive paths 
nice_df = pd.DataFrame(columns=["x", "y", "z"])
for row_n in range(np.shape(df)[0]): 
  df_row = list(df.iloc[row_n].dropna().to_numpy())
  row_z = zs[row_n]

  for i in range(len(df_row)):
    row_dict = {}
    row_dict['x'] = [df_row[i][:,0]]
    row_dict['y'] = [df_row[i][:,1]]
    row_dict['z'] = row_z
    row_df = pd.DataFrame(row_dict)
    nice_df = nice_df.append(row_df, ignore_index=True)

# make sure the nice_df looks nice 
#print(nice_df.head(7))


# plot contours according to consecutive paths 
# fig_1 = plt.figure()
# ax = fig_1.gca(projection='3d')
# for i in zs:
#   plot_dif_contour(nice_df, i)
# plt.show()

# want to bring all the zs up to the z = 0 level at least 
min_z = nice_df['z'][0]
if min_z < 0:
  nice_df['z'] = nice_df['z'] + abs(min_z)

# turn the nice_df into a list of points for printing 
print("appending paths")
path = []
# go through all z 
for lev in range(nice_df.shape[0]):
  # all the x,y at a certain z 
  curr_level = nice_df['x'][lev]
  for element in range(len(curr_level)):
    point = (nice_df['x'][lev][element], nice_df['y'][lev][element], nice_df['z'][lev]+1)
    path.append(point)
# save to a constant 
PATH = path

# turn into a nice_df into path_df for data handling
path_df = pd.DataFrame(columns=["x", "y", "z"])
path_rows = []
# go through all z 
for lev in range(nice_df.shape[0]):
  # all the x,y at a certain z 
  curr_level = nice_df['x'][lev]
  for element in range(len(curr_level)):
      nu_dict = {'x': nice_df['x'][lev][element], 'y': nice_df['y'][lev][element], 'z': nice_df['z'][lev]}
      path_rows.append(nu_dict)
path_df = pd.DataFrame(path_rows)


