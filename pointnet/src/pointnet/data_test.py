import h5py
import os
import sys

from numpy.core.fromnumeric import size
import provider
import numpy as np
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TRAIN_FILES = provider.getDataFiles(  \
    os.path.join(BASE_DIR, 'data/modelnet40_ply_hdf5_2048/train_files.txt')
    )
train_file_idxs =  np.arange(0 , len(TRAIN_FILES))
np.random.shuffle(train_file_idxs)   
current_data , current_label = provider.loadDataFile(TRAIN_FILES[train_file_idxs[0]])
current_data = current_data[: ,0:1024, :]
current_data , current_label, _ = provider.shuffle_data(current_data , np.squeeze(current_label))
print(current_data)