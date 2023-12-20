import numpy as np
import matplotlib.pyplot as plt
epoch = [0,1,2,3,4]
train_acc = [0.827762,0.933281, 0.959989,0.966986,0.977611,0.977477,0.981693,0.982480,0.987074]
eval_acc = [0.951068,0.981033,0.980495, 0.990737,0.991242,0.993258,0.992167,0.991002,0.995468]
plt.plot(epoch, train_acc, color = 'red', label = "train accuracy", linewidth = 5)
plt.plot(epoch, eval_acc, color = 'blue' ,label = "evaluation accuracy" , linewidth = 5)
plt.xlabel("Epochs")
plt.ylabel("Accuracy")
plt.legend(loc = 'lower right')
plt.title("Accuracy plot for Pointnet")
plt.show()
