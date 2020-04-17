"""
MNIST database training by ensembling 15 CNN and expanding the MNIST dataset in order to obtain better results
"""
import numpy as np
import pandas as pd
import seaborn as sns
from sklearn.model_selection import train_test_split
import itertools
from keras.utils.np_utils import to_categorical # convert to one-hot-encoding
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPool2D
from keras.optimizers import RMSprop #adaptive learning rate method
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import ReduceLROnPlateau
from keras.callbacks import LearningRateScheduler

#Load data
train = pd.read_csv("train.csv")
test = pd.read_csv("test.csv")
Y_train = train["label"]
#Drop label column from train 
X_train = train.drop(labels = ["label"],axis=1)
#free space
del train
#Check data to see how many samples for each digit we have
g = sns.countplot(Y_train)
Y_train.value_counts()
#Check if there are null values
X_train.isnull().any().describe()
test.isnull().any().describe()
#Normalize the data
X_train = X_train / 255.0
test = test / 255.0
#Reshape image in three dimension (height = 28px width = 28px channel = 1)
X_train = X_train.values.reshape(-1,28,28,1) #channel = 1 grayscale image
test = test.values.reshape(-1,28,28,1)
# Encode labels to one hot vectors
Y_train = to_categorical(Y_train, num_classes = 10)
random_seed = 2
#Split into train and validation for the fitting
X_train, X_val, Y_train, Y_val = train_test_split(X_train, Y_train, test_size = 0.1, random_state = random_seed)
#Model architecture -> [[Conv2D->relu]*2 -> MaxPool2D -> Dropout]*2 -> Flatten -> Dense -> Dropout -> Out
nets = 15 #stacking 15 models to obtain better results
model = [0] * nets
history = [0] * nets
epochs = 45
for i in range(nets):
    model[i] = Sequential()
    model[i].add(Conv2D(filters = 32, kernel_size = (5,5), padding = 'Same', activation = "relu", input_shape = (28,28,1)))
    model[i].add(Conv2D(filters = 32, kernel_size = (5,5), padding = 'Same', activation = "relu", input_shape = (28,28,1)))
    model[i].add(MaxPool2D(pool_size = (2,2)))
    model[i].add(Dropout(0.25))

    model[i].add(Conv2D(filters = 64, kernel_size = (3,3), padding = 'Same', activation = "relu", input_shape = (28,28,1)))
    model[i].add(Conv2D(filters = 64, kernel_size = (3,3), padding = 'Same', activation = "relu", input_shape = (28,28,1)))
    model[i].add(MaxPool2D(pool_size = (2,2), strides=(2,2)))
    model[i].add(Dropout(0.25))

    model[i].add(Flatten())
    model[i].add(Dense(256, activation="relu"))
    model[i].add(Dropout(0.5))
    model[i].add(Dense(10, activation="softmax"))
    model[i].compile(optimizer="adam", loss="categorical_crossentropy", metrics=["accuracy"])
#decrease learning rate after each epoch
annealer = LearningRateScheduler(lambda x: 1e-3 * 0.95 ** x)
#Data augmentation because we don't have lots of samples 
datagen = ImageDataGenerator(
featurewise_center = False, samplewise_center = False,
featurewise_std_normalization = False, samplewise_std_normalization = False,
zca_whitening = False, rotation_range = 20, zoom_range = 0.3,
width_shift_range = 0.3, height_shift_range = 0.3, 
horizontal_flip = False, vertical_flip = False)
datagen.fit(X_train)
#Training
for j in range(nets):
    X_train2, X_val2, Y_train2, Y_val2 = train_test_split(X_train, Y_train, test_size = 0.1)
    history[j] = model[j].fit_generator(datagen.flow(X_train2,Y_train2, batch_size=64),
        epochs = epochs, steps_per_epoch = X_train2.shape[0]//64,  
        validation_data = (X_val2,Y_val2), callbacks=[annealer], verbose=0)
    print("CNN {0:d}: Epochs={1:d}, Train accuracy={2:.5f}, Validation accuracy={3:.5f}".format(
        j+1,epochs,max(history[j].history['accuracy']),max(history[j].history['val_accuracy']) ))
#Save results 
results = np.zeros( (test.shape[0],10) ) 
for j in range(nets):
    results = results + model[j].predict(test)
results = np.argmax(results,axis = 1)
results = pd.Series(results,name="Label")
save_res = pd.concat([pd.Series(range(1,28001),name = "ImageId"),results],axis = 1)
save_res.to_csv("MNIST-results.csv",index=False)
