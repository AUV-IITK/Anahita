import numpy
import pandas
from keras.models import Sequential
from keras.layers import Dense
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
import os
from keras.models import model_from_json
from keras.models import load_model
import h5py

num_attr = 15 # number of features fed into the network

# load dataset
dataframe = pandas.read_csv("depth_test.csv", delim_whitespace=True, header=None)
dataset = dataframe.values
print ('data loaded')

# split into input (X) and output (Y) variables
X = dataset[:, 0:num_attr]
Y = dataset[:,num_attr]

# define base model
def baseline_model():
    # create model
    print ('model called')
    model = Sequential()
    model.add(Dense(num_attr, input_dim=num_attr, kernel_initializer='normal', activation='relu'))
    model.add(Dense(int(num_attr/2), kernel_initializer='normal', activation='relu'))
    model.add(Dense(1, kernel_initializer='normal'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam')
    return model

# fit the model
regressor = KerasRegressor(build_fn=baseline_model, batch_size=20, epochs=1)
print ('model fit start')
regressor.fit(X, Y)
print ('model fit end')

# evaluate model with standardized dataset
seed = 7
numpy.random.seed(seed)
kfold = KFold(n_splits=10, random_state=seed)
results = cross_val_score(regressor, X, Y, cv=kfold)
print("Results: %.2f (%.2f) MSE" % (results.mean(), results.std()))

# serialize model to JSON
depth_model_json = regressor.model.to_json()
with open("model.json", "w+") as json_file:
    json_file.write(depth_model_json)
# # serialize weights to HDF5
regressor.model.save_weights("depth_model.h5")
print("Saved model to disk")

# Instantiate the model as you please (we are not going to use this)
model2 = KerasRegressor(build_fn=baseline_model, batch_size=5, epochs=100, verbose=1)

# This is where you load the actual saved model into new variable.
model2.model = load_model("/home/ironman/anahita_ws/src/Anahita/master_layer/src/master_layer/depth_model.h5")

# Now you can use this to predict on new data (without fitting model2, because it uses the older saved model)
model2.predict(X)