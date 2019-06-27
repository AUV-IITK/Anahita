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
from sklearn.externals import joblib
import pickle
import matplotlib.pyplot as plt

num_attr = 4 # number of features fed into the network

# load dataset
dataframe = pandas.read_csv("../../data/marker_depth_train.csv", delim_whitespace=True, header=None)
dataset = dataframe.values
print ('data loaded')

# split into input (X) and output (Y) variables
X = dataset[:, 0:num_attr]
Y = dataset[:,num_attr]

training_type = "simple"

# define base model
def baseline_model():
    # create model
    print ('model called')
    model = Sequential()
    model.add(Dense(8, input_dim=num_attr, kernel_initializer='normal', activation='relu'))
    model.add(Dense(4, kernel_initializer='normal', activation='relu'))
    model.add(Dense(1, kernel_initializer='normal'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam')
    return model

seed = 7

if training_type == "simple":
    # fit the model
    regressor = KerasRegressor(build_fn=baseline_model, batch_size=5, epochs=100)
    print ('model fit start')
    regressor.fit(X, Y)
    print ('model fit end')

    # evaluate model with standardized dataset
    numpy.random.seed(seed)
    kfold = KFold(n_splits=10, random_state=seed)
    results = cross_val_score(regressor, X, Y, cv=kfold)
    print("Results: %.2f (%.2f) MSE" % (results.mean(), results.std()))

    # serialize model to JSON
    depth_model_json = regressor.model.to_json()
    with open("../../models/marker_depth_model.json", "w+") as json_file:
        json_file.write(depth_model_json)
    # serialize weights to HDF5
    regressor.model.save_weights("../../models/marker_depth_model.h5")
    print("Saved model to disk")

if training_type == "standard":

    print ('training the standard way')
    # evaluate model with standardized dataset
    numpy.random.seed(seed)
    estimators = []
    estimators.append(('standardize', StandardScaler()))
    estimators.append(('mlp', KerasRegressor(build_fn=baseline_model, epochs=100, batch_size=5, verbose=1)))
    pipeline = Pipeline(estimators)

    std_ = pipeline.named_steps['standardize']
    std_.fit(X, Y)
    print ('fit std again')

    reg_ = pipeline.named_steps['mlp']
    reg_.fit(X, Y)
    print ('fit regressor again')

    kfold = KFold(n_splits=10, random_state=seed)
    results = cross_val_score(pipeline, X, Y, cv=kfold)
    print("Standardized: %.2f (%.2f) MSE" % (results.mean(), results.std()))

    folder_name = 'model'

    pickle.dump(pipeline.named_steps['standardize'], open(folder_name + '/' + 'standard_scaler.pkl', 'wb'))
    print ('standard scaler dumped')

    model_json = pipeline.named_steps['mlp'].model.to_json()
    with open(folder_name + '/' + 'mlp.json', "w+") as json_file:
        json_file.write(model_json)
    pipeline.named_steps['mlp'].model.save_weights(folder_name + '/' + 'mlp.h5')
    print ('mlp dumped')
