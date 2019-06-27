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
import matplotlib.pyplot as plt
from sklearn.externals import joblib
import pickle

num_attr = 4 # number of features fed into the network

# load dataset
dataframe = pandas.read_csv("../../data/marker_depth_test.csv", delim_whitespace=True, header=None)
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
    model.add(Dense(8, input_dim=num_attr, kernel_initializer='normal', activation='relu'))
    model.add(Dense(4, kernel_initializer='normal', activation='relu'))
    model.add(Dense(1, kernel_initializer='normal'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam')
    return model

test_type = "simple"

def load_simple (file_name):
    model = baseline_model()
    model.load_weights(file_name)
    return model

def load_complex (scaler_filename, regressor_filename):
    standard_scaler = pickle.load(open(scaler_filename, 'rb'))
    print ('loaded standard scaler')
    p_model = KerasRegressor(build_fn=baseline_model)
    p_model.fit(X, Y)
    print ('training again')

    p_model.model.load_weights(regressor_filename)
    print ('loaded mlp')

    predictor = Pipeline([
        ('standardize', standard_scaler),
        ('mlp', p_model)
    ])
    return predictor

def plot (Y, Y_pred):
    fig, ax = plt.subplots()
    ax.scatter(Y, Y_pred)
    ax.plot([Y.min(), Y.max()], [Y.min(), Y.max()], 'k--', lw=4)
    ax.set_xlabel('Measured')
    ax.set_ylabel('Predicted')
    plt.savefig('../../img/marker_depth_model.png')
    plt.show()

if (test_type == "simple"):
    model = load_simple('../../models/marker_depth_model.h5')
    Y_pred = model.predict(X)
    plot (Y, Y_pred)
else:
    model = load_complex('../../models/standard_scaler.pkl', '../../models/gate_model_2.h5')
    Y_pred = model.predict(X)

sum_ = 0
for i in range(len(Y)):
    sum_ += abs(Y[i] - Y_pred[i])
accuracy = sum_/len(Y)
print ('accuracy: {}'.format(accuracy))
