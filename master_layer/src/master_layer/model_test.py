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

num_attr = 12 # number of features fed into the network

# load dataset
dataframe = pandas.read_csv("../../data/torpedo_yaw_test.csv", delim_whitespace=True, header=None)
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
    model.add(Dense(13, input_dim=num_attr, kernel_initializer='normal', activation='relu'))
    model.add(Dense(6, kernel_initializer='normal', activation='relu'))
    model.add(Dense(1, kernel_initializer='normal'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam')
    return model

test_type = "simple"

MODEL_DIR = '/home/ironman/anahita_ws/src/Anahita/master_layer/models'

def load_simple (model_name):
    json_filename = os.path.join(MODEL_DIR, model_name + '.json')
    json_file = open(json_filename, 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    loaded_model = model_from_json(loaded_model_json)
    weights_filename = os.path.join(MODEL_DIR, model_name + '.h5')
    loaded_model.load_weights(weights_filename)
    return loaded_model

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

def mean_error (Y, Y_pred):
    sum_ = 0
    for i in range(len(Y)):
        sum_ += abs(Y[i] - Y_pred[i])
    accuracy = sum_/len(Y)
    print ('mean error: {}'.format(accuracy))

def plot (Y, Y_pred):
    _, ax = plt.subplots()
    ax.scatter(Y, Y_pred)
    ax.plot([Y.min(), Y.max()], [Y.min(), Y.max()], 'k--', lw=4)
    ax.set_xlabel('Measured')
    ax.set_ylabel('Predicted')
    plt.savefig('../../img/torpedo_yaw_model.png')
    plt.show()

if (test_type == "simple"):
    model = load_simple('torpedo_yaw_model')
    Y_pred = model.predict(X)
    mean_error(Y, Y_pred)
    plot (Y, Y_pred)
else:
    model = load_complex('../../models/standard_scaler.pkl', '../../models/gate_model_2.h5')
    Y_pred = model.predict(X)
