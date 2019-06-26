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

num_attr = 15 # number of features fed into the network

# load dataset
dataframe = pandas.read_csv("depth_data.csv", delim_whitespace=True, header=None)
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
# regressor = KerasRegressor(build_fn=baseline_model, batch_size=100, epochs=1)
# print ('model fit start')
# regressor.fit(X, Y)
# print ('model fit end')

# evaluate model with standardized dataset
seed = 7
# numpy.random.seed(seed)
# kfold = KFold(n_splits=10, random_state=seed)
# results = cross_val_score(regressor, X, Y, cv=kfold)
# print("Results: %.2f (%.2f) MSE" % (results.mean(), results.std()))

#-------------------------
# # evaluate model with standardized dataset
numpy.random.seed(seed)
estimators = []
estimators.append(('standardize', StandardScaler()))
estimators.append(('mlp', KerasRegressor(build_fn=baseline_model, epochs=1, batch_size=100, verbose=1)))
pipeline = Pipeline(estimators)
kfold = KFold(n_splits=10, random_state=seed)
results = cross_val_score(pipeline, X, Y, cv=kfold)
print("Standardized: %.2f (%.2f) MSE" % (results.mean(), results.std()))

#--------------------------------
# # serialize model to JSON
# depth_model_json = regressor.model.to_json()
# with open("model.json", "w+") as json_file:
#     json_file.write(depth_model_json)
# # serialize weights to HDF5
# regressor.model.save_weights("temp.h5")
# print("Saved model to disk")


# -----------------------------
folder_name = 'model'

std_ = pipeline.named_steps['standardize']
std_.fit(X, Y)
print 'fit std'

pickle.dump(pipeline.named_steps['standardize'], open(folder_name + '/' + 'standard_scalar.pkl', 'wb'))
print 'standard scaler dumped'

reg_ = pipeline.named_steps['mlp']
reg_.fit(X, Y)
print 'fit again'

model_json = pipeline.named_steps['mlp'].model.to_json()
with open(folder_name + '/' + 'mlp.json', "w+") as json_file:
    json_file.write(model_json)
pipeline.named_steps['mlp'].model.save_weights(folder_name + '/' + 'mlp.h5')
print 'mlp dumped'

standard_scaler = pickle.load(open(folder_name + '/' + 'standard_scalar.pkl', 'rb'))
print 'loaded standard scaler'
mlp = baseline_model()
p_model = KerasRegressor(build_fn=baseline_model, epochs=1, batch_size=100, verbose=1)
p_model.fit(X, Y)
print 'training again'

p_model.model.load_weights(folder_name + '/' + 'mlp.h5')
print 'loaded mlp'

predictor = Pipeline([
    ('standardize', standard_scaler),
    ('mlp', p_model)
])

print predictor.predict(X[:100])



#-----------------------------------

# print joblib.dump(pipeline, 'depth_model.pkl', compress=1)
# print("Saved model to disk")

# clf_load = joblib.load('depth_model.pkl')
# print 'model loaded from depth_model.pkl'
# print clf_load.score(X[:100])
