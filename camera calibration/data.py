import pickle

# Open the pickle file for reading in binary mode ('rb')
# with open('/Users/starboy/Documents/workspace_python/calibration/calibration.pkl', 'rb') as file:
#     # Load the data from the pickle file
#     data = pickle.load(file)

# # Now you can work with the loaded data
# print("calibration matrix>>>>")
# print(data)

with open('/Users/starboy/Documents/workspace_python/calibration/cameraMatrix.pkl', 'rb') as file:
    # Load the data from the pickle file
    data = pickle.load(file)

# Now you can work with the loaded data
print()
print("<<<< camera matrix >>>>\n")
print(data)
print()

with open('/Users/starboy/Documents/workspace_python/calibration/dist.pkl', 'rb') as file:
    # Load the data from the pickle file
    data = pickle.load(file)

# Now you can work with the loaded data
print("<<<< distortion >>>>\n")
print(data)
print()

with open('/Users/starboy/Documents/workspace_python/calibration/rotation.pkl', 'rb') as file:
    # Load the data from the pickle file
    rot = pickle.load(file)

# Now you can work with the loaded data
print("<<<< rotation >>>>\n")
print(rot)
print()

with open('/Users/starboy/Documents/workspace_python/calibration/translation.pkl', 'rb') as file:
    # Load the data from the pickle file
    tra = pickle.load(file)

# Now you can work with the loaded data
print("<<<< translation >>>>\n")
print(tra)
print()