import serial
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import plotly.express as px


def getCalibParameters(df, R):
    x,y,z = df.iloc[:,0], df.iloc[:,1], df.iloc[:,2]
    D = pd.DataFrame({'x^2':x**2, 'y^2':y**2, 'z^2':z**2, '2yz':2*y*z, '2xz':2*x*z, '2xy':2*x*y, '2x':2*x, '2y':2*y, '2z':2*z, '1':np.ones(len(x))})
    S = D.T.dot(D)
    S11, S12, S21, S22 = S.iloc[0:6, 0:6], S.iloc[0:6, 6:], S.iloc[6:, 0:6], S.iloc[6:, 6:]
    S22_inv = np.linalg.inv(S22)
    S22a = np.dot(S22_inv, S21)
    S22b = np.dot(S12, S22a)
    SS = S11 - S22b
    C = np.array([[-1,1,1,0,0,0],[1,-1,1,0,0,0],[1,1,-1,0,0,0],[0,0,0,-4,0,0],[0,0,0,0,-4,0],[0,0,0,0,0,-4]])
    E = np.linalg.inv(C).dot(SS)
    eigen_values, eigen_vectors = np.linalg.eig(E) # these are normalized eigen vectors
    l = [(idx,eigVal) for idx,eigVal in enumerate(eigen_values) if eigVal > 0]
    if len(l) >= 1: 
        # chose eigen vector with heighest eigen value out of the positive eigen values
        idx = max(l, key=lambda x:x[1])[0]
        v1 = eigen_vectors[:,idx]
    if v1[0] < 0.0: v1 = -v1
    v2 = np.dot(S22a, v1)
    V = np.array([v1[0], v1[1], v1[2], v1[3], v1[4], v1[5], -v2[0], -v2[1], -v2[2], -v2[3]])
    A,B,C,D,E,F,G,H,I,J = V[0], V[1], V[2], V[5], V[4], V[3], V[6], V[7], V[8], V[9]
    Q = np.array([A, D, E, D, B, F, E, F, C]).reshape(3, 3)
    U = np.array([G, H, I])
    bias = -np.linalg.inv(Q).dot(U)
    lmb, V = np.linalg.eig(Q) # these are normalized eigen vectors
    lmb1, lmb2, lmb3 = lmb
    V_inv = np.linalg.inv(V)
    const = R/np.sqrt(((bias.T.dot(Q).dot(bias))-J))
    sqrtEigDiag = np.array([[np.sqrt(lmb1), 0, 0], [0, np.sqrt(lmb2), 0], [0, 0, np.sqrt(lmb3)]])
    matMult = const*(np.dot(V, np.dot(sqrtEigDiag, V_inv)))
    return matMult, bias
def dfScatter3D(df_raw, df_calib):
    # Create a 3D scatter plot
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(df_raw['x'], df_raw['y'], df_raw['z'], c='b', marker='o')
    ax.scatter(df_calib['x'], df_calib['y'], df_calib['z'], c='r', marker='o')
    ax.set_title('3D Scatter Plot of Raw and Calibrated Data')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.legend(['raw', 'calibrated'])
    # ax.set_aspect('auto')
    # Create projections on XY, XZ, and YZ planes
    fig, axes = plt.subplots(1, 3, figsize=(18,3))
    # XY plane projection
    axes[0].scatter(df_raw['x'], df_raw['y'], c='b', marker='o')
    axes[0].scatter(df_calib['x'], df_calib['y'], c='r', marker='o', alpha=0.1)
    axes[0].set_title('XY Plane Projection')
    axes[0].set_xlabel('X-axis')
    axes[0].set_ylabel('Y-axis')
    axes[0].legend(['raw', 'calibrated'])
    axes[0].set_aspect('equal')
    # XZ plane projection
    axes[1].scatter(df_raw['x'], df_raw['z'], c='b', marker='o')
    axes[1].scatter(df_calib['x'], df_calib['z'], c='r', marker='o', alpha=0.1)
    axes[1].set_title('XZ Plane Projection')
    axes[1].set_xlabel('X-axis')
    axes[1].set_ylabel('Z-axis')
    axes[1].legend(['raw', 'calibrated'])
    axes[1].set_aspect('equal')
    # YZ plane projection
    axes[2].scatter(df_raw['y'], df_raw['z'], c='b', marker='o')
    axes[2].scatter(df_calib['y'], df_calib['z'], c='r', marker='o', alpha=0.1)
    axes[2].set_title('YZ Plane Projection')
    axes[2].set_xlabel('Y-axis')
    axes[2].set_ylabel('Z-axis')
    axes[2].legend(['raw', 'calibrated'])
    axes[2].set_aspect('equal')
    plt.show()
def df_plotly3D(rawdf, calibdf):
    fig = px.scatter_3d(rawdf, x='x', y='y', z='z', opacity=0.5)
    fig.add_scatter3d(x=calibdf['x'], y=calibdf['y'], z=calibdf['z'], mode='markers')
    fig.update_traces(marker=dict(size=1))
    fig.show()
def readData_to_csv(numRows):
    df = pd.DataFrame(columns=['x', 'y', 'z', 'resultant'])
    ser = serial.Serial("COM9", 115200, timeout=1)
    i = 0
    while i<numRows:
        response = ser.readline().decode().strip('\r\n').split(' , ')
        response = np.array(response)
        df = df.append({'x': response[0], 'y': response[1], 'z': response[2], 'resultant': response[3]}, ignore_index=True)
        i += 1
    ser.close()
    df.to_csv('data.csv', index=False, header=None)


readData_to_csv(10000)

rawdf = pd.read_csv(r"C:\Users\Samarth Walse\Desktop\flight controller material\EllipsoidFitting python code\data.csv", header=None, names=['x', 'y', 'z', 'resultant'])
rawdf = rawdf.drop(columns=['resultant'])
multFactor, bias = getCalibParameters(rawdf, 1.0)
calibdf = rawdf.apply(lambda x: np.dot(multFactor, (x.T - bias)), axis=1).to_list()
calibdf = pd.DataFrame(calibdf, columns =['x','y','z']) 

dfScatter3D(rawdf, calibdf)
df_plotly3D(rawdf, calibdf)

print("Bias Matrix: ")
print(bias)
print("Multiplication Factor Matrix: ")
print(multFactor)




