'''
Author: Xiyue Chen
Date:   2024-11-18
Usage:  Linear approximation for power consumption for UAVs at hovering states.
'''
# import from other codes
import os
import sys

# Get absolute path to parent directory
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)
from input.config import Physics, UAV

import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression


# Define constants
UAV = UAV()
g = Physics.GRAVITY  # gravitational acceleration (m/s²)
rho = Physics.AIR_DENSITY
A = UAV.ROTOR_AREA         # arbitrary area value
# m0_ls = [3, 5, 7, 9]         # initial mass
# m0_ls = [25, 30, 35, 40]
m0_ls =[5]
color_ls = ['salmon', 'gold', 'lightseagreen', 'royalblue']


def generate_data(m_values, m0):
    """Generate y values based on the given formula"""
    constant = np.sqrt((g**3)/(2 * rho * A))
    y_values = ((m0 + m_values)**1.5) * constant
    return y_values

if __name__ == "__main__":
    # Generate data points
    m = np.linspace(0, UAV.WEIGHT_CAPACITY, 100)  # generate 100 points from 0 to 5
    y_true = []
    y_pred = []
    for m0 in m0_ls:
        y = generate_data(m, m0)
        y_true.append(y)

        # Reshape data for sklearn
        X = m.reshape(-1, 1)
        y = y.reshape(-1, 1)

        # Perform linear regression
        model = LinearRegression()
        model.fit(X, y)

        # Generate predictions
        y_hat = model.predict(X)
        y_pred.append(y_hat)

        # Print regression results
        print(f"Linear Regression Results for inital weight {m0}:")
        print(f"Slope: {model.coef_[0][0]:.4f}")
        print(f"Intercept: {model.intercept_[0]:.4f}")
        print(f"R² Score: {model.score(X, y):.4f}")

    # Plot results
    plt.figure(figsize=(10, 6))
    for i in range(len(y_true)):
        plt.scatter(m, y_true[i], color='gray', s=0.3)
        plt.plot(m, y_pred[i], color=color_ls[i], label=f'$m_0$={m0_ls[i]}kg (fitting)')

    plt.xlabel('m (mass)')
    plt.ylabel('power consumption p (Watt)')
    # plt.title('Data Generated from Formula with Linear Regression Fit')
    plt.legend()
    # plt.grid()



    plt.show()