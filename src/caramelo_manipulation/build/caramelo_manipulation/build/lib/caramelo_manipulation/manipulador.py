import numpy as np

def dh_transform(a, alpha, d, theta):
    alpha = np.deg2rad(alpha)
    theta = np.deg2rad(theta)
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

class Manipulador5DOF:
    def __init__(self, A2=10, A3=10, A4=5, d1=10, d5=2):
        self.A2 = A2
        self.A3 = A3
        self.A4 = A4
        self.d1 = d1
        self.d5 = d5

    def encoder_to_deg(self, pos):
        # 0 - 4095 mapeado para 0 - 360 graus
        return (pos % 4096) * 360 / 4096

    def get_pose_from_encoders(self, encoders):
        thetas_deg = [self.encoder_to_deg(p) for p in encoders]
        return self.get_pose(thetas_deg)

    def get_pose(self, thetas):
        θ1, θ2, θ3, θ4, θ5 = thetas

        T1 = dh_transform(0, -90, self.d1, θ1)
        T2 = dh_transform(self.A2, 0, 6, θ2)
        T3 = dh_transform(self.A3, 0, 6, θ3)
        T4 = dh_transform(self.A4, 90, 0, θ4)
        T5 = dh_transform(0, 0, self.d5, θ5)

        T = T1 @ T2 @ T3 @ T4 @ T5
        pos = T[:3, 3]
        return pos, T

    def check_collision(self, encoders, limite_z=2.0):
        pos, _ = self.get_pose_from_encoders(encoders)
        return pos[2] < limite_z
