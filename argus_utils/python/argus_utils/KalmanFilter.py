import numpy as np
from collections import namedtuple
from scipy.stats import multivariate_normal as mvn

KalmanPredict = namedtuple('KalmanPredict',
                           ['dt', 'lastPpost', 'A', 'Ppre'])
KalmanUpdate = namedtuple('KalmanUpdate',
                          ['inno', 'K', 'Cv', 'Ppost', 'C', 'res'])
KalmanPreUpdate = namedtuple('KalmanPreUpdate',
                             ['inno', 'C'])


class KalmanFilter(object):

    def __init__(self, Afunc=None, C=None, Q=None, R=None):
        self.Afunc = Afunc
        self.C = C
        self.Q = Q
        self.R = R
        self.x = None
        self.P = None
        self.t = None

    def initialize(self, x, P, t):
        self.x = x
        self.P = P
        self.t = t

    def predict(self, t, Q=None):
        dt = t - self.t
        if dt < 0:
            raise ValueError('KF negative dt')
        A = self.Afunc(dt)

        if Q is None:
            Q = self.Q

        self.x = np.dot(A, self.x)
        lastPpost = self.P
        self.P = np.dot(np.dot(A, self.P), A.T) + Q
        self.t = t
        return KalmanPredict(dt=dt, lastPpost=lastPpost, A=A, Ppre=self.P)

    def compute_obs_ll(self, t, y, C=None, R=None):
        if C is None:
            C = self.C
        if R is None:
            R = self.R

        if t != self.t:
            raise ValueError('KF update wrong time')

        ypred = np.dot(C, self.x)
        inno = y - ypred
        Cv = np.dot(np.dot(C, self.P), C.T) + R
        try:
            return mvn.logpdf(x=inno, cov=Cv)
        except ValueError:
            print 'inno'
            print inno
            print 'Cv'
            print Cv
            print 'R'
            print R
            raise RuntimeError

    def update(self, t, y, C=None, R=None):
        if C is None:
            C = self.C
        if R is None:
            R = self.R

        if t != self.t:
            raise ValueError('KF update wrong time')

        ypred = np.dot(C, self.x)
        inno = y - ypred
        Cv = np.dot(np.dot(C, self.P), C.T) + R
        K = np.transpose(np.linalg.solve(Cv, np.dot(C, self.P)))
        dx = np.dot(K, inno)
        self.x = self.x + dx
        residual = y - np.dot(C, self.x)
        L = np.identity(self.P.shape[0]) - np.dot(K, C)
        self.P = np.dot(np.dot(L, self.P), L.T) + \
            np.dot(np.dot(K, R), K.T)
        return KalmanUpdate(inno=inno, K=K, Cv=Cv,
                            Ppost=self.P, C=C, residual=residual)
