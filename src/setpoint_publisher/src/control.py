from time import time

class PID(object):
    def __init__(self,p,i=0,d=0):
        self.integrated = 0
        self.previous_error = 0
        self.time = time()
        self.coef_p = p
        self.coef_i = i
        self.coef_d = d
        self.action = self.__firstIteration

    def setProportional(self, value):
        self.coef_p = value

    def setIntegral(self, value):
        self.coef_i = value

    def setDerivative(self, value):
        self.coef_d = value

    def setCoefficients(self, coefficients):
        self.coef_p, self.coef_i, self.coef_d = coefficients

    def __firstIteration(self, error):
        self.previous_error = error
        self.time = time()
        self.action = self.__loop
        return self.coef_p * error

    def __loop(self, error):
        now = time()
        time_difference = now - self.time
        derivetive = (error - self.previous_error) / time_difference
        self.integrated += error * time_difference
        action = self.coef_p * error + self.coef_i * self.integrated + self.coef_d * derivetive
        self.time = now
        return action


from time import time

class PID2(object):
    def __init__(self,p,i=0,d=0):
        self.integrated = 0
        self.previous_error = 0
        self.time = time()
        self.coef_p = p
        self.coef_i = i
        self.coef_d = d
        self.action = self.__firstIteration

    def setProportional(self, value):
        self.coef_p = value

    def setIntegral(self, value):
        self.coef_i = value

    def setDerivative(self, value):
        self.coef_d = value

    def setCoefficients(self, coefficients):
        self.coef_p, self.coef_i, self.coef_d = coefficients

    def __firstIteration(self, error):
        self.previous_error = error
        self.time = time()
        self.action = self.__loop
        return self.coef_p * error

    def __loop(self, error):
        from time import time
        now = time()
        time_difference = now - self.time
        derivetive = (error - self.previous_error) / time_difference
        self.integrated += error * time_difference
        action = self.coef_p * error + self.coef_i * self.integrated + self.coef_d * derivetive
        self.time = now
        return action



def time_test():
    count = 10000

    m=PID(1)
    start = time()
    for i in range(count):
        m.action(.5)
    print(time()-start)

if __name__ == "__main__":
    time_test()
