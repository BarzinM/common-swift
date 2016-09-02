from time import time


class ControlError(Exception):
    pass


def saturate(value, upper_limit, lower_limit):
    return min(max(value, lower_limit), upper_limit)


class PID(object):
    def __init__(self, p, i, d, windup):
        self.integrated = 0
        self.previous_error = 0
        self.time = time()
        self.coef_p = p
        self.coef_i = i
        self.coef_d = d
        self.action = self.__firstIteration
        if windup is None:
            self.upper_limit = None
        elif type(windup) is list:
            if len(windup) > 2:
                raise ControlError('Length of `windup` argument shouldn\'t be more than 2.')
            if len(windup) == 1:
                windup.append(-windup[0])
            self.upper_limit = max(windup)
            self.lower_limit = min(windup)
        elif type(1. * windup) is float:
            if windup <= 0:
                raise ControlError('Value for windup argument should be greater than 0.')
            self.upper_limit = windup
            self.lower_limit = -windup

    def setProportional(self, value):
        self.coef_p = value

    def setIntegral(self, value):
        self.coef_i = value

    def setDerivative(self, value):
        self.coef_d = value

    def setCoefficients(self, pid_coefficients):
        self.coef_p, self.coef_i, self.coef_d = pid_coefficients

    def __firstIteration(self, error):
        self.time = time()
        self.previous_error = error
        if self.upper_limit is None:
            self.action = self.__loop
        else:
            self.previous_windup = 0
            self.action = self.__loopWindup
        return self.coef_p * error

    def __loopWindup(self, error):
        now = time()
        time_difference = now - self.time
        derivetive = (error - self.previous_error) / time_difference
        local_integrated = error * time_difference + self.integrated

        # self.integrated += (error + self.previous_windup) * time_difference
        action = self.coef_p * error + self.coef_i * local_integrated + self.coef_d * derivetive
        action_difference = action - saturate(action, self.upper_limit, self.lower_limit)
        if action_difference == 0.:
            self.integrated = local_integrated
        self.time = now
        return action - action_difference

    def __loop(self, error):
        now = time()
        time_difference = now - self.time
        derivetive = (error - self.previous_error) / time_difference
        self.integrated += error * time_difference
        action = self.coef_p * error + self.coef_i * self.integrated + self.coef_d * derivetive
        return action


class MassSpringDamper(object):
    def __init__(self, mass, stiffness, damping):
        self.mass = float(mass)
        self.stiffness = float(stiffness)
        self.damping = float(damping)
        self.position = 0
        self.velocity = 0

    def applyForce(self, force, duration):
        acceleration = force / self.mass - self.damping * self.velocity - self.stiffness * self.position
        velocity_difference = acceleration * duration
        self.position += (self.velocity + velocity_difference / 2) * duration
        self.velocity += velocity_difference
        return self.position


def forceTest():
    import matplotlib.pyplot as plt
    obj = MassSpringDamper(5, 10, 1)
    history_position = [obj.applyForce(1, .01)]
    length = 3000
    for _ in range(length):
        position = obj.applyForce(0, 0.01)
        history_position.append(position)
    plt.plot(range(length + 1), history_position)
    plt.show()


def controlTest():
    from time import sleep
    from numpy import sin
    import matplotlib.pyplot as mlt
    count = 200
    history_action = []

    obj = MassSpringDamper(.1, 0, 6)
    obj.position = 1
    error = 0 - obj.applyForce(0, .01)
    history_error = [error]
    controller = PID(20, 1, .015, windup=-1.5)
    for i in range(count):
        act = saturate(controller.action(error), 1.5, -1.5)
        error = 0 - obj.applyForce(act, 0.01)
        print('Error: %+0.2f | Action: %+0.2f' % (error, act))
        sleep(.01)
        history_error.append(error)
        history_action.append(act)

    print('overshoot', max(history_error))
    mlt.plot(range(count + 1), history_error)
    mlt.plot(range(count), history_action)
    mlt.show()

if __name__ == "__main__":
    forceTest()
    controlTest()
    pass
