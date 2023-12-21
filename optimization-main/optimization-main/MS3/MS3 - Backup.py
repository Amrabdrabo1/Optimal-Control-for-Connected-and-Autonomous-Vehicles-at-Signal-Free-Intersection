import numpy as np
import random
import plotly.graph_objects as go
import matplotlib.pyplot as plt


current_car = 0


class CAV:
    def __init__(self, direction="NS", To=0, Vi=20):
        self.direction = direction
        self.To = To
        self.Tf = 0
        self.Vi = Vi
        self.x = [[-10]*2]*60
        self.v = [[-10]*2]*60
        self.u = [[-10]*2]*60


class problem:
    def __init__(self, Vmax=25, Vmin=5, Umax=5, Umin=-5, Vc=15, L=300, S=50, safe_distance=20, delta_t=1):
        self.CAVs = []
        self.solution = []
        self.Vmax = Vmax                    # Maximum velocity in CZ
        self.Vmin = Vmin                    # Minimum velocity in cz
        self.Umax = Umax                    # Maximum acc in CZ
        self.Umin = Umin                 # Minimum acc in CZ
        self.Vc = Vc                      # MZ velocity
        self.L = L                          # Length of CZ
        self.S = S                          # Length of MZ
        self.safe_distance = safe_distance
        self.Tmax = 60
        self.delta_t = delta_t

    def initializeCAVs(self):
        self.CAVs = [
            CAV("NS", 0, 20),
            CAV("NS", 1, 10),
            CAV("EW", 3, 14),
            CAV("EW", 4, 22),
        ]

    def setRandomSol(self, CAV):
        startFakes = False
        for t in range(self.Tmax):
            if(startFakes):
                CAV.u[t] = [CAV.To+self.delta_t*t, -10]
                CAV.v[t] = [CAV.To+self.delta_t*t, -10]
                CAV.x[t] = [CAV.To+self.delta_t*t, -10]
            elif (t == 0):
                CAV.u[t] = [CAV.To+self.delta_t*t,
                            random.randint(self.Umin, self.Umax)]
                CAV.v[t] = [CAV.To+self.delta_t*t, CAV.Vi]
                CAV.x[t] = [CAV.To+self.delta_t*t, 0]
            else:
                CAV.u[t] = [CAV.To+self.delta_t*t,
                            random.randint(self.Umin, self.Umax)]
                CAV.x[t] = [CAV.To+self.delta_t*t, CAV.x[t-1][1] + CAV.v[t-1]
                            [1]*self.delta_t + 0.5*CAV.u[t-1][1]*pow(self.delta_t, 2)]

                validVelocity = True
                while validVelocity:
                    CAV.v[t] = [CAV.To+self.delta_t*t, CAV.v[t-1]
                                [1] + CAV.u[t-1][1]*self.delta_t]
                    if (CAV.v[t][1] >= self.Vmin and CAV.v[t][1] <= self.Vmax):
                        validVelocity = False
                    else:
                        CAV.u[t-1] = [CAV.To+self.delta_t *
                                      (t-1), random.randint(self.Umin, self.Umax)]

                if (CAV.x[t][1] >= self.L):
                    CAV.Tf = CAV.u[t][0]
                    startFakes = True

                    if (CAV.v[t][1] != self.Vc):

                        if((self.Vc-CAV.v[t-3][1])/self.delta_t > self.Umax):
                            CAV.u[t-2] = [CAV.To+self.delta_t*(t-2), self.Umax]
                        elif((self.Vc-CAV.v[t-3][1])/self.delta_t < self.Umin):
                            CAV.u[t-2] = [CAV.To+self.delta_t*(t-2), self.Umin]
                        else:
                            CAV.u[t-2] = [CAV.To+self.delta_t *
                                          (t-2), (self.Vc-CAV.v[t-3][1])/self.delta_t]

                        CAV.v[t-2] = [CAV.To+self.delta_t *
                                      (t-2), CAV.v[t-3][1] + CAV.u[t-3][1]*self.delta_t]
                        CAV.x[t-2] = [CAV.To+self.delta_t*(
                            t-2), CAV.x[t-3][1] + CAV.v[t-3][1]*self.delta_t + 0.5*CAV.u[t-3][1]*pow(self.delta_t, 2)]

                        if((self.Vc-CAV.v[t-2][1])/self.delta_t > self.Umax):
                            CAV.u[t-1] = [CAV.To+self.delta_t*(t-1), self.Umax]
                        elif((self.Vc-CAV.v[t-2][1])/self.delta_t < self.Umin):
                            CAV.u[t-1] = [CAV.To+self.delta_t*(t-1), self.Umin]
                        else:
                            CAV.u[t-1] = [CAV.To+self.delta_t *
                                          (t-1), (self.Vc-CAV.v[t-2][1])/self.delta_t]

                        CAV.v[t-1] = [CAV.To+self.delta_t *
                                      (t-1), CAV.v[t-2][1] + CAV.u[t-2][1]*self.delta_t]
                        CAV.x[t-1] = [CAV.To+self.delta_t*(
                            t-1), CAV.x[t-2][1] + CAV.v[t-2][1]*self.delta_t + 0.5*CAV.u[t-2][1]*pow(self.delta_t, 2)]

                        if((self.Vc-CAV.v[t-1][1])/self.delta_t > self.Umax):
                            CAV.u[t] = [CAV.To+self.delta_t*t, self.Umax]
                        elif((self.Vc-CAV.v[t-1][1])/self.delta_t < self.Umin):
                            CAV.u[t] = [CAV.To+self.delta_t*t, self.Umin]
                        else:
                            CAV.u[t] = [CAV.To+self.delta_t*t,
                                        (self.Vc-CAV.v[t-1][1])/self.delta_t]

                        CAV.v[t] = [CAV.To+self.delta_t*t, CAV.v[t-1]
                                    [1] + CAV.u[t-1][1]*self.delta_t]
                        CAV.x[t] = [CAV.To+self.delta_t*t, CAV.x[t-1][1] + CAV.v[t-1]
                                    [1]*self.delta_t + 0.5*CAV.u[t-1][1]*pow(self.delta_t, 2)]

    def obj_func_time(self, CAVs):  # to calculate total time
        total_time = 0
        for i in range(len(CAVs)):
            time = CAVs[i].Tf - CAVs[i].To
            total_time += time
        return total_time

    def obj_func_fuel(self, CAVs):  # To calculate total consumption fuel
        total_fuel = 0
        for i in range(len(CAVs)):
            for j in range(len(CAVs[i].u)):
                acc = CAVs[i].u[j][1]
                if(acc != -10):
                    total_fuel += acc**2
        return total_fuel

    def cost_func(self, CAVs):
        return self.obj_func_fuel(self.CAVs) + self.obj_func_time(self.CAVs)

    def isFeasible(self, cars):
        v_min = self.Vmin
        v_max = self.Vmax
        u_min = self.Umin
        u_max = self.Umax
        L = self.L
        S = self.S
        Vc = self.Vc
        safeDistance = self.safe_distance
        delta_t = self.delta_t
        cars_NS = [car3 for car3 in cars if car3.direction == "NS"]
        cars_EW = [car2 for car2 in cars if car2.direction == "EW"]

        # # Check over min and max velocity and acceleration
        # Check over rear collision
        # NS direction
        for i in range(len(cars_NS) - 1):
            car_i = cars[i]
            car_k = cars[i + 1]
            if car_k.Tf < car_i.Tf:
                return False
            for t in range(car_k.To, car_k.Tf, delta_t):
                car_kxt = car_k.x[int((t - car_k.To) / delta_t)]
                car_ixt = [(x, y) for x, y in car_i.x if x == t]
                if len(car_ixt) != 0:
                    if abs(car_ixt[0][1] - car_kxt[1]) < safeDistance:
                        return False
        # EW direction
        for i in range(len(cars_EW) - 1):
            car_i = cars[i]
            car_k = cars[i + 1]
            if car_k.Tf < car_i.Tf:
                return False
            for t in range(car_k.To, car_k.Tf, delta_t):
                car_kxt = car_k.x[int((t - car_k.To) / delta_t)]
                car_ixt = [(x, y) for x, y in car_i.x if x == t]
                if len(car_ixt) != 0:
                    if abs(car_ixt[0][1] - car_kxt[1]) < safeDistance:
                        return False
        # Check over side collision
        for i in range(len(cars_NS)):
            for j in range(len(cars_EW)):
                car_i = cars_NS[i]
                car_j = cars_EW[j]
                if car_i.To < car_j.To:
                    if car_i.Tf + (S / Vc) > car_j.Tf:
                        return False
                else:
                    if car_j.Tf + (S / Vc) > car_i.Tf:
                        return False
        return True


class SimulatedAnnealing:
    def __init__(self, problem, max_iter=100, init_temp=100, final_temp=1e-03, iter_per_temp=5, cooling_schedule="linear", beta=5, alpha=0.9):
        self.problem = problem
        self.max_iter = max_iter
        self.init_temp = init_temp
        self.final_temp = final_temp
        self.iter_per_temp = iter_per_temp
        self.cooling_schedule = cooling_schedule
        self.beta = min(beta, (init_temp - final_temp)/max_iter)
        self.alpha = alpha
        self.curr_temp = None
        self.sols = None
        self.fvalues = None
        self.best_sol = None
        self.best_fvalue = None
        self.current_sol = []

    def cool_down_temp(self, curr_iter):
        schedules = {
            "linear": self.curr_temp - self.beta,
            "geometric": self.curr_temp * self.alpha
        }
        return schedules.get(self.cooling_schedule, schedules.get("linear"))

    def Array_solutions(self, CAVs):
        list_randsol = []
        for i in range(len(CAVs)):
            list_randsol.append(self.problem.setRandomSol(CAVs[i]))

        while(self.problem.isFeasible(CAVs) == False):
            list_randsol = []
            for i in range(len(CAVs)):
                list_randsol.append(self.problem.setRandomSol(CAVs[i]))

        return list_randsol

    def perform_algorithm(self):
        # initial solution for all cars
        self.Current_sol = self.Array_solutions(self.problem.CAVs)

        self.curr_temp = self.init_temp
        self.sols = [self.current_sol]  # u's
        self.fvalues = [self.problem.cost_func(self.problem.CAVs)]  # f(u)
        # decision variable which is acceleration
        self.best_sol = self.current_sol
        self.best_fvalue = self.problem.cost_func(self.problem.CAVs)  # f(u)
        for iter in range(self.max_iter):
            for _ in range(self.iter_per_temp):
                sol_neighbour = self.Array_solutions(
                    self.problem.CAVs)  # random solution for u
                fvalue_neighbour = self.problem.cost_func(
                    sol_neighbour)  # f(new u)

                # check if f(new u) < f(old u
                if fvalue_neighbour < self.fvalues[-1]:
                    # neighbour is better and is accpeted
                    self.sols.append(sol_neighbour)  # u
                    self.fvalues.append(fvalue_neighbour)  # f(u)
                else:
                    p = np.exp(-(fvalue_neighbour -
                               self.fvalues[-1]) / self.curr_temp)
                    if np.random.rand() < p:
                        # neighbour is worse and is accepted according to probability
                        self.sols.append(sol_neighbour)
                        self.fvalues.append(fvalue_neighbour)
                    else:
                        # neighbour is worse and is rejected
                        self.sols.append(self.sols[-1])
                        self.fvalues.append(self.fvalues[-1])
                # update best solution reached so far
                if self.fvalues[-1] < self.best_fvalue:
                    self.best_sol = self.sols[-1]
                    self.best_fvalue = self.fvalues[-1]
            # update temperature
            self.curr_temp = self.cool_down_temp(iter)

    def visualize(self):
        # convergence plot
        fig1 = go.Figure(data=go.Scatter(x=np.arange(
            0, self.max_iter*self.iter_per_temp), y=self.fvalues, mode="lines"))
        fig1.update_layout(
            title="Convergence Plot",
            xaxis_title="Iteration Number",
            yaxis_title="Objective Function Value"
        )
        fig1.show()

    def graf(self, delta):
        car = self.problem.CAVs[0]
        time = []
        for i in range((int((car.Tf-car.To)/delta))):
            time.append(car.To+i*delta)
        # time.append(car.Tf)

        u_empty = []
        v_empty = []
        x_empty = []
        for i in range((int((car.Tf-car.To)/delta))):
            u_empty.append(car.u[i][1])
            v_empty.append(car.v[i][1])
            x_empty.append(car.x[i][1])

        plt.plot(time, u_empty)
        plt.xlabel("Time")
        plt.ylabel("Acceleration")
        plt.show()

        plt.plot(time, v_empty)
        plt.xlabel("Time")
        plt.ylabel("Velocity")
        plt.show()

        plt.plot(time, x_empty)
        plt.xlabel("Time")
        plt.ylabel("position")
        plt.show()


myproblem = problem(Vmax=25, Vmin=5, Umax=5, Umin=-5, Vc=15,
                    L=300, S=50, safe_distance=20, delta_t=1)

myproblem.setCAVs()

SA = SimulatedAnnealing(myproblem, max_iter=500, init_temp=100, final_temp=1e-03,
                        iter_per_temp=10, cooling_schedule="linear", beta=5)


# def graf(car, delta):

#     time = [range(car.To, car.Tf, delta)]
#     time = time.append(car.Tf)
#     u_empty = []
#     v_empty = []
#     x_empty = []
#     for i in range(len((Tf-To)/delta)):
#         u_empty.append(car.u[i][1])
#         v_empty.append(car.v[i][1])
#         x_empty.append(car.x[i][1])
#     plt.plot(time, u_empty)
#     plt.show()


SA.perform_algorithm()

for i in range(len(myproblem.CAVs)):
    print(myproblem.CAVs[i].u)
print(SA.best_fvalue)
SA.visualize()
print(myproblem.CAVs[0].Tf)
SA.graf(1)
