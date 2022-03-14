import numpy as np
import random
import math
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import copy as copy
import scipy.io as scipy
import os
import time

matlab_path = 'D:\GUC\Semester 9\Optimization\Prjoect\Driving_Scenario'
S = 50                    # Length of MZ
L=300                     # Length of CZ
dL=5                      # CZ step size
n=int(L/dL)               # no. of steps
Vmax = 25                 # Maximum velocity in CZ
Vmin = 5                  # Minimum velocity in cz
Umax = 50                  # Maximum acc in CZ
Umin = -50                 # Minimum acc in CZ
Vm = 25                   # MZ velocity
Td = 0.5                  # Response time of the brakes

def create_cars():
    return [
            CAV(0, "EW"),
            CAV(10, "EW"),
            CAV(15, "NS"),
            CAV(20, "NS"),
        ]



def is_rear_collision(CAVs):
    CAVs_NS = [CAV for CAV in CAVs if CAV.direction == "NS"]
    CAVs_EW = [CAV for CAV in CAVs if CAV.direction == "EW"]
    CAVs_NS.sort(key=lambda CAV: CAV.To)
    CAVs_EW.sort(key=lambda CAV: CAV.To)
    for k in range(len(CAVs_NS)):
        CAV_k=CAVs_NS[k]
        for i in range(k+1, len(CAVs_NS)):
            CAV_i = CAVs_NS[i]
            for j in range(len(CAV_k.t)):
                if CAV_i.t[j]-CAV_k.t[j] < (Td + CAV_i.v[j]/abs(Umin)):
                    return [CAV_k, CAV_i]
    for k in range(len(CAVs_EW)):
        CAV_k=CAVs_EW[k]
        for i in range(k+1, len(CAVs_EW)):
            CAV_i = CAVs_EW[i]
            for j in range(len(CAV_k.t)):
                if CAV_i.t[j]-CAV_k.t[j] < (Td + CAV_i.v[j]/abs(Umin)):
                    return [CAV_k, CAV_i]
    return []


def is_lateral_collision(CAVs):
    CAVs_NS = [CAV for CAV in CAVs if CAV.direction == "NS"]
    CAVs_EW = [CAV for CAV in CAVs if CAV.direction == "EW"]
    CAVs_NS.sort(key=lambda CAV: CAV.To)
    CAVs_EW.sort(key=lambda CAV: CAV.To)
    for i in range(len(CAVs_NS)):
        CAV_i = CAVs_NS[i]
        for j in range(len(CAVs_EW)):
            CAV_j = CAVs_EW[j]
            if CAV_i.To < CAV_j.To:
                if CAV_i.t[-1] + S/Vm > CAV_j.t[-1]:
                    return [CAV_i, CAV_j]
            else:
                if CAV_j.t[-1] + S/Vm > CAV_i.t[-1]:
                    return [CAV_j, CAV_i]
    return []


def is_violation(CAVs):
    firstViolation=[]
    secondViolation=[]
    thirdViolation=[]
    fourthViolation=[]
    for CAV in CAVs:
        for i in range(len(CAV.u)):
            if CAV.u[i]>Umax or CAV.u[i]<Umin or CAV.v[i]>Vmax or CAV.v[i]<Vmin:
                firstViolation.append(CAV)
        if CAV.v[-1] != Vm:
            j = 55
            vj = CAV.get_vi(j)
            if not (Umin < (pow(Vm, 2)-pow(vj, 2))/(2*(n-50)*dL) < Umax):
                secondViolation.append(CAV)
    thirdViolation = is_lateral_collision(CAVs)
    fourthViolation = is_rear_collision(CAVs)
    if (not firstViolation) and (not secondViolation) and (not thirdViolation) and (not fourthViolation):
        return []
    else:
        return [firstViolation, secondViolation, thirdViolation, fourthViolation]


def make_feasible(CAVs):
    violation = is_violation(CAVs)
    not_feasible = len(violation) != 0
    while not_feasible:
        if len(violation[0]) != 0:
            for CAV in violation[0]:
                CAV.generate_random_car_semi_feasible_solution()
        elif len(violation[1]) != 0:
            for CAV in violation[1]:
                CAV.generate_random_car_semi_feasible_from_index(55)
        elif len(violation[2]) != 0:
            r = random.randint(0, 1)
            violation[2][r].generate_random_car_semi_feasible_solution()
            # for CAV in CAVs:
            #     CAV.generate_random_car_semi_feasible_solution()
        elif len(violation[3]) != 0:
            r = random.randint(0, 1)
            violation[3][r].generate_random_car_semi_feasible_solution()
            # for CAV in CAVs:
            #     CAV.generate_random_car_semi_feasible_solution()
        violation = is_violation(CAVs)
        not_feasible = len(violation) != 0
    return CAVs


def generate_random_feasible_solution(CAVs):
    for CAV in CAVs:
        CAV.generate_random_car_semi_feasible_solution()
    make_feasible(CAVs)
    return CAVs


def obj_func_time(CAVs):  # to calculate total time
    total_time = 0
    for i in range(len(CAVs)):
        time = CAVs[i].t[-1] - CAVs[i].To
        total_time += time
    return total_time


def obj_func_fuel(CAVs):  # To calculate total consumption fuel
    total_fuel = 0
    for i in range(len(CAVs)):
        for j in range(len(CAVs[i].u)-1):
            acc = CAVs[i].u[j]
            delta_t = CAVs[i].t[j+1] - CAVs[i].t[j]
            total_fuel += delta_t*(acc**2)
    return total_fuel


def cost_func(CAVs):
    return (1/(Umax*10))*obj_func_fuel(CAVs) + obj_func_time(CAVs)


class CAV:
    def __init__(self,  To=0, direction="NS"):
        self.direction = direction
        self.To = To
        while True:
            self.Vo = random.uniform(Vmin,Vmax)
            if self.is_velocity_valid(0):
                break
        self.v = [-10]*(n+1)
        self.u = [-10]*(n+1)
        self.t = [-10]*(n+1)

    def min_max_feasible_u(self, vi, i):
        u_max = (pow(Vmax, 2) - pow(vi, 2)) / (2 * dL)
        u_min = (pow(Vmin, 2) - pow(vi, 2)) / (2 * dL)
        return [max(u_min, Umin), min(u_max, Umax)]

    def sum_u(self, i):
        sum = 0
        for j in range(i+1):
            sum = sum + self.u[j]
        return sum

    def get_vi(self, i):
        if (self.Vo ** 2) + 2*dL*self.sum_u(i-1) > 0:
            return math.sqrt((self.Vo ** 2) + 2*dL*self.sum_u(i-1))
        else:
            vi = self.Vo
            for j in range(i):
                vi = vi + self.u[j] * (self.t[j+1]-self.t[j])
            return vi

    def get_dt(self, i):
        vi = self.v[i]
        ui = self.u[i]
        return (-vi+math.sqrt(pow(vi, 2)+2*ui*dL))/ui

    def sum_dt(self, i):
        sum = 0
        for j in range(i+1):
            sum = sum + self.get_dt(j)
        return sum

    def get_T(self, i):
        return self.To + self.sum_dt(i-1)

    def is_velocity_valid(self, i):
        if (self.Vo ** 2) + 2*dL*self.sum_u(i-1) < 0:
            return False
        vi = self.get_vi(i)
        if Vmin <= vi <= Vmax and Umin < (pow(Vm, 2)-pow(vi, 2))/(2*(n-i)*dL) < Umax:
            return True
        else:
            return False

    def generate_random_car_semi_feasible_solution(self):
        # Respects first two constraints
        for i in range(len(self.u) - 1):
            vi = self.get_vi(i)
            self.v[i] = vi
            self.t[i] = self.get_T(i)
            if i == len(self.u) - 2:
                self.u[i] = (pow(Vm, 2) - pow(vi, 2)) / (2 * (n - i) * dL)
                self.u[i + 1] = 0
                self.v[i + 1] = Vm
                self.t[i + 1] = self.get_T(i + 1)
            else:
                feasible_u = self.min_max_feasible_u(vi, i)
                # while True:
                self.u[i] = random.uniform(feasible_u[0], feasible_u[1])
                # if self.is_velocity_valid(i+1):
                #     break
        return self

    def generate_random_car_semi_feasible_from_index(self, j):
        # Respects first two constraints
        for i in range(j, len(self.u) - 1):
            vi = self.get_vi(i)
            self.v[i] = vi
            self.t[i] = self.get_T(i)
            if i == len(self.u) - 2:
                self.u[i] = (pow(Vm, 2) - pow(vi, 2)) / (2 * (n - i) * dL)
                self.u[i + 1] = 0
                self.v[i + 1] = Vm
                self.t[i + 1] = self.get_T(i + 1)
            else:
                feasible_u = self.min_max_feasible_u(vi, i)
                # while True:
                self.u[i] = random.uniform(feasible_u[0], feasible_u[1])
                # if self.is_velocity_valid(i+1):
                #     break
        return self

    def calculate_v_and_t(self):
        for i in range(len(self.u)):
            vi = self.get_vi(i)
            self.v[i] = vi
            self.t[i] = self.get_T(i)

class Particle:
    def __init__(self, position):
        self.position = position
        self.Pbest_pos = position
        self.Pbest_fvalue = cost_func(position)
        self.current_velocity = np.array([[0]*(n+1)]*len(position))

    def update_position(self, Nbest, w, c1, c2):
        not_feasible = True
        first_term = np.multiply(w, self.current_velocity)
        original = copy.deepcopy(self)
        z = 0
        while not_feasible:
            self = original
            for i in range(len(self.position)):
                for j in range(len(self.position[i].u) - 1):
                    self.position[i].t[j] = self.position[i].get_T(j)
                    vj = self.position[i].get_vi(j)
                    self.position[i].v[j] = vj

                    if j == len(self.position[i].u) - 2:
                        self.position[i].u[j] = (pow(Vm, 2) - pow(vj, 2)) / (2 * (n - j) * dL)
                        self.position[i].u[j + 1] = 0
                        self.position[i].v[j + 1] = Vm
                        self.position[i].t[j + 1] = self.position[i].get_T(j + 1)
                    else:
                        feasible_u = self.position[i].min_max_feasible_u(vj, j)
                        k = 0
                        old_u = self.position[i].u[j]
                        while True:
                            first_velocity = first_term[i][j]
                            second_velocity = c1 * (np.random.uniform(0, 1) * (self.Pbest_pos[i].u[j] - old_u))
                            third_velocity = c2 * (np.random.uniform(0, 1) * (Nbest.position[i].u[j] - old_u))
                            if k < 50 and z < 50:
                                total_velocity = first_velocity + second_velocity + third_velocity
                            else:
                                if k >= 50:
                                    k = 0
                                if z >= 50:
                                    self = Particle(generate_random_feasible_solution(create_cars()));
                                    if cost_func(self.position) < original.Pbest_fvalue:
                                        self.Pbest_pos = copy.deepcopy(self.position)
                                        self.Pbest_fvalue = cost_func(self.position)
                                    return self
                                total_velocity = random.uniform(feasible_u[0] - old_u, feasible_u[1] - old_u)
                            self.current_velocity[i][j] = total_velocity
                            self.position[i].u[j] = old_u + total_velocity
                            k = k + 1
                            if self.position[i].is_velocity_valid(j + 1):
                                break
            violation = is_violation(self.position)
            not_feasible = len(violation) != 0
            z = z+1
        if cost_func(self.position) < original.Pbest_fvalue:
            self.Pbest_pos = copy.deepcopy(self.position)
            self.Pbest_fvalue = cost_func(self.position)

        return self

class PSO_Algorithm:
    def __init__(self, n_pop=50, max_iter=100):
        self.Nbest = None
        self.c1 = 1.4944
        self.c2 = 1.4944
        self.particles = []
        self.n_pop = n_pop
        self.max_iter = max_iter
        self.best_particle_in_each_iteration = []
        self.best_fvalue_in_each_iteration = []
        self.time_array = []
        self.fuel_array = []
        self.w = 0.792

    def initRandomPopulation(self):
        for _ in range(self.n_pop):
            particle = Particle(generate_random_feasible_solution(create_cars()))
            corresponding_fvalue = cost_func(particle.position)
            self.particles.append(particle)
            if len(self.best_particle_in_each_iteration) == 0:
                self.Nbest = particle
                self.best_particle_in_each_iteration.append(particle)
                self.best_fvalue_in_each_iteration.append(corresponding_fvalue)
            elif corresponding_fvalue < cost_func(self.best_particle_in_each_iteration[0].position):
                self.Nbest = particle
                self.best_particle_in_each_iteration[0], self.best_fvalue_in_each_iteration[0] = particle, corresponding_fvalue

    def perform_algorithm(self):
        self.initRandomPopulation()
        print("Best Initial Solution ", self.best_fvalue_in_each_iteration[0])
        for iter in range(self.max_iter):
            print("iteration: ", iter)
            best_particle = self.Nbest
            best_fvalue = cost_func(self.Nbest.position)
            for i in range(len(self.particles)):
                self.particles[i] = self.particles[i].update_position(self.Nbest, self.w, self.c1, self.c2)
                if cost_func(self.particles[i].position) < best_fvalue:
                    best_particle = self.particles[i]
                    best_fvalue = cost_func(self.particles[i].position)
            self.Nbest = best_particle
            self.best_particle_in_each_iteration.append(best_particle)
            self.best_fvalue_in_each_iteration.append(best_fvalue)
            self.time_array.append(obj_func_time(best_particle.position))
            self.fuel_array.append(obj_func_fuel(best_particle.position))

    def visualize(self):
        # convergence plot
        plt.figure(1)
        plt.plot(np.arange(0, self.max_iter + 1), self.best_fvalue_in_each_iteration, label="Convergence Plot")
        plt.xlabel("Iteration Number")
        plt.ylabel("Fitness Value of Best So Far")
        plt.legend()
        plt.show()
        self.barChart()
        plt.show()

    def barChart(self):
        plt.figure(2)
        barWidth = 0.25
        fig = plt.subplots(figsize=(12, 8))

        # set height of bar
        time = [self.time_array[0], self.time_array[9], self.time_array[19], self.time_array[29], self.time_array[39],
                self.time_array[49], self.time_array[59], self.time_array[69], self.time_array[79], self.time_array[89],
                self.time_array[99]]
        fuel = [(1 / (Umax * 10)) * self.fuel_array[0], (1 / (Umax * 10)) * self.fuel_array[9],
                (1 / (Umax * 10)) * self.fuel_array[19],
                (1 / (Umax * 10)) * self.fuel_array[29], (1 / (Umax * 10)) * self.fuel_array[39],
                (1 / (Umax * 10)) * self.fuel_array[49],
                (1 / (Umax * 10)) * self.fuel_array[59], (1 / (Umax * 10)) * self.fuel_array[69],
                (1 / (Umax * 10)) * self.fuel_array[79],
                (1 / (Umax * 10)) * self.fuel_array[89], (1 / (Umax * 10)) * self.fuel_array[99]]

        # Set position of bar on X axis
        br1 = np.arange(len(time))
        br2 = [x + barWidth for x in br1]

        # Make the plot
        plt.bar(br1, time, color='r', width=barWidth,
                edgecolor='grey', label='time')
        plt.bar(br2, fuel, color='g', width=barWidth,
                edgecolor='grey', label='(1/Umax*10)*fuel')

        # Adding Xticks
        plt.xlabel('Iterations', fontweight='bold', fontsize=15)
        plt.ylabel('Cost function', fontweight='bold', fontsize=15)
        plt.xticks([r + barWidth for r in range(len(time))],
                   ['0', '10', '20', '30', '40', '50', '60', '70', '80', '90', '100'])

        plt.legend()

    def toMatlabVelocities(self, variable_name):
        solution = self.best_particle_in_each_iteration[-1].position
        velocities=np.zeros(shape=(len(solution),len(solution[0].v)))
        for i in range(len(solution)):
            velocities[i] = solution[i].v
        file_path = 'velocities.mat'
        scipy.savemat(os.path.join(matlab_path, file_path), {str(variable_name): velocities})

    def toMatlabArrivalTimes(self, variable_name):
        solution = self.best_particle_in_each_iteration[-1].position
        arrival_times = np.zeros(shape=(len(solution)))
        for i in range(len(solution)):
            arrival_times[i] = solution[i].To
        file_path = 'arrivalTimes.mat'
        scipy.savemat(os.path.join(matlab_path, file_path), {str(variable_name): arrival_times})

    def toMatlabDirections(self, variable_name):
        solution = self.best_particle_in_each_iteration[-1].position
        directions = np.zeros(shape=(len(solution)), dtype='object')
        for i in range(len(solution)):
            directions[i] = solution[i].direction
        file_path = 'directions.mat'
        scipy.savemat(os.path.join(matlab_path, file_path), {str(variable_name): directions})

    def toMatlabExitTimes(self, variable_name):
        solution = self.best_particle_in_each_iteration[-1].position
        exitTimes = np.zeros(shape=(len(solution)))
        for i in range(len(solution)):
            exitTimes[i] = solution[i].t[-1]+(L+S)/Vm
        file_path = 'exitTimes.mat'
        scipy.savemat(os.path.join(matlab_path, file_path), {str(variable_name): exitTimes})


PSO = PSO_Algorithm()
PSO.perform_algorithm()
print(PSO.best_fvalue_in_each_iteration)
PSO.visualize()
PSO.toMatlabVelocities('velocities')
PSO.toMatlabArrivalTimes('arrivalTimes')
PSO.toMatlabDirections('directions')
PSO.toMatlabExitTimes('exitTimes')


# CAVs1 = create_cars()
# CAVs2 = create_cars()
# generate_random_feasible_solution(CAVs1)
# generate_random_feasible_solution(CAVs2)
# print(CAVs1[3].u[2])
# crossover_test(CAVs1,CAVs2)
# print(CAVs1[3].u[2])