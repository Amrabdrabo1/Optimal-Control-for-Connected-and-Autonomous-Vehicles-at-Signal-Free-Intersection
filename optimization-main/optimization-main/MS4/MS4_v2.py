import numpy as np
import random
import math
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import copy as copy
import scipy.io as scipy
import os
import time

S = 10  # Length of MZ
L = 50  # Length of CZ
dL = 0.5  # CZ step size
n = int(L / dL)  # no. of steps
Vmax = 5                 # Maximum velocity in CZ
Vmin = 1                  # Minimum velocity in cz
Umax = 10                  # Maximum acc in CZ
Umin = -10                 # Minimum acc in CZ
Vm = Vmax                   # MZ velocity
Td = 0.1                  # Response time of the brakes

matlab_path = 'D:\GUC\Semester 9\Optimization\Prjoect\Driving_Scenario'


def mutation_test(child):
    # Random noise
    not_feasible = True
    while not_feasible:
        for i in range(len(child)):
            for j in range(len(child[i].u) - 1):
                vi = child[i].get_vi(j)
                child[i].v[j] = vi
                child[i].t[j] = child[i].get_T(i)
                if j == len(child[i].u) - 2:
                    child[i].u[i] = (pow(Vm, 2) - pow(vi, 2)) / (2 * (n - i) * dL)
                    child[i].u[j + 1] = 0
                    child[i].v[j + 1] = Vm
                    child[i].t[j + 1] = child[i].get_T(i + 1)
                else:
                    while True:
                        new_u = child[i].u[j] + random.uniform(max(-1, Umin - child[i].u[j]),
                                                               min(1, Umax - child[i].u[j]))
                        child[i].u[j] = new_u
                        if child[i].is_velocity_valid(j + 1):
                            break
        violation = is_violation(child)
        not_feasible = len(violation) != 0
    return child


def create_cars():
    return [
        CAV(0, "EW"),
        CAV(10, "EW"),
        CAV(15, "NS"),
        CAV(20, "NS"),
    ]


# def is_FIFO(CAVs):
#     CAVs_sorted = sorted(CAVs, key=lambda CAV: CAV.To)
#     for i in range(len(CAVs_sorted)):
#         for j in range(i+1, len(CAVs_sorted)):
#             if CAVs_sorted[j].t[-1] < CAVs_sorted[j].t[-1]:
#                 return False
#     return True


def is_rear_collision(CAVs):
    CAVs_NS = [CAV for CAV in CAVs if CAV.direction == "NS"]
    CAVs_EW = [CAV for CAV in CAVs if CAV.direction == "EW"]
    CAVs_NS.sort(key=lambda CAV: CAV.To)
    CAVs_EW.sort(key=lambda CAV: CAV.To)
    for k in range(len(CAVs_NS)):
        CAV_k = CAVs_NS[k]
        for i in range(k + 1, len(CAVs_NS)):
            CAV_i = CAVs_NS[i]
            for j in range(len(CAV_k.t)):
                if CAV_i.t[j] - CAV_k.t[j] < (Td + CAV_i.v[j] / abs(Umin)):
                    return [CAV_k, CAV_i]
    for k in range(len(CAVs_EW)):
        CAV_k = CAVs_EW[k]
        for i in range(k + 1, len(CAVs_EW)):
            CAV_i = CAVs_EW[i]
            for j in range(len(CAV_k.t)):
                if CAV_i.t[j] - CAV_k.t[j] < (Td + CAV_i.v[j] / abs(Umin)):
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
                if CAV_i.t[-1] + S / Vm > CAV_j.t[-1]:
                    return [CAV_i, CAV_j]
            else:
                if CAV_j.t[-1] + S / Vm > CAV_i.t[-1]:
                    return [CAV_j, CAV_i]
    return []


def is_violation(CAVs):
    firstViolation = []
    secondViolation = []
    thirdViolation = []
    fourthViolation = []
    for CAV in CAVs:
        for i in range(len(CAV.u)):
            if CAV.u[i] > Umax or CAV.u[i] < Umin or CAV.v[i] > Vmax or CAV.v[i] < Vmin:
                firstViolation.append(CAV)
        if CAV.v[-1] != Vm:
            j = 55
            vj = CAV.get_vi(j)
            if not (Umin < (pow(Vm, 2) - pow(vj, 2)) / (2 * (n - 50) * dL) < Umax):
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
        for j in range(len(CAVs[i].u) - 1):
            acc = CAVs[i].u[j]
            delta_t = CAVs[i].t[j + 1] - CAVs[i].t[j]
            total_fuel += delta_t * (acc ** 2)
    return total_fuel


def cost_func(CAVs):
    return (1 / (Umax * 10)) * obj_func_fuel(CAVs) + obj_func_time(CAVs)


class CAV:
    def __init__(self, To=0, direction="NS"):
        self.direction = direction
        self.To = To
        while True:
            self.Vo = random.uniform(Vmin, Vmax)
            if self.is_velocity_valid(0):
                break
        self.v = [-10] * (n + 1)
        self.u = [-10] * (n + 1)
        self.t = [-10] * (n + 1)

    def min_max_feasible_u(self, vi, i):
        u_max = (pow(Vmax, 2) - pow(vi, 2)) / (2 * dL)
        u_min = (pow(Vmin, 2) - pow(vi, 2)) / (2 * dL)
        return [max(u_min, Umin), min(u_max, Umax)]

    def sum_u(self, i):
        sum = 0
        for j in range(i + 1):
            sum = sum + self.u[j]
        return sum

    def get_vi(self, i):
        if (self.Vo ** 2) + 2 * dL * self.sum_u(i - 1) > 0:
            return math.sqrt((self.Vo ** 2) + 2 * dL * self.sum_u(i - 1))
        else:
            vi = self.Vo
            for j in range(i):
                vi = vi + self.u[j] * (self.t[j + 1] - self.t[j])
            return vi

    def get_dt(self, i):
        vi = self.v[i]
        ui = self.u[i]
        return (-vi + math.sqrt(pow(vi, 2) + 2 * ui * dL)) / ui

    def sum_dt(self, i):
        sum = 0
        for j in range(i + 1):
            sum = sum + self.get_dt(j)
        return sum

    def get_T(self, i):
        return self.To + self.sum_dt(i - 1)

    def is_velocity_valid(self, i):
        if (self.Vo ** 2) + 2 * dL * self.sum_u(i - 1) < 0:
            return False
        vi = self.get_vi(i)
        if Vmin <= vi <= Vmax and Umin < (pow(Vm, 2) - pow(vi, 2)) / (2 * (n - i) * dL) < Umax:
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


class GeneticAlgorithm:
    def __init__(self, n_pop=50, max_iter=100, p_elite=0.1, p_crossover=0.8, p_mutation=0.1,
                 parents_selection="Random", tournament_size=5, mutation_selection="Worst",
                 survivors_selection="Fitness"):
        # self.problem = problem
        self.n_pop = n_pop
        self.max_iter = max_iter
        self.p_elite = p_elite
        self.p_crossover = p_crossover
        self.p_mutation = p_mutation
        self.parents_selection = parents_selection
        self.tournament_size = tournament_size if tournament_size < n_pop else n_pop
        self.mutation_selection = mutation_selection
        self.survivors_selection = survivors_selection
        self.gen_sols = None
        self.gen_fvalues = None
        self.gen_ages = None
        self.best_sols = None
        self.best_fvalues = None
        self.time_array = []
        self.fuel_array = []

    def initRandomPopulation(self):
        self.gen_sols = []
        self.gen_fvalues = []
        self.gen_ages = []
        self.best_sols = []
        self.best_fvalues = []
        for _ in range(self.n_pop):
            new_sol = generate_random_feasible_solution(create_cars())
            new_fvalue = cost_func(new_sol)
            self.gen_sols.append(new_sol)
            self.gen_fvalues.append(new_fvalue)
            self.gen_ages.append(0)
            if len(self.best_sols) == 0:
                self.best_sols.append(new_sol)
                self.best_fvalues.append(new_fvalue)
            elif (new_fvalue < self.best_fvalues[0]):
                self.best_sols[0], self.best_fvalues[0] = new_sol, new_fvalue

    def selectParents(self, numParents, criteria):
        gen_probs = 1 / (1 + np.square(self.gen_fvalues))
        gen_probs = gen_probs / sum(gen_probs)
        lambda_rank = 1.5  # (between 1 and 2) offspring created by best individual
        gen_ranks = list(map(lambda i: np.argwhere(np.argsort(self.gen_fvalues) == i)[0, 0], np.arange(self.n_pop)))
        gen_ranks = ((2 - lambda_rank) + np.divide(gen_ranks, self.n_pop - 1) * (2 * lambda_rank - 2)) / self.n_pop
        selection_criteria = {
            "Random": lambda n: np.random.choice(self.n_pop, size=(n,), replace=False),
            "RouletteWheel": lambda n: np.random.choice(self.n_pop, size=(n,), replace=True, p=gen_probs),
            "SUS": lambda n: np.random.choice(self.n_pop, size=(n,), replace=False, p=gen_probs),
            "Rank": lambda n: np.random.choice(self.n_pop, size=(n,), replace=False, p=gen_ranks),
            "Tournament": lambda n: np.array([np.amin(list(map(lambda i: [self.gen_fvalues[i], i],
                                                               np.random.choice(self.n_pop,
                                                                                size=(self.tournament_size,),
                                                                                replace=False))),
                                                      axis=0)[1] for _ in range(n)], dtype=int),
            "Worst": lambda n: np.argsort(self.gen_fvalues)[self.n_pop - n:]
        }
        parents_idx = selection_criteria.get(criteria, selection_criteria["Random"])(numParents)
        return parents_idx

    def crossover(self, p1_idx, p2_idx):
        # Whole Arithmetic Combination
        not_feasible = True
        while not_feasible:
            child1 = copy.deepcopy(self.gen_sols[p1_idx])
            child2 = copy.deepcopy(self.gen_sols[p2_idx])
            for i in range(len(child1)):
                for j in range(len(child1[i].u) - 1):
                    child1[i].t[j] = child1[i].get_T(j)
                    vi1 = child1[i].get_vi(j)
                    child1[i].v[j] = vi1

                    child2[i].t[j] = child2[i].get_T(j)
                    vi2 = child2[i].get_vi(j)
                    child2[i].v[j] = vi2

                    if j == len(child1[i].u) - 2:
                        child1[i].u[j] = (pow(Vm, 2) - pow(vi1, 2)) / (2 * (n - j) * dL)
                        child1[i].u[j + 1] = 0
                        child1[i].v[j + 1] = Vm
                        child1[i].t[j + 1] = child1[i].get_T(j + 1)

                        child2[i].u[j] = (pow(Vm, 2) - pow(vi2, 2)) / (2 * (n - j) * dL)
                        child2[i].u[j + 1] = 0
                        child2[i].v[j + 1] = Vm
                        child2[i].t[j + 1] = child2[i].get_T(j + 1)
                    else:
                        old_u1 = child1[i].u[j]
                        old_u2 = child2[i].u[j]
                        feasible_u1 = child1[i].min_max_feasible_u(vi1, j)
                        feasible_u2 = child2[i].min_max_feasible_u(vi2, j)
                        k = 0
                        while True:
                            if k < 50:
                                alpha = np.random.uniform(0, 1)

                                new_u1 = alpha * old_u1 + (1 - alpha) * old_u2
                                new_u2 = (1 - alpha) * old_u1 + alpha * old_u2
                            else:
                                k = 0

                                new_u1 = random.uniform(feasible_u1[0], feasible_u1[1])
                                new_u2 = random.uniform(feasible_u2[0], feasible_u2[1])

                            if new_u1 > Umax:
                                new_u1 = Umax
                            elif new_u1 < Umin:
                                new_u1 = Umin

                            if new_u2 > Umax:
                                new_u2 = Umax
                            elif new_u2 < Umin:
                                new_u2 = Umin

                            child1[i].u[j] = new_u1
                            child2[i].u[j] = new_u2
                            k = k + 1
                            if child1[i].is_velocity_valid(j + 1) and child2[i].is_velocity_valid(j + 1):
                                break
            violation1 = is_violation(child1)
            violation2 = is_violation(child2)
            not_feasible = len(violation1) != 0 or len(violation2) != 0
        return child1, child2

    def mutation(self, p_idx):
        # Random noise
        not_feasible = True
        while not_feasible:
            child = self.gen_sols[p_idx]
            for i in range(len(child)):
                for j in range(len(child[i].u) - 1):
                    vi = child[i].get_vi(j)
                    child[i].v[j] = vi
                    child[i].t[j] = child[i].get_T(j)
                    if j == len(child[i].u) - 2:
                        child[i].u[j] = (pow(Vm, 2) - pow(vi, 2)) / (2 * (n - j) * dL)
                        child[i].u[j + 1] = 0
                        child[i].v[j + 1] = Vm
                        child[i].t[j + 1] = child[i].get_T(j + 1)
                    else:
                        while True:
                            new_u = child[i].u[j] + random.uniform(max(-1, Umin - child[i].u[j]),
                                                                   min(1, Umax - child[i].u[j]))
                            child[i].u[j] = new_u
                            if child[i].is_velocity_valid(j + 1):
                                break
            violation = is_violation(child)
            not_feasible = len(violation) != 0

        return child

    def selectSurvivors(self, numSurvivors, criteria):
        selection_criteria = {
            "Age": lambda n: np.argsort(self.gen_ages)[:n],
            "Fitness": lambda n: np.argsort(self.gen_fvalues)[:n]
        }
        survivors_idx = selection_criteria.get(criteria, selection_criteria["Fitness"])(numSurvivors)
        return survivors_idx

    def perform_algorithm(self):
        self.initRandomPopulation()
        print("Best Initial Solution ", self.best_fvalues[0])
        n_crossovers = int(np.ceil(self.p_crossover * self.n_pop / 2))
        n_mutations = int(self.p_mutation * self.n_pop)
        n_elite = int(self.p_elite * self.n_pop)
        n_survivors = self.n_pop - int(self.p_crossover * self.n_pop) - n_mutations - n_elite
        for iter in range(self.max_iter):
            # Crossover and Parents Selection
            print("iteration: ", iter)
            parents_idx = self.selectParents(numParents=n_crossovers * 2, criteria=self.parents_selection)
            new_gen_sols = []
            new_gen_fvalues = []
            new_gen_ages = []
            for i in range(0, n_crossovers * 2, 2):
                [ch1, ch2] = self.crossover(parents_idx[i], parents_idx[i + 1])
                # print("Crossover done: ", i)
                new_gen_sols.append(ch1)
                new_gen_fvalues.append(cost_func(ch1))
                new_gen_ages.append(0)
                if len(new_gen_sols) == int(self.p_crossover * self.n_pop):
                    break
                new_gen_sols.append(ch2)
                new_gen_fvalues.append(cost_func(ch2))
                new_gen_ages.append(0)
            # Mutation and Parents Selection
            parents_idx = self.selectParents(numParents=n_mutations, criteria=self.mutation_selection)
            for i in range(n_mutations):
                ch = self.mutation(parents_idx[i])
                # print("Mutation done: ", i)
                new_gen_sols.append(ch)
                new_gen_fvalues.append(cost_func(ch))
                new_gen_ages.append(0)
            # Elite Members
            elite_idx = self.selectSurvivors(numSurvivors=n_elite, criteria="Fitness")
            for i in range(n_elite):
                new_gen_sols.append(self.gen_sols[elite_idx[i]])
                new_gen_fvalues.append(self.gen_fvalues[elite_idx[i]])
                new_gen_ages.append(self.gen_ages[elite_idx[i]] + 1)
            # Survivors (if any)
            survivors_idx = self.selectSurvivors(numSurvivors=n_survivors, criteria=self.survivors_selection)
            for i in range(n_survivors):
                new_gen_sols.append(self.gen_sols[survivors_idx[i]])
                new_gen_fvalues.append(self.gen_fvalues[survivors_idx[i]])
                new_gen_ages.append(self.gen_ages[survivors_idx[i]] + 1)
            assert (len(new_gen_sols) == self.n_pop)
            assert (len(new_gen_fvalues) == self.n_pop)
            assert (len(new_gen_ages) == self.n_pop)
            # New generation becomes current one
            self.gen_sols = new_gen_sols
            self.gen_fvalues = new_gen_fvalues
            self.gen_ages = new_gen_ages
            # update best solution reached so far
            best_idx = np.argmin(self.gen_fvalues)
            if self.gen_fvalues[best_idx] < self.best_fvalues[-1]:
                self.best_sols.append(self.gen_sols[best_idx])
                self.best_fvalues.append(self.gen_fvalues[best_idx])
            else:
                self.best_sols.append(self.best_sols[-1])
                self.best_fvalues.append(self.best_fvalues[-1])
            self.time_array.append(obj_func_time(self.best_sols[-1]))
            self.fuel_array.append(obj_func_fuel(self.best_sols[-1]))

    def visualize(self):
        # convergence plot
        plt.figure(1)
        plt.plot(np.arange(0, self.max_iter + 1), self.best_fvalues, label="Convergence Plot")
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
        solution = self.best_sols[-1]
        velocities = np.zeros(shape=(len(solution), len(solution[0].v)))
        for i in range(len(solution)):
            velocities[i] = solution[i].v
        file_path = 'velocities.mat'
        scipy.savemat(os.path.join(matlab_path, file_path), {str(variable_name): velocities})

    def toMatlabVelocities(self, variable_name):
        solution = self.best_sols[-1]
        velocities = np.zeros(shape=(len(solution), len(solution[0].v)))
        for i in range(len(solution)):
            velocities[i] = solution[i].v
        file_path = 'velocities.mat'
        scipy.savemat(os.path.join(matlab_path, file_path), {str(variable_name): velocities})

    def toMatlabArrivalTimes(self, variable_name):
        solution = self.best_sols[-1]
        arrival_times = np.zeros(shape=(len(solution)))
        for i in range(len(solution)):
            arrival_times[i] = solution[i].To
        file_path = 'arrivalTimes.mat'
        scipy.savemat(os.path.join(matlab_path, file_path), {str(variable_name): arrival_times})

    def toMatlabDirections(self, variable_name):
        solution = self.best_sols[-1]
        directions = np.zeros(shape=(len(solution)), dtype='object')
        for i in range(len(solution)):
            directions[i] = solution[i].direction
        file_path = 'directions.mat'
        scipy.savemat(os.path.join(matlab_path, file_path), {str(variable_name): directions})

    def toMatlabExitTimes(self, variable_name):
        solution = self.best_sols[-1]
        exitTimes = np.zeros(shape=(len(solution)))
        for i in range(len(solution)):
            exitTimes[i] = solution[i].t[-1] + (L + S) / Vm
        file_path = 'exitTimes.mat'
        scipy.savemat(os.path.join(matlab_path, file_path), {str(variable_name): exitTimes})


GA = GeneticAlgorithm()
GA.perform_algorithm()
print(GA.best_fvalues)
GA.visualize()
GA.toMatlabVelocities('velocities')
GA.toMatlabArrivalTimes('arrivalTimes')
GA.toMatlabDirections('directions')
GA.toMatlabExitTimes('exitTimes')

# CAVs1 = create_cars()
# CAVs2 = create_cars()
# generate_random_feasible_solution(CAVs1)
# generate_random_feasible_solution(CAVs2)
# print(CAVs1[3].u[2])
# crossover_test(CAVs1,CAVs2)
# print(CAVs1[3].u[2])