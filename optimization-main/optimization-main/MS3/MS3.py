import numpy as np
import random
import math
import plotly.graph_objects as go
import matplotlib.pyplot as plt

S = 50                    # Length of MZ
L = 300                     # Length of CZ
dL = 5                      # CZ step size
n = int(L/dL)               # no. of steps
Vmax = 25                 # Maximum velocity in CZ
Vmin = 5                  # Minimum velocity in cz
Umax = 10                  # Maximum acc in CZ
Umin = -10                 # Minimum acc in CZ
Vm = 25                   # MZ velocity
Td = 0.5                  # Response time of the brakes


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
                    old_u = child[i].u[j]
                    while True:
                        new_u = old_u + random.uniform(max(-1, Umin - old_u),
                                                               min(1, Umax - old_u))
                        child[i].u[j] = new_u
                        if child[i].is_velocity_valid(j + 1):
                            break
        violation = is_violation(child)
        not_feasible = len(violation) != 0
    return child


def crossover_test(child1o, child2o):
    # Whole Arithmetic Combination
    not_feasible = True
    while not_feasible:
        for i in range(len(child1)):
            for j in range(len(child1[i].u) - 1):
                child1[i].t[j] = child1[i].get_T(j)
                vi1 = child1[i].get_vi(j)
                child1[i].v[j] = vi1

                child2[i].t[j] = child2[i].get_T(j)
                vi2 = child2[i].get_vi(j)
                child2[i].v[j] = vi2

                if j == len(child1[i].u) - 2:
                    child1[i].u[i] = (pow(Vm, 2) - pow(vi1, 2)) / (2 * (n - i) * dL)
                    child1[i].u[j + 1] = 0
                    child1[i].v[j + 1] = Vm
                    child1[i].t[j + 1] = child1[i].get_T(i + 1)

                    child2[i].u[i] = (pow(Vm, 2) - pow(vi2, 2)) / (2 * (n - i) * dL)
                    child2[i].u[j + 1] = 0
                    child2[i].v[j + 1] = Vm
                    child2[i].t[j + 1] = child2[i].get_T(i + 1)
                else:
                    old_u1 = child1[i].u[j]
                    old_u2 = child2[i].u[j]
                    k = 1
                    feasible_u1 = min_max_feasible_u(vi1)
                    feasible_u2 = min_max_feasible_u(vi2)
                    # s = 0
                    while True:
                        # s = s+1
                        # print(s)
                        if k < 50:
                            alpha = np.random.uniform(0, 1)

                            new_u1 = alpha * old_u1 + (1 - alpha) * old_u2
                            new_u2 = (1 - alpha) * old_u1 + alpha * old_u2
                        else:
                            new_u1 = random.uniform(feasible_u1[0], feasible_u1[1])
                            new_u2 = random.uniform(feasible_u2[0], feasible_u2[1])

                        # print(old_u1, ",", old_u2, ",", new_u1, ",", new_u2)
                        print(feasible_u1[0], ",", feasible_u1[1], ",", new_u1)

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
                        k = k+1
                        # print(child1[i].is_velocity_valid(j + 1) and child2[i].is_velocity_valid(j + 1))
                        if child1[i].is_velocity_valid(j + 1) and child2[i].is_velocity_valid(j + 1):
                            break

        violation1 = is_violation(child1)
        violation2 = is_violation(child2)
        not_feasible = len(violation1) != 0 or len(violation2) != 0
    return child1, child2


def create_cars():
    return [
            CAV(0, "EW"),
            CAV(10, "EW"),
            CAV(15, "NS"),
            CAV(20, "NS"),
        ]


def min_max_feasible_u(vi):
    u_max = (pow(Vmax, 2) - pow(vi, 2))/(2*dL)
    u_min = (pow(Vmin, 2) - pow(vi, 2))/(2*dL)
    return [max(u_min, Umin), min(u_max, Umax)]

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
            if CAV.u[i]>Umax or CAV.u[i]<Umin or CAV.v[i]>Vmax  or CAV.v[i]<Vmin:
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
    return obj_func_fuel(CAVs) + obj_func_time(CAVs)

class CAV:
    def __init__(self,  To=0, direction="NS"):
        self.direction = direction
        self.To = To
        while True:
            self.Vo = random.uniform(Vmin, Vmax)
            if self.is_velocity_valid(0):
                break
        self.v = [-10]*(n+1)
        self.u = [-10]*(n+1)
        self.t = [-10]*(n+1)

    def sum_u(self, i):
        sum = 0
        for j in range(i+1):
            sum = sum + self.u[j]
        return sum

    def get_vi(self, i):
        if (self.Vo ** 2) + 2*dL*self.sum_u(i-1) > 0:
            return math.sqrt((self.Vo ** 2) + 2*dL*self.sum_u(i-1))
        else:
            print("hh")
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
        if (pow(self.Vo, 2)) + 2*dL*self.sum_u(i-1) < 0:
            return False
        vi = self.get_vi(i)
        if Vmin <= vi <= Vmax: #and Umin < (pow(Vm, 2)-pow(vi, 2))/(2*(n-i)*dL) < Umax:
            return True
        else:
            return False

    def generate_random_car_semi_feasible_solution(self):
        # Respects first two constraints
        for i in range(len(self.u)-1):
            vi = self.get_vi(i)
            self.v[i] = vi
            self.t[i] = self.get_T(i)
            if i == len(self.u)-2:
                self.u[i] = (pow(Vm, 2)-pow(vi, 2))/(2*(n-i)*dL)
                self.u[i+1] = 0
                self.v[i+1] = Vm
                self.t[i+1] = self.get_T(i+1)
            else:
                feasible_u = min_max_feasible_u(vi)
                while True:
                    self.u[i] = random.uniform(feasible_u[0], feasible_u[1])
                    if self.is_velocity_valid(i+1):
                        break
        return self

    def generate_random_car_semi_feasible_from_index(self, j):
        # Respects first two constraints
        for i in range(j, len(self.u)-1):
            vi = self.get_vi(i)
            self.v[i] = vi
            self.t[i] = self.get_T(i)
            if i == len(self.u)-2:
                self.u[i] = (pow(Vm, 2)-pow(vi, 2))/(2*(n-i)*dL)
                self.u[i+1] = 0
                self.v[i+1] = Vm
                self.t[i+1] = self.get_T(i+1)
            else:
                while True:
                    feasible_u = min_max_feasible_u(vi)
                    self.u[i] = random.uniform(feasible_u[0], feasible_u[1])
                    if self.is_velocity_valid(i+1):
                        break
        return self

    def calculate_v_and_t(self):
        for i in range(len(self.u)):
            vi = self.get_vi(i)
            self.v[i] = vi
            self.t[i] = self.get_T(i)


class GeneticAlgorithm:
    def __init__(self, problem, n_pop=50, max_iter=100, p_elite=0.1, p_crossover=0.8, p_mutation=0.1,
                 parents_selection="Random", tournament_size=5, mutation_selection="Worst",
                 survivors_selection="Fitness"):
        self.problem = problem
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

    def mutation(self, p_idx):
        # Random noise
        child = self.gen_sols[p_idx]
        not_feasible = True
        while not_feasible:
            for i in range(len(child)):
                for j in range(len(child[i].u)-1):
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
                            new_u = child[i].u[j] + random.uniform(max(-1, Umin - child[i].u[j]), min(1, Umax - child[i].u[j]))
                            child[i].u[j] = new_u
                            if child[i].is_velocity_valid(j+1):
                                break
            violation = is_violation(child)
            not_feasible = len(violation) != 0

        return child

    def crossover(self, p1_idx, p2_idx):
        # Whole Arithmetic Combination
        child1 = self.gen_sols[p1_idx]
        child2 = self.gen_sols[p2_idx]
        not_feasible = True
        while not_feasible:
            for i in range(len(child1)):
                for j in range(len(child1[i].u) - 1):
                    child1[i].t[j] = child1[i].get_T(i)
                    vi1 = child1[i].get_vi(j)
                    child1[i].v[j] = vi1

                    child2[i].t[j] = child2[i].get_T(i)
                    vi2 = child2[i].get_vi(j)
                    child2[i].v[j] = vi2

                    if j == len(child1[i].u) - 2:
                        child1[i].u[i] = (pow(Vm, 2) - pow(vi1, 2)) / (2 * (n - i) * dL)
                        child1[i].u[j + 1] = 0
                        child1[i].v[j + 1] = Vm
                        child1[i].t[j + 1] = child1[i].get_T(i + 1)

                        child2[i].u[i] = (pow(Vm, 2) - pow(vi2, 2)) / (2 * (n - i) * dL)
                        child2[i].u[j + 1] = 0
                        child2[i].v[j + 1] = Vm
                        child2[i].t[j + 1] = child2[i].get_T(i + 1)
                    else:
                        old_u1 = child1[i].u[j]
                        old_u2 = child2[i].u[j]
                        k = 1
                        feasible_u1 = min_max_feasible_u(vi1)
                        feasible_u2 = min_max_feasible_u(vi2)
                        while True:
                            if k < 50:
                                alpha = np.random.uniform(0, 1)

                                new_u1 = alpha * old_u1 + (1 - alpha) * old_u2
                                new_u2 = (1 - alpha) * old_u1 + alpha * old_u2
                            else:
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



child1 = create_cars()
child2 = create_cars()

generate_random_feasible_solution(child1)
generate_random_feasible_solution(child2)

child1, child2 = crossover_test(child1, child2)

# for CAV in CAVs:
#     print(CAV.t[-1])
#
# print(obj_func_time(CAVs))
# print(obj_func_fuel(CAVs))