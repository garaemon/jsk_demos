#!/usr/bin/env python
import csv
import numpy as np
from scipy import signal, interpolate
import matplotlib as mpl
import matplotlib.pyplot as plt

from rospkg import RosPack
import os
def resolve_ros_path(path):
    if path.startswith("package://"):
        package_name = path.split("/")[2]
        rest_path = path.split("/")[3:]
        rp = RosPack()
        pkg_path = rp.get_path(package_name)
        return os.path.join(*([pkg_path] + rest_path))
    else:
        return path

class QualityTable():
    def __init__(self, name, file_name, epsilon_scale):
        self.name = name
        self.file_name = resolve_ros_path(file_name)
        self.epsilon_scale = epsilon_scale
        # read value
        # print "Reading", file_name
        with open(self.file_name, "r") as f:
            reader = csv.reader(f)
            self.labels = reader.next()
            # self.data["epsilon"] => [0, 0.1, ..., 1.0]
            self.data = dict()
            for label in self.labels:
                self.data[label] = []
            for row in reader:
                for i in range(len(self.labels)):
                    label = self.labels[i]
                    self.data[label].append(float(row[i]))
        # initialize linear interpolator
        sigmas = np.array(self.data['sigma'])
        epsilons = np.array(self.data['epsilon'])
        keys = sorted(range(0, len(sigmas)), key=lambda x: sigmas[x])
        self.sigmas_sorted = np.array([sigmas[x] for x in keys])
        self.epsilons_sorted = np.array([epsilons[x] for x in keys])
        self.sigma2epsilon_interpolation = interpolate.interp1d(
            self.sigmas_sorted, self.epsilons_sorted, kind="linear")
        self.epsilon2sigma_interpolation = interpolate.interp1d(
            self.epsilons_sorted, self.sigmas_sorted, kind="linear")
    def plot(self, c):
        range = np.linspace(self.min_sigma(), self.max_sigma(), 100)
        plt.plot(range, self.epsilon_scale * self.sigma2epsilon_interpolation(range),
                 label='${1:.2f}{0}$'.format(self.name, self.epsilon_scale),
                 c=c)
    def min_sigma(self):
        return self.sigmas_sorted[0]
    def max_sigma(self):
        return self.sigmas_sorted[-1]
    def min_epsilon(self):
        return self.epsilon_scale * self.epsilons_sorted[0]
    def max_epsilon(self):
        return self.epsilon_scale * self.epsilons_sorted[-1]
    def sigma2epsilon(self, sigma):
        return self.epsilon_scale * self.sigma2epsilon_interpolation(sigma)
    def epsilon2sigma(self, epsilon):
        return 1.0 / self.epsilon_scale * self.epsilon2sigma_interpolation(epsilon)
    def delta(self, sigma, delta_sigma):
        return self.sigma2epsilon(sigma + delta_sigma) - self.sigma2epsilon(sigma)
    def other_params(self):
        return [label for label in self.labels if label != "sigma" and label != "epsilon"]
    def lookup_value(self, from_label, from_value, target_label):
        # run linear interpolation
        from_data = np.array(self.data[from_label])
        to_data = np.array(self.data[target_label])
        keys = sorted(range(0, len(from_data)), key=lambda x: from_data[x])
        sorted_from_data = np.array([from_data[x] for x in keys])
        sorted_to_data = np.array([to_data[x] for x in keys])
        inter = interpolate.interp1d(
            sorted_from_data, sorted_to_data, kind="linear")
        return inter(from_value)
    def lookup_value2(self, from_label1, from_label2, from_value1, from_value2, target_label):
        # run linear interpolation
        for a, b, c in zip(self.data[from_label1], self.data[from_label2], self.data[target_label]):
            if a == from_value1 and b == from_value2:
                return c

class OptimizationContainer():
    def __init__(self):
        self.iteration_times = 0
        self.tables = []
        self.initial_sigmas = []
        self.current_sigmas = []
        self.tracking_sigmas = []
    def register_quality_table(self, table, initial_sigma):
        self.tables.append(table)
        self.initial_sigmas.append(initial_sigma)
        self.current_sigmas.append(initial_sigma)
        self.tracking_sigmas.append([initial_sigma])
    def possible_min_sigma(self):
        return sum([t.min_sigma() for t in self.tables])
    def current_sigma(self):
        return sum(self.current_sigmas)
    def current_epsilon(self):
        return sum([t.sigma2epsilon(s) for t, s in zip(self.tables, self.current_sigmas)])
    def initial_sigma(self):
        return sum(self.initial_sigmas)
    def initial_epsilon(self):
        return sum([t.sigma2epsilon(s) for t, s in zip(self.tables, self.initial_sigmas)])
    def max_epsilon(self):
        return sum([t.max_epsilon() for t in self.tables])
    def is_sigma_bigger(self, sigma):
        return self.current_sigma() < sigma
    def setDirectionPositve(self):
        self.direction = 1
    def setDirectionNegative(self):
        self.direction = -1
    def delta_sigma(self):
        # return 0.1 * self.direction
        return 0.3 * self.direction
    def is_converged(self, sigma_d):
        if self.direction == 1:
            return sigma_d < self.current_sigma()
        else:
            return sigma_d > self.current_sigma()
    def is_sigma_valid(self, tp, sigma):
        for t in self.tables:
            if t.name == tp:
                return sigma >= t.min_sigma() and sigma < t.max_sigma()
    def proc(self):
        self.iteration_times = self.iteration_times + 1
        # print "k=", self.iteration_times
        delta_sigma = self.delta_sigma()
        # check is valid sigma
        ds = []
        is_valid = False
        for t, current_sigma in zip(self.tables, self.current_sigmas):
            if self.is_sigma_valid(t.name, current_sigma + delta_sigma):
                d = t.delta(current_sigma, delta_sigma)
                ds.append(abs(d))
                is_valid = True
            else:
                if self.direction < 0:
                    ds.append(1000.0) # large enough value
                else:
                    ds.append(-100.0)
        if not is_valid:
            # print "already converged"
            return False
        keys = sorted(range(0, len(ds)), key=lambda x: ds[x])
        if self.direction < 0:
            min_d = ds[keys[0]]
            min_table = self.tables[keys[0]]
            self.current_sigmas[keys[0]] = self.current_sigmas[keys[0]] + delta_sigma
            self.tracking_sigmas[keys[0]].append(self.current_sigmas[keys[0]])
        else:
            min_d = ds[keys[-1]]
            min_table = self.tables[keys[-1]]
            self.current_sigmas[keys[-1]] = self.current_sigmas[keys[-1]] + delta_sigma
            self.tracking_sigmas[keys[-1]].append(self.current_sigmas[keys[-1]])
        # print "choose", min_table.name
        return True
    def printOverview2(self, deadline):
        print "{0},{1}".format(self.current_sigma(), self.current_epsilon())
    def printOverview(self, deadline):
        print "initial condition:"
        print "  all sigma:", self.initial_sigma()
        print "  all epsilon:", self.initial_epsilon()
        for t, s in zip(self.tables, self.initial_sigmas):
            print "  " + t.name
            print "    sigma:", s
            print "    epsilon:", t.sigma2epsilon(s)
            for p in t.other_params():
                print "    {0}: {1}".format(p, t.lookup_value("sigma",s, p))
        print "deadline sigma:", deadline
        print "minimum sigma:", sum([t.min_sigma() for t in self.tables])
        print "final result:"
        print "  all sigma:", self.current_sigma()
        print "  all epsilon:", self.current_epsilon()
        print "  rel epsilon", self.current_epsilon() / self.max_epsilon()
        for t, s in zip(self.tables, self.current_sigmas):
            print "  " + t.name
            print "    sigma:", s
            print "    epsilon:", t.sigma2epsilon(s)
            for p in t.other_params():
                print "    {0}: {1}".format(p, t.lookup_value("sigma",s, p))
    def draw(self, ax):
        ax.cla()
        ax.set_xlabel('$t$', fontsize=22)
        ax.set_ylabel('$q$', fontsize=22)
        max_sigma = max([t.max_sigma() for t in self.tables])
        max_epsilon = max([t.max_epsilon() for t in self.tables])
        for t, current_sigma, trackings, c in zip(self.tables, self.current_sigmas,
                                                  self.tracking_sigmas,
                                                  ["#1f77b4",
                                                   "#ff7f0e",
                                                   "#2ca02c",
                                                   "#d62728",
                                                   "#9467bd",
                                                   "#8c564b",
                                                   "#e377c2",
                                                   "#7f7f7f",
                                                   "#bcbd22",
                                                   "#17becf"]):
            current_epsilon = t.sigma2epsilon(current_sigma)
            t.plot(c)
            plt.plot(trackings,
                     t.sigma2epsilon(trackings),
                     'o', c=c)
        ax.set_xlim(0, max_sigma)
        ax.set_ylim(0, max_epsilon)
        ax.grid()
        plt.legend(loc=4)
        # plt.text(max_sigma * 5.0 / 8, max_epsilon * 0.1,
        #          "$k={2}$\n$\epsilon = {0:.1f}$\n$\sigma = {1:.1f}$".format(self.current_epsilon(), self.current_sigma(),
        #                                                                     self.iteration_times),
        #          ha='left', va='center')
        plt.draw()

        
