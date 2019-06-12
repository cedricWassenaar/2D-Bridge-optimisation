import numpy as np
import math
import Graphics
from typing import List
import json
from scipy.optimize import fmin_powell
Vector = List[float]
import time

class Node (object):
    """A object that defines a position"""
    def __init__(self, name: str, pos, constraint_x=0, constraint_y=0):
        """Node: has a name, position and constraints. The loads are are added when the distributed weight is placed
        on the beam. A optional value is optimize, for each dimension the position of the node can be optimized
        t optimise the construction"""
        self.name: str = name
        self.pos = np.array(pos)
        self.load: Vector = np.array([0, 0])
        self.load_list = np.array([0])
        self.constraint_x = constraint_x
        self.constraint_y = constraint_y
        self.optimize: List = np.array([0, 0])

    def __str__(self):
        text: str = self.name
        text += ": " + str(self.pos)
        return text


class Beam (object):
    """A beam or rod that is positioned between two nodes
    A beam knows the two nodes it is placed between and therefore its length,
    with other data as density and cross-section area the weight can be determined,
    the placed load is divided for the two nodes."""
    def __init__(self, name: str, nodes, v_load, a, b):
        self.name: str = name
        self.length: float = self.absolute(nodes[a].pos-nodes[b].pos)
        self.a_node = a
        self.b_node = b
        self.pos1: Vector = nodes[a].pos
        self.pos2: Vector = nodes[b].pos
        self.load: Vector = np.array(v_load)
        self.load_nodes: Vector = 0.5 * np.array(v_load) * self.length
        self.delta_0: Vector = nodes[a].pos-nodes[b].pos
        self.delta_1: Vector = nodes[b].pos - nodes[a].pos
        self.angle_0: float = math.atan2(self.delta_0[1], self.delta_0[0])
        self.angle_1: float = math.atan2(self.delta_1[1], self.delta_1[0])
        self.area = 0.10
        self.E_modulus = 210 * 1e+9
        self.density = 7850
        self.yield_strength = 250 * 1e+6
        self.internal_force = 0
        self.weight = 0.0
        self.connections = np.zeros(len(2 * nodes))
        self.connections[2 * a] = math.cos(self.angle_0)
        self.connections[2 * a + 1] = math.sin(self.angle_0)
        self.connections[2 * b] = math.cos(self.angle_1)
        self.connections[2 * b + 1] = math.sin(self.angle_1)

    @staticmethod
    def absolute(arr):
        """Return the absolute length of a vector"""
        return np.linalg.norm(arr)

    def calculate_beam_weight(self, new_force):
        """
        calculates weight of a beam using the internal force of the beam and yield strength of the material
        :param new_force:
        :return: -
        """
        self.internal_force = abs(new_force)
        if new_force >= 0:
            # Force is stretching beam
            self.area = self.internal_force / self.yield_strength
        else:
            # Force is compressing beam
            self.area = math.pow(((self.internal_force * (0.5 * self.length) ** 2 / (
                math.pi ** 2 * self.E_modulus)) / (math.pi / 4)), 1 / 2) * math.pi
        self.weight = self.area * self.length * self.density

    def __str__(self):
        """
        Overwrites str method, prints important data of the beam
        :return text:
        """
        text: str = "\n"
        text += "Beam: " + self.name + "\n"
        text += "\tLength: {0:.2f} m\n".format(round(self.length, 2))
        text += "\tArea: {0:.2f} mm²\n".format(round(self.area * 1e6, 2))
        text += "\tWeight: {0:.3f} kg\n".format(round(self.weight, 3))
        return text

    def single_line(self):
        text: str = self.name
        text += ": {0:.2f}m".format(round(self.length, 2))
        text += ", {0:.2f}mm²".format(round(self.area * 1e6, 2))
        text += ", {0:.3f}kg".format(round(self.weight, 3))
        return text


class Construction(object):
    def __init__(self, name: str, nodes: List, beam_list: List, load_list: List):
        """
        Creates a construction with the given nodes, beam, loads and constraints
        :param name:
        :param nodes:
        :param beam_list:
        """
        self.temp_beams = beam_list
        self.materials = {}
        self.material: str = ""
        self.name: str = name
        self.window = Graphics.Construction("Bridge 1", 1280, 720)
        self.nodes: List = nodes
        self.beams: List = []
        self.current_loads = 0
        self.load_list = load_list
        self.beams = []
        self.last_iteration = False
        self.max_beams = []
        self.set_beams()
        self.optional_loads: List = []
        self.iteration = 0

        # Declare later used data
        self.matrix = []
        self.B = []
        self.X = []
        self.weight = np.inf
        self.get_materials()
        self.inter_plot = False
        print("Construction created...")

    def set_beams(self):
        """
        Rebuilds all beams between the nodes with the new values
        :return:
        """
        self.beams = []
        for x in range(0, len(self.temp_beams)):
            self.beams.append(Beam(str(self.temp_beams[x][0]),
                                   self.nodes,
                                   self.load_list[self.current_loads][x],
                                   self.temp_beams[x][1],
                                   self.temp_beams[x][2]))

    def optimize(self, active=True, inter_plot=True):
        """
        Optimize will generate a construction with minimal weight for the load that is given
        Optional: active will activate the minimization function to create a highly optimized construction
        :param active:
        :param inter_plot:
        :return:
        """
        self.inter_plot = inter_plot
        initial_guess = []
        for x in range(0, len(self.nodes)):
            if not np.any(self.nodes[x].optimize):
                continue
            for val in range(0, len(self.nodes[x].optimize)):
                if self.nodes[x].optimize[val] != 0:
                    initial_guess.append(self.nodes[x].pos[val])

        initial_guess = np.array(initial_guess)
        print("Initial Guess", initial_guess)
        print("Calculating Construction....")
        constructions_weights = []
        load_nr_max_weight = []
        results = []
        self.max_beams = []
        for a in range(0, len(self.load_list)):
            # Loop through all loads
            self.current_loads = a
            print("\n\nCalculating construction for load: ", self.current_loads)
            # Create optimal for current load
            if active:
                result = fmin_powell(self.set_and_calculate, initial_guess, xtol=0.01, ftol=0.005)
            else:
                result = self.set_and_calculate(initial_guess)
            self.plot_construction()
            constructions_weights.append(self.weight)
            load_nr_max_weight.append(a)
            results.append(result)
            self.max_beams.append(self.beams)
            for y in range(0, len(self.load_list)):
                # Make construction strong so that current optimal can hold all loads
                if a == y:
                    continue
                self.current_loads = y
                self.set_and_calculate(result)
                for t in range(0, len(self.beams)):
                    if self.max_beams[a][t].weight < self.beams[t].weight:
                        self.max_beams[a][t] = self.beams[t]
                # Calculate the weight of current strong optimal
                self.weight = 0
                for t in range(0, len(self.beams)):
                    self.beams[t] = self.max_beams[a][t]
                    self.weight += self.beams[t].weight

                if self.weight > constructions_weights[a]:
                    constructions_weights[a] = self.weight
                    load_nr_max_weight[a] = y

        minimum = min(constructions_weights)
        load_index = constructions_weights.index(minimum)
        self.current_loads = load_nr_max_weight[load_index]
        self.set_and_calculate(results[load_index])
        self.beams = self.max_beams[load_index]
        self.weight = minimum
        print("\n\nThe best weight for all loads is:", minimum, "kg")

        print("This is bridge is optimized for load nr: ", load_index)
        self.plot_construction(finished=True)
        while True:
            self.window.hold()

    def set_and_calculate(self, new_values):
        """
        Sets the variable positions, rebuilds all the beams and calculates the weight of the construction
        :return:
        """
        self.iteration += 1
        t = 0
        for x in range(0, len(self.nodes)):
            if not np.any(self.nodes[x].optimize):
                continue
            for val in range(0, len(self.nodes[x].optimize)):
                if self.nodes[x].optimize[val] != 0:
                    self.nodes[x].pos[val] = new_values[t]
                    t += 1
        self.set_beams()
        self.get_weight()
        if self.inter_plot:
            try:
                self.plot_construction()
            except:
                print("\nWarning plot failed \n")
        return self.weight

    def get_weight(self):
        lightest_weight = np.inf
        best_material = {}
        for material in self.materials:
            self.set_material(self.materials[material])
            self.calculate_weight()
            if self.weight < lightest_weight:
                best_material = material
                lightest_weight = self.weight

        self.set_material(self.materials[best_material])
        self.material = str(best_material)
        self.calculate_weight()

    def get_max_beams(self):
        pass

    def calculate_weight(self):
        """
        Calculates the weight of each beam and the total weight of the construction using linear algebra
        :return:
        """
        self.matrix = []
        for x in range(0, len(self.beams)):
            self.matrix.append(self.beams[x].connections)

        self.matrix = np.array(self.matrix)
        self.matrix = self.matrix.transpose()

        size = np.shape(self.matrix)
        missing = size[0] - size[1]
        for x in range(0, missing):
            zeros = np.array([np.zeros(size[0])])
            self.matrix = np.concatenate((self.matrix, zeros.T), axis=1)

        t = size[1]
        for x in range(0, len(self.nodes)):
            if self.nodes[x].constraint_x != 0:
                self.matrix[2 * x][t] = self.nodes[x].constraint_x
                t += 1
            if self.nodes[x].constraint_y != 0:
                self.matrix[2 * x + 1][t] = self.nodes[x].constraint_y
                t += 1

        self.B = np.zeros(np.shape(self.matrix)[0])
        for x in range(0, len(self.nodes)):
            self.nodes[x].load = np.array([0, 0])

        for x in range(0, len(self.beams)):
            self.nodes[self.beams[x].a_node].load = \
                self.nodes[self.beams[x].a_node].load + self.beams[x].load_nodes
            self.nodes[self.beams[x].b_node].load = \
                self.nodes[self.beams[x].b_node].load + self.beams[x].load_nodes

        for x in range(0, len(self.nodes)):
            self.B[2 * x] = self.nodes[x].load[0]
            self.B[2 * x + 1] = self.nodes[x].load[1]

        self.weight = 0
        try:
            self.X = np.dot(np.linalg.inv(self.matrix), self.B)
        except np.linalg.linalg.LinAlgError:
            print("\nWarning linear algebra Error\n")
            self.X = np.full(size[0], 1e20)

        for x in range(0, len(self.beams)):
            self.beams[x].calculate_beam_weight(self.X[x])
            self.weight += self.beams[x].weight

        return self.weight

    def set_material(self, current_material: dict):
        """Sets the currently selected material"""
        for beam in self.beams:
            beam.yield_strength = current_material["yield_strength"]
            beam.E_modulus = current_material["E_modulus"]
            beam.density = current_material["density"]

    def get_materials(self):
        """Gets all available materials from the materials.json dictionary"""
        with open("materials.json", "r") as read_file:
            self.materials = json.load(read_file)
        read_file.close()
        self.set_material(self.materials[list(self.materials.keys())[0]])

    def __str__(self):
        """Overwritten method to print its data in a certain format when using print() or str()"""
        text: str = "\n  "
        text += "\nA =\n" + str(self.matrix)
        text += "\n\nB = \n" + str(self.B)
        text += "\n\nX = \n" + str(self.X)
        text += "\n\n\t  "

        for x in range(0, len(self.beams)):
            text += str(self.beams[x])

        text += "\n\nTotal weight bridge: {0:.3f} kg\n".format(round(self.weight, 3))
        return text

    def plot_construction(self, finished=False):
        offset: Vector = (200, 400)

        def inv(pos: Vector):
            pos: Vector = pos * np.array([1, -1])  # invert y-axis for graphics
            pos: Vector = pos * 200 + offset
            return pos

        for beam in self.beams:
            self.window.draw_beam(beam.name,
                                  inv(beam.pos1),
                                  inv(beam.pos2),
                                  beam.internal_force,
                                  size=int((beam.area * 1e6)**0.7))

        for node in self.nodes:
            self.window.draw_node(node.name, inv(node.pos))
            self.window.draw_force(node.name, inv(node.pos), node.load)
            if node.constraint_x != 0:
                self.window.draw_constraint_x(node.name + "x", inv(node.pos))
            if node.constraint_y != 0:
                self.window.draw_constraint_y(node.name + "y", inv(node.pos))
            if np.any(node.optimize):
                self.window.draw_editable(inv(node.pos))

        self.window.add_text((50, 50), "Weight: {0:.3f} kg".format(round(self.weight, 3)))
        self.window.add_text((50, 70), "Material: " + self.material)
        self.window.add_text((50, 90), "Iteration: " + str(self.iteration))
        if finished:
            self.window.add_text((50, 30), "OPTIMAL SOLUTION FOUND: ")
            self.window.add_text((50, 520), "NODES: ")
            for x in range(0, len(self.nodes)):
                b = 50 + (x // 5) * 150
                h = (x % 5) * 30 + 550
                self.window.add_text((b, h), str(self.nodes[x]))
                self.window.add_text((400, 520), "BEAMS: ")
            for x in range(0, len(self.beams)):
                b = 400 + (x // 5) * 300
                h = (x % 5) * 30 + 550
                self.window.add_text((b, h), self.beams[x].single_line())
        self.window.show()


if __name__ == "__main__":
    np.set_printoptions(precision=2)
    scale: float = 1  # meter
    load: float = 1000  # Newton

    # A list of all the nodes in the construction
    o_nodes = [
        Node("A", (0.00001, 0.00001), constraint_x=-1, constraint_y=-1),
        Node("B", (1.00001 * scale, 0.00001)),
        Node("C", (1.99999 * scale, 0.00001)),
        Node("D", (3.00001 * scale, 0.00001)),
        Node("E", (4.00001 * scale, 0.00001), constraint_y=-1),
        Node("F", (3.00002 * scale, 1.00002 * scale)),
        Node("G", (2.00001 * scale, 1.000001 * scale)),
        Node("H", (1.00003 * scale, 1.00003 * scale))
    ]

    # A list of all the beams or rods that connect to certain nodes
    o_beams = [
        ["AB", 0, 1],
        ["AH", 0, 7],
        ["BC", 1, 2],
        ["BH", 1, 7],
        ["BG", 1, 6],
        ["CD", 2, 3],
        ["CG", 2, 6],
        ["DE", 3, 4],
        ["DF", 3, 5],
        ["DG", 3, 6],
        ["EF", 4, 5],
        ["FG", 5, 6],
        ["GH", 6, 7],
    ]

    # A list of all the different loads placed on the beams
    o_loads = [
                    [
                        [0, -1 * load],
                        [0, 0],
                        [0, -1 * load],
                        [0, 0],
                        [0, 0],
                        [0, -1 * load],
                        [0, 0],
                        [0, -1 * load],
                        [0, 0],
                        [0, 0],
                        [0, 0],
                        [0, 0],
                        [0, 0]
                    ]
                    ,
                    [
                        [0, -2 * load],
                        [0, 0],
                        [0, -1 * load],
                        [0, 0],
                        [0, 0],
                        [0, -0.5 * load],
                        [0, 0],
                        [0, -1 * load],
                        [0, 0],
                        [0, 0],
                        [0, 0],
                        [0, 0],
                        [0, 0]
                    ]
                    ,
                    [
                        [0, -3 * load],
                        [0, 0],
                        [0, -1 * load],
                        [0, 0],
                        [0, 0],
                        [0, -4 * load],
                        [0, 0],
                        [0, -1 * load],
                        [0, 0],
                        [0, 0],
                        [0, 0],
                        [0, 0],
                        [0, 0]
                    ]


                ]

    # All dimensions of the nodes that will be optimized are given a 1 value
    o_nodes[1].optimize = np.array([1, 0])
    o_nodes[2].optimize = np.array([1, 0])
    o_nodes[3].optimize = np.array([1, 0])
    o_nodes[5].optimize = np.array([1, 1])
    o_nodes[6].optimize = np.array([1, 1])
    o_nodes[7].optimize = np.array([1, 1])

    # Creates a construction with the given nodes and beams
    bridge_1 = Construction("Bridge 1", o_nodes, o_beams, o_loads)

    # The bridge is calculated for most optimal weight/load ratio
    bridge_1.optimize(active=True, inter_plot=True)
    print(bridge_1)
