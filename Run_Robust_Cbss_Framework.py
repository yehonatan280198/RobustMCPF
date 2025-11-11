import math
from collections import defaultdict
from queue import PriorityQueue

from FindConflict import FindConflict
from LowLevelPlan import LowLevelPlan
from NodeStateConstClasses import Node
from Verify import Verify
from kBestSequencing import kBestSequencing
from kBestSequencingWithGLKH import kBestSequencingWithGLKH


class Robust_Cbss_framework:

    def __init__(self, Positions, GoalLocations, no_collision_prob, delaysProb, MapAndDims, verifyAlpha, algorithm, configStr):
        self.Positions = Positions  # Initial positions of agents
        self.GoalLocations = GoalLocations  # Locations of goals

        self.algorithm = algorithm
        self.ResolvedConflicts = 0

        self.OPEN = PriorityQueue()  # Open list for CBS nodes, prioritized by cost
        self.Num_roots_generated = 0  # Counter for the number of root nodes generated
        self.K_optimal_sequences = {}  # Dictionary to store k-optimal sequences of allocations

        if algorithm in ["RCbssEff", "IDP", "IRC"]:
            self.K_Best_Seq_Solver = kBestSequencingWithGLKH(self.Positions, self.GoalLocations, MapAndDims, configStr)
        else:
            self.K_Best_Seq_Solver = kBestSequencing(self.Positions, self.GoalLocations, MapAndDims, configStr)

        self.LowLevelPlanner = LowLevelPlan(MapAndDims, self.Positions, self.K_Best_Seq_Solver.cost_without_rotations, algorithm)
        self.verify_algorithm = Verify(delaysProb, no_collision_prob, verifyAlpha, algorithm)
        self.findConflict_algorithm = FindConflict(algorithm)

        self.Solution = self.run()

    ####################################################### run ############################################################

    def run(self):
        # Calculate the best sequence of task allocations (k=1)
        self.K_optimal_sequences[1] = self.K_Best_Seq_Solver.find_k_best_solution(k=1)
        # Increment root node counter
        self.Num_roots_generated += 1

        # Create the root node
        Root = Node()
        # Assign the best sequence of task allocations for all agents to the root node
        Root.sequence = self.K_optimal_sequences[1]
        # Generate paths and calculate the cost for the root node
        self.LowLevelPlanner.runLowLevelPlan(Root, list(range(len(self.Positions))))

        # Add the root node to the open list
        self.OPEN.put((Root.g, Root))

        # Continue processing nodes in the open list until it is empty
        while not self.OPEN.empty():

            # Get the node with the lowest cost
            _, N = self.OPEN.get()
            # Check if a new root needs to be generated
            N = self.CheckNewRoot(N)
            if N is None:
                continue

            # If the paths in the current node are verified as valid, avoiding collisions with probability P, return them as the solution
            if (not N.isPositiveNode) and self.verify_algorithm.verify(N.paths):
                return [N.paths, self.K_Best_Seq_Solver.Counter_Solver_TSP_For_Test,
                        self.LowLevelPlanner.Counter_LowLevel_For_Test, self.Num_roots_generated, self.ResolvedConflicts, N.g]

            # Identify the first conflict in the paths
            conflict = self.findConflict_algorithm.findConflict(N)
            if conflict is None:
                continue
            else:
                self.ResolvedConflicts += 1
                _, _, _, x, agent1AndTime, agent2AndTime = conflict

            # Generate child nodes with constraints to resolve the conflict and add child nodes to the open list
            if agent1AndTime[1] != 0:
                A1 = self.GenChild(N, (agent1AndTime[0], x, agent1AndTime[1]))
                if A1 is not None:
                    self.OPEN.put((A1.g, A1))

            if agent2AndTime[1] != 0:
                A2 = self.GenChild(N, (agent2AndTime[0], x, agent2AndTime[1]))
                if A2 is not None:
                    self.OPEN.put((A2.g, A2))

            if self.algorithm != "IDP":
                A3 = self.GenChild(N, (agent1AndTime[0], agent2AndTime[0], x, agent1AndTime[1], agent2AndTime[1]))
                self.OPEN.put((A3.g, A3))

    ####################################################### Check new root ############################################################

    def CheckNewRoot(self, N):
        # print(f"{N.g}, {self.K_optimal_sequences[self.Num_roots_generated]['Cost']}")

        # If the current node cost is within the threshold of the current optimal sequence
        if N.g <= self.K_optimal_sequences[self.Num_roots_generated]["Cost"]:
            return N

        # Generate a new root with an updated sequence
        self.Num_roots_generated += 1
        self.K_optimal_sequences[self.Num_roots_generated] = self.K_Best_Seq_Solver.find_k_best_solution(
            k=self.Num_roots_generated)

        if self.K_optimal_sequences[self.Num_roots_generated]["Cost"] == math.inf:
            return N

        # Create a new root node
        newRoot = Node()
        newRoot.sequence = self.K_optimal_sequences[self.Num_roots_generated]
        # Calculate paths and cost for the new root
        self.LowLevelPlanner.runLowLevelPlan(newRoot, list(range(len(self.Positions))))

        self.OPEN.put((newRoot.g, newRoot))
        self.OPEN.put((N.g, N))
        return None

    ####################################################### Get conflict ############################################################

    def GenChild(self, N, NewCons):
        A = Node()
        A.negConstraints = defaultdict(set, {agent: constraints.copy() for agent, constraints in N.negConstraints.items()})
        A.posConstraints = defaultdict(set, {agent: constraints.copy() for agent, constraints in N.posConstraints.items()})
        A.paths = defaultdict(list, {agent: path[:] for agent, path in N.paths.items()})
        A.sequence = N.sequence
        A.g = N.g

        if len(NewCons) == 3:
            agent, _, _ = NewCons
            A.negConstraints[agent].add(NewCons)
            if not self.LowLevelPlanner.runLowLevelPlan(A, [agent]):
                return None

        else:
            A.isPositiveNode = True
            agent1, agent2, _, _, _ = NewCons
            A.posConstraints[agent1].add(NewCons)
            A.posConstraints[agent2].add(NewCons)

        return A
