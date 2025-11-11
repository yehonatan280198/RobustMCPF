import math
import os
import subprocess
from collections import defaultdict
import numpy as np
from queue import PriorityQueue
from collections import deque


####################################################### generate egtsp Parameters file ############################################################
def generate_EGTSP_parameters_file(configStr):
    lines = [
        f"PROBLEM_FILE = {os.getcwd()}/EGSTP_runtime_files/{configStr}_mEgtsp.gtsp\n",
        f"RUNS = 10\n",
        f"OUTPUT_TOUR_FILE = {os.getcwd()}/EGSTP_runtime_files/{configStr}_mEgtsp.tour\n"
    ]

    with open(f"{os.getcwd()}/EGSTP_runtime_files/{configStr}_mEgtsp.par", mode="w+") as filePar:
        filePar.writelines(lines)


####################################################### Create 4 copies of specific goal ############################################################
def create_copy_of_goals(GoalLocations):
    return [(goal, direct) for goal in GoalLocations for direct in range(4)]


######################################################### kBestSequencingWithGLKH class ###############################################################
class kBestSequencingWithGLKH:
    def __init__(self, Positions, GoalLocations, dict_of_map_and_dim, configStr):
        self.Positions = Positions
        self.GoalLocations = GoalLocations
        self.AllCopyOfGoals = create_copy_of_goals(GoalLocations)
        self.AllPosAndGoals = self.Positions + self.AllCopyOfGoals
        self.configStr = configStr

        self.MapAndDims = dict_of_map_and_dim
        self.OPEN = PriorityQueue()
        self.Solutions = {}

        self.Counter_Solver_TSP_For_Test = 0

        self.cost_without_rotations, self.cost_with_rotations = self.precompute_costs()

        generate_EGTSP_parameters_file(self.configStr)

    ######################################################### Find K Best Solution ###############################################################

    def find_k_best_solution(self, k):
        # If k is 1, find and return the best (first) solution
        if k == 1:
            # Save the first allocation along with the include/exclude sets
            self.Solutions[k] = (set(), set(), self.solve_tsp_with_constraints(set(), set()))
            # Return optimal allocation
            return self.Solutions[k][2]

        # Retrieve the previous solution for k-1
        includeE, excludeE, optimalSequences = self.Solutions[k - 1]

        # Iterate over the edges in the current solution's allocation
        for index, (v0, v1) in enumerate(optimalSequences["Alloc_edges"]):
            # Create a new include set by adding edges up to the current index
            newIncludeE = includeE | set(optimalSequences["Alloc_edges"][:index])
            # Create a new exclude set by adding the current edge (only locs)
            newExcludeE = excludeE | {(v0, v1)}

            if newIncludeE.intersection(newExcludeE):
                continue

            # Solve the TSP problem with the new constraints
            PotentialOptimalSequences = self.solve_tsp_with_constraints(newIncludeE, newExcludeE)

            alloc_edges_set = set(PotentialOptimalSequences["Alloc_edges"])

            if not newIncludeE <= alloc_edges_set:
                continue

            if newExcludeE & alloc_edges_set:
                continue

            # Add the valid solution to the priority queue
            self.OPEN.put((PotentialOptimalSequences["Cost"], (newIncludeE, newExcludeE, PotentialOptimalSequences)))

        # If the priority queue is empty, return a default solution indicating no more allocations
        if self.OPEN.empty():
            return {"Allocations": {}, "Alloc_edges": [], "Cost": math.inf}

        # Retrieve the next best solution from the queue
        _, (includeE, excludeE, optimalSequences) = self.OPEN.get()
        # Save the new solution in the list of solutions
        self.Solutions[k] = (includeE, excludeE, optimalSequences)
        # Return the optimal solution found
        return optimalSequences

    ######################################################### Solve tsp with constraints ###############################################################
    def solve_tsp_with_constraints(self, includeE, excludeE):
        # Create the cost matrix
        costMatrix = self.create_cost_matrix(includeE, excludeE)
        # Generate the mEgtsp input file
        self.generate_EGTSP_problem_file(costMatrix)
        # Run the LKH solver and return the result
        return self.invoke_GLKH(includeE)

    ######################################################### Create cost matrix ###############################################################
    def create_cost_matrix(self, includeE, excludeE):
        cmat = np.zeros((len(self.AllPosAndGoals), len(self.AllPosAndGoals)))
        for row, (rowLoc, rowDirect) in enumerate(self.AllPosAndGoals):
            for col, (colLoc, colDirect) in enumerate(self.AllPosAndGoals):

                if rowLoc != colLoc:
                    if (rowLoc, colLoc) in includeE:
                        cmat[row, col] = -(
                                1000000 - self.cost_with_rotations[((rowLoc, rowDirect), (colLoc, colDirect))])
                    elif (rowLoc, colLoc) in excludeE:
                        cmat[row, col] = 1000000
                    elif (row < len(self.Positions) and col < len(self.Positions)) or (
                            row >= len(self.Positions) > col):
                        cmat[row, col] = 0
                    else:
                        cmat[row, col] = self.cost_with_rotations[((rowLoc, rowDirect), (colLoc, colDirect))]

        return cmat

    ####################################################### Generate egtsp problem file ############################################################
    def generate_EGTSP_problem_file(self, costMatrix):
        nx, _ = costMatrix.shape
        totalSets = len(self.Positions) + len(self.GoalLocations)
        lines = [
            "NAME : mEgtspf\n",
            "TYPE : AGTSP\n",
            f"DIMENSION : {nx}\n",
            f"GTSP_SETS : {totalSets}\n",
            "EDGE_WEIGHT_TYPE : EXPLICIT\n",
            "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n",
            "EDGE_WEIGHT_SECTION\n"
        ]

        lines.extend(" ".join(map(str, row.astype(int))) + "\n" for row in costMatrix)
        lines.append("GTSP_SET_SECTION\n")

        for agent in range(1, len(self.Positions) + 1):
            lines.append(f"{agent} {agent} -1\n")

        currSet = len(self.Positions) + 1
        index = len(self.Positions) + 1
        for _ in range(len(self.GoalLocations)):
            lines.append(f"{currSet} {index} {index + 1} {index + 2} {index + 3} -1\n")
            currSet += 1
            index += 4

        with open(f"{os.getcwd()}/EGSTP_runtime_files/{self.configStr}_mEgtsp.gtsp", mode="w") as FileGtsp:
            FileGtsp.writelines(lines)

    ############################################################# Invoke GLKH ####################################################################
    def invoke_GLKH(self, includeE):
        cmd = [f"{os.getcwd()}/GLKH-1.1/GLKH", f"{os.getcwd()}/EGSTP_runtime_files/{self.configStr}_mEgtsp.par"]
        self.Counter_Solver_TSP_For_Test += 1
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process.wait()

        mEgtsp_tours = {"Allocations": {}, "Alloc_edges": []}

        with open(f"{os.getcwd()}/EGSTP_runtime_files/{self.configStr}_mEgtsp.tour", mode="r") as FileTour:
            lines = FileTour.readlines()
            mEgtsp_tours["Cost"] = int(lines[1].split("=")[1])
            ix = 6  # Starting index of the tour in the output file
            val = int(lines[ix])
            currAgentTour = []
            agent = -1
            first = True

            # Read until the end of the tour
            while val != -1:
                goalLoc, _ = self.AllPosAndGoals[val - 1]
                if first:
                    agent = val - 1
                    currAgentTour.append(goalLoc)
                    first = False

                # If it's a new agent
                elif not first and val <= len(self.Positions):
                    mEgtsp_tours["Allocations"][agent] = currAgentTour
                    currAgentTour = [goalLoc]
                    agent = val - 1
                else:
                    mEgtsp_tours["Alloc_edges"].append((currAgentTour[-1], goalLoc))
                    currAgentTour.append(goalLoc)

                ix = ix + 1
                val = int(lines[ix])

            # Add the final agent's tour
            mEgtsp_tours["Allocations"][agent] = currAgentTour

            mEgtsp_tours["Cost"] += (len(includeE) * 1000000)

        return mEgtsp_tours

    ############################################################# Precompute all the costs ####################################################################
    def precompute_costs(self):
        precomputed_cost = defaultdict(lambda: 1000000)
        dict_for_h_val = defaultdict(lambda: 1000000)

        for goal in self.GoalLocations:
            self.BFS_without_rotations(goal, dict_for_h_val)

        for pos in self.AllPosAndGoals:
            self.BFS_with_rotations(pos, precomputed_cost)

        return dict_for_h_val, precomputed_cost

    def BFS_with_rotations(self, pos, precomputed_cost):
        counter_of_reach_goals = 0
        visited = np.zeros((self.MapAndDims["Cols"] * self.MapAndDims["Rows"], 4), dtype=bool)
        queue = deque([(pos, 0)])

        while queue:
            current_pos, cost = queue.popleft()
            loc, direct = current_pos

            if visited[loc, direct]:
                continue
            visited[loc, direct] = True

            if current_pos in self.AllCopyOfGoals:
                precomputed_cost[(pos, current_pos)] = cost
                counter_of_reach_goals += 1
                if counter_of_reach_goals == len(self.AllCopyOfGoals):
                    return

            for neighbor_pos, new_cost in self.get_neighbors_for_bfs_with_rotations(current_pos, cost):
                loc, direct = neighbor_pos
                if not visited[loc, direct]:
                    queue.append((neighbor_pos, new_cost))

    def get_neighbors_for_bfs_with_rotations(self, current_pos, cost):
        neighbors = []
        loc, direct = current_pos

        direction_moves = (loc + 1, loc + self.MapAndDims["Cols"], loc - 1, loc - self.MapAndDims["Cols"])

        loc_after_move = direction_moves[direct]
        if self.validate_move(loc_after_move, loc):
            neighbors.append(((loc_after_move, direct), cost + 1))

        neighbors.append(((loc, (direct - 1) % 4), cost + 1))
        neighbors.append(((loc, (direct + 1) % 4), cost + 1))

        return neighbors

    def validate_move(self, loc_after_move, loc):
        # Extract the agent's location and direction before taking the next step

        # If the agent is at the top or bottom boundary, it cannot move up or down
        if not (0 <= loc_after_move < self.MapAndDims["Cols"] * self.MapAndDims["Rows"]):
            return False

        # If the agent is at the right boundary, it cannot move right
        if loc % self.MapAndDims["Cols"] == self.MapAndDims["Cols"] - 1 and loc_after_move % \
                self.MapAndDims["Cols"] == 0:
            return False

        # If the agent is at the left boundary, it cannot move left
        if loc % self.MapAndDims["Cols"] == 0 and loc_after_move % self.MapAndDims["Cols"] == \
                self.MapAndDims["Cols"] - 1:
            return False

        if self.MapAndDims["Map"][loc_after_move] != 0:
            return False

        return True

    def BFS_without_rotations(self, goal, dict_fo_h_val):
        visited = np.zeros(self.MapAndDims["Cols"] * self.MapAndDims["Rows"], dtype=bool)
        queue = deque([(goal, 0)])

        while queue:
            current_loc, cost = queue.popleft()

            if visited[current_loc]:
                continue
            visited[current_loc] = True

            dict_fo_h_val[(current_loc, goal)] = cost

            for neighbor_loc, new_cost in self.get_neighbors_for_bfs_without_rotations(current_loc, cost):
                if not visited[neighbor_loc]:
                    queue.append((neighbor_loc, new_cost))

    def get_neighbors_for_bfs_without_rotations(self, current_loc, cost):
        neighbors = []

        for neighborLoc in [current_loc + 1, current_loc + self.MapAndDims["Cols"], current_loc - 1,
                            current_loc - self.MapAndDims["Cols"]]:
            if self.validate_move(neighborLoc, current_loc):
                neighbors.append((neighborLoc, cost + 1))

        return neighbors

