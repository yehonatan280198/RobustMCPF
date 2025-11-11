import math
import os
import subprocess
from collections import defaultdict
import numpy as np
from queue import PriorityQueue
from collections import deque


####################################################### generate Atsp Parameters file ############################################################
def generateMtspPar(configStr):
    lines = [
        f"PROBLEM_FILE = {os.getcwd()}/ATSP_runtime_files/{configStr}_Mtsp.atsp\n",
        f"RUNS = 10\n",
        f"OUTPUT_TOUR_FILE = {os.getcwd()}/ATSP_runtime_files/{configStr}_Mtsp.tour\n"
    ]

    with open(f"{os.getcwd()}/ATSP_runtime_files/{configStr}_Mtsp.par", mode="w+") as filePar:
        filePar.writelines(lines)

    ####################################################### Generate Atsp problem file ############################################################


def generateMtspFile(costMatrix, configStr):
    nx, ny = costMatrix.shape
    with open(f"ATSP_runtime_files/{configStr}_Mtsp.atsp", mode="w+") as ftsp:
        file_content = ["NAME : mtspf\n", "COMMENT : file for mtspf test\n", "TYPE : ATSP\n", f"DIMENSION : {nx}\n",
                        "EDGE_WEIGHT_TYPE : EXPLICIT\n", "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n", "EDGE_WEIGHT_SECTION\n",
                        "\n".join(" ".join(map(str, costMatrix[ix, :].astype(int))) for ix in range(nx)) + "\n"]

        ftsp.writelines(file_content)


######################################################### kBestSequencing class ###############################################################

class kBestSequencing:
    def __init__(self, Positions, GoalLocations, dict_of_map_and_dim, configStr):
        self.Positions = Positions
        self.OnlyLocOfPosition = [pos for pos, _ in Positions]  # Extract only locations, ignoring direction
        self.GoalLocations = GoalLocations  # Locations of goals
        self.AllLocPosAndGoals = self.OnlyLocOfPosition + self.GoalLocations
        self.configStr = configStr

        self.MapAndDims = dict_of_map_and_dim
        self.OPEN = PriorityQueue()
        self.Solutions = {}

        self.Counter_Solver_TSP_For_Test = 0

        self.cost_without_rotations = self.precompute_costs()

        generateMtspPar(self.configStr)

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

    def solve_tsp_with_constraints(self, includeE, excludeE):
        # Create the cost matrix
        costMatrix = self.Create_Cost_Matrix(includeE, excludeE)
        # Generate the MTSP input file
        generateMtspFile(costMatrix, self.configStr)
        # Run the LKH solver and return the result
        return self.invoke_lkh(includeE)

    def Create_Cost_Matrix(self, includeE, excludeE):
        cmat = np.zeros((len(self.AllLocPosAndGoals), len(self.AllLocPosAndGoals)))
        for row, rowLoc in enumerate(self.AllLocPosAndGoals):
            for col, colLoc in enumerate(self.AllLocPosAndGoals):

                # if not same location
                if rowLoc != colLoc:
                    if (rowLoc, colLoc) in includeE:
                        cmat[row, col] = -1000000
                    elif (rowLoc, colLoc) in excludeE:
                        cmat[row, col] = 1000000
                    elif (row < len(self.Positions) and col < len(self.Positions)) or (
                            row >= len(self.Positions) > col):
                        cmat[row, col] = 0
                    else:
                        cmat[row, col] = self.cost_without_rotations[(rowLoc, colLoc)]

        return cmat

    def invoke_lkh(self, includeE):
        cmd = [f"{os.getcwd()}/LKH-3.0.11/LKH", f"{os.getcwd()}/ATSP_runtime_files/{self.configStr}_Mtsp.par"]
        self.Counter_Solver_TSP_For_Test += 1
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process.wait()

        mtsp_tours = {"Allocations": {}, "Alloc_edges": []}

        with open(f"{os.getcwd()}/ATSP_runtime_files/{self.configStr}_Mtsp.tour", mode="r") as FileTour:
            lines = FileTour.readlines()
            mtsp_tours["Cost"] = int(lines[1].split("=")[1])
            ix = 6  # Starting index of the tour in the output file
            val = int(lines[ix])
            currAgentTour = []
            agent = -1
            first = True

            # Read until the end of the tour
            while val != -1:
                goalLoc = self.AllLocPosAndGoals[val - 1]
                if first:
                    agent = val - 1
                    currAgentTour.append(goalLoc)
                    first = False

                # If it's a new agent
                elif not first and val <= len(self.Positions):
                    mtsp_tours["Allocations"][agent] = currAgentTour
                    currAgentTour = [goalLoc]
                    agent = val - 1
                else:
                    mtsp_tours["Alloc_edges"].append((currAgentTour[-1], goalLoc))
                    currAgentTour.append(goalLoc)

                ix = ix + 1
                val = int(lines[ix])

            # Add the final agent's tour
            mtsp_tours["Allocations"][agent] = currAgentTour
            for edge in includeE:
                mtsp_tours["Cost"] += (1000000 + self.cost_without_rotations[edge])

        return mtsp_tours

    def precompute_costs(self):
        precomputed_cost = defaultdict(lambda: 1000000)

        for loc in self.AllLocPosAndGoals:
            self.BFS_without_rotations(loc, precomputed_cost)

        return precomputed_cost

    def BFS_without_rotations(self, goal, precomputed_cost):
        visited = np.zeros(self.MapAndDims["Cols"] * self.MapAndDims["Rows"], dtype=bool)
        queue = deque([(goal, 0)])

        while queue:
            current_loc, cost = queue.popleft()

            if visited[current_loc]:
                continue
            visited[current_loc] = True

            precomputed_cost[(current_loc, goal)] = cost

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
