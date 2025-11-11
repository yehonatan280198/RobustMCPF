import heapq
from NodeStateConstClasses import State


########################################################## Extract path #####################################################3
def extractPath(state):
    path = []
    while state is not None:
        path.insert(0, state.CurPosition)
        state = state.parent

    return path

########################################################## LowLevelPlan Class #####################################################3


class LowLevelPlan:
    def __init__(self, dict_of_map_and_dim, Positions, dict_cost_for_Heuristic_value, algorithm):
        self.MapAndDims = dict_of_map_and_dim
        self.Positions = Positions
        self.dict_cost_for_Heuristic_value = dict_cost_for_Heuristic_value

        self.algorithm = algorithm
        self.Counter_LowLevel_For_Test = 0

    ########################################################## Low level plan #####################################################

    def runLowLevelPlan(self, Node, agent_that_need_update_path):
        if self.algorithm in ["RCbssEff", "RCbssBase", "IDP"]:
            return self.runLowLevelPlanWithRotations(Node, agent_that_need_update_path)
        else:
            return self.runLowLevelPlanWithoutRotations(Node, agent_that_need_update_path)

    ########################################################## run Low Level Plan With Rotations #####################################################
    def runLowLevelPlanWithRotations(self, Node, agent_that_need_update_path):
        self.Counter_LowLevel_For_Test += 1

        for agent in agent_that_need_update_path:
            sequence = Node.sequence["Allocations"][agent]

            # If no allocations are present
            if len(sequence) == 1:
                Node.paths[agent] = [self.Positions[agent]]
                continue

            # Decrease the previous path cost of the current agent
            Node.g -= (max(1, len(Node.paths[agent])) - 1)

            findPath = False
            OpenList = []
            visited = {}

            S = State(self.Positions[agent], sequence=[self.Positions[agent][0]])
            heapq.heappush(OpenList, (self.calc_cost_for_Heuristic_value(S, sequence), S))

            while OpenList:
                _, S = heapq.heappop(OpenList)
                loc, direct = S.CurPosition

                if (loc, direct, tuple(S.sequence)) in visited:
                    continue
                visited[(loc, direct, tuple(S.sequence))] = True

                if len(S.sequence) == len(sequence):
                    findPath = True
                    break

                for Sl in self.GetNeighbors(S, agent, visited, Node, sequence):
                    loc, direct = Sl.CurPosition
                    if not visited.get((loc, direct, tuple(Sl.sequence)), False):
                        heapq.heappush(OpenList, (self.calc_cost_for_Heuristic_value(Sl, sequence) + Sl.g, Sl))

            if not findPath:
                return False

            # Extract the path from the final goal back to the start
            Node.paths[agent] = extractPath(S)
            Node.g += (len(Node.paths[agent]) - 1)
        return True

    ########################################################## calc cost for Heuristic value #####################################################
    def calc_cost_for_Heuristic_value(self, S, sequence):
        if len(S.sequence) == len(sequence):
            return 0

        nextIndex = len(S.sequence)
        h_val = self.dict_cost_for_Heuristic_value[S.CurPosition[0], sequence[nextIndex]]
        return h_val + sum(self.dict_cost_for_Heuristic_value[sequence[i], sequence[i + 1]] for i in range(nextIndex, len(sequence) - 1))

    ########################################################## Get neighbors #####################################################

    def GetNeighbors(self, state, agent, visited, Node, sequence):
        neighbors = []
        loc, direct = state.CurPosition

        # Define movement candidates for the agent
        direction_moves = (loc + 1, loc + self.MapAndDims["Cols"], loc - 1, loc - self.MapAndDims["Cols"])

        # Try moving in the current direction
        loc_after_move = direction_moves[direct]
        canMove = self.validateMove(loc_after_move, agent, state, Node)

        if canMove == 1:
            afterMoveStateSequence = state.sequence + [sequence[len(state.sequence)]] if (
                    loc_after_move == sequence[len(state.sequence)] and state.sequence == sequence[:len(state.sequence)]
            ) else state.sequence

            neighbors.append(State((loc_after_move, direct), state.g + 1, state, afterMoveStateSequence))

        canStay = self.validateMove(loc, agent, state, Node) == 1
        if canStay == 1:
            neighbors.append(State((loc, (direct - 1) % 4), state.g + 1, state, state.sequence[:]))
            neighbors.append(State((loc, (direct + 1) % 4), state.g + 1, state, state.sequence[:]))

        # Stay in the same place but increment g (cost)
        if canMove == -1:
            neighbors.append(State((loc, direct), state.g + 1, state, state.sequence[:]))
            del visited[(loc, direct, tuple(state.sequence))]

        return neighbors

    ########################################################## validate Move #####################################################
    def validateMove(self, loc_after_move, agent, state, Node):
        # Extract the agent's location and direction before taking the next step
        loc, _ = state.CurPosition
        cols, rows = self.MapAndDims["Cols"], self.MapAndDims["Rows"]
        max_cells = cols * rows

        # If the agent is at the top or bottom boundary, it cannot move up or down
        if not (0 <= loc_after_move < max_cells):
            return 0

        col_loc = loc % cols
        col_after = loc_after_move % cols

        if (col_loc == 0 and col_after == cols - 1) or (col_loc == cols - 1 and col_after == 0):
            return 0

        if self.MapAndDims["Map"][loc_after_move] != 0:
            return 0

        # Check if the move violates any negative constraints
        for z, x, t in Node.negConstraints[agent]:
            if t == state.g + 1 and (x == loc_after_move or x == frozenset((loc, loc_after_move))):
                return -1

        for agent1, agent2, x, t1, t2 in Node.posConstraints[agent]:
            if agent1 == agent and t1 == state.g + 1 and (
                    x != loc_after_move and x != frozenset((loc, loc_after_move))):
                return 0

            elif agent2 == agent and t2 == state.g + 1 and (
                    x != loc_after_move and x != frozenset((loc, loc_after_move))):
                return 0

        return 1

    ########################################################## runLowLevelPlanWithoutRotations #####################################################

    def runLowLevelPlanWithoutRotations(self, Node, agent_that_need_update_path):
        self.Counter_LowLevel_For_Test += 1

        for agent in agent_that_need_update_path:
            sequence = Node.sequence["Allocations"][agent]

            # If no allocations are present
            if len(sequence) == 1:
                Node.paths[agent] = [self.Positions[agent]]
                continue

            # Decrease the previous path cost of the current agent
            Node.g -= (max(1, len(Node.paths[agent])) - 1)

            findPath = False
            OpenList = []
            visited = {}

            S = State(self.Positions[agent], sequence=[self.Positions[agent][0]])
            heapq.heappush(OpenList, (self.calc_cost_for_Heuristic_value(S, sequence), S))

            while OpenList:
                _, S = heapq.heappop(OpenList)
                loc, _ = S.CurPosition

                if (loc, tuple(S.sequence)) in visited:
                    continue
                visited[(loc, tuple(S.sequence))] = True

                if len(S.sequence) == len(sequence):
                    findPath = True
                    break

                for Sl in self.GetNeighborsWithoutRotations(S, agent, visited, Node, sequence):
                    loc, _ = Sl.CurPosition
                    if not visited.get((loc, tuple(Sl.sequence)), False):
                        heapq.heappush(OpenList, (self.calc_cost_for_Heuristic_value(Sl, sequence) + Sl.g, Sl))

            if not findPath:
                return False

            # Extract the path from the final goal back to the start
            Node.paths[agent] = extractPath(S)
            Node.g += (len(Node.paths[agent]) - 1)
        return True

    def GetNeighborsWithoutRotations(self, state, agent, visited, Node, sequence):
        neighbors = []
        loc, _ = state.CurPosition
        stay = False

        # Define movement candidates for the agent
        direction_moves = (loc + 1, loc + self.MapAndDims["Cols"], loc - 1, loc - self.MapAndDims["Cols"])

        for loc_after_move in direction_moves:
            canMove = self.validateMove(loc_after_move, agent, state, Node)

            if canMove == 1:
                afterMoveStateSequence = state.sequence + [sequence[len(state.sequence)]] if (
                        loc_after_move == sequence[len(state.sequence)] and state.sequence == sequence[
                                                                                              :len(state.sequence)]
                ) else state.sequence

                neighbors.append(State((loc_after_move, _), state.g + 1, state, afterMoveStateSequence))

            # Stay in the same place but increment g (cost)
            if canMove == -1 and not stay:
                stay = True
                neighbors.append(State((loc, _), state.g + 1, state, state.sequence[:]))
                del visited[(loc, tuple(state.sequence))]

        return neighbors
