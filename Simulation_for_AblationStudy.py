class Simulation_for_type2_test:

    def __init__(self, plan, algorithm, delaysProb, Positions, GoalLocations, callToPlannerCounter, randGen):
        self.plan = plan
        self.algorithm = algorithm
        self.delaysProb = delaysProb
        self.positions = Positions
        self.remainGoals = GoalLocations
        self.callToPlanner = callToPlannerCounter
        self.randGen = randGen
        self.SOC = 0

    def runSimulation(self):
        if self.algorithm in ["RCbssEff", "IDP"]:
            return self.run_simulation_for_RCbssEffOrIDP()
        else:
            return self.run_simulation_for_IRC()

    def run_simulation_for_RCbssEffOrIDP(self):
        # Initialize the set of active agents (agents that have not finished their path)
        active_agents = {agent for agent, path in self.plan.items() if len(path) > 1}
        # Flag to indicate if a collision occurs
        collision = False

        while active_agents:
            new_pos = []
            new_locs = set()
            # Set to track current agent locations
            locsAndEdge = set()
            # Set to track agents that have completed their paths
            finish_agents = set()

            for agent, path in self.plan.items():
                # Current path of the agent
                lastLoc = path[0][0]

                # Simulate agent movement with a delay probability
                if len(path) != 1 and self.randGen.random() > self.delaysProb[agent]:
                    # Remove the first step if the agent moves
                    path.pop(0)

                new_pos.append(path[0])
                new_locs.add(path[0][0])

                # Current location of the agent
                loc = path[0][0]
                # Check for collision
                if loc in locsAndEdge or (loc, lastLoc) in locsAndEdge:
                    collision = True
                    break
                locsAndEdge.add(loc)
                locsAndEdge.add((lastLoc, loc))

                # If the agent has reached its destination, mark it for removal
                if len(path) == 1:
                    finish_agents.add(agent)

            # Stop the simulation if a collision occurs
            if collision:
                return False

            self.SOC += len(active_agents)
            # Remove agents that have completed their paths
            active_agents -= finish_agents

            self.positions = new_pos
            self.remainGoals = list(set(self.remainGoals) - new_locs)

        return True

    def run_simulation_for_IRC(self):
        # Initialize the set of active agents (agents that have not finished their path)
        active_agents = {agent for agent, path in self.plan.items() if len(path) > 1}
        # Flag to indicate if a collision occurs
        collision = False

        while active_agents:
            new_pos = self.positions.copy()
            new_locs = set()
            # Set to track current agent locations
            locsAndEdge = set()
            # Set to track agents that have completed their paths
            finish_agents = set()

            for agent, path in self.plan.items():
                # Current path of the agent
                lastPos = new_pos[agent]

                # Simulate agent movement with a delay probability
                if len(path) != 1 and self.randGen.random() > self.delaysProb[agent]:
                    canMoveToNextLoc, direct = self.find_next_rotation_and_if_can_move_to_next_loc(lastPos, path[1][0])
                    if canMoveToNextLoc:
                        path.pop(0)

                    path[0] = (path[0][0], direct)

                new_pos[agent] = path[0]
                new_locs.add(path[0][0])

                # Current location of the agent
                loc = path[0][0]
                # Check for collision
                if loc in locsAndEdge or (loc, lastPos[0]) in locsAndEdge:
                    collision = True
                    break
                locsAndEdge.add(loc)
                locsAndEdge.add((lastPos[0], loc))

                # If the agent has reached its destination, mark it for removal
                if len(path) == 1:
                    finish_agents.add(agent)

                # Stop the simulation if a collision occurs
            if collision:
                return False

            self.SOC += len(active_agents)
            # Remove agents that have completed their paths
            active_agents -= finish_agents

            self.positions = new_pos
            self.remainGoals = list(set(self.remainGoals) - new_locs)

        return True

    def find_next_rotation_and_if_can_move_to_next_loc(self, currPos, newtLoc):
        loc, direct = currPos
        desired_direct = -1

        diff = newtLoc - loc
        if diff == 1:
            desired_direct = 0  # Right
        elif diff == -1:
            desired_direct = 2  # Left
        elif diff > 0:
            desired_direct = 1  # Down
        elif diff < 0:
            desired_direct = 3  # Up

        if desired_direct == -1 or desired_direct == direct:
            return True, direct

        delta = (desired_direct - direct) % 4
        if delta == 1:
            return False, (direct + 1) % 4
        elif delta == 3:
            return False, (direct - 1) % 4
        else:
            return False, (direct + 1) % 4
