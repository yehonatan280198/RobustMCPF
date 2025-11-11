import heapq
import random
from itertools import combinations


def create_loc_times(path):
    locTimes = {}
    for i, (loc, _) in enumerate(path):
        locTimes.setdefault(loc, i)
    return locTimes


def create_edge_times(path):
    edgeTimes = {}
    for i in range(len(path) - 1):
        if path[i][0] != path[i + 1][0]:
            edge = (path[i][0], path[i + 1][0])
            edgeTimes.setdefault(edge, i + 1)
    return edgeTimes


def findConflictWithoutDelays(N):
    for agent1, agent2 in combinations(N.paths.keys(), 2):
        path1 = N.paths[agent1]
        path2 = N.paths[agent2]

        # Create loc-time dictionaries
        locTimes1 = set()
        for i, (loc, _) in enumerate(path1):
            locTimes1.add((i, loc))

        locTimes2 = set()
        for i, (loc, _) in enumerate(path2):
            locTimes2.add((i, loc))

        # Detect location conflicts
        common_locs = locTimes1 & locTimes2
        if len(common_locs) != 0:
            time, loc = min(common_locs)
            return 0, time, None, loc, (agent1, time), (agent2, time)

        edgeTimes1 = set()
        for i in range(len(path1) - 1):
            if path1[i][0] != path1[i + 1][0]:
                edge = (path1[i][0], path1[i + 1][0])
                edgeTimes1.add((i + 1, edge))

        edgeTimes2 = set()
        for i in range(len(path2) - 1):
            if path2[i][0] != path2[i + 1][0]:
                edge = (path2[i][0], path2[i + 1][0])
                edgeTimes2.add((i + 1, edge))

        # Detect edge conflicts, including reversed edges
        for time, edge1 in edgeTimes1:
            reversed_edge1 = (edge1[1], edge1[0])
            if (time, reversed_edge1) in edgeTimes2:
                return 0, time, None, frozenset(edge1), (agent1, time), (agent2, time)

    return None


class FindConflict:
    def __init__(self, algorithm):
        self.algorithm = algorithm
        self.randGen = random.Random(42)

    def findConflict(self, N):
        if self.algorithm in ["RCbssEff", "RCbssBase", "IRC"]:
            return self.findConflictWithDelays(N)
        else:
            return findConflictWithoutDelays(N)

    def findConflictWithDelays(self, N):
        heap = []

        posConstraintsDict = {
            agent: {
                (x, (agent1, t1), (agent2, t2)) for agent1, agent2, x, t1, t2 in N.posConstraints[agent]
            }
            for agent in N.paths
        }

        agent_data = {
            agent: {
                "locTimes": create_loc_times(N.paths[agent]),
                "edgeTimes": create_edge_times(N.paths[agent])
            }
            for agent in N.paths
        }

        for agent1, agent2 in combinations(N.paths.keys(), 2):
            allPosConstDict = posConstraintsDict[agent1] | posConstraintsDict[agent2]

            locTimes1, edgeTimes1 = agent_data[agent1]["locTimes"], agent_data[agent1]["edgeTimes"]
            locTimes2, edgeTimes2 = agent_data[agent2]["locTimes"], agent_data[agent2]["edgeTimes"]

            for loc in locTimes1.keys() & locTimes2.keys():
                time1, time2 = locTimes1[loc], locTimes2[loc]
                delta = abs(time1 - time2)
                Time = min(time1, time2)

                agent1_time, agent2_time = (Time, Time + delta) if time1 <= time2 else (Time + delta, Time)

                if (loc, (agent1, agent1_time), (agent2, agent2_time)) not in allPosConstDict:
                    heapq.heappush(heap, (
                        delta, Time, self.randGen.random(), loc, (agent1, agent1_time), (agent2, agent2_time)))

            for edge1, time1 in edgeTimes1.items():
                reversed_edge1 = (edge1[1], edge1[0])
                if reversed_edge1 in edgeTimes2:
                    time2 = edgeTimes2[reversed_edge1]
                    delta = abs(time1 - time2)
                    Time = min(time1, time2)

                    agent1_time, agent2_time = (Time, Time + delta) if time1 <= time2 else (Time + delta, Time)

                    if (frozenset(edge1), (agent1, agent1_time), (agent2, agent2_time)) not in allPosConstDict:
                        heapq.heappush(heap, (
                            delta, Time, self.randGen.random(), frozenset(edge1), (agent1, agent1_time),
                            (agent2, agent2_time)))

        return heapq.heappop(heap) if heap else None
