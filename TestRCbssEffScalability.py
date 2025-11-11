import os
import time
import csv
import ast
import sys

from multiprocessing import Process, Queue
from Run_Robust_Cbss_Framework import Robust_Cbss_framework


####################################################### Create Map #################################################################################
def create_map(map_name):
    file_path = f"OurResearch.domain/{map_name}.map"
    with open(file_path, "r") as file:
        lines = file.readlines()
    map_start_index = lines.index("map\n") + 1
    map_lines = lines[map_start_index:]

    currMap = []
    rows, cols = 0, 0
    for line in map_lines:
        cols = len(line.strip())
        rows += 1
        for char in line.strip():
            currMap += [0] if char == "." else [1]

    return {"Rows": rows, "Cols": cols, "Map": currMap}


####################################################### Global Variables ######################################################################
mapName = sys.argv[1]
mapAndDim = create_map(mapName)
p_safe = float(sys.argv[2])
configStr = f"{mapName}_p_safe_{p_safe}_typeOfTest_1"
cache_successes = {}


mapsList = ["empty-32-32", "random-32-32-20", "maze-32-32-2", "room-32-32-4", "den312d", "ht_chantry", "lak303d", "den520d"]
num_of_agentsList = [10, 15, 20, 25, 30, 40, 50]
num_of_goalsList = [20, 30, 40, 50, 60, 80, 100]
no_collision_probList = [0.8, 0.9, 0.95]
delays_probList = [0.1, 0.3]
instances = 20
verifyAlpha = 0.05
max_time = 60


####################################################### Write the header of a CSV file ############################################################
if not os.path.exists("typeOfTest_1_Output_files"):
    os.makedirs("typeOfTest_1_Output_files")

with open(f"typeOfTest_1_Output_files/Output_{configStr}.csv", mode="w", newline="",
          encoding="utf-8") as file:
    columns = ["Map", "Safe prob", "Delay prob", "Number of agents", "Number of goals", "Instance", "Algorithm",
               "Runtime", "Calls to EGTSP\\TSP", "Calls to LowLevel", "Number of roots generated", "Resolved conflicts"]
    writer = csv.DictWriter(file, fieldnames=columns)
    writer.writeheader()


####################################################### Read locs from file #################################################################################
def read_locs_from_file(num_of_instance):
    file_agents_name = f"Agent_Goal_locations_files/{mapName.split('.')[0]}_Map_Agent_Locs_instance_{num_of_instance}.txt"
    file_goals_name = f"Agent_Goal_locations_files/{mapName.split('.')[0]}_Map_Goal_Locs_instance_{num_of_instance}.txt"

    with open(file_agents_name, "r") as f:
        Agents_Positions = [ast.literal_eval(line.strip()) for _, line in zip(range(CurrNumAgents), f)]

    with open(file_goals_name, "r") as f:
        Goals_Locations = [ast.literal_eval(line.strip()) for _, line in zip(range(CurrNumGoals), f)]

    return Agents_Positions, Goals_Locations


####################################################### run Test  #################################################################################
def run_Test(queue, Positions, GoalLocations, DelaysProbDict, CurrAlgorithm):

    p = Robust_Cbss_framework(Positions, GoalLocations, p_safe, DelaysProbDict, mapAndDim, verifyAlpha, CurrAlgorithm, configStr)
    queue.put((p.Solution[1], p.Solution[2], p.Solution[3], p.Solution[4]))


####################################################### run Tests #################################################################################

def run_instances():
    delaysProbDict = {i: CurrDelaysProb for i in range(CurrNumAgents)}
    curr_cache_successes = {
        ("RobustCbss", (CurrDelaysProb, CurrNumAgents, CurrNumGoals)): 0
    }

    for instance in range(instances):
        AgentsPositions, GoalsLocations = read_locs_from_file(instance)

        for CurrAlgorithm in ["RobustCbss"]:

            if not checkIfNeedRunThisTest(CurrAlgorithm):
                addRecordToCsv(instance, CurrAlgorithm, None, None, None, None, None)
                continue

            print(f"typeOfTest: 1, algorithm: {CurrAlgorithm}, map: {mapName}, safe prob: {p_safe}, delay prob: {CurrDelaysProb}, agents: {CurrNumAgents}, goals: {CurrNumGoals}, instance: {instance}")
            print(f"AgentsPositions: {AgentsPositions} \n GoalsLocations: {GoalsLocations}")

            queue = Queue()

            process = Process(target=run_Test, args=(queue, AgentsPositions, GoalsLocations, delaysProbDict, CurrAlgorithm))

            process.start()
            start_time = time.time()
            process.join(timeout=max_time)

            if process.is_alive():
                process.terminate()
                process.join()
                print(f"Skipped due to timeout after {max_time} seconds.")
                print("--------------------------------------------------------------------------")
                addRecordToCsv(instance, CurrAlgorithm, None, None, None, None, None)
                continue

            runtime = round(time.time() - start_time, 5)

            # If the process completed in time, retrieve the solution
            CallToEGTSP, CallToLowLevel, Roots, ResolvedConflicts = queue.get()
            print("Pass!")
            print("--------------------------------------------------------------------------")

            curr_cache_successes[(CurrAlgorithm, (CurrDelaysProb, CurrNumAgents, CurrNumGoals))] += 1
            addRecordToCsv(instance, CurrAlgorithm, runtime, CallToEGTSP, CallToLowLevel, Roots, ResolvedConflicts)

    cache_successes[("RobustCbss", (CurrDelaysProb, CurrNumAgents, CurrNumGoals))] = curr_cache_successes[("RobustCbss", (CurrDelaysProb, CurrNumAgents, CurrNumGoals))]


def addRecordToCsv(instance, CurrAlgorithm, runtime, CallToEGTSP, CallToLowLevel, Roots, ResolvedConflicts):
    record = [mapName, p_safe, CurrDelaysProb, CurrNumAgents, CurrNumGoals, instance + 1, CurrAlgorithm, runtime,
              CallToEGTSP, CallToLowLevel, Roots, ResolvedConflicts]

    with open(f"typeOfTest_1_Output_files/Output_{configStr}.csv", mode="a", newline="", encoding="utf-8") as file:
        writerRecord = csv.writer(file)
        writerRecord.writerow(record)


def checkIfNeedRunThisTest(CurrAlgorithm):
    currTuple = (CurrDelaysProb, CurrNumAgents, CurrNumGoals)
    for (alg, record), successes in cache_successes.items():
        if CurrAlgorithm == alg and all(a <= b for a, b in zip(record, currTuple)) and successes == 0:
            return False

    return True


for CurrDelaysProb in delays_probList:
    for CurrNumAgents in num_of_agentsList:
        for CurrNumGoals in num_of_goalsList:
            run_instances()
