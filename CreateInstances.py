import os
import random


def create_positions_for_agents_And_Locs_For_Goals(NumAgents, NumGoals, MapAndDim):
    zero_indices = [i for i, value in enumerate(MapAndDim["Map"]) if value == 0]
    chosen_indices_for_agents = random.sample(zero_indices, NumAgents)
    position_for_agents = [(num, random.randint(0, 3)) for num in chosen_indices_for_agents]

    chosen_set = set(chosen_indices_for_agents)
    remaining_indices_for_goals = [idx for idx in zero_indices if idx not in chosen_set]
    location_for_agents = random.sample(remaining_indices_for_goals, NumGoals)

    return position_for_agents, location_for_agents


def read_map_file(file_path):
    with open("Maps/" + file_path, 'r') as f:
        lines = f.readlines()

    map_start_index = lines.index("map\n") + 1
    map_lines = lines[map_start_index:]

    currMap = []
    rows, cols = 0, 0
    for line in map_lines:
        cols = len(line.strip())
        rows += 1
        for char in line.strip():
            currMap += [0] if char == "." else [1]
    print(rows, cols)
    return {"Rows": rows, "Cols": cols, "Map": currMap}


mapsList = ["empty-32-32.map", "random-32-32-20.map", "maze-32-32-2.map", "room-32-32-4.map", "den312d.map", "ht_chantry.map", "lak303d.map", "den520d.map"]
output_dir = "Agent_Goal_locations_files"
os.makedirs(output_dir, exist_ok=True)

for map_name in mapsList:
    mapAndDim = read_map_file(map_name)
    for instance in range(100):
        AgentsPositions, GoalsLocations = create_positions_for_agents_And_Locs_For_Goals(50, 100, mapAndDim)

        agents_file = os.path.join(output_dir,
                                   f"{map_name.split('.')[0]}_Map_Agent_Locs_instance_{instance}.txt")
        goals_file = os.path.join(output_dir,
                                  f"{map_name.split('.')[0]}_Map_Goal_Locs_instance_{instance}.txt")

        with open(agents_file, "w") as f:
            for item in AgentsPositions:
                f.write(f"{item}\n")

        with open(goals_file, "w") as f:
            for item in GoalsLocations:
                f.write(f"{item}\n")