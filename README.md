# Robust Multi-Agent Combinatorial Path Finding  

### Official code for the paper **"Robust Multi-Agent Combinatorial Path Finding"** accepted for publication at **AAAI-2026**

**Authors:** Yehonatan Kidushim, Avraham Natan, Roni Stern, Meir Kalech  

---

It implements the **Robust CBSS framework**, including both algorithms presented in the paper:
- **RCbssBase** – a baseline algorithm using TSP‑based allocation  
- **RCbssEff** – an improved and efficient variant using orientation‑aware E‑GTSP allocation  

The framework extends the **Conflict‑Based Steiner Search (CBSS)** to support:
- **Dynamic goal allocation** with K‑best‑Sequencing  
- **Orientation‑aware kinematic constraints**  
- **Probabilistic robustness** against stochastic execution delays (p‑Robust CBS)  

This repository includes full source code, benchmark data, experiment scripts, and a technical appendix to enable **full reproducibility** of the results presented in the paper.

---

## Repository Structure

- **Agent_Goal_locations_files/** – Agent and goal configuration files for experiments.  
- **ExperimentalResults/** – Processed experimental results from all experiments.  
- **Maps/** – Benchmark maps used in experiments.  
- **Run_Robust_Cbss_Framework.py** – Runs the main Robust CBSS framework (RCbssEff and RCbssBase).  
- **Simulation_for_AblationStudy.py** – Runs simulations for ablation studies.  
- **TestRCbssEffAblationStudy.py** – Executes ablation experiments.  
- **TestRCbssEffScalability.py** – Executes scalability experiments.  
- **TestRCbssEffVsRCbssBase.py** – Compares RCbssEff and RCbssBase performance.  
- **createMap.py** – Generates agent and goal configurations for maps.  
- **FindConflict.py** – Detects conflicts between agents’ paths.  
- **LowLevelPlan.py** – Computes individual agent paths under constraints.  
- **NodeStateConstClasses.py** – Defines data structures for nodes, states, and constraints.  
- **Verify.py** – Verifies solution robustness using simulations.  
- **kBestSequencing.py** – K‑best‑Sequencing algorithm using TSP.  
- **kBestSequencingWithGLKH.py** – K‑best‑Sequencing algorithm using E‑GTSP.  
- **AAAI‑2026_technicalAppendix.pdf** – Technical appendix with proofs and supplementary results.  

---

## Requirements & Installation

### Python
- **Python** ≥ 3.10  
- Recommended: **Ubuntu 20.04+** (tested on Ubuntu 24.04, AMD EPYC 7702P, 16 cores)  
- **numpy==2.2.6**  
- **scipy==1.15.3**  

### External Solvers
This project relies on two external solvers:
- **LKH‑3.0.11** – for solving multi‑agent TSP (used in RCbssBase).  
- **GLKH‑1.1** – for solving multi‑agent E‑GTSP with orientation handling (used in RCbssEff).  

Download and compile them from their official sources:  
- LKH: [http://www.akira.ruc.dk/~keld/research/LKH/](http://www.akira.ruc.dk/~keld/research/LKH/)  
- GLKH: [http://www.akira.ruc.dk/~keld/research/GLKH/](http://www.akira.ruc.dk/~keld/research/GLKH/)  

Compile each solver:
```bash
tar -xvzf LKH-3.0.11.tar.gz
cd LKH-3.0.11
make

tar -xvzf GLKH-1.1.tar.gz
cd GLKH-1.1
make
```

---

## Randomization & Seeds
All randomized components are initialized with fixed seeds for reproducibility:
- **Monte Carlo verification** (Verify.py): `seed = 47`
- **Conflict resolution** when multiple conflicts exist (FindConflict.py): `seed = 42`
- **Ablation experiments** (TestRCbssEffAblationStudy.py): `seed = 44`

Other components are fully deterministic given identical inputs.

