
# Distributed Traffic Signal Control under Stochastic Traffic Conditions

## Introduction
This repository contains the source code used in the computational experiments of the paper: 
[**Traffic Signal Control under Stochastic Traffic Demand and Vehicle Turning via Decentralized Decomposition Approaches**](https://www.sciencedirect.com/science/article/pii/S0377221723002941).

We build a two-stage stochastic program based on the Cell Transmission Model (CTM) for traffic signal control under stochastic traffic conditions.
We design a distributed algorithm combining ADMM and Benders decomposition to solve the model. 
Here is the detailed code.

## Citation
> [**Traffic Signal Control under Stochastic Traffic Demand and Vehicle Turning via Decentralized Decomposition Approaches**](https://www.sciencedirect.com/science/article/pii/S0377221723002941) <br />
> Xinyu Fei, Xingmin Wang, Xian Yu, Yiheng Feng, Henry Liu, Siqian Shen, Yafeng Yin <br />
> *European Journal of Operational Research, 2023*
> ```
> @article{fei2023traffic,
>   title={Traffic signal control under stochastic traffic demand and vehicle turning via decentralized decomposition approaches},
>   author={Fei, Xinyu and Wang, Xingmin and Yu, Xian and Feng, Yiheng and Liu, Henry and Shen, Siqian and Yin, Yafeng},
>   journal={European Journal of Operational Research},
>   volume={310},
>   number={2},
>   pages={712--736},
>   year={2023},
>   publisher={Elsevier}
> }
> ```

## Installation and Setup
### Prerequisites
- Python >= 3.8 (our test is based on Python 3.8)
- Libraries: 
    - gurobipy
    - matplotlib
    - networkx
    - numpy>=1.20.3
    - osmnx
    - scipy
    - tqdm



### Installation Steps
1. Clone the repository
2. Set up a virtual environment (optional)
3. Install dependencies by
```shell 
pip install -r requirements.txt
```

## Usage
### Quick Start
You can test our synthetic data instances by running
```shell 
python fake_instance_main.py
```

### Detailed Usage Guide
1. If you want to test your own instances, you can set your own osm map file as input.
You can also change the number of scenarios, ADMM iterations, and Benders decomposition iterations. 
An example command line is
```shell 
python fake_instance_main.py --mapfile="map_data/fake_instance/1-8.osm" \
  --scenario=20 --numadmm=10 --numbenders=4
```

2. If you want to generate a baseline signal plan, you can call function
```
for node_id, node in fake_instance_simulator.nodes.items():
    if node.type == "signalized":
        fake_instance_simulator.generate_naive_signal_plan(node_id)
```
3. If you want to simply run the evaluation of a given traffic signal plan, you can comment the following lines and 
rerun the file ```fake_instance_main.py```
```
fake_instance_optimizer.run()
fake_instance_optimizer.draw_signal()
```
4. If you want to solve the centralized two-stage stochastic program directly by Gurobi, please
follow the commands:
 ```
cd opt/centralized
python Gurobi_Stochastic.py
``` 
You can change the synthetic network size and other parameters in ```opt/centralized/data_global.py```.

## Code Structure
### Directory Layout
Here is the layout of this repository.
```
ctm-signal-opt-sharing/
    |- apps
        |- evaluate_signal.py
    |- ctm
      |- build_ctm.py
      ...
    |- map_data
      |- fake_instance
        |- 1-1.osm
        ...
    |- mtldp
      |- apps
      |- mtlmap
      |- tools
    |- opt
      |- centralized
      |- ADMM_optimizer.py
      ...
    fake_instance_main.py
```

### Important Modules
1. ```apps```: the function to evaluate a given traffic signal plan
2. ```ctm```: all the functions to build a ctm model
3. ```map_data```: our sample synthetic data
4. ```mtldp```: our tools to generate and read osm map files
5. ```opt```: our optimizer algorithm code, including centralized version in the folder
```centralized```, and decentralized algorithms.


## Developers
Xinyu Fei (xinyuf@umich.edu), for algorithms and CTM model

Xingmin Wang (xingminw@umich.edu), for data processing and CTM model

## Contact
Xinyu Fei (xinyuf@umich.edu)

Siqian Shen (siqian@umich.edu)