# Carcará
Team: Sofia Paiva, Artur Costa, João Baião, Leonardo Santos.

Institution: Universidade Federal de Minas Gerais (UFMG)

This repository contains:

* `sarc_carcara`: package containing our proposed solution;

* `vectorfield_stack`: a modified version of the original repository found at this [link](https://github.com/adrianomcr/vectorfield_stack);

* `sarc_environment`: package containing the simulation environment. You can check the original repository at this [link](https://github.com/2nd-sarc-barinet-aerospace-competition/sarc_environment).


## How to run
1. Clone this repository to your workspace

```
cd ~/workspace/src
git clone https://github.com/leohmcs/carcara.git
```

2. Build the packages
```
catkin build
```

3. Run the proposed solution
```
cd carcara/sarc_environment/start/start_map
./start.sh
```

Note: check the `solution` pane on tmux while running the simulation.