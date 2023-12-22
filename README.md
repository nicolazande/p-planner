# p-planner (Parallel Planner)

p-planner is a C++ project that implements a parallelized path planning algorithm. It takes advantage of GPUs and multi-core hardware to achieve higher performance by leveraging parallelism. The project includes a Qt GUI that showcases the effectiveness of the planner, allowing users to configure the number of threads and planners running against each other.

## Introduction

Path planning is traditionally a sequential problem, which limits its performance on modern hardware with parallel capabilities. p-planner aims to exploit parallelization by allowing multiple computational units to compete on the same planning task using different means. By doing so, p-planner can provide robust and efficient planning solutions.

The project introduces the concept of parallel planning, where computational units with different batch sizes compete to find the optimal solution. This approach reduces the impact of a bad initialization, speeds up the initial solution, and ensures a globally optimal path is found. The provided Qt GUI allows users to interactively configure and observe the parallel planning process.

![MbYPOknw](https://github.com/nicolazande/p-planner/assets/115359494/54b08e50-62f7-4b61-b97a-74bc19afe2ca)

## Features

- Parallelized path planning algorithm
- Utilizes multi-core hardware for enhanced performance
- Qt GUI for interactive configuration and visualization

## Results
![xSX4oM4t](https://github.com/nicolazande/p-planner/assets/115359494/17259d72-e31d-4218-8491-4a1ee3c31cf6)

   
