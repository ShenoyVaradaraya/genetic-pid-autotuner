# Genetic Algorithm PID Controller Autotuner

This repository contains a Python implementation of a genetic algorithm-based PID (Proportional-Integral-Derivative) controller autotuner. The system automatically optimizes PID controller parameters for dynamic systems without requiring manual tuning.

## Overview

PID controllers are widely used in industrial processes, robotics, and control applications, but manual tuning can be time-consuming and challenging. This autotuner efficiently optimizes PID controller parameters using genetic algorithms, providing a valuable tool for engineers and practitioners in control systems.

## Features

- Genetic algorithm-based optimization for PID controller parameters
- Dynamic mutation rate adaptation over generations
- Multiple independent runs to find globally optimal solutions
- Anti-windup implementation in the PID controller
- Simulation of first-order systems with delay
- Comprehensive performance metrics calculation (rise time, settling time, overshoot, steady-state error)
- Visualization of system response with optimized PID parameters

## System Parameters

The genetic algorithm uses the following parameters:

- **Population Size**: 500
- **Crossover Rate**: 0.9
- **Generations**: 200
- **Target Fitness**: 0.01
- **Number of Runs**: 20
- **PID Controller Parameter Ranges**:
  - Kp range: (5.0, 10.0)
  - Ki range: (0.5, 5.0)
  - Kd range: (0.5, 1.0)
- **System Parameters**:
  - Process gain: 2.0
  - Process time constant: 3.0
  - Delay: 1
- **Simulation Parameters**:
  - Simulation time: 10 seconds
  - Time step: 0.1 seconds

## Requirements

- Python 3.x
- NumPy
- Matplotlib
- tqdm

## Installation

Clone the repository and install the required dependencies:

```bash
git clone https://github.com/yourusername/genetic-pid-autotuner.git
cd genetic-pid-autotuner
pip install numpy matplotlib tqdm
```

## Usage

Run the main script to execute the genetic algorithm for PID controller tuning:

```bash
python pid_autotune.py
```

The script will execute the genetic algorithm with the specified parameters, and the results, including performance metrics and system response plots, will be displayed.

## Performance Metrics

The algorithm evaluates PID controller performance using the following metrics:

1. **Overshoot (Mp)**:
   - Formula: Mp = (Cmax - Cfinal) / Cfinal × 100

2. **Steady-State Error (ess)**:
   - Formula: ess = |Cfinal - Cdesired|

3. **Settling Time (Ts)**:
   - Formula: Ts = tsettled - tstart

4. **Rise Time (Tr)**:
   - Formula: Tr = t90% - t10%

## Results

The genetic algorithm-based autotuner demonstrates its capability to fine-tune controller parameters for dynamic systems. The optimization process results in PID controllers with satisfactory performance metrics:

- Rise Time: ~1.80 seconds
- Overshoot: 20-25%
- Settling Time: ~10.00 seconds
- Steady-State Error: ~0.01-0.02

## Implementation Details

The implementation includes:

- PID Controller with anti-windup
- First-order system simulation with delay
- Genetic algorithm operations (selection, crossover, mutation)
- Performance metrics calculation
- Visualization of system response

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Citation

If you use this code in your research or project, please cite:

```
@software{genetic_pid_autotuner,
  author = {Varadaraya Ganesh Shenoy},
  title = {Genetic Algorithm based PID Controller Autotuner},
  year = {2025},
  url = {https://github.com/yourusername/genetic-pid-autotuner}
}
```

## References

1. D. C. Meena and A. Devanshu, "Genetic algorithm tuned PID controller for process control," 2017 International Conference on Inventive Systems and Control (ICISC), Coimbatore, India, 2017, pp. 1-6, doi: 10.1109/ICISC.2017.8068639.

2. H. Noshahri and H. Kharrati, "PID controller design for unmanned aerial vehicle using genetic algorithm," 2014 IEEE 23rd International Symposium on Industrial Electronics (ISIE), Istanbul, Turkey, 2014, pp. 213-217, doi: 10.1109/ISIE.2014.6864613.

3. R. H. Dinger, "Engineering design optimization with genetic algorithms," Northcon/98. Conference Proceedings (Cat. No.98CH36264), Seattle, WA, USA, 1998, pp. 114-119, doi: 10.1109/NORTHC.1998.731522.
