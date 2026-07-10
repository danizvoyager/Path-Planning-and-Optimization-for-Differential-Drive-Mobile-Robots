
# Quick User Guide

## DWA Motion Planning with Metaheuristic Optimization

### A Short Guide for Users

---

## 1. What This Code Does

This MATLAB software implements **Dynamic Window Approach (DWA)** for mobile robot navigation with **metaheuristic optimization** of DWA parameters. It enables a differential-drive robot to navigate through static and dynamic obstacles while automatically finding the best DWA weights using four optimization algorithms:

- **GA** - Genetic Algorithm
- **GWO** - Grey Wolf Optimizer  
- **ACO** - Ant Colony Optimization
- **PSO** - Particle Swarm Optimization

---

## 2. Quick Start

### Run All Optimizers (Recommended)
```matlab
compare_optimizers
```
This runs all four optimizers, generates comparison plots, and shows statistical results.

### Run a Single Optimizer
```matlab
ga_dwa      % Genetic Algorithm
gwo_dwa     % Grey Wolf Optimizer
aco_dwa     % Ant Colony Optimization
pso_dwa     % Particle Swarm Optimization
```

### Test a Weight Set with Animation
```matlab
weights = [0.6, 0.5, 0.3];  % [alpha, beta, gamma]
[J, sim] = evaluateWeights(weights, true);
```

---

## 3. Understanding the Cost Function

The optimization minimizes:
```
J = λ₁·T_goal + λ₂·L - λ₃·Dmin + λ₄·ω²_int
```

| Term | Description | Default Weight |
|------|-------------|----------------|
| T_goal | Time to reach goal | λ₁ = 1/40 |
| L | Path length | λ₂ = 1/15 |
| Dmin | Minimum obstacle clearance | λ₃ = 1/5 |
| ω²_int | Motion smoothness | λ₄ = 1/10 |

**Lower J = Better Performance**
- J < 1.5: Good
- J > 2.0: Needs improvement
- J > 5.0: Poor

---

## 4. DWA Weights Explained

The DWA evaluates trajectories using:
```
score = α·Heading + β·Obstacle + γ·Velocity
```

| Weight | Purpose | High Value Effect |
|--------|---------|-------------------|
| α (Heading) | Move toward goal | More direct path |
| β (Obstacle) | Avoid obstacles | Safer, more conservative |
| γ (Velocity) | Move fast | Faster travel |

**Common Weight Sets:**
- Open spaces: `[0.8, 0.1, 0.4]`
- Cluttered environments: `[0.4, 0.8, 0.2]`
- Dynamic environments: `[0.6, 0.5, 0.3]`

---

## 5. Two Simulation Modes

### Baseline Mode
- Kinematic robot model
- No global planner
- Faster simulation

```matlab
cfg = getSimulationConfig('baseline');
[J, sim] = evaluateWeights(weights, true, [], cfg);
```

### Enhanced Mode (Recommended)
- Dynamic robot model with LQI controller
- D* Lite global planner
- More realistic

```matlab
cfg = getSimulationConfig('enhanced');
[J, sim] = evaluateWeights(weights, true, [], cfg);
```

---

## 6. Working with Environments

### Fixed Environment
```matlab
scenario = generateScenarioFixed();
[J, sim] = evaluateWeights(weights, true, scenario);
```

### Random Environment
```matlab
scenario = generateScenarioRandom();
[J, sim] = evaluateWeights(weights, true, scenario);
```

### Custom Environment
```matlab
scenario = struct();
scenario.goal = [10; 2];

% Static obstacles
scenario.staticObs = struct([]);
scenario.staticObs(1).pos = [3; 1];
scenario.staticObs(1).radius = 0.3;

% Dynamic obstacles  
scenario.dynamicObs = struct([]);
scenario.dynamicObs(1).pos = [5; 2];
scenario.dynamicObs(1).vel = [0.3; 0];
scenario.dynamicObs(1).radius = 0.2;
```

---

## 7. Understanding Results

### Simulation Output Structure
```matlab
sim.reached       % true/false - goal reached
sim.T_goal        % Time to goal (seconds)
sim.L             % Path length (meters)
sim.Dmin_global   % Minimum obstacle clearance (meters)
sim.hist_state    % Robot state history
sim.hist_u        % Control command history
sim.hist_time     % Time history
```

### Reading the Plots

**Convergence Plot:**
- X-axis: Iteration number
- Y-axis: Best cost value
- Lower and flatter = better

**Path Comparison:**
- Green: Robot position
- Green diamond: Goal
- Red circles: Static obstacles
- Blue circles: Dynamic obstacles
- Colored lines: Different methods

**Control Output:**
- Top: Linear velocity
- Bottom: Angular velocity
- Smoother = better

---

## 8. Customizing Configuration

Edit `getSimulationConfig.m`:

### Robot Parameters
```matlab
cfg.robot.radius = 0.135;      % Robot radius (m)
cfg.safetyMargin = 0.12;       % Safety margin (m)
cfg.params.v_max = 1.2;        % Max speed (m/s)
cfg.params.v_min = 0.0;        % Min speed (m/s)
```

### Simulation Parameters
```matlab
cfg.dt = 0.1;                  % Time step (s)
cfg.Tmax = 40;                 % Max simulation time (s)
cfg.predictTime = 2.0;         % Prediction horizon (s)
cfg.goalTol = 0.3;             % Goal tolerance (m)
```

### Cost Weights
```matlab
cfg.cost.lambda1 = 1/40;       % Time weight
cfg.cost.lambda2 = 1/15;       % Path length weight
cfg.cost.lambda3 = 1/5;        % Safety weight
cfg.cost.lambda4 = 1/10;       % Smoothness weight
```

### Optimizer Parameters
```matlab
popSize = 20;    % Population size
maxIter = 30;    % Maximum iterations
```

---

## 9. Interpreting Comparison Results

### Example Output
```
Fixed DWA:
  Mean Cost   = 11.3935
  Goal Rate   = 80.00%

GA:
  Mean Cost   = 1.2282
  Goal Rate   = 100%

GWO:
  Mean Cost   = 1.2203
  Goal Rate   = 100%

PSO:
  Mean Cost   = 1.2150
  Goal Rate   = 100%
```

### What to Look For:
1. **Mean Cost**: Lower is better
2. **Goal Rate**: Should be 100%
3. **Runtime**: Lower is better
4. **Path Length**: Lower is better
5. **Convergence**: Should be smooth

---

## 10. Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| Optimization not converging | Increase maxIter or popSize |
| Robot collides with obstacles | Increase safetyMargin or β weight |
| Slow simulation | Increase dt, decrease v_res or w_res |
| Memory errors | Decrease Tmax or reduce population size |

### Quick Fixes

**If robot doesn't reach goal:**
```matlab
weights = [0.6, 0.5, 0.3];  % Balanced weights
cfg.goalTol = 0.5;           % More tolerant
cfg.Tmax = 60;               % More time
```

**If robot hits obstacles:**
```matlab
weights = [0.3, 0.8, 0.2];  % Higher obstacle weight
cfg.safetyMargin = 0.15;     % More safety margin
cfg.params.v_max = 0.8;      % Slower speed
```

**If simulation is slow:**
```matlab
cfg.dt = 0.15;               % Larger time step
cfg.params.v_res = 0.08;     % Coarser resolution
doPlot = false;              % No animation
```

---

## 11. File Descriptions

| File | Purpose |
|------|---------|
| **Optimizers** | |
| `ga_dwa.m` | Genetic Algorithm |
| `gwo_dwa.m` | Grey Wolf Optimizer |
| `aco_dwa.m` | Ant Colony Optimization |
| `pso_dwa.m` | Particle Swarm Optimization |
| `compare_optimizers.m` | Compare all optimizers |
| **Core Functions** | |
| `dwaControl.m` | DWA controller |
| `evaluateWeights.m` | Evaluate performance |
| `globalCost.m` | Cost function |
| `ddmrDynamics.m` | Robot dynamics |
| **Environment** | |
| `generateScenarioFixed.m` | Fixed environment |
| `generateScenarioRandom.m` | Random environment |
| `getSimulationConfig.m` | Configuration |

---

## 12. Hardware Deployment

### Quick Steps

1. **Generate C++ code:**
```matlab
cfg_coder = coder.config('exe');
cfg_coder.TargetLang = 'C++';
codegen dwaControl -config cfg_coder -args {x, goal, staticObs, dynamicObs, cfg, controllerState}
```

2. **Deploy on ROS:**
```bash
roslaunch dwa_controller dwa_hardware.launch
```

3. **Run on embedded system:**
- Raspberry Pi: Copy compiled code and run
- NVIDIA Jetson: Use GPU acceleration
- Industrial PC: Standard deployment

---

## 13. Quick Reference

### Most Common Commands
```matlab
% Run all optimizers
compare_optimizers

% Test weights
[J, sim] = evaluateWeights([0.6, 0.5, 0.3], true);

% Change mode
cfg = getSimulationConfig('enhanced');

% Custom environment
scenario = generateScenarioRandom();
[J, sim] = evaluateWeights(weights, true, scenario);
```

### Best Practices
1. **Start with fixed weights** to understand behavior
2. **Run compare_optimizers** to find best method
3. **Use enhanced mode** for realistic results
4. **Increase population** for better optimization
5. **Run multiple trials** for statistical significance
6. **Save results** for analysis and reporting

---

## 14. Getting Help

### Common Problems and Solutions

**MATLAB errors:**
- Ensure all toolboxes are installed
- Add all folders to path: `addpath(genpath(pwd))`

**Long optimization times:**
- Reduce maxIter to 20
- Reduce population size to 15
- Use baseline mode for testing

**Poor performance:**
- Increase population size
- Increase maxIter
- Tune cost weights

---

## 15. Summary

| Step | Action | Command |
|------|--------|---------|
| 1 | Run optimization | `ga_dwa` (or gwo/pso/aco) |
| 2 | Compare methods | `compare_optimizers` |
| 3 | Visualize results | `evaluateWeights(weights, true)` |
| 4 | Customize | Edit `getSimulationConfig.m` |
| 5 | Deploy | Generate C++ code |

**Remember:**
- Lower J = Better performance
- 100% goal rate is ideal
- Balance weights for your environment
- Run multiple trials for reliable results

