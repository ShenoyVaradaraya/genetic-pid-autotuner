import random
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

# Constants
POPULATION_SIZE = 500
CROSSOVER_RATE = 0.9
GENERATIONS = 200
TARGET_FITNESS = 0.01
RUNS = 20  # Number of runs

# PID Controller Parameters
Kp_range = (5.0, 10.0)
Ki_range = (0.5, 5.0)
Kd_range = (0.5, 1.0)

# System parameters
process_gain = 2.0
process_time_constant = 3.0
delay = 1

# Simulation time and step
simulation_time = 10
time_step = 0.1

# Variables to store step response characteristics
generation_rise_times = []
generation_settling_times = []
generation_overshoots = []
generation_steady_state_errors = []

# Saturate function to limit the control output
def saturate(value, min_value, max_value):
    return max(min(value, max_value), min_value)

# PID controller with anti-windup
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def calculate_output(self, error, time_step):
        self.integral += error * time_step
        derivative = (error - self.prev_error) / time_step
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = saturate(output, -1.0, 1.0)  # Saturate the control output
        self.prev_error = error
        return output

# System response function for a first-order system with delay
def simulate_system(Kp, Ki, Kd):
    setpoint = np.ones(int(simulation_time / time_step) + 1)
    output = np.zeros_like(setpoint)

    pid_controller = PIDController(Kp, Ki, Kd)

    for i in range(1, len(setpoint)):
        error = setpoint[i - 1] - output[i - 1]

        # Calculate control output using the PID controller
        control_output = pid_controller.calculate_output(error, time_step)

        # Simulate process dynamics with delay
        process_input = control_output if i >= delay else 0
        output[i] = output[i - 1] + (process_gain * process_input - output[i - 1]) * (time_step / process_time_constant)

    # Calculate fitness as the integral of absolute error
    fitness = np.sum(np.abs(setpoint - output)) * time_step

    # Calculate step response characteristics
    steady_state_error = np.abs(setpoint[-1] - output[-1])
    rise_time, settling_time, overshoot = calculate_step_response_characteristics(setpoint, output)

    return output, fitness, steady_state_error, rise_time, settling_time, overshoot

# Initialize population with a more diverse range of PID parameters
def initialize_population():
    population = []
    for _ in range(POPULATION_SIZE):
        Kp = random.uniform(*Kp_range)
        Ki = random.uniform(*Ki_range)
        Kd = random.uniform(*Kd_range)
        population.append((Kp, Ki, Kd))
    return population

# Calculate fitness for each individual in the population
def calculate_fitness(population):
    fitness_scores = []
    for ind in population:
        _, fitness, _, _, _, _ = simulate_system(*ind)
        fitness_scores.append(fitness)
    return fitness_scores

# Select parents based on roulette wheel selection
def select_parents(population, fitness_scores):
    total_fitness = sum(fitness_scores)
    probabilities = [fit / total_fitness for fit in fitness_scores]
    parents = random.choices(population, weights=probabilities, k=2)
    return parents

# Perform crossover to create offspring
def crossover(parent1, parent2):
    crossover_point = random.randint(0, len(parent1) - 1)
    child1 = parent1[:crossover_point] + parent2[crossover_point:]
    child2 = parent2[:crossover_point] + parent1[crossover_point:]
    return child1, child2

# Perform mutation on an individual
def mutate(ind, generation):
    mutated_ind = list(ind)
    mutation_rate = 1.0 / (generation + 1)  # Dynamic mutation rate

    for i in range(len(mutated_ind)):
        if random.random() < mutation_rate:
            mutated_ind[i] += random.uniform(-0.5, 0.5)  # Increased the range of mutation
            if i == 0:
                mutated_ind[i] = np.clip(mutated_ind[i], *Kp_range)
            elif i == 1:
                mutated_ind[i] = np.clip(mutated_ind[i], *Ki_range)
            else:
                mutated_ind[i] = np.clip(mutated_ind[i], *Kd_range)

    return tuple(mutated_ind)

# Calculate step response characteristics
def calculate_step_response_characteristics(setpoint, output):
    rise_time = calculate_rise_time(setpoint, output)
    settling_time = calculate_settling_time(setpoint, output)
    overshoot = calculate_overshoot(setpoint, output)
    return rise_time, settling_time, overshoot

# Calculate rise time
def calculate_rise_time(setpoint, output):
    for i in range(len(output)):
        if output[i] >= 0.9 * setpoint[-1]:
            return i * time_step
    return float('inf')

# Calculate settling time
def calculate_settling_time(setpoint, output):
    for i in range(len(output) - 1, 0, -1):
        if abs(output[i] - setpoint[-1]) >= 0.02 * setpoint[-1]:
            return i * time_step
    return float('inf')

# Calculate overshoot
def calculate_overshoot(setpoint, output):
    peak_index = np.argmax(output)
    overshoot = (output[peak_index] - setpoint[-1]) / setpoint[-1] * 100
    return overshoot

# Main Genetic Algorithm loop
def genetic_algorithm():
    best_params_across_runs = None
    best_fitness_across_runs = float('inf')

    for run in tqdm(range(RUNS), desc="Genetic Algorithm Runs"):
        population = initialize_population()

        # Use tqdm for a progress bar
        for generation in tqdm(range(GENERATIONS), desc=f"Run {run + 1} Progress", leave=False):
            fitness_scores = calculate_fitness(population)
            best_fitness = min(fitness_scores)

            if best_fitness < TARGET_FITNESS:
                break

            best_individual_index = fitness_scores.index(min(fitness_scores))
            best_individual = population[best_individual_index]

            new_population = []

            while len(new_population) < POPULATION_SIZE:
                parent1, parent2 = select_parents(population, fitness_scores)
                child1, child2 = crossover(parent1, parent2)
                child1 = mutate(child1, generation)
                child2 = mutate(child2, generation)
                new_population.extend([child1, child2])

            population = new_population

        if best_fitness < best_fitness_across_runs:
            best_params_across_runs = best_individual
            best_fitness_across_runs = best_fitness

    print("Best PID Parameters Across Runs:", best_params_across_runs)
    print("Best Fitness Across Runs:", best_fitness_across_runs)

    # Simulate the system with the best individual across runs
    best_output, _, _, _, _, _ = simulate_system(*best_params_across_runs)

    # Plot the response
    time_points = np.arange(0, simulation_time + time_step, time_step)
    plt.plot(time_points, best_output, label="System Response")
    plt.plot(time_points, np.ones_like(time_points), label="Setpoint", linestyle='--')
    plt.title("System Response with Best PID Parameters Across Runs")
    plt.xlabel("Time (s)")
    plt.ylabel("Output")
    plt.legend()
    plt.show()

    # Calculate performance metrics
    rise_time, overshoot, settling_time, steady_state_error = calculate_performance_metrics(time_points, best_output)

    # Print performance metrics
    print("\nPerformance Metrics:")
    print(f"Rise Time: {rise_time:.2f} seconds")
    print(f"Overshoot: {overshoot:.2f}%")
    print(f"Settling Time: {settling_time:.2f} seconds")
    print(f"Steady-State Error: {steady_state_error:.4f}")


def calculate_performance_metrics(time_points, output):
    # Calculate rise time
    rise_time_index = np.where(output >= 0.9)[0][0]
    rise_time = time_points[rise_time_index]

    # Calculate overshoot
    peak_index = np.argmax(output)
    overshoot = ((output[peak_index] - 1) / 1) * 100

    # Calculate settling time
    settling_time_index = np.where((output >= 0.95) & (output <= 1.05))[0][-1]
    settling_time = time_points[settling_time_index]

    # Calculate steady-state error
    steady_state_error = np.abs(output[-1] - 1)

    return rise_time, overshoot, settling_time, steady_state_error


# Run the genetic algorithm
genetic_algorithm()
