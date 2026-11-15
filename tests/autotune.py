import optuna
import numpy as np

# Suppress Optuna logging noise for clean terminal output
optuna.logging.set_verbosity(optuna.logging.WARNING)

def evaluate_pid_performance(kp, ki, kd):
    'Simulates the warehouse robot response with given PID gains and returns the cost (RMSE/ITAE).'
    dt = 0.05
    steps = 200
    setpoint = 1.0
    current_value = 0.0
    
    previous_error = 0.0
    integral = 0.0
    
    error_sum = 0.0
    control_effort = 0.0
    
    for _ in range(steps):
        error = setpoint - current_value
        integral += error * dt
        derivative = (error - previous_error) / dt
        
        # PID Output
        output = (kp * error) + (ki * integral) + (kd * derivative)
        
        # Plant simulation model with slight inertia
        current_value += output * 0.1
        
        error_sum += abs(error) * dt
        control_effort += (output ** 2) * dt
        previous_error = error
        
    # Cost function: Penalize tracking error and excessive control effort
    cost = error_sum + 0.01 * control_effort
    return cost

def objective(trial):
    kp = trial.suggest_float('kp', 0.1, 10.0)
    ki = trial.suggest_float('ki', 0.0, 1.0)
    kd = trial.suggest_float('kd', 0.0, 5.0)
    
    cost = evaluate_pid_performance(kp, ki, kd)
    return cost

if __name__ == '__main__':
    print('--- Starting Bayesian Hyperparameter Optimization for PID Gains ---')
    
    # Run Optuna study to find optimal gains
    study = optuna.create_study(direction='minimize')
    study.optimize(objective, n_trials=50)
    
    print('\n[SUCCESS] Optimization Complete!')
    print(f'Optimal Gains Found:')
    print(f'  Kp: {study.best_params[\'kp\']:.4f}')
    print(f'  Ki: {study.best_params[\'ki\']:.4f}')
    print(f'  Kd: {study.best_params[\'kd\']:.4f}')
    print(f'  Best Cost: {study.best_value:.4f}')
