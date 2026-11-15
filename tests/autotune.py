import optuna
import numpy as np

optuna.logging.set_verbosity(optuna.logging.WARNING)

def evaluate_pid_performance(kp, ki, kd):
    'Simulates the warehouse robot response with given PID gains and returns the cost.'
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
        output = (kp * error) + (ki * integral) + (kd * derivative)
        current_value += output * 0.1
        error_sum += abs(error) * dt
        control_effort += (output ** 2) * dt
        previous_error = error
        
    return error_sum + 0.01 * control_effort

def objective(trial):
    kp = trial.suggest_float('kp', 0.1, 10.0)
    ki = trial.suggest_float('ki', 0.0, 1.0)
    kd = trial.suggest_float('kd', 0.0, 5.0)
    return evaluate_pid_performance(kp, ki, kd)

if __name__ == '__main__':
    print('--- Starting Bayesian Hyperparameter Optimization for PID Gains ---')
    study = optuna.create_study(direction='minimize')
    study.optimize(objective, n_trials=50)
    
    bp = study.best_params
    print('\n[SUCCESS] Optimization Complete!')
    print('Optimal Gains Found:')
    print(f'  Kp: {bp["kp"]:.4f}')
    print(f'  Ki: {bp["ki"]:.4f}')
    print(f'  Kd: {bp["kd"]:.4f}')
    print(f'  Best Cost: {study.best_value:.4f}')
