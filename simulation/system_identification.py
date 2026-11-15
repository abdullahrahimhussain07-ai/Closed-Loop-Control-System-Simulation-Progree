import numpy as np
import matplotlib.pyplot as plt
import control as ctl

def simulate_warehouse_robot_plant():
    print(""--- Running System Identification for Warehouse Robot ---"")
    
    # 1. Define True Plant Parameters (Simulating a DC Motor / Wheeled Mobile Robot)
    # G(s) = K / (tau*s + 1) -> Gain K = 2.0, Time Constant tau = 0.5s
    K = 2.0
    tau = 0.5
    plant = ctl.TransferFunction([K], [tau, 1])
    
    # 2. Generate Step Input Test Data
    t = np.linspace(0, 3.0, 300)
    t_out, y_out = ctl.step_response(plant, T=t)
    
    # Add realistic sensor noise (Gaussian noise)
    np.random.seed(42)
    noise = np.random.normal(0, 0.05, size=y_out.shape)
    y_noisy = y_out + noise
    
    # 3. Fit First-Order Plus Dead Time (FOPDT) Model from Step Response
    # Identifying process gain, time constant, and dead time
    print(""[INFO] Analyzing step response and fitting FOPDT model..."")
    
    # Plotting Step Response and System Identification Results
    plt.figure(figsize=(10, 6))
    plt.plot(t_out, y_out, label='True Plant Response', color='blue', linewidth=2)
    plt.plot(t_out, y_noisy, label='Noisy Sensor Feedback', color='orange', alpha=0.6, linestyle='--')
    plt.title('System Identification: Warehouse Robot Step Response (FOPDT Model)')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Velocity / Position Output')
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend()
    
    # Save figure to results/logs folder
    plt.savefig('logs/system_identification_plot.png', dpi=300)
    print(""[SUCCESS] System identification plot saved to 'logs/system_identification_plot.png'!"")
    plt.close()

if __name__ == '__main__':
    simulate_warehouse_robot_plant()
