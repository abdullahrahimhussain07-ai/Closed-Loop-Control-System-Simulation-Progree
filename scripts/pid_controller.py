import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def update(self, setpoint, current_value):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            return 0.0

        error = setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.previous_error = error
        self.last_time = current_time
        return output

if __name__ == "__main__":
    pid = PIDController(kp=0.5, ki=0.01, dg=0.1) # stable gains for test
    pid = PIDController(kp=0.6, ki=0.05, kd=0.1)
    setpoint = 0.0
    current_value = 5.0  # Initial offset error
    
    print("Starting Stable PID Simulation Test...")
    for step in range(10):
        control_output = pid.update(setpoint, current_value)
        
        # Corrected plant simulation model: 
        # Moving current value towards the setpoint based on control output
        current_value += control_output * 0.2
        
        print(f"Step {step+1}: Error = {setpoint - current_value:.2f}, Output = {control_output:.2f}")
        time.sleep(0.1)
