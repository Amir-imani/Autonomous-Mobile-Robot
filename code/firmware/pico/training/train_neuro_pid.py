import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np

class NeuroPIDNetwork(nn.Module):
    def __init__(self):
        super(NeuroPIDNetwork, self).__init__()
        self.hidden = nn.Linear(3, 5)
        self.output = nn.Linear(5, 3)
        self.tanh = nn.Tanh()
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        x = self.hidden(x)
        x = self.tanh(x)
        x = self.output(x)
        x = self.sigmoid(x)
        return x

class DCMotorSimulation:
    def __init__(self):
        # Keep simulated state in the tensor computation graph
        self.current_rpm = torch.tensor(0.0)
        self.dt = torch.tensor(0.02)
        self.a = torch.tensor(0.85)
        self.b = torch.tensor(0.45)

    def step(self, pwm):
        # Apply tensor-based saturation to the motor command
        pwm = torch.clamp(pwm, -100.0, 100.0)
        self.current_rpm = (self.a * self.current_rpm) + (self.b * pwm)
        return self.current_rpm

    def reset(self):
        self.current_rpm = torch.tensor(0.0)

def train_neuro_pid():
    model = NeuroPIDNetwork()
    optimizer = optim.Adam(model.parameters(), lr=0.01)
    
    epochs = 300
    sim_steps = 100
    target_rpm = torch.tensor(40.0)
    
    print("Starting Neuro-PID Training Loop...")
    
    for epoch in range(epochs):
        motor = DCMotorSimulation()
        motor.reset()
        
        last_error = torch.tensor(0.0)
        integral_error = torch.tensor(0.0)
        loss = torch.tensor(0.0)
        
        for t in range(sim_steps):
            error = target_rpm - motor.current_rpm
            d_error = (error - last_error) / motor.dt
            integral_error = integral_error + (error * motor.dt)
            last_error = error
            input_tensor = torch.stack([error, d_error, integral_error])
            nn_output = model(input_tensor)
            kp = nn_output[0] * 50.0
            ki = nn_output[1] * 10.0
            kd = nn_output[2] * 5.0
            pwm_cmd = (kp * error) + (ki * integral_error) + (kd * d_error)
            motor.step(pwm_cmd)
            time_weight = torch.tensor(float(t))
            loss = loss + (time_weight * (error ** 2))
        
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        if (epoch + 1) % 50 == 0:
            print(f"Epoch [{epoch+1}/{epochs}] -> Optimization Loss (ITAE): {loss.item():.2f}")
            
    print("\nTraining Completed Successfully!")
    export_to_cpp(model)

def export_to_cpp(model):
    state_dict = model.state_dict()
    w1 = state_dict['hidden.weight'].detach().numpy()
    b1 = state_dict['hidden.bias'].detach().numpy()
    w2 = state_dict['output.weight'].detach().numpy()
    b2 = state_dict['output.bias'].detach().numpy()
    print("\n" + "="*50)
    print("   COPY THESE ARRAYS INTO YOUR Neuro_PID.cpp   ")
    print("="*50 + "\n")
    print("void Neuro_PID::initWeights() {")
    print("    // Layer 1 Weights (Hidden Layer)")
    for i in range(5):
        print(f"    W1[{i}][0] = {w1[i][0]:.6f}f; W1[{i}][1] = {w1[i][1]:.6f}f; W1[{i}][2] = {w1[i][2]:.6f}f;")
    print("\n    // Layer 1 Biases")
    for i in range(5):
        print(f"    B1[{i}] = {b1[i]:.6f}f;")
    print("\n    // Layer 2 Weights (Output Layer)")
    for i in range(3):
        row_str = ", ".join([f"W2[{i}][{j}] = {w2[i][j]:.6f}f" for j in range(5)])
        print(f"    {row_str};")
    print("\n    // Layer 2 Biases")
    for i in range(3):
        print(f"    B2[{i}] = {b2[i]:.6f}f;")
    print("}")
    print("\n" + "="*50)

if __name__ == "__main__":
    train_neuro_pid()