import torch
import torch.nn as nn
import torch.optim as optim
import random

# ==========================================
# 1. شبکه عصبی (مغز ربات - کاملاً سازگار با پیکو)
# ==========================================
class NeuroPIDNetwork(nn.Module):
    def __init__(self):
        super(NeuroPIDNetwork, self).__init__()
        self.hidden = nn.Linear(3, 5)
        self.output = nn.Linear(5, 3)
        self.tanh = nn.Tanh()
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        x = self.tanh(self.hidden(x))
        return self.sigmoid(self.output(x))

# ==========================================
# 2. شبیه‌ساز فیزیکی غیرخطی (Digital Twin)
# ==========================================
class AdvancedDCMotor:
    def __init__(self):
        self.current_rpm = torch.tensor(0.0)
        self.dt = torch.tensor(0.02) # 50Hz
        
        # پارامترهای فیزیکی واقعی
        self.inertia = torch.tensor(0.90)       # مقاومت در برابر تغییر سرعت
        self.torque_const = torch.tensor(0.35)  # ضریب تبدیل ولتاژ به گشتاور
        self.stiction_limit = torch.tensor(12.0) # حداقل PWM برای حرکت گیربکس (اصطکاک ایستایی)

    def step(self, pwm, disturbance=0.0):
        # 1. محدودیت درایور موتور
        pwm = torch.clamp(pwm, -100.0, 100.0)
        
        # 2. شبیه‌سازی اصطکاک ایستایی (Deadzone)
        # اگر PWM کمتر از حد اصطکاک باشد، زور موتور به گیربکس نمی‌رسد
        effective_pwm = torch.where(torch.abs(pwm) > self.stiction_limit, 
                                    pwm - (torch.sign(pwm) * self.stiction_limit), 
                                    torch.tensor(0.0))
        
        # 3. شبیه‌سازی نویز انکودر (Gaussian Noise) - ایجاد لرزش تصادفی 1 الی 2 دور
        noise = torch.randn(1)[0] * 1.5 
        
        # 4. معادله دیفرانسیل با در نظر گرفتن بارِ خارجی (مثل دست تو روی چرخ)
        self.current_rpm = (self.inertia * self.current_rpm) + (self.torque_const * effective_pwm) - disturbance + noise
        
        return self.current_rpm

    def reset(self):
        self.current_rpm = torch.tensor(0.0)

# ==========================================
# 3. الگوریتم آموزش خشن و همه‌جانبه (Robust Training)
# ==========================================
def train_advanced_neuro_pid():
    model = NeuroPIDNetwork()
    # استفاده از L2 Regularization (weight_decay) برای جلوگیری از تولید اعداد بیش از حد بزرگ توسط شبکه
    optimizer = optim.Adam(model.parameters(), lr=0.005, weight_decay=1e-4)
    
    epochs = 400
    sim_steps = 150
    target_rpm = torch.tensor(40.0)
    
    print("🚀 Starting Advanced High-Fidelity Training...")
    
    for epoch in range(epochs):
        motor = AdvancedDCMotor()
        motor.reset()
        
        last_error = torch.tensor(0.0)
        integral_error = torch.tensor(0.0)
        loss = torch.tensor(0.0)
        
        # در 30 درصد از چرخه‌های آموزشی، یک نیروی مزاحم (دست شما) را شبیه‌سازی می‌کنیم
        apply_disturbance = random.random() > 0.7 
        disturbance_val = torch.tensor(15.0) if apply_disturbance else torch.tensor(0.0)
        
        for t in range(sim_steps):
            # اعمال اغتشاش فقط در وسط مسیر (ثانیه 1 به بعد)
            current_dist = disturbance_val if t > 50 else torch.tensor(0.0)
            
            error = target_rpm - motor.current_rpm
            d_error = (error - last_error) / motor.dt
            
            # Anti-windup نرم‌افزاری در شبیه‌ساز
            integral_error = torch.clamp(integral_error + (error * motor.dt), -50.0, 50.0)
            
            # اعمال یک فیلتر پایین‌گذر روی مشتق (دقیقاً مثل کد ++C)
            filtered_d_error = (0.2 * d_error) + (0.8 * last_error)
            last_error = error
            
            # نرمال‌سازی ورودی‌ها قبل از تزریق به شبکه
            input_tensor = torch.stack([error * 0.1, filtered_d_error * 0.01, integral_error * 0.1])
            
            nn_output = model(input_tensor)
            
            # محدوده ضرایب را منطقی در نظر می‌گیریم (Scaling ایمن)
            kp = nn_output[0] * 10.0
            ki = nn_output[1] * 2.0
            kd = nn_output[2] * 0.5
            
            pwm_cmd = (kp * error) + (ki * integral_error) + (kd * filtered_d_error)
            
            # اضافه کردن پیش‌خور (Feedforward) به شبیه‌ساز
            feedforward = target_rpm * 0.6
            total_cmd = feedforward + pwm_cmd
            
            motor.step(total_cmd, current_dist)
            
            # تابع هزینه ترکیبی: هم خطای سرعت را جریمه کن، هم تغییرات ناگهانی ضرایب (لگد زدن) را!
            time_weight = torch.tensor(float(t))
            loss = loss + (time_weight * (error ** 2)) + (0.5 * (pwm_cmd ** 2))
        
        optimizer.zero_grad()
        loss.backward()
        
        # بریدن گرادیان‌ها (Gradient Clipping) برای جلوگیری از انفجار شبکه در برابر نویز
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()
        
        if (epoch + 1) % 50 == 0:
            print(f"Epoch [{epoch+1}/{epochs}] -> Loss: {loss.item():.2f} | Disturbance Handled: {apply_disturbance}")
            
    print("\n✅ Advanced Training Completed!")
    export_to_cpp(model)

def export_to_cpp(model):
    state_dict = model.state_dict()
    w1, b1 = state_dict['hidden.weight'].detach().numpy(), state_dict['hidden.bias'].detach().numpy()
    w2, b2 = state_dict['output.weight'].detach().numpy(), state_dict['output.bias'].detach().numpy()
    
    print("\n" + "="*60)
    print("   COPY THESE RUGGED ARRAYS INTO YOUR Neuro_PID.cpp   ")
    print("="*60 + "\n")
    
    print("void Neuro_PID::initWeights() {")
    for i in range(5): print(f"    W1[{i}][0] = {w1[i][0]:.6f}f; W1[{i}][1] = {w1[i][1]:.6f}f; W1[{i}][2] = {w1[i][2]:.6f}f;")
    print()
    for i in range(5): print(f"    B1[{i}] = {b1[i]:.6f}f;")
    print()
    for i in range(3): print(f"    W2[{i}][0] = {w2[i][0]:.6f}f; W2[{i}][1] = {w2[i][1]:.6f}f; W2[{i}][2] = {w2[i][2]:.6f}f; W2[{i}][3] = {w2[i][3]:.6f}f; W2[{i}][4] = {w2[i][4]:.6f}f;")
    print()
    for i in range(3): print(f"    B2[{i}] = {b2[i]:.6f}f;")
    print("}")
    print("\n" + "="*60)

if __name__ == "__main__":
    train_advanced_neuro_pid()