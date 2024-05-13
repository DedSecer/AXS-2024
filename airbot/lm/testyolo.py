from ultralytics import YOLO

model = YOLO("/root/Workspace/AXS_baseline/ckpt/yolo-v8/best_real.pt")

res = model("/root/Workspace/AXS_baseline/ICRA2024-Sim2Real-AXS/src/airbot/lm/white.png")
print(res)