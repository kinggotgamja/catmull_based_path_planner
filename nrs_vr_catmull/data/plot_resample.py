import numpy as np
import matplotlib.pyplot as plt

# 파일 경로
filename = 'resample_path.txt'  # ← 여기에 실제 파일 경로로 바꿔줘

# 텍스트 파일에서 전체 9열 데이터 불러오기
data = np.loadtxt(filename, usecols=range(9))  # 0~8열

# 시간축 (0.002초 간격)
time = np.arange(len(data)) * 0.002

# 서브플롯 준비
fig, axs = plt.subplots(3, 3, figsize=(14, 8))
fig.suptitle("Position / Orientation / Force vs Time", fontsize=16)

labels = [
    ['Position X', 'Orientation Roll', 'Force X'],
    ['Position Y', 'Orientation Pitch', 'Force Y'],
    ['Position Z', 'Orientation Yaw', 'Force Z']
]

# 각 subplot에 데이터 그리기
for row in range(3):      # 행 (x/y/z or roll/pitch/yaw or fx/fy/fz)
    for col in range(3):  # 열 (pos, ori, force)
        ax = axs[row][col]
        data_col = col * 3 + row  # 순서대로 0~8열
        ax.plot(time, data[:, data_col], label=labels[row][col], linewidth=1)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(labels[row][col])
        ax.grid(True)
        ax.legend(loc="upper right", fontsize=8)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # 제목 공간 확보
plt.show()
