# catmull_based_path_planner

Catmull-Rom spline은 주어진 waypoints (discrete points) 들을 자연스럽게 연결해주는 곡선을 만드는 interpolation 방식.

🎯 특징: 각 포인트를 정확히 지나가는 곡선을 만들어줌 (interpolating spline)

🌊 곡선이 부드럽고 자연스럽게 이어져서 path planning, 애니메이션, 로봇 경로 생성 등에 널리 사용

📐 일정한 간격으로 샘플링하면 일정한 속도로 움직이는 path도 만들 수 있음 (속도 보정을 추가하면 완벽함)

## Credit Requirement

This repository's code is free to use, modify, and distribute.
However, the following conditions must be met:

- **Credit the Source**: When using this code, you must include the following information:
    ```
    Source: https://github.com/kinggotgamja/catmull_based_path_planner
    Author: Jaeyoon Shim
    ```
- No other license restrictions are applied.
