"""完整闭环仿真的基本契约测试。"""

import numpy as np

from fr3_control.simulation import simulate


def test_shortened_contract_via_full_nominal_run() -> None:
    """运行标称计算力矩实验并检查输出形状、有限性和电压约束。"""
    log, metrics = simulate("ctc", "nominal", save=False)
    assert np.all(np.isfinite(log["q"]))
    assert log["time"].shape[0] == log["q"].shape[0]
    assert metrics["ee_rmse_mm"] >= 0.0
    assert metrics["peak_voltage_v"] <= 48.0 + 1e-8
