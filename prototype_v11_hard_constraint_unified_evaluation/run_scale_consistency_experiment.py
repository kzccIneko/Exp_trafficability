"""可选：跨尺度路径一致性检验接口。"""
from __future__ import annotations
from pathlib import Path

def main():
    print('跨尺度路径一致性检验为可选模块。建议在提供 12.5m 与 30m 同区 DEM 后运行。')
    print('本模块设计目标：在 30m 上规划路径，再在 12.5m 参考 DEM 上复核车辆能力利用率。')
    print('当前 v10 主流程不强制运行该实验。')

if __name__ == '__main__':
    main()
