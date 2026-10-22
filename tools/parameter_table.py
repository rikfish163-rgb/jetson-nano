#!/usr/bin/env python3
"""Generate the tuning reference from source, without touching ROS parameters."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/robot'


def flatten(value, prefix=''):
    for key, item in value.items():
        name = prefix + str(key)
        if isinstance(item, dict):
            for row in flatten(item, name + '/'):
                yield row
        else:
            yield name, item


def cell(value):
    return json.dumps(value, ensure_ascii=False).replace('|', '\\|')


def main():
    cfg = yaml.safe_load((PACKAGE / 'config/competition.yaml').read_text())
    full = ET.parse(str(PACKAGE / 'launch/full.launch')).getroot()
    stack = ET.parse(str(PACKAGE / 'launch/stack.launch')).getroot()
    args = {a.get('name'): a.get('default', '') for a in full.findall('arg')}
    mappings = {}
    for param in stack.findall('param'):
        value = param.get('value', '')
        if value.startswith('$(arg ') and value.endswith(')'):
            key = param.get('name', '').replace('/competition/config/', '')
            mappings[key] = value[6:-1]
    rows = ['# 比赛参数表（自动生成）', '',
            '来源：`competition.yaml`、`full.launch`、`stack.launch`。不是当前 ROS 参数快照。', '',
            '修改 YAML 后重启对应进程生效；非空 launch 默认值和命令行参数会覆盖 YAML。',
            '独立左右转参数未设置时回退到通用 `turn_*`。raw 是指令档值，距离单位为米，',
            '角度通常为弧度；名称带 `_deg` 的参数为度。相机标定矩阵不要当作普通调参项。', '',
            '重新生成：`python3 tools/parameter_table.py`（Nano 工作区根目录，无车辆动作）。', '',
            '| YAML 参数 | YAML 值 | full.launch 覆盖入口 | launch 默认覆盖值 |',
            '|---|---|---|---|']
    for name, value in flatten(cfg):
        arg = mappings.get(name, '')
        override = args.get(arg, '')
        rows.append('| `%s` | `%s` | %s | %s |' % (
            name, cell(value), '`'+arg+'`' if arg else '—',
            '`'+override+'`' if override else '保留 YAML'))
    rows += ['', '## 完整启动参数', '',
             '| 参数 | full.launch 默认值 |', '|---|---|']
    for name, value in args.items():
        rows.append('| `%s` | %s |' % (name, '`'+value+'`' if value else '空：使用下层配置'))
    output = ROOT / 'docs/PARAMETERS.md'
    output.parent.mkdir(exist_ok=True)
    output.write_text('\n'.join(rows)+'\n')
    print(output)


if __name__ == '__main__':
    main()
