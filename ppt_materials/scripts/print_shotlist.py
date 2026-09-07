#!/usr/bin/env python3
"""Print 擂主级 PPT shot list from catalog.json. Does not drive the robot."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

CAT = Path(__file__).resolve().parents[1] / "catalog.json"


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--class", dest="cls", choices=list("ABCD"), help="only one class")
    p.add_argument("--next", action="store_true", help="only the next recording block")
    args = p.parse_args()
    cat = json.loads(CAT.read_text())
    shots = cat["shots"]
    if args.cls:
        shots = [s for s in shots if s["class"] == args.cls]

    print("=" * 60)
    print(cat["title"])
    print("=" * 60)
    print("\n【当前真实可展示】")
    for k, v in cat["capabilities"].items():
        st = v["status"]
        if st in ("absent", "code_exists_not_live_ros2_demo"):
            continue
        print(f"  - {k}: {st}")
        print(f"    {v.get('evidence', v.get('note', ''))}")

    print("\n【当前缺失 / 不许写成已完成】")
    for x in cat["missing_for_judges"]:
        print(f"  - {x}")
    for x in cat["do_not_claim"]:
        print(f"  - 禁止宣称：{x}")
    d = cat["capabilities"]["D_agent_goto_oa"]
    print(f"  - Agent 导航/避障代码存在但非本场 ROS2 演示：{d['note']}")

    print("\n【可马上录（车可以先不动）】")
    for s in shots:
        if not s.get("needs_drive") and s.get("kind") != "roadmap_slide":
            print(f"  {s['id']} {s['title']}  → {s['drop_into']}")

    print("\n【需要实际开车】")
    for s in shots:
        if s.get("needs_drive"):
            drive = s.get("drive", "")
            print(f"  {s['id']} {s['title']}")
            if drive:
                print(f"      动作: {drive}")

    print("\n【只能做技术路线图】")
    for s in shots:
        if s.get("kind") == "roadmap_slide":
            print(f"  {s['id']} {s['title']}")
            print(f"      {s['record']}")

    nxt = cat["next_recording"]
    print("\n【下一步最优先现场录制】")
    print("  顺序:", " → ".join(nxt["order"]))
    print("  原因:", nxt["why_first"])
    print("  RViz:", nxt["rviz"])
    print("  安全:", nxt["safety"])
    if args.next:
        print("\n先录 A1（静止点云 12s），确认墙体清楚后再连录 A2+B1（慢转）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
