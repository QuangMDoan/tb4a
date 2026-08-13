#!/usr/bin/env python3
"""Draw the live ROS 2 graph for the running TurtleBot 4 / Nav2 stack.

Queries the running ROS 2 system for topic publishers/subscribers, filters out
infrastructure noise, groups nodes into subsystems, and renders a PNG via
Graphviz (dot).

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 ros2-graph.py [-o /home/ubuntu/ros2-graph.png] [--all]

Requirements: a running ROS 2 graph, ros2 CLI on PATH, and graphviz (dot).
"""
import argparse
import shutil
import subprocess
import sys
from datetime import date

# Nodes to hide (infrastructure / duplicated helpers).
SKIP_NODE_PREFIXES = ("/transform_listener_impl", "/_ros2cli", "/_do_not_use")

# Topics to hide (pure infrastructure / bookkeeping).
SKIP_TOPICS = {"/rosout", "/parameter_events", "/bond", "/diagnostics"}
SKIP_TOPIC_SUFFIXES = ("/transition_event",)

# Subsystem grouping: node name -> (cluster id, cluster label, fill color).
CLUSTERS = {
    "sensors": ("Robot base & sensors", "#eef7ee", "#88bb88",
                ["/create3_repub", "/turtlebot4_node", "/turtlebot4_base_node",
                 "/rplidar_composition", "/oakd", "/joint_state_publisher",
                 "/robot_state_publisher"]),
    "loc": ("Localization", "#f7f0ee", "#bb9988",
            ["/map_server", "/amcl"]),
    "nav": ("Nav2 navigation", "#eef3f7", "#8899bb",
            ["/bt_navigator", "/planner_server", "/controller_server",
             "/smoother_server", "/behavior_server", "/global_costmap",
             "/local_costmap", "/velocity_smoother", "/collision_monitor",
             "/waypoint_follower"]),
}


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True).stdout


def collect_edges(show_all):
    """Return a set of (src_node, dst_node, topic) edges from the live graph."""
    topics = run(["ros2", "topic", "list"]).split()
    edges = set()
    for topic in topics:
        if not show_all and (topic in SKIP_TOPICS
                             or topic.endswith(SKIP_TOPIC_SUFFIXES)):
            continue
        info = run(["ros2", "topic", "info", topic, "-v"])
        pubs, subs, section = [], [], None
        for line in info.splitlines():
            s = line.strip()
            if s.startswith("Publisher count:"):
                section = "pub"
            elif s.startswith("Subscription count:"):
                section = "sub"
            elif s.startswith("Node name:"):
                node = "/" + s.split(":", 1)[1].strip().lstrip("/")
                if not show_all and node.startswith(SKIP_NODE_PREFIXES):
                    continue
                (pubs if section == "pub" else subs).append(node)
        for p in pubs:
            for sub in subs:
                edges.add((p, sub, topic))
    return edges


def node_id(name):
    return "n_" + name.strip("/").replace("/", "_").replace("-", "_")


def build_dot(edges, show_all):
    nodes = {n for e in edges for n in (e[0], e[1])}
    assigned = set()
    lines = [
        "digraph ros2_graph {",
        "    rankdir=LR;",
        '    graph [fontname="Helvetica", labelloc="t", '
        f'label="TurtleBot 4 + Nav2 - Live ROS 2 Graph ({date.today()})"];',
        '    node [shape=box, style="rounded,filled", fontname="Helvetica", '
        'fillcolor="#e8eef7"];',
        '    edge [fontname="Helvetica", fontsize=9, color="#555555"];',
    ]
    # Clustered subsystems.
    for cid, (label, fill, border, members) in CLUSTERS.items():
        present = [m for m in members if m in nodes]
        if not present:
            continue
        lines.append(f"    subgraph cluster_{cid} {{")
        lines.append(f'        label="{label}"; style="rounded,filled"; '
                     f'fillcolor="{fill}"; color="{border}";')
        for m in present:
            lines.append(f'        {node_id(m)} [label="{m}"];')
            assigned.add(m)
        lines.append("    }")
    # Any remaining nodes (e.g. /rviz2, or extras when --all).
    for n in sorted(nodes - assigned):
        lines.append(f'    {node_id(n)} [label="{n}", fillcolor="#f7f7e0"];')
    # Edges.
    for src, dst, topic in sorted(edges):
        lines.append(f'    {node_id(src)} -> {node_id(dst)} [label="{topic}"];')
    lines.append("}")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="Render the live ROS 2 graph to PNG.")
    ap.add_argument("-o", "--output", default="/home/ubuntu/ros2-graph.png",
                    help="Output PNG path (default: /home/ubuntu/ros2-graph.png)")
    ap.add_argument("--dot", help="Also write the intermediate Graphviz .dot file here")
    ap.add_argument("--all", action="store_true",
                    help="Include infrastructure nodes/topics (rosout, tf listeners, etc.)")
    args = ap.parse_args()

    if shutil.which("ros2") is None:
        sys.exit("error: 'ros2' not found. Source your ROS 2 setup first.")
    if shutil.which("dot") is None:
        sys.exit("error: 'dot' not found. Install graphviz (apt install graphviz).")

    edges = collect_edges(args.all)
    if not edges:
        sys.exit("error: no edges found. Is the ROS 2 graph running?")

    dot_src = build_dot(edges, args.all)
    if args.dot:
        with open(args.dot, "w") as f:
            f.write(dot_src)

    proc = subprocess.run(["dot", "-Tpng", "-o", args.output],
                          input=dot_src, text=True,
                          capture_output=True)
    if proc.returncode != 0:
        sys.exit(f"error: dot failed:\n{proc.stderr}")

    print(f"Wrote {args.output} ({len(edges)} edges, "
          f"{len({n for e in edges for n in (e[0], e[1])})} nodes)")


if __name__ == "__main__":
    main()
