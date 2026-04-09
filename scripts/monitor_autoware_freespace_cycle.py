#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_internal_planning_msgs.msg import Scenario
from autoware_planning_msgs.msg import LaneletRoute
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def iso_now(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).astimezone().isoformat(timespec="milliseconds")


def speed_mps(msg: Odometry) -> float:
    linear = msg.twist.twist.linear
    return math.hypot(linear.x, linear.y)


def pose_distance_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


class Monitor(Node):
    def __init__(self, output_prefix: str, stop_threshold_mps: float, stop_hold_sec: float) -> None:
        super().__init__("autoware_freespace_cycle_monitor")

        self.output_prefix = Path(output_prefix)
        self.output_prefix.parent.mkdir(parents=True, exist_ok=True)
        self.events_path = self.output_prefix.with_suffix(".events.jsonl")
        self.summary_path = self.output_prefix.with_suffix(".summary.txt")
        self.status_path = self.output_prefix.with_suffix(".status.json")

        self.stop_threshold_mps = stop_threshold_mps
        self.stop_hold_sec = stop_hold_sec

        self.start_wall = time.time()
        self.start_mono = time.monotonic()
        self.last_status_write = 0.0

        self.event_first_time: dict[str, float] = {}
        self.current: dict[str, Any] = {
            "autonomous_enabled": False,
            "operation_mode": None,
            "route_state": None,
            "scenario": None,
            "force_parking": False,
            "parking_completed": False,
            "obstacle_stop_active": False,
            "obstacle_stop_factor_count": 0,
            "speed_mps": None,
            "parking_trajectory_points": 0,
            "final_trajectory_points": 0,
            "rejoin_goal": None,
            "ego_xy": None,
            "distance_to_rejoin_goal_m": None,
        }

        self.route_state_labels = self._extract_labels(RouteState)
        self.op_mode_labels = self._extract_labels(OperationModeState)

        self._stopped_since_mono: float | None = None
        self._seen_parking = False

        self.create_subscription(OperationModeState, "/api/operation_mode/state", self._on_operation_mode, 10)
        self.create_subscription(RouteState, "/api/routing/state", self._on_route_state, 10)
        self.create_subscription(Scenario, "/planning/scenario_planning/scenario", self._on_scenario, 10)
        self.create_subscription(Bool, "/planning/freespace_bypass/force_parking", self._on_force_parking, 10)
        self.create_subscription(Bool, "/planning/scenario_planning/parking/is_completed", self._on_parking_completed, 10)
        self.create_subscription(PlanningFactorArray, "/planning/planning_factors/obstacle_stop", self._on_obstacle_stop, 10)
        self.create_subscription(Trajectory, "/planning/scenario_planning/parking/trajectory", self._on_parking_trajectory, 10)
        self.create_subscription(Trajectory, "/planning/trajectory", self._on_final_trajectory, 10)
        self.create_subscription(LaneletRoute, "/planning/freespace_bypass/route", self._on_temp_route, 10)
        self.create_subscription(Odometry, "/localization/kinematic_state", self._on_odometry, 20)

        self.create_timer(0.5, self._on_timer)

        self._record_event("monitor_started", {"output_prefix": str(self.output_prefix)}, once=True)
        self._write_outputs(force=True)

    @staticmethod
    def _extract_labels(cls: type) -> dict[int, str]:
        labels: dict[int, str] = {}
        for name in dir(cls):
            if not name.isupper():
                continue
            value = getattr(cls, name, None)
            if isinstance(value, int):
                labels[value] = name
        return labels

    def _uptime_sec(self) -> float:
        return time.monotonic() - self.start_mono

    def _current_ego_xy(self) -> tuple[float, float] | None:
        value = self.current.get("ego_xy")
        if not value:
            return None
        return float(value["x"]), float(value["y"])

    def _update_rejoin_distance(self) -> None:
        goal = self.current.get("rejoin_goal")
        ego_xy = self._current_ego_xy()
        if not goal or ego_xy is None:
            self.current["distance_to_rejoin_goal_m"] = None
            return
        self.current["distance_to_rejoin_goal_m"] = pose_distance_xy(
            ego_xy[0], ego_xy[1], float(goal["x"]), float(goal["y"])
        )

    def _record_event(self, name: str, details: dict[str, Any] | None = None, once: bool = False) -> None:
        if once and name in self.event_first_time:
            return

        wall = time.time()
        entry = {
            "time": iso_now(wall),
            "uptime_sec": round(self._uptime_sec(), 3),
            "event": name,
            "details": details or {},
        }
        with self.events_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=True) + "\n")

        self.event_first_time.setdefault(name, wall)
        self._write_outputs(force=True)

    def _write_outputs(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_status_write < 1.0:
            return
        self.last_status_write = now

        status = {
            "generated_at": iso_now(time.time()),
            "uptime_sec": round(self._uptime_sec(), 3),
            "current": self.current,
            "event_first_time": {k: iso_now(v) for k, v in self.event_first_time.items()},
            "durations_sec": self._durations(),
        }
        self.status_path.write_text(json.dumps(status, ensure_ascii=True, indent=2), encoding="utf-8")
        self.summary_path.write_text(self._build_summary_text(status), encoding="utf-8")

    def _durations(self) -> dict[str, float | None]:
        def delta(a: str, b: str) -> float | None:
            ta = self.event_first_time.get(a)
            tb = self.event_first_time.get(b)
            if ta is None or tb is None:
                return None
            return round(tb - ta, 3)

        return {
            "start_to_autonomous_enabled": delta("monitor_started", "autonomous_enabled"),
            "autonomous_enabled_to_route_set": delta("autonomous_enabled", "route_set"),
            "route_set_to_obstacle_stop_active": delta("route_set", "obstacle_stop_active"),
            "obstacle_stop_active_to_vehicle_stopped": delta("obstacle_stop_active", "vehicle_stopped_after_obstacle_stop"),
            "vehicle_stopped_to_force_parking_enabled": delta("vehicle_stopped_after_obstacle_stop", "force_parking_enabled"),
            "force_parking_enabled_to_scenario_parking_entered": delta("force_parking_enabled", "scenario_parking_entered"),
            "scenario_parking_entered_to_parking_trajectory_ready": delta("scenario_parking_entered", "parking_trajectory_ready"),
            "scenario_parking_entered_to_parking_completed": delta("scenario_parking_entered", "parking_completed"),
            "scenario_parking_entered_to_lane_driving_returned": delta("scenario_parking_entered", "scenario_lane_driving_returned"),
            "force_parking_enabled_to_force_parking_disabled": delta("force_parking_enabled", "force_parking_disabled"),
        }

    def _build_summary_text(self, status: dict[str, Any]) -> str:
        lines = [
            "Autoware Freespace Cycle Monitor",
            f"generated_at: {status['generated_at']}",
            f"uptime_sec: {status['uptime_sec']}",
            "",
            "Current State:",
        ]
        for key, value in status["current"].items():
            lines.append(f"{key}: {value}")

        lines += ["", "Event First Seen:"]
        for key, value in status["event_first_time"].items():
            lines.append(f"{key}: {value}")

        lines += ["", "Durations (sec):"]
        for key, value in status["durations_sec"].items():
            lines.append(f"{key}: {value}")

        return "\n".join(lines) + "\n"

    def _on_operation_mode(self, msg: OperationModeState) -> None:
        mode = int(msg.mode)
        enabled = bool(msg.is_autoware_control_enabled)
        previous_mode = self.current.get("operation_mode")
        previous_enabled = self.current.get("autonomous_enabled")
        self.current["operation_mode"] = self.op_mode_labels.get(mode, str(mode))
        self.current["autonomous_enabled"] = enabled and mode == OperationModeState.AUTONOMOUS

        if previous_mode != self.current["operation_mode"] or previous_enabled != self.current["autonomous_enabled"]:
            self._record_event(
                "operation_mode_changed",
                {
                    "mode": self.current["operation_mode"],
                    "autoware_control_enabled": enabled,
                    "in_transition": bool(msg.is_in_transition),
                },
            )

        if self.current["autonomous_enabled"]:
            self._record_event("autonomous_enabled", {"mode": self.current["operation_mode"]}, once=True)

    def _on_route_state(self, msg: RouteState) -> None:
        state = int(msg.state)
        label = self.route_state_labels.get(state, str(state))
        if label != self.current.get("route_state"):
            self.current["route_state"] = label
            self._record_event("route_state_changed", {"state": label})
        if state == RouteState.SET:
            self._record_event("route_set", {"state": label}, once=True)
        if state == RouteState.ARRIVED:
            self._record_event("route_arrived", {"state": label}, once=True)

    def _on_scenario(self, msg: Scenario) -> None:
        scenario = str(msg.current_scenario)
        previous = self.current.get("scenario")
        self.current["scenario"] = scenario
        if scenario != previous:
            self._record_event("scenario_changed", {"scenario": scenario})
        if scenario == Scenario.PARKING:
            self._seen_parking = True
            self._record_event("scenario_parking_entered", {"scenario": scenario}, once=True)
        if self._seen_parking and scenario == Scenario.LANEDRIVING:
            self._record_event("scenario_lane_driving_returned", {"scenario": scenario}, once=True)

    def _on_force_parking(self, msg: Bool) -> None:
        previous = bool(self.current.get("force_parking"))
        current = bool(msg.data)
        self.current["force_parking"] = current
        if current != previous:
            self._record_event("force_parking_changed", {"force_parking": current})
        if current:
            self._record_event("force_parking_enabled", {"force_parking": current}, once=True)
        elif "force_parking_enabled" in self.event_first_time:
            self._record_event("force_parking_disabled", {"force_parking": current}, once=True)

    def _on_parking_completed(self, msg: Bool) -> None:
        previous = bool(self.current.get("parking_completed"))
        current = bool(msg.data)
        self.current["parking_completed"] = current
        if current != previous:
            self._record_event("parking_completed_changed", {"parking_completed": current})
        if current:
            self._record_event("parking_completed", {"parking_completed": current}, once=True)

    def _on_obstacle_stop(self, msg: PlanningFactorArray) -> None:
        factor_count = len(msg.factors)
        modules = sorted({factor.module for factor in msg.factors if factor.module})
        active = factor_count > 0
        previous_active = bool(self.current.get("obstacle_stop_active"))
        previous_count = int(self.current.get("obstacle_stop_factor_count", 0))
        self.current["obstacle_stop_active"] = active
        self.current["obstacle_stop_factor_count"] = factor_count
        if active != previous_active or factor_count != previous_count:
            self._record_event(
                "obstacle_stop_state_changed",
                {"active": active, "factor_count": factor_count, "modules": modules},
            )
        if active:
            self._record_event("obstacle_stop_active", {"factor_count": factor_count, "modules": modules}, once=True)

    def _on_parking_trajectory(self, msg: Trajectory) -> None:
        points = len(msg.points)
        previous = int(self.current.get("parking_trajectory_points", 0))
        self.current["parking_trajectory_points"] = points
        if points != previous:
            self._record_event("parking_trajectory_changed", {"points": points})
        if points > 1:
            self._record_event("parking_trajectory_ready", {"points": points}, once=True)

    def _on_final_trajectory(self, msg: Trajectory) -> None:
        points = len(msg.points)
        previous = int(self.current.get("final_trajectory_points", 0))
        self.current["final_trajectory_points"] = points
        if points != previous:
            self._record_event("final_trajectory_changed", {"points": points})
        if self._seen_parking and self.current.get("scenario") == Scenario.LANEDRIVING and points > 1:
            self._record_event("lane_driving_trajectory_ready", {"points": points}, once=True)

    def _on_temp_route(self, msg: LaneletRoute) -> None:
        goal = {
            "x": float(msg.goal_pose.position.x),
            "y": float(msg.goal_pose.position.y),
            "z": float(msg.goal_pose.position.z),
        }
        previous = self.current.get("rejoin_goal")
        self.current["rejoin_goal"] = goal
        self._update_rejoin_distance()
        if previous != goal:
            self._record_event("rejoin_goal_updated", goal)

    def _on_odometry(self, msg: Odometry) -> None:
        current_speed = speed_mps(msg)
        self.current["speed_mps"] = round(current_speed, 4)
        self.current["ego_xy"] = {
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
        }
        self._update_rejoin_distance()

        if current_speed <= self.stop_threshold_mps:
            if self._stopped_since_mono is None:
                self._stopped_since_mono = time.monotonic()
        else:
            self._stopped_since_mono = None

    def _on_timer(self) -> None:
        if (
            self.current.get("obstacle_stop_active")
            and self._stopped_since_mono is not None
            and time.monotonic() - self._stopped_since_mono >= self.stop_hold_sec
        ):
            self._record_event(
                "vehicle_stopped_after_obstacle_stop",
                {
                    "speed_mps": self.current.get("speed_mps"),
                    "hold_sec": self.stop_hold_sec,
                },
                once=True,
            )

        distance = self.current.get("distance_to_rejoin_goal_m")
        if distance is not None and distance <= 1.0:
            self._record_event("rejoin_goal_reached", {"distance_to_rejoin_goal_m": round(distance, 3)}, once=True)

        self._write_outputs(force=False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--stop-threshold-mps", type=float, default=0.05)
    parser.add_argument("--stop-hold-sec", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = Monitor(args.output_prefix, args.stop_threshold_mps, args.stop_hold_sec)
    try:
        rclpy.spin(node)
    finally:
        node._write_outputs(force=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
