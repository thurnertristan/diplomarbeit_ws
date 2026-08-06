#!/usr/bin/env python3
"""Einfaches Beispiel für die Steuerung eines Roboters.

Die Klasse RobotController stellt grundlegende Bewegungsbefehle bereit,
welche in echte Hardware-APIs oder in eine Simulation eingebunden werden
können.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass


@dataclass
class RobotState:
    """Aktueller Zustand des Roboters."""

    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0


class RobotController:
    """Minimaler Roboter-Controller mit Basisbewegungen."""

    def __init__(self, wheel_base: float = 0.5) -> None:
        self.wheel_base = wheel_base
        self.state = RobotState()
        self.is_moving = False

    def move(self, distance: float) -> RobotState:
        """Bewegt den Roboter entlang seiner aktuellen Blickrichtung."""
        self.is_moving = True
        self.state.x += distance * __import__("math").cos(__import__("math").radians(self.state.heading))
        self.state.y += distance * __import__("math").sin(__import__("math").radians(self.state.heading))
        return self.state

    def rotate(self, angle_deg: float) -> RobotState:
        """Dreht den Roboter um den angegebenen Winkel in Grad."""
        self.state.heading += angle_deg
        return self.state

    def stop(self) -> RobotState:
        """Stoppt den Roboter."""
        self.is_moving = False
        return self.state

    def report(self) -> str:
        return (
            f"Position: x={self.state.x:.2f}, y={self.state.y:.2f}, "
            f"heading={self.state.heading:.2f}°, moving={self.is_moving}"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Steuerung eines Roboters")
    parser.add_argument("command", choices=["move", "turn", "stop", "status"])
    parser.add_argument("value", nargs="?", type=float, default=0.0)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    robot = RobotController()

    if args.command == "move":
        robot.move(args.value)
    elif args.command == "turn":
        robot.rotate(args.value)
    elif args.command == "stop":
        robot.stop()

    if args.command == "status":
        print(robot.report())
    else:
        print(robot.report())


if __name__ == "__main__":
    main()
