---
name: rh-ros2-health-check
description: Template skill for read-only ROS 2 health checks (graph snapshot, topic/QoS, TF, diagnostics). Requires a ROS 2 system or rosbag; when none is available, explain what is needed instead of pretending.
whenToUse: Use when the user has a ROS 2 system/rosbag and asks for health checks. Without ROS 2 tooling this skill cannot run — say so clearly.
modelInvocable: true
userInvocable: true
---

# ROS 2 read-only health check (template)

This skill is a placeholder for the planned ROS 2 bridge (phase 5 of the roadmap). The Robotic Harness demo does not ship a ROS 2 bridge yet.

1. **Check availability.** Verify that `ros2` CLI and the target ROS 2 distribution are available in the environment. If not, stop here and report exactly what is missing (e.g. "ROS 2 Humble not installed", "ros2 bag not available").
2. **Graph snapshot.** When available, collect: node list, topic list with types, service/action lists.
3. **Topic profile.** For each topic of interest: publisher/subscriber counts and message rate (sampled, not unbounded).
4. **TF audit.** Build the frame tree, check for missing parents, staleness and timeouts.
5. **Diagnostics.** Read `/diagnostics` and list active errors/warnings.
6. **Report.** Facts only; no write operations are allowed by this skill. Everything here is read-only.

## Contribution note

The repository welcomes contributors to implement the ROS 2 bridge (see CONTRIBUTING.md). Until then, this skill documents the intended check order so the interface stays stable.
