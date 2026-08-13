# Security Policy

## Scope

Robotic Harness is an **experimental, simulation-first** DeepSeek Harness
plugin suite. It is a research and teaching tool, **not** a functional-safety
system and **not** a certified robot controller. See
[docs/safety-boundary.md](docs/safety-boundary.md) for the complete boundary
statement.

## Supported status

| Version | Supported |
|---|---|
| 0.1.x (demo) | Source-level fixes; no maintenance guarantee |

## Reporting a vulnerability

Do **not** open a public issue for security-sensitive findings. Report them
privately to the maintainers (GitHub Security Advisories when available, or a
direct message to the repository owners).

Please include:

- the affected version and environment (OS, Python, DSH version);
- a minimal reproduction;
- the impact (what an attacker could do) and your suggested fix.

## What we treat as in-scope

- Command injection or path traversal in the worker CLI or the DSH tools
  (the worker spawns a configured interpreter — a malicious `pythonPath` is
  trusted configuration, not a vulnerability).
- Unsafe deserialization in the JSON protocol.
- Any tool that could issue an unapproved physical action.

## Out of scope / by design

- The plugin never commands real hardware; there is intentionally **no**
  arbitrary topic-publish or real-robot write tool.
- Simulation results are evidence, not safety certification.
- The suction grasp in the demo is kinematic by design and documented as such
  in run configs and reports.
