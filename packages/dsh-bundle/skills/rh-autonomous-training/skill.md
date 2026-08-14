---
name: rh-autonomous-training
description: Orchestrate the full training workflow — check the configured training server, plan the training (objective, hyperparameters, data), discover supplementary datasets, prepare the job, and only with explicit human confirmation submit it remotely, then track and report.
whenToUse: Use when the user wants to train (or continue training) a model and expects the agent to organize the server, plan, data and job execution.
modelInvocable: true
userInvocable: true
---

# Autonomous training workflow

Follow this order; every step is auditable and reversible except the final remote submission, which requires explicit human confirmation.

1. **Server.** Call `rh_train_server_check`. If no server is configured (`backend: unavailable`), stop and tell the user exactly what to put in `<storeRoot>/train-servers.json` (`{"servers": [{"id","host","user","keyPath?","port?","workDir?"}]}`). Never invent a server.
2. **Plan.** Ask the user for the training objective (and model/hyperparameters if known). Call `rh_train_plan_create` → returns plan JSON + Markdown with `status: draft`. Show the plan summary and get the user's approval of hyperparameters before continuing.
3. **Data.** If the plan lacks data or the user wants supplements, call `rh_train_data_discovery` with a query derived from the objective. Present dataset candidates (id, downloads, tags) and note that licenses/quality must be checked (data-quality tools) before use. Add agreed sources to the plan via `rh_train_plan_create` (same planId) or note them for the job.
4. **Prepare (dry-run).** Call `rh_train_job_prepare` with `dryRun: true` (default). It generates `train.py`, `launcher.sh` and a plan snapshot locally. Review the generated script with the user — the script is a template placeholder; say so clearly.
5. **Confirm + submit.** Only after the user explicitly agrees: call `rh_train_job_prepare` with `dryRun: false` and `confirm: true` in the same call. The worker checks the server, uploads only the generated artifacts, and starts the allowlisted launcher. If the user has not confirmed, refuse politely and stay in dry-run.
6. **Track.** Poll `rh_train_job_status` with the job id (plan id): local log tail, or remote tail for submitted jobs. Report loss/epoch progress from the log.
7. **Report.** When the job finishes, call `rh_train_report` (jobId or logPath) → statistical convergence verdict + Markdown report. Remind the user that this is not a release verdict; model evaluation needs validation and real-scenario tests.

## Rules

- Never submit a job remotely without the user's explicit confirmation in the same turn you call the submit command.
- Never fabricate server connectivity, dataset results, or training progress — `backend: unavailable` is an honest answer.
- Training scripts are template-based placeholders; state that limitation instead of implying a real model is being trained.
- The plan stays a draft until the user approves it; record choices in the plan file for auditability.
