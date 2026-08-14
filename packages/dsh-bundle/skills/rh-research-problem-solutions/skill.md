---
name: rh-research-problem-solutions
description: Find evidence-backed solution options for a user's current problem (at any stage — experiment, model, simulation, data, control, perception) by searching public literature, then present ranked candidates with sources so the user can choose.
whenToUse: Use when the user hits a problem and wants to know what approaches exist in the literature before deciding what to try.
modelInvocable: true
userInvocable: true
---

# Research problem → candidate solutions

1. **Clarify the problem.** Ask for (or infer) the stage: `experiment`, `model`, `simulation`, `data`, `control`, `perception`, or `general`. Collect constraints (hardware, environment, what has already been tried).
2. **Search literature.** Call `rh_problem_solutions` with `problem`, `stage`, `context`. It returns evidence cards (title, year, URL, abstract excerpt, matched keywords) plus a proposal scaffold.
   - If the backend is `unavailable`, report that honestly and offer offline alternatives (rh_memory_retrieve for past cases, rh_manual_search for local docs) instead of inventing papers.
3. **Synthesize options, don't adjudicate.** Turn the candidates into 1–3 concrete options the user can choose between. For each option state:
   - what it is (one or two sentences, grounded in the paper abstract),
   - where the evidence comes from (URL),
   - what would be needed to validate it in this project (data, experiment, model change).
4. **Let the user choose.** Present the trade-offs and ask which option to pursue. Do not silently pick one.
5. **After the choice**, propose the smallest validation step (a run, a simulation, an experiment spec) that would confirm or reject the option, and offer to carry it out with the corresponding tools (experiment/benchmark/diagnose).

## Rules

- Every candidate must carry its source URL; never present a paper's claim as verified fact.
- The worker's relevance match is keyword-based — say so when ranking, and don't over-trust the order.
- The final decision belongs to the user; the Agent proposes, the user disposes.
- Literature alone is never a verdict. If the search returns nothing relevant, say so and suggest rephrasing or using offline memory/docs.
