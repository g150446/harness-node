# Repository agent instructions

## Terminal window lifecycle

- When an agent opens a macOS Terminal window or tab for a command that must run
  with Terminal permissions, record the exact window or tab identifier.
- Close that window or tab as soon as the command and its required verification
  have finished. Do not leave an idle Terminal session open for possible later
  reuse.
- Before the final response, close every Terminal window or tab opened by the
  agent during the task.
- Never close a Terminal window or tab that existed before the agent's task or
  was opened by the user. Resolve and close only the exact identifiers created
  by the agent.

## Interactive hardware tests

- Before starting a test that requires the user to perform a physical action,
  tell the user exactly what action will be required, how many trials will run,
  and what sound or message marks the start of each trial.
- Ask the user to confirm that they are ready, and do not launch the test or its
  countdown until the user explicitly confirms readiness.
- Run trials one at a time when the agent must interpret a result or give
  corrective guidance between trials. Ask for readiness again before resuming
  after an interruption or a materially changed test procedure.
