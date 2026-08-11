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
