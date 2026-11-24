# Robot vacuum coordinator: how to set it up

Why this file exists: the coordinator used to be a **Cowork** scheduled task.
Cowork runs cannot push to GitHub ("not in this task's sources"), so its work
from 13 to 23 Nov stayed in its own chats. It has to be a **Claude Code**
scheduled task instead, created by the owner in the claude.ai web app, so it
gets both the repository and the connectors.

## Settings

| Setting | Value |
|---|---|
| Kind | Claude Code scheduled task (routine), not Cowork |
| Name | Robot vacuum coordinator |
| Repository | `abdelfatta7/v8-carrier` |
| Connectors | Claude Docs (the build manual), Google Drive (the hand-off folder) |
| Environment | Default |
| Model | Opus (it picks smaller models for its agents itself) |
| Schedule | Monday and Thursday, 06:37, Europe/London |
| Notifications | Push on |
| Prompt | the whole of `prompt.md` in this folder |

After the new task exists, the old Cowork task "Robot vacuum coordinator"
gets switched off (not deleted, so its run history stays).
