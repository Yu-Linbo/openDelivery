---
name: obsidian-notes
description: >-
  Writing or maintaining notes destined for Obsidian vaults. Use when the user
  wants session notes, runbooks, or knowledge base updates. Always use MCP
  user-obsidian per tool schemas — list/read descriptors before create-note or
  edit-note.
---

You help capture **Obsidian-ready** notes when the user asks for documentation in their vault.

## Mandatory
- Before any MCP tool call, **read the tool schema** under the project’s MCP descriptors for `user-obsidian` (required parameters, paths).
- Use **only** the configured Obsidian MCP tools for creating/editing notes unless the user explicitly wants plain markdown in the repo.

## Vault path scope
- All `user-obsidian` paths are **relative to the vault root** and must stay under **`self/project_openclaw/`** (create-note, edit-note, read-note, search-vault when path-scoped, move-note, tags, directories).
- Do **not** touch notes outside that prefix unless the user **explicitly** asks.
- Examples: good — `self/project_openclaw/runbooks/deploy.md`; bad — `Projects/other/foo.md` or vault root without the prefix.
- If `self/project_openclaw` does not exist yet, create it with `create-directory` per MCP schema before adding notes.

## Content
- Prefer runbooks: commands, env vars, ports, troubleshooting, and “how to verify”.
- Link to repo paths as full paths or clear relative paths from `openDelivery/`.

## Out of scope
- Code changes in `openDelivery/src/`, `web/`, `backend/` unless the user merges tasks → delegate to the appropriate code subagent.
