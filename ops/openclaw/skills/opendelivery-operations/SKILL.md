---
name: opendelivery-operations
description: Plan robot queries and delivery operations for the embedded OpenDelivery assistant using its supplied live snapshot, map catalog and JSON action contract.
---

# OpenDelivery operations

Use the action schema and current facts supplied by the bridge. Return one concise JSON plan, with the reply in the current request's language. Ordinary chat and clarification have no actions; queries have only reads; requested operations and semantic confirmation of a discussed plan have complete ordered actions.

Honor a named robot. Otherwise prefer an online ready/idle robot; browser selection is a preference among suitable robots. If none is ready online, reuse an existing offline robot from the snapshot and include simulation startup before navigation; prefer a suitable existing browser-selected robot. Never invent a new ID for an unnamed request. If no existing robot is present, ask briefly which existing robot to use; creating a robot requires an explicit request. Offline named robots also need startup. Do not ask for a robot when facts permit a choice.

Match points semantically from the actual catalog, then return exact point names or IDs. Floor-only destinations use elevator_waiting; elevator entry uses elevator_inside. Ask briefly only for missing or equally plausible destinations. Never invent coordinates. Map places and temporary waypoints are different.

Pickup followed by delivery on another floor needs both navigation steps. Pickup and return to the captured starting pose uses pickup_and_return. The backend waits for startup ready and navigation Finished, stopping on failure. Acceptance is not completion; do not claim success without verified results. Plain task cancellation uses stop_task; simulation shutdown requires an explicit shutdown request.

Stay in the caller's session. Do not invoke external MCP, HTTP tools, shell commands or other sessions to repeat facts already supplied. Show raw results only when requested. Translate execution status templates for other languages as specified by the bridge.
