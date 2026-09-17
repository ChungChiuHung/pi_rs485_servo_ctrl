# Lessons Learned (Deep Memory) — pi_rs485_servo_ctrl

Dated, verified, incident-specific lessons from real development on this
repo. CLAUDE.md stays lean; this file can grow indefinitely. Only add
entries once verified in practice — not speculative.

Format:

```
## [YYYY-MM-DD] Short title
**Relates to:** CLAUDE.md §N (or "new pattern")
**What happened:** ...
**Lesson:** ...
**Status:** active | promoted to CLAUDE.md | superseded
```

---

## [2026-07-22] Canonical module structure resolved
**Relates to:** CLAUDE.md §2
**What happened:** Investigated the module-folder question via git
archaeology (commit counts, last-touched dates per folder) plus direct
diff between `servo_comm_shihlin/` and `servo_comm_shihlin_50W/`, then
confirmed the interpretation with the user in three rounds of Q&A.
Findings: `servo_comm_shihlin*` = Type 2 (Shihlin) motor, actively
developed, run via `osc_2.py`; `servo_communication/` = Type 1 (other
brand) motor; `web/` = Flask UI for `servo_communication/`, still live
despite stale git history (main.py boots it); `examples/` = scratch
sandbox, confirmed not deployed. Initial git-history-only guess (that
web/servo_communication/examples were all uniformly "abandoned") was
**wrong** — user corrected that `servo_communication/` and `web/` are
both still actively used, just for a different hardware variant than
the one with the most recent commits.
**Lesson:** Commit recency/count alone is not sufficient to judge
whether a module is "live" vs "dead" in a multi-hardware-variant repo —
a folder can look stale simply because that variant's code is stable,
not because it's abandoned. Always confirm with the user before
labeling anything legacy, even with strong-looking git evidence.
**Status:** promoted to CLAUDE.md §2

## [2026-07-22] Web UI integrated into each parallel project's app.py
**Relates to:** CLAUDE.md §2, §5
**What happened:** User reported having integrated "the Web" into each
parallel project's own `app.py`. Verified by reading all four `app.py`
files: `servo_comm_shihlin/app.py` and `servo_comm_shihlin_50W/app.py`
are now identical (177 lines, full Shihlin action-route set) —
confirming `app.py` is now part of their manual-sync surface, not just
the previously-documented motor-config files. `servo_communication/app.py`
gained a full, self-contained action-route set and local imports
(no longer a stub). `web/app.py` was rewritten to import directly from
the `servo_communication` package instead of duplicating its helper
modules, and its routes now just mirror `servo_communication/app.py`.
User confirmed `web/` (and by extension `main.py`, which still boots
it) is now the superseded path — `servo_communication/app.py` is the
real entrypoint going forward.
**Lesson:** Structural claims from the user ("I integrated X") are
worth verifying against the actual files even when plausible — in this
case the claim was fully accurate, but verifying surfaced a knock-on
fact the user hadn't mentioned: `main.py` was NOT updated to point at
the new entrypoint, so the documented boot path and the real one have
now diverged. Surface knock-on inconsistencies like this rather than
only checking the specific claim made.
**Status:** promoted to CLAUDE.md §2 and §5

## [2026-07-22] main.py's simplicity is a paused plan, not neglect
**Relates to:** CLAUDE.md §5
**What happened:** User clarified that `main.py`'s stale, minimal state
(just `from web.app import app`) isn't an oversight — the original
intent was for `main.py` plus a JSON config file to select which
motor/scenario module to run at boot. Checked git history to confirm:
the very first commit touching `main.py` (7a73528, 2024-03-21) imported
both `web` and `servo_communication` behind a `main()` stub with the
comment "Main application logic here" — consistent with a planned
dispatcher. Ten days later (6618a27, 2024-03-31) it was simplified to
directly boot `web.app`, and has been untouched since. The
config-driven selector was never implemented; per the user, that task
is idle, not scrapped.
**Lesson:** A file that looks stale/neglected in git history can
actually be a paused feature with real, still-intended future scope.
Don't reduce "known issue" documentation to just "this is wrong, fix
it eventually" — capture the original intent so a future agent doesn't
"fix" it by prematurely collapsing it to one hardcoded path (e.g. just
repointing to `servo_communication.app`), which would foreclose the
actual planned functionality (multi-motor config-driven selection).
**Status:** promoted to CLAUDE.md §5

<!-- New verified entries go below this line -->
