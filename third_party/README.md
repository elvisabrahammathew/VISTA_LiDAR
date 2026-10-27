# Third-party reference source

This repository-level folder retains external code used as protocol or design
reference. The current C++ build does not include or link any folder here.

- `unilidar_sdk2` is a pinned Unitree protocol reference. VISTA's own decoder
  does not link its platform-specific vendor binaries.
- `others` is reserved for future reference material.

All source required to build `embedded_C` now lives in
[`embedded_C/library/`](../embedded_C/library/README.md). The retired FAST-LIO
history remains available in earlier repository revisions.
