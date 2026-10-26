# Historical moving-point branch (retired)

The experimental moving-point branch added on 2026-10-21 is replaced by
[RoomMap foreground subtraction](ROOM_MAP_FOREGROUND.md).

It previously combined free-space evidence, candidate groups and short-term
geometric displacement. Stationary RAW tests showed that not every output could
be called a moving object. The active application now displays points outside
confirmed background without that claim. Displacement/history/clustering settings
and that worker are no longer active. Future motion classification/tracking can
be added after foreground with dedicated ground-truth evaluation.

Ground remains retired separately; see `GROUND_PROCESSING_RETIRED.md`.
