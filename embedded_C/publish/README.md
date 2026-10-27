# Standalone Release output

When `embedded_C` is copied or checked out without the repository's sibling
`publish/` and `third_party/README.md`, successful Release builds write into
`embedded_C/publish/<platform>/`. The executable, local runtime libraries and
configuration files are created there. Debug builds do not update this folder.

When the full VISTA repository layout is present, the existing repository-level
`publish/<platform>/` continues to be used. See `docs/BUILDING.md` for the
commands, OS requirements and configuration notes.
