# gpSP core (vendored)

`core/` is an unmodified snapshot of `components/gpsp/core` from:

- Repository: https://github.com/midworld/gpsp-esp32
- Commit: `296be4a` (docs(readme): add menu screenshot)
- Core upstream: https://github.com/midworld/gpsp (gpSP with an RV32 JIT)

Local changes to `core/` are kept as separate commits on top of the import so they can be
re-applied on the next sync. Everything else in `gbsp/` (component CMake, frontend, JIT
platform glue) is retro-go code.

## License

Most of `core/` is GPL-2.0-or-later. The headers in `core/include/` and the files in
`core/tests/` are GPL-3.0-or-later, so the gbsp app as a whole is distributed under
GPL-3.0-or-later. See the license exceptions in the top-level README.

## Syncing

```sh
git -C <gpsp-esp32 clone> archive <commit> components/gpsp/core | tar -x --strip-components=2 -C gbsp/components/gpsp
```

Then update the commit above and re-check `CMakeLists.txt` against upstream's
`components/gpsp/CMakeLists.txt` (source list and defines).
