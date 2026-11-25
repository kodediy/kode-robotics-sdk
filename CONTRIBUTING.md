# Contributing

Thanks for helping with the SDK for the Robotics module (KDROB001) on the Kode Dot.

## Talk first

Open an issue before a large change; the templates in `.github/ISSUE_TEMPLATE` cover bugs, build problems, API proposals, documentation and tasks. A question or an idea that is not a proposal yet goes to [Discussions](https://github.com/kodediy/kode-robotics-sdk/discussions).

## Branches and pull requests

- `main` holds the releases: every merge into it is a version, with its tag. `develop` holds the next version, and pull requests go there. Start your branch from `develop`, one branch per issue.
- A version is one pull request into `develop`, `chore(release): Set the SDK's version to X.Y.Z`, made with `tools/set_version.sh X.Y.Z`; then one pull request from `develop` into `main`, `chore(release): kode-robotics-sdk X.Y.Z`; then the annotated tag `vX.Y.Z` on `main`. The release workflow checks the tag with `tools/check_version.sh` and runs the checks again before it publishes.
- The title of a pull request is the commit it becomes: `type(scope): Summary`, the scope being the component.
- The SDK's maintainer ([@vmlorenzorod](https://github.com/vmlorenzorod), the code owner) reviews and approves every pull request, into `develop` and into `main`.
- Name the branch with the kind of change, then the issue number and what it does, in lower case with hyphens: `feat/`, `fix/`, `docs/`, `refactor/`, `test/`, `ci/` or `chore/`. For example `fix/12-stepper-stop`.
- Commit by path, not with `git add -A`.
- Fill in the pull request template. Say what was **measured** on hardware and what was only **read in the code**, and on which board.
- Anything that touches hardware is not done until it has run on a board. Test with the motor, the stepper or the servo free to turn and clear of hands, and read [Safety](docs/overview.md#safety) first.

## Style

- Code, comments, README and documentation in English.
- The layers are kode-sdk's: `robotics.h` (tasks: calls block and return `bool`, `robotics_error()` says why) and `robotics_sys.h` (Active Objects: requests with `QActive *me` first, answers in `ROBOTICS_*_RSP_SIG`). Names are whole words, no abbreviations.
- No unreachable `assert` from the API and no `portMAX_DELAY` in the facade.
- Every function of `robotics.h` and `robotics_sys.h` is in `ALL[]` of `tests/target/main/main.c`; `tools/check_layers.sh` checks it.

## Before you open a pull request

With ESP-IDF v6.1 exported:

```sh
tools/host_tests.sh                    # seven host suites; --sanitize adds ASan and UBSan
tools/check_layers.sh                  # the layer rules
tools/check_example_pins.sh
idf.py -C tests/target build           # when components/ changed
tools/check_ram.sh tests/target/build/robotics_target.elf tests/target/ram_budget.txt
for p in module console_actions motors stepper servos outputs i2c; do idf.py -C examples/$p build; done
```

Test cases are moved between suites, never deleted: the total does not go down. `tools/check_misra.sh` (cppcheck on `PATH`, see `tools/build_cppcheck.sh`) takes a baseline file that is not in the repository.
