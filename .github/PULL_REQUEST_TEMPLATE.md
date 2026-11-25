*By completing this PR sufficiently, you help us review it quicker and keep the release notes honest.*

### Checklist
1. [ ] A title in the commit format, `type(scope): Summary` (*eg. "fix(robotics_aos): Pause on brake before a reversal"*)
2. [ ] Related links below (*eg. the issue this PR closes*)
3. [ ] Relevant documentation updated - header comments, README - if the change touches what they say
4. [ ] `tools/host_tests.sh`, `tools/check_layers.sh` and `tools/check_example_pins.sh` run, and green; `tests/target` built when `components/` changed
5. [ ] The branch is named for the change: `feat/`, `fix/`, `hotfix/`, `docs/`, `refactor/`, `test/`, `chore/`
6. [ ] If a module was used: its revision, the firmware on the Dot, and what was wired to it

*This entire section above can be deleted if all items are checked.*

-----------
## Description of Change
Please describe the proposed change and its impact.

## Test Scenarios
Please describe on what hardware and software this was tested, and how. Tell what ran on hardware from what was read in the code.

(*eg. host suites, all cases passing; tests/target on v6.1; module KDROB001 rev1.0 on a Dot over USB, two N20 motors on J501/J502, both turned both ways*)

## Related links
Please provide links to related issues, PRs etc.

(*eg. Closes #number of issue*)
