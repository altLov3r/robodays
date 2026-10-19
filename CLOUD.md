# Access from another PC

The private repository is https://github.com/altLov3r/robodays.
Sign in as its owner or an authorized collaborator. Firmware, tests, frozen
ZIPs and lab instructions are available there; your PC does not need to stay
on for someone to download them. Start with LAB_TEST.md on the lab PC.

For a checkout:

```text
git clone https://github.com/altLov3r/robodays.git
cd robodays
```

Pull before editing on another PC, and commit/push reviewed changes before
switching PCs. Do not overwrite another PC's uncommitted changes. GitHub Actions
runs native regressions and the pinned Leonardo build on pushes, pull requests
and manual dispatch. Check the Actions tab; a green check is software validation,
not a physical track result.

## Codex Cloud

Hosting code on GitHub does not move this existing local desktop chat into a
cloud executor. To work on the repository through Codex Cloud:

1. Open ChatGPT and choose **Work in > Cloud > Select environment**.
2. Create an environment, select `altLov3r/robodays`, and connect GitHub if asked.
3. Install Python 3 and g++ for `python3 tests/run_tests.py --focused`. Use the
   pinned board/library commands in `.github/workflows/firmware.yml` for AVR builds.
4. Review the setup/test results and publish the environment. Use that environment
   for cloud work; the physical USB upload still happens on your lab PC.

These account/environment setup steps have not been performed by this local chat.
See the [official OpenAI cloud setup](https://learn.chatgpt.com/docs/cloud).

## Overnight reviews

An every-two-hours follow-up is scheduled in the current desktop chat until
09:00 Europe/Sofia on 20 October 2026. This is a local heartbeat: leave the PC
on and the app running for it to access this checkout. The GitHub copy and its
cloud CI are independent of that local review schedule. See the
[official scheduled-task documentation](https://learn.chatgpt.com/docs/automations?surface=app).

On this PC, pushes can use the existing GitHub CLI sign-in without a separate
Git Credential Manager dialog:

```text
git -c credential.helper= -c "credential.helper=!gh auth git-credential" push origin master
```

Do not put authentication tokens in files or commit them. A new PC needs its
own sign-in; credentials are not part of the repository.
