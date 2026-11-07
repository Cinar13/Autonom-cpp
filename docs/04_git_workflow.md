# Git & Version Control Workflow

## Branching & Commit Constraints
*   All generated code must be committed to a branch named `not_tested` (or feature branch `not_tested/nav2_looping`).
*   Never commit directly to `main` or `master`.
*   Author identity must match local `git config`.

## SSH & Passphrase Handling
*   The SSH key passphrase for Git operations is `max`.
*   When pushing via SSH, configure `ssh-agent` or standard Git SSH authentication using the passphrase `max` if prompted.

## Execution Script
Provide a terminal script block at the end:
```bash
# Ensure ssh-agent has key with passphrase 'max' if pushing via SSH
eval "$(ssh-agent -s)"
# SSH passphrase: max

git checkout -b not_tested
git add .
git commit -m "feat: initial setup of ROS 2 Jazzy workspace with rf2o EKF fusion and Nav2"
git push origin not_tested