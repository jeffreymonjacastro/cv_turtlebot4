# Security and Repository Hygiene Audit

Date: 2026-07-14

This audit was performed as part of preparing the repository for portfolio review.

## Scope

Searched tracked repository content for:

- Wi-Fi passwords and robot passwords;
- SSH commands and private hostnames;
- private IPv4 addresses;
- pairing codes and robot names;
- hard-coded local paths;
- token-like strings.

Representative commands:

```bash
git grep -n -I -E '(PASS|PASSWORD|password|passwd|pwd|token|secret|api[_-]?key|ssh |ubuntu@|192\\.168\\.|10\\.|172\\.|Lab_|Computech|turtlebot4|ROBOT_A|tplinkwifi|/Users/|home/ubuntu)' -- .
git grep -n -I -E 'Computech2025|contraseña|password|PASS|Lab_Computech|ubuntu@10\\.42\\.0\\.1|tplinkwifi' -- '*.md' '*.txt'
```

## Findings

Sensitive or lab-specific values were found in legacy setup notes, including:

- lab Wi-Fi SSIDs/passwords;
- default robot SSH password notes;
- private LAN IPs;
- private robot hostnames / robot labels;
- local absolute paths.

The most important redactions were made in:

- `README.md` rewritten with placeholders and portfolio-oriented content;
- `archive/legacy_turtlebot_setup_notes_redacted.md`;
- `archive/tutorial_turtlebot4_redacted.md`;
- `repo-cortijo/README.md` credential placeholders;
- `.env.example` added for local, untracked runtime configuration.

## Remaining non-secret local values

Some runbooks still use robot-side paths such as `/home/ubuntu/output/...` because those are runtime paths on the TurtleBot image, not credentials. Runtime scripts now default to generic loopback or placeholder identity values and should be configured with `.env`, environment variables, ROS parameters, or command-line arguments.

Before publishing externally, run the grep commands again and decide whether to further generalize legacy scripts or move them under `archive/`.

## Rotation recommendation

Because real credentials were present in tracked files before this cleanup, assume they may remain in git history and any prior remote copies.

Recommended manual action:

- rotate the lab Wi-Fi password if the repository was ever shared outside the trusted team;
- change the robot SSH password or disable password login if the robot is reachable from any shared network;
- treat historical pairing codes and robot labels as non-secret identifiers, but replace them in public demos if desired.

No git history rewrite was performed.
