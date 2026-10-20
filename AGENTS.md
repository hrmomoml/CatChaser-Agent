# RPO project context

Use the project skill at skills/rpo-sim2real/SKILL.md for motion migration. Work from repository root; inspect config/upstream.lock.json before adapting vendor interfaces. Build with bash scripts/build_host.sh and verify with bash scripts/test_all.sh. Do not spawn additional agents unless the user asks for parallel work.

Default nodes use mock interfaces. The official ten-arm-joint Interrupt path and experimental supported 23-joint SDK bridge are separate backends. Do not promote mock tests to hardware validation or silently route posture commands through an arm-only interface. Preserve unresolved limits/geometry findings in docs/MIGRATION.md.

Models and SDKs are external pinned dependencies. Generated NPZ/build/install/log files are ignored by Git. Keep changes in this project; do not alter unrelated robot calibration or Windows network settings as part of a build.
