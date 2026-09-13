# Original Pipe µ Git history

This branch preserves the exact original commits and pre-pivot tag in a verified, self-contained Git bundle. The working-file snapshot is published separately on `feature/pipe-micro-reset-20260912`.

Original implementation head: `28c0f425bedc9ab3075a07c41465b4ff98a329dd`.
Original source tree: `c3f469f8310b8f3db340747d950633b3ac5f208c`.
Evidence source commit: `16cc2da` (full identity is in the evidence matrix).
Legacy tag: `legacy/pre-pipe-micro-20260912` at `4ccbd72c954fa8dd699b43b1eb900428942cfdbb`.

After downloading the bundle:

```sh
git bundle verify pipe-micro-original-commits.bundle
git clone -b feature/pipe-micro-reset-20260912 pipe-micro-original-commits.bundle restored-pipe
```

The bundle contains the complete reachable history and the legacy tag. It permits checking out the exact original evidence revisions. GitHub connector publication uses a new commit identity; it does not relabel the original evidence as a run of the publication commit.
