# Build and deploy the documentation website

The website is built entirely from **the independent orphan `documentation` branch** with Sphinx, MyST for Markdown, `sphinxcontrib-mermaid` for diagrams and the Furo theme. There is no application code, ROS workspace, generated HTML or compiled native library in this branch.

## Local build

Python 3.12 or another compatible interpreter is sufficient:

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r docs/requirements.txt
sphinx-build -b html --keep-going docs docs/_build/html
```

Then open `docs/_build/html/index.html`. All documentation lives under `docs/`; the site entry point is `docs/index.md`. Mermaid diagrams are rendered from Markdown fences, and the canonical source for the node diagram is `docs/diagrams/acoustic_interaction_flow.mmd`.

## GitHub Pages

The independent branch owns `.github/workflows/docs-pages.yml`. It builds and deploys the Sphinx output on changes to `documentation`. The workflow **does not merge `main`** and does not copy implementation sources into the documentation branch.

To enable hosting, configure repository **Settings → Pages → Build and deployment → Source: GitHub Actions**. The expected URL is:

https://ardev1161.github.io/audition_ros2/

The URL works only after Pages is enabled and the deployment action completes. Build CI may run without Pages enabled, but deployment may be blocked by repository settings or permissions.

## Updating code-dependent docs

When an implementation changes, manually update the Markdown on `documentation` and link to the corresponding implementation PR or exact commit. **Never merge `main` or the code feature branch into `documentation`**; their histories and trees are deliberately independent.

Source code, launch/config YAML and runtime tests belong to [PR #1](https://github.com/ARDev1161/audition_ros2/pull/1), not this documentation-only branch. The code may evolve independently of this documentation; clearly label not-yet-implemented features.

## Maintenance rules

- Keep all narrative Markdown and Mermaid under `docs/`.
- Do not commit `docs/_build`, `.venv` or generated site HTML.
- New pages require an entry in the MyST `toctree` in `docs/index.md`.
- Do not use manually drawn static flowcharts as the canonical interaction diagram; update the Mermaid source and its Markdown rendering together.
- Only add documentation tooling to this branch. Do not create a ROS package skeleton here.
