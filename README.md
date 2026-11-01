# audition_ros2 — documentation

This is the **standalone documentation branch** for [audition_ros2](https://github.com/ARDev1161/audition_ros2). It is intentionally an **orphan branch**: no ROS source code, build artifacts, model files or implementation history are included.

The documentation website is built with **Sphinx + MyST Markdown + Mermaid + Furo**, following the presentation approach used by [libaudition](https://ardev1161.github.io/libaudition/).

**Documentation site (once GitHub Pages is enabled):** https://ardev1161.github.io/audition_ros2/

## Start reading

- [Engineering roadmap](docs/ROADMAP.md): M0–M7, acceptance gates and future capabilities.
- [ROS node architecture](docs/ROS2_ARCHITECTURE.md): node count, responsibilities, interaction diagram, process boundaries and QoS.
- [libaudition integration and user configuration](docs/LIBAUDITION_INTEGRATION.md): native C++ adapters, proposed YAML profiles, ROS topics and optional backends.
- [Human spatial hearing and robotics](docs/HUMAN_SPATIAL_HEARING.md): bio-inspired approaches and engineering trade-offs.
- [M0 implementation notes](docs/M0_IMPLEMENTATION.md), [M1 implementation notes](docs/M1_IMPLEMENTATION.md).

## Build the website locally

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r docs/requirements.txt
sphinx-build -b html docs docs/_build/html
```

Open `docs/_build/html/index.html`. The docs build does **not** install ROS, compile native audio backends or download speech models.

## Branch separation

- **`documentation`**: independent Sphinx documentation site, editable Markdown and Mermaid, Pages deployment only.
- **`feat/m0-ros2-foundation`**: ROS 2 implementation and tests, tracked in [PR #1](https://github.com/ARDev1161/audition_ros2/pull/1).
- **`main`**: left unchanged. The documentation workflow **never merges or copies source code from `main`**.

Documentation status labels explicitly distinguish implemented adapters from proposed designs and physical hardware validation. GitHub Pages publishing requires **Settings → Pages → Source: GitHub Actions**.
