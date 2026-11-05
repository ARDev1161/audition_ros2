"""Sphinx documentation for the ROS 2 integration layer."""
from pathlib import Path
project = "audition_ros2"
author = "audition_ros2 contributors"
extensions = ["myst_parser", "sphinxcontrib.mermaid"]
source_suffix = {".md": "markdown"}
root_doc = "index"
myst_fence_as_directive = ["mermaid"]
myst_heading_anchors = 3
mermaid_config = {"securityLevel": "loose"}
mermaid_fullscreen = False
exclude_patterns = ["_build"]
html_theme = "furo"
html_title = "audition_ros2 documentation"
html_baseurl = "https://ardev1161.github.io/audition_ros2/"
html_static_path = ["_static"]
html_js_files = ["diagram-viewer.js"]
html_css_files = ["layout.css", "diagram-viewer.css"]
html_theme_options = {
    "source_repository": "https://github.com/ARDev1161/audition_ros2/",
    "source_branch": "feat/m0-ros2-foundation",
    "source_directory": "docs/",
}
suppress_warnings = ["toc.not_included"]
