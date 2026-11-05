"""Sphinx site for the independent audition_ros2 documentation branch."""
project = "audition_ros2"
author = "audition_ros2 contributors"
version = "0.1"
release = "0.1-dev"

extensions = ["myst_parser", "sphinxcontrib.mermaid"]
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
root_doc = "index"
myst_fence_as_directive = ["mermaid"]
myst_heading_anchors = 3
mermaid_fullscreen = False
mermaid_config = {"securityLevel": "loose"}
html_static_path = ["_static"]
html_js_files = ["diagram-viewer.js"]
html_css_files = ["layout.css", "diagram-viewer.css"]
exclude_patterns = ["_build", ".DS_Store", "Thumbs.db"]

html_theme = "furo"
html_title = "audition_ros2 documentation"
html_baseurl = "https://ardev1161.github.io/audition_ros2/"
html_theme_options = {
    "source_repository": "https://github.com/ARDev1161/audition_ros2/",
    "source_branch": "documentation",
    "source_directory": "docs/",
}
