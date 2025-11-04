# Copyright 2025 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.


from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

# Doxygen
subprocess.call("cd ../doxygen; doxygen Doxyfile", shell=True)
html_extra_path = ["../doxygen/_build"]  # Export files to "sphinx/_build"

# -- Project information -----------------------------------------------------

project = "staubli_driver_ros2"
copyright = "2025, ICUBE Laboratory, University of Strasbourg"
author = "Thibault Poignonec"

# Read version from package.xml
package_xml_path = Path(__file__).parent.parent / "package.xml"
tree = ET.parse(package_xml_path)
root = tree.getroot()
version_element = root.find("version")
if version_element is not None:
    release = version_element.text
else:
    release = "0.0.1"  # Fallback version

# The short X.Y version
version = ".".join(release.split(".")[:2])


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx_rtd_theme",
    "sphinx_mdinclude",
    "sphinx.ext.imgmath",
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
    "sphinx.ext.graphviz",
    "sphinx.ext.inheritance_diagram",
    "sphinx.ext.githubpages",
    "sphinxcontrib.plantuml",
    "sphinxcontrib.video",
    "sphinx_copybutton",
    "breathe",
]

breathe_default_project = "staubli_driver_ros2"


# Automatic numbering
numfig = True


def get_package(package: str):
    path = Path(__file__).parent.parent.parent.joinpath(f"{package}/include/{package}")
    files_gen = path.glob("*.hpp")
    files = []
    for file in files_gen:
        files.append(file.name)
    return (path, files)


templates_path = ["_templates"]
primary_domain = "cpp"
highlight_language = "cpp"

breathe_projects = {
    "staubli_driver_ros2": "_build/xml/",
}
breathe_default_project = "staubli_driver_ros2"
breathe_default_members = ("members", "undoc-members")


# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------

html_theme = "sphinx_rtd_theme"  # "sphinx_book_theme"
html_theme_options = {
    "canonical_url": "",
    "analytics_id": "",  # Provided by Google in your dashboard
    "prev_next_buttons_location": "bottom",
    "style_external_links": False,
    "logo_only": False,
    # Toc options
    "collapse_navigation": True,
    "sticky_navigation": True,
    "navigation_depth": 4,
    "includehidden": True,
    "titles_only": False,
}

html_static_path = ["_static"]
html_logo = "images/logo-icube.png"
html_css_files = ["css/custom.css"]
pygments_style = "sphinx"

# -- Options for PDF output


latex_engine = "pdflatex"
latex_elements = {
    "papersize": "a4paper",
    "pointsize": "11pt",
    "preamble": r"""
        % Replace Unicode emojis with text representations
        \DeclareUnicodeCharacter{00A9}{\copyright}

        % Add spacing before chapter entries in TOC
        \usepackage{tocloft}
        \renewcommand{\cftchappresnum}{\hspace{1.5em}}  % Space before number
        \setlength{\cftchapnumwidth}{3em}  % Width for number including space

        % Customize chapter headers
        \usepackage{fancyhdr}
        \pagestyle{fancy}
        \fancyhf{}

        % Customize chapter title format
        \usepackage{titlesec}
        \titleformat{\chapter}[display]
            {\normalfont\huge\bfseries}{\chaptertitlename\ \thechapter}{20pt}{\Huge}
        \titlespacing*{\chapter}{0pt}{0pt}{20pt}
    """,
    "figure_align": "htbp",
    "extraclassoptions": "openany,oneside",
}

latex_documents = [
    (
        "index_pdf",
        "staubli_driver_ros2.tex",
        "Staubli ROS2 Driver Documentation",
        "Thibault Poignonec",
        "manual",
    ),
]

latex_logo = "images/logo-icube.png"
latex_show_pagerefs = True
# latex_show_urls = 'footnote'
latex_toplevel_sectioning = "chapter"
