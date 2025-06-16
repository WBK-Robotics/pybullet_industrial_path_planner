# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

import sys
import os
import mock

MOCK_MODULES = ['ompl', 'ompl.base']
for mod_name in MOCK_MODULES:
    sys.modules[mod_name] = mock.Mock()

sys.path.insert(0, os.path.abspath('../../src'))
sys.setrecursionlimit(1500)


# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'pybullet_industrial_path_planner'
copyright = '2025, Philipp Schulz, Malte Hansjosten'
author = 'Philipp Schulz, Malte Hansjosten'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx.ext.autodoc',
              'sphinx.ext.todo',
              'sphinx.ext.coverage',
              'sphinx.ext.mathjax',
              'sphinx.ext.ifconfig',
              'sphinx.ext.viewcode',
              'sphinx.ext.githubpages',
              'sphinx.ext.napoleon']

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'style_nav_header_background': '#009682', 'logo_only': True}
html_logo = "images/header_logo.png"
# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

source_suffix = '.rst'