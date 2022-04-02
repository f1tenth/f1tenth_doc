# -*- coding: utf-8 -*-
#
# Godot Engine documentation build configuration file

import sys
import os

# -- General configuration ------------------------------------------------

needs_sphinx = '1.3'

# Sphinx extension module names and templates location
sys.path.append(os.path.abspath('extensions'))
extensions = ['gdscript', 'sphinx_tabs.tabs', 'sphinx.ext.imgmath', 'sphinx.ext.todo', 'sphinx_copybutton']

templates_path = ['_templates']

# You can specify multiple suffix as a list of string: ['.rst', '.md']
source_suffix = '.rst'
source_encoding = 'utf-8-sig'

# The master toctree document
master_doc = 'index'

# General information about the project

project = 'F1TENTH - Build'
# copyright = '2018-2020, f1tenth community (CC-BY-NC-SA 4.0)'
# author = 'f1tenth community'

# Version info for the project, acts as replacement for |version| and |release|
# The short X.Y version
version = 'latest'
# The full version, including alpha/beta/rc tags
release = 'latest'

# Parse Sphinx tags passed from RTD via environment
env_tags = os.getenv('SPHINX_TAGS')
if env_tags != None:
   for tag in env_tags.split(','):
       print("Adding Sphinx tag: %s" % tag.strip())
       tags.add(tag.strip())

# Language / i18n
language = os.getenv('READTHEDOCS_LANGUAGE', 'en')
is_i18n = tags.has('i18n')

exclude_patterns = ['_build']

# GDScript syntax highlighting
from gdscript import GDScriptLexer
from sphinx.highlighting import lexers
lexers['gdscript'] = GDScriptLexer()

# Pygments (syntax highlighting) style to use
pygments_style = 'sphinx'
highlight_language = 'gdscript'

# -- Options for HTML output ----------------------------------------------

# on_rtd is whether we are on readthedocs.org, this line of code grabbed from docs.readthedocs.org
on_rtd = os.environ.get('READTHEDOCS', None) == 'True'

import sphinx_rtd_theme
html_theme = 'sphinx_rtd_theme'
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]
if on_rtd:
    using_rtd_theme = True

# Theme options
html_theme_options = {
    # 'typekit_id': 'hiw1hhg',
    # 'analytics_id': '',
    # 'sticky_navigation': True  # Set to False to disable the sticky nav while scrolling.
    'logo_only': False,  # if we have a html_logo below, this shows /only/ the logo with no title text
    'collapse_navigation': False,  # Collapse navigation (False makes it tree-like)
    'prev_next_buttons_location': 'bottom',
    # 'display_version': True,  # Display the docs version
    # 'navigation_depth': 4,  # Depth of the headers shown in the navigation bar
}

# VCS options: https://docs.readthedocs.io/en/latest/vcs.html#github
html_context = {
    "display_github": not is_i18n, # Integrate GitHub
    "github_user": "f1tenth", # Username
    "github_repo": "f1tenth_doc", # Repo name
    "github_version": "master", # Version
    "conf_py_path": "/", # Path in the checkout to the docs root
}

# html_logo = 'img/logo/f1_stickers_02.png'
html_favicon = 'img/logo/f1_stickers_02.png'

# These folders are copied to the documentation's HTML output
html_static_path = ['_static']

html_extra_path = ['robots.txt']

# These paths are either relative to html_static_path
# or fully qualified paths (eg. https://...)
html_css_files = [
    'css/custom.css',
]

html_js_files = [
    'js/custom.js',
]

# Hide copyright in footer
html_show_copyright = True
html_show_sphinx = True
html_last_updated_fmt = '%b %d, %Y'

# Output file base name for HTML help builder
htmlhelp_basename = 'F1TENTH_build'

# -- Options for reStructuredText parser ----------------------------------

# Enable directives that insert the contents of external files
file_insertion_enabled = False

# -- Options for LaTeX output ---------------------------------------------

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
  (master_doc, 'F1tenth.tex', 'F1tenth Build Documentation',
   'f1tenth community', 'manual'),
]

# -- Options for linkcheck builder ----------------------------------------

# disable checking urls with about.html#this_part_of_page anchors
linkcheck_anchors = False

linkcheck_timeout = 10

# -- I18n settings --------------------------------------------------------

locale_dirs = ['../sphinx/po/']
gettext_compact = False


# Exclude class reference when marked with tag i18n.
if is_i18n:
    exclude_patterns = ['classes']
