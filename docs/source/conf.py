# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import subprocess
import sys

sys.path.insert(0,os.path.abspath('../../scripts'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'ExpRobLab Assignment 1'
copyright = '2022, Aurora Durante'
author = 'Aurora Durante'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx.ext.autodoc',
	      'sphinx.ext.doctest',
	      'sphinx.ext.intersphinx',
	      'sphinx.ext.todo',
	      'sphinx.ext.coverage',
	      'sphinx.ext.mathjax',
	      'sphinx.ext.ifconfig',
	      'sphinx.ext.viewcode',
	      'sphinx.ext.githubpages',
	      'sphinx.ext.inheritance_diagram',
	      'breathe']

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Extension configuration -------------------------------------------------
# -- Options for intersphinx extension ---------------------------------------
# Example configuration for intersphinx: refer to the Python standard library.

intersphinx_mapping = {'https://docs.python.org/': None}

# -- Options for todo extension ----------------------------------------------
# If true, `todo` and `todoList` produce output, else they produce nothing.

todo_include_todos = True

# -- Options for breathe

breathe_projects = {
		    "assignment1": "_build/xml/"
}
breathe_default_project = "assignment1"
breathe_default_members = ('members', 'undoc-members')

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

def skip(app, what, name, obj, would_skip, options):
    if name == "__init__":
        return False
    return would_skip

def setup(app):
    app.connect("autodoc-skip-member", skip)
