# F1tenth 한국어 번역작업

작성자: 최윤도 ([@zygn](https://github.com/zygn)), 류승재 ([@RSJ397](https://github.com/RSJ397)), 김강진 ([@syndicater123](https://github.com/syndicater123))


#### VSCode reStructuredText 오류시

- `python -m pip install sphinx_rtd_theme` 설치

# F1tenth documentation

This repository contains the source files of [F1tenth](http://f1tenth.org)'s documentation, in reStructuredText markup language (reST).

## Contributing changes

**Pull Requests should use the `master` branch by default. Only make Pull Requests against other branches (e.g. `2.1` or `3.0`) if your changes only apply to that specific version of f1tenth.**

Though arguably less convenient to edit than a wiki, this git repository is meant to receive pull requests to always improve the documentation, add new pages, etc. Having direct access to the source files in a revision control system is a big plus to ensure the quality of our documentation.

### Editing existing pages

To edit an existing page, locate its .rst source file and open it in your favorite text editor. You can then commit the changes, push them to your fork and make a pull request.

### Adding new pages

To add a new page, create a .rst file with a meaningful name in the section you want to add a file to, e.g. `going_forward/more_cool_stuff.rst`. Write its content like you would do for any other file, and make sure to define a reference name for Sphinx at the beginning of the file (check other files for the syntax), based on the file name with a "doc_" prefix (e.g. `.. _doc_cool_stuff:`).

You should then add your page to the relevant "toctree" (table of contents, e.g. `tutorials/3d/index.rst`). By convention, the files used to define the various levels of toctree are prefixed with an underscore, so in the above example the file should be referenced in `tutorials/3d/_3d_graphics.rst`. Add your new filename to the list on a new line, using a relative path and no extension, e.g. here `light_baking`.

### Sphinx and reStructuredText syntax

Check Sphinx's [reST Primer](https://www.sphinx-doc.org/en/stable/rest.html) and the [official reference](http://docutils.sourceforge.net/rst.html) for details on the syntax.

Sphinx uses specific reST comments to do specific operations, like defining the table of contents (`:toctree:`) or cross-referencing pages. Check the [official Sphinx documentation](https://www.sphinx-doc.org/en/stable/index.html) for more details, or see how things are done in existing pages and adapt it to your needs.

### Adding images and attachments

To add images, please put them in an `img/` folder next to the .rst file with a meaningful name and include them in your page with:
```rst
.. image:: img/image_name.png
```

Similarly, you can include attachments (like assets as support material for a tutorial) by placing them into a `files/` folder next to the .rst file, and using this inline markup:
```rst
:download:`myfilename.zip <files/myfilename.zip>`
```

## Building with Sphinx

To build the HTML website (or any other format supported by Sphinx, like PDF, EPUB or LaTeX), you need to install [Sphinx](https://www.sphinx-doc.org/) >= 1.3 as well as (for the HTML) the [readthedocs.org theme](https://github.com/snide/sphinx_rtd_theme). Only the Python 3 flavor was tested, though the Python 2 versions might work too.

Those tools are best installed using [pip](https://pip.pypa.io), Python's module installer. The Python 3 version might be provided (on Linux distros) as `pip3` or `python3-pip`. You can then run:

```sh
pip3 install sphinx
pip3 install sphinx_rtd_theme
```

You can then build the HTML documentation from the root folder of this repository with:

```sh
make html
```

or:

```sh
make SPHINXBUILD=~/.local/bin/sphinx-build html
```

The compilation might take some time as the `classes/` folder contains many files to parse.
You can then test the changes live by opening `_build/html/index.html` in your favorite browser.

### Building with Sphinx on Windows

On Windows, you need to:
* Download the Python installer [here](https://www.python.org/downloads/).
* Install Python. Don't forget to check the "Add Python to PATH" box.
* Use the above `pip` commands.

Building is still done at the root folder of this repository using the provided `make.bat`:
```sh
make.bat html
```

Alternatively, you can build with this command instead:
```sh
sphinx-build -b html ./ _build
```

Note that during the first build, various installation prompts may appear and ask to install LaTeX plugins.
Make sure you don't miss them, especially if they open behind other windows, else the build may appear to hang until you confirm these prompts.

You could also install a normal `make` toolchain (for example via MinGW) and build the docs using the normal `make html`.

### Building with Sphinx and virtualenv

If you want your Sphinx installation scoped to the project, you can install it using virtualenv.
Execute this from the root folder of this repository:

```sh
virtualenv --system-site-packages env/
. env/bin/activate
pip3 install sphinx
pip3 install sphinx_rtd_theme
```

Then do `make html` like above.

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
