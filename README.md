# RoboRacer Build Documentation

Source for the [RoboRacer](https://roboracer.ai) build documentation, the guide to building, configuring, and driving a RoboRacer (F1TENTH) autonomous vehicle. Pages are written in [reStructuredText](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html), built with [Sphinx](https://www.sphinx-doc.org/), and published on [Read the Docs](https://f1tenth.readthedocs.io/).

## Branch and version model

Each ROS generation lives on its own branch, published as a separate Read the Docs version:

| Branch  | Contents                                   | Status |
| ------- | ------------------------------------------ | ------ |
| `main`  | Current docs — ROS 2 **Humble**            | Active — Read the Docs default; **open PRs here** |
| `foxy`  | ROS 2 **Foxy** docs                        | Frozen (legacy) |
| `ros1`  | Original ROS 1 docs                        | Frozen (legacy) |

Target your pull requests at **`main`** unless a change applies *only* to a frozen legacy version.

## Building locally

Dependencies are managed with [uv](https://docs.astral.sh/uv/) (see `pyproject.toml` / `uv.lock`). uv installs the right Python and packages for you — no manual virtualenv.

```sh
uv sync                                       # create the environment from the lockfile
uv run sphinx-build -b html . _build/html     # build the site
```

Open `_build/html/index.html` in your browser.

For a live-reloading preview while you edit (serves on http://127.0.0.1:8000 and rebuilds on save):

```sh
uv run --with sphinx-autobuild sphinx-autobuild . _build/html
```

### Match CI before you push

CI builds with **warnings treated as errors** — a broken cross-reference, a missing image, or a malformed heading fails the build. Run the same strict build locally to catch it first:

```sh
uv run sphinx-build -W --keep-going -b html . _build/html
```

## Contributing

1. Fork the repo and create a branch off `main`.
2. **Edit a page:** find its `.rst` source, edit it, and build locally to check the result.
3. **Add a page:** create a `.rst` file with a meaningful name in the relevant section.
   Give it a Sphinx label on the first line using the `doc_` prefix convention, e.g. `.. _doc_gap_finding:`, so other pages can cross-reference it with `:ref:`.
4. **Wire it into the navigation:** add the new file (relative path, no extension) to the nearest `toctree` — usually the section's `index.rst`. A page that isn't in a `toctree` won't appear in the sidebar.
5. **Images:** place them in an `img/` folder next to the `.rst` and include them with `.. image:: img/your_image.png`. Unreferenced images are periodically pruned, so only commit images a page actually uses.
6. Open a pull request into `main`. The `docs` CI check (strict build + link check) must pass before merge.

New to reStructuredText? See Sphinx's [reST primer](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html), or copy the patterns already used in neighboring pages.

## Reporting issues

Found something wrong, outdated, or unclear? Open an issue at [github.com/f1tenth/f1tenth_doc/issues](https://github.com/f1tenth/f1tenth_doc/issues).
Please include:

- the page URL (or `.rst` path),
- the ROS distro / branch it concerns (Humble / `main`, Foxy, ROS 1), and
- what's wrong and, if you can, how it should read.

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
