#!/usr/bin/env python3
"""dump_tree.py — build a BT offline with StubFacade and print its structure.

Given a tree-building function referenced as ``module.path:function``, import
it, call it with a :class:`~xyz_bt_lib.core.stub_facade.StubFacade` (and stub
``logger`` / ``tick_id_getter``) injected for whichever of those parameters the
function accepts, then print the tree with explicit ``[memory=...]`` tags via
:mod:`xyz_bt_lib.core.tree_render`.

No ROS, no hardware, read-only: it constructs the tree (nodes' ``__init__`` do
no ROS I/O per xyz_bt_lib design rule 4) but never calls ``setup()`` or ticks.

Usage:
    python3 tools/dump_tree.py <module>:<function> [--ascii|--unicode] [-p DIR]...

Examples:
    # The bundled demo tree in examples/ (add examples/ to the path):
    python3 tools/dump_tree.py demo_composites_tree:build_tree -p examples

    # A project tree living outside the lib — add its src dir to the path:
    python3 tools/dump_tree.py sprint.tree.sprint_bt:build_tree -p /path/to/project/src

The build function may take any of ``facade`` / ``logger`` / ``tick_id_getter``
(as keyword or leading positional params); each is injected when present and
skipped otherwise. If it needs other required arguments, dump_tree reports the
function's signature and exits rather than guessing.
"""
import argparse
import importlib
import inspect
import sys

from xyz_bt_lib.core.stub_facade import StubFacade
from xyz_bt_lib.core.tree_render import render_ascii, render_unicode


def _parse_target(target):
    """Split 'pkg.mod:func' into (module_path, func_name)."""
    if ':' not in target:
        raise SystemExit(
            "error: target must be 'module.path:function' (got {!r})".format(target))
    mod_path, _, func_name = target.partition(':')
    if not mod_path or not func_name:
        raise SystemExit(
            "error: target must be 'module.path:function' (got {!r})".format(target))
    return mod_path, func_name


def _build_kwargs(func):
    """Inject stub facade/logger/tick_id_getter for whatever params `func` accepts."""
    stubs = {
        'facade': StubFacade(),
        'logger': None,
        'tick_id_getter': (lambda: 0),
    }
    sig = inspect.signature(func)
    params = sig.parameters
    has_var_kw = any(p.kind == inspect.Parameter.VAR_KEYWORD for p in params.values())
    kwargs = {}
    for name, value in stubs.items():
        if name in params or has_var_kw:
            kwargs[name] = value
    return kwargs, sig


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Build a BT offline with StubFacade and print its structure.")
    parser.add_argument('target', help="tree builder as 'module.path:function'")
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--unicode', dest='style', action='store_const',
                       const='unicode', help='Unicode glyphs (default)')
    group.add_argument('--ascii', dest='style', action='store_const',
                       const='ascii', help='ASCII glyphs')
    parser.add_argument('-p', '--path', action='append', default=[],
                        metavar='DIR',
                        help='directory to prepend to sys.path (repeatable)')
    parser.set_defaults(style='unicode')
    args = parser.parse_args(argv)

    for d in reversed(args.path):
        sys.path.insert(0, d)
    sys.path.insert(0, '')   # allow imports relative to CWD

    mod_path, func_name = _parse_target(args.target)

    try:
        module = importlib.import_module(mod_path)
    except Exception as exc:
        raise SystemExit("error: could not import module {!r}: {}".format(mod_path, exc))

    func = getattr(module, func_name, None)
    if func is None or not callable(func):
        raise SystemExit(
            "error: {!r} has no callable {!r}".format(mod_path, func_name))

    kwargs, sig = _build_kwargs(func)
    try:
        root = func(**kwargs)
    except TypeError as exc:
        raise SystemExit(
            "error: could not call {}:{}{} with injected stubs "
            "(facade/logger/tick_id_getter).\n  {}\n"
            "The builder needs argument(s) dump_tree can't supply — call it from "
            "your own script instead.".format(mod_path, func_name, sig, exc))

    if root is None:
        raise SystemExit(
            "error: {}:{} returned None (expected a tree root Behaviour)".format(
                mod_path, func_name))

    render = render_ascii if args.style == 'ascii' else render_unicode
    print(render(root), end='')


if __name__ == '__main__':
    main()
