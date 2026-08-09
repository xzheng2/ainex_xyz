#!/usr/bin/env python3
"""Offline structural rendering of xyz_bt_lib trees, with explicit memory values.

py_trees' own ``display.ascii_tree`` / ``display.unicode_tree`` DO encode a
composite's ``memory`` in the node glyph (``[-]`` vs ``{-}`` for Sequence,
``[o]`` vs ``{o}`` for Selector) — but that distinction is one glyph and easy
to misread, and ``memory`` is exactly the field most often gotten wrong. These
renderers reproduce py_trees' native layout *and* append an explicit
``[memory=True|False]`` tag to every Sequence / Selector, so the contract is
unmissable in a code review or a printed tree.

Faithfulness: the walk reuses py_trees' own ``ascii_symbols`` /
``unicode_symbols`` tables and 4-spaces-per-depth indentation, so the output is
byte-for-byte the native tree plus the appended tag on composite lines.

No ROS, no Blackboard — pure structural inspection. Combine with
:class:`~xyz_bt_lib.core.stub_facade.StubFacade` to build and render a tree
entirely off-robot.
"""
import py_trees
from py_trees import behaviour, composites, decorators, display


def _root_behaviour(root):
    """Accept either a Behaviour or a BehaviourTree; return the root Behaviour."""
    return root.root if isinstance(root, py_trees.trees.BehaviourTree) else root


def _symbol_for(node, symbols):
    """Mirror py_trees.display._generate_text_tree's glyph selection exactly."""
    if isinstance(node, composites.Parallel):
        key = composites.Parallel
    elif isinstance(node, decorators.Decorator):
        key = decorators.Decorator
    elif isinstance(node, composites.Sequence):
        key = 'sequence_with_memory' if node.memory else composites.Sequence
    elif isinstance(node, composites.Selector):
        key = 'selector_with_memory' if node.memory else composites.Selector
    else:
        key = behaviour.Behaviour
    return symbols[key]


def _render(root, symbols):
    root = _root_behaviour(root)
    lines = []

    def walk(node, depth):
        indent = '    ' * depth
        line = '{}{} {}'.format(indent, _symbol_for(node, symbols), node.name)
        # Only Sequence / Selector carry a `memory` contract; Parallel uses a
        # policy and leaves/decorators have none, so tag only composites.
        if isinstance(node, (composites.Sequence, composites.Selector)):
            line += '  [memory={}]'.format(node.memory)
        lines.append(line)
        for child in node.children:      # [] for leaves, [decorated] for decorators
            walk(child, depth + 1)

    walk(root, 0)
    return '\n'.join(lines) + '\n'


def render_ascii(root):
    """Return an ASCII structure diagram of `root` with explicit memory tags.

    Same layout as ``py_trees.display.ascii_tree(root)``, but every Sequence /
    Selector line ends with ``[memory=True|False]``.

    Args:
        root: a py_trees Behaviour (tree root / subtree) or a BehaviourTree.
    """
    return _render(root, display.ascii_symbols)


def render_unicode(root):
    """Return a Unicode structure diagram of `root` with explicit memory tags.

    Same layout as ``py_trees.display.unicode_tree(root)``, but every Sequence /
    Selector line ends with ``[memory=True|False]``.

    Args:
        root: a py_trees Behaviour (tree root / subtree) or a BehaviourTree.
    """
    return _render(root, display.unicode_symbols)
