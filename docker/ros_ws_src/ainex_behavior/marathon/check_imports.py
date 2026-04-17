#!/usr/bin/env python3
"""AST-based import constraint checker for marathon layer isolation.

Checks each layer directory against its allowed/forbidden dependency rules.
Uses AST to inspect actual import statements and call sites (not string grep).

Usage:
    python3 marathon/check_imports.py [--marathon-dir PATH]

Exit code:
    0 — all checks pass
    1 — one or more violations found
"""
import ast
import sys
import argparse
from pathlib import Path


# ---------------------------------------------------------------------------
# Rules per layer
# ---------------------------------------------------------------------------
# bad_imports:      module name prefixes forbidden from being imported
# bad_calls:        (module, attr) pairs — flags `module.attr(...)` call sites
# bad_method_names: method names forbidden on ANY receiver  (e.g. emit_comm())
# bad_names:        bare name references forbidden (e.g. ManagerProxy)
# ---------------------------------------------------------------------------

RULES = {
    'app': {
        # app/ is the assembly layer: it creates adapters and wires up the tree,
        # but must not own any ROS subscribers or emit ros_in/ros_out directly.
        # rospy.Subscriber belongs exclusively in ainex_bt_edu/input_adapters/.
        # emit_comm belongs in comm/comm_facade.py (ros_out) and input_adapters/ (ros_in).
        'bad_imports':      [],
        'bad_calls':        [('rospy', 'Subscriber')],
        'bad_method_names': ['emit_comm'],
        'bad_names':        [],
    },
    'behaviours': {
        'bad_imports':      ['marathon.comm', 'marathon.infra'],
        'bad_calls':        [('rospy', 'Publisher'),
                             ('rospy', 'Subscriber'),
                             ('rospy', 'ServiceProxy')],
        'bad_method_names': ['emit_comm'],
        'bad_names':        [],
    },
    'semantics': {
        'bad_imports':      ['marathon.infra',
                             'bt_observability.ros_comm_tracer'],
        'bad_calls':        [('rospy', 'Publisher'),
                             ('rospy', 'ServiceProxy')],
        'bad_method_names': ['emit_comm'],
        'bad_names':        ['ManagerProxy'],
    },
    'algorithms': {
        'bad_imports':      ['rospy', 'marathon.comm',
                             'marathon.infra', 'bt_observability'],
        'bad_calls':        [],
        'bad_method_names': ['emit_comm', 'set_step',
                             'run_action', 'set_servos_position'],
        'bad_names':        [],
    },
    'tree': {
        'bad_imports':      ['marathon.comm', 'marathon.infra',
                             'marathon.algorithms'],
        'bad_calls':        [('rospy', 'Publisher'),
                             ('rospy', 'ServiceProxy')],
        'bad_method_names': [],
        'bad_names':        ['CommFacade', 'GaitManager', 'MotionManager'],
    },
}


# ---------------------------------------------------------------------------
# AST visitors
# ---------------------------------------------------------------------------

class LayerChecker(ast.NodeVisitor):
    """Walk one source file and collect violations against a rule set."""

    def __init__(self, rules, filepath):
        self.rules = rules
        self.filepath = filepath
        self.violations = []

    def _viol(self, node, msg):
        self.violations.append((self.filepath, node.lineno, msg))

    # ── Import checks ────────────────────────────────────────────────────

    def visit_Import(self, node):
        for alias in node.names:
            self._check_import(node, alias.name)
        self.generic_visit(node)

    def visit_ImportFrom(self, node):
        if node.module:
            self._check_import(node, node.module)
        self.generic_visit(node)

    def _check_import(self, node, module_name):
        for bad in self.rules.get('bad_imports', []):
            if module_name == bad or module_name.startswith(bad + '.'):
                self._viol(node, f"forbidden import: {module_name!r} (matches rule {bad!r})")

    # ── Call checks ──────────────────────────────────────────────────────

    def visit_Call(self, node):
        func = node.func

        # Check module.attr(...) calls — e.g. rospy.Publisher(...)
        if isinstance(func, ast.Attribute) and isinstance(func.value, ast.Name):
            for mod, attr in self.rules.get('bad_calls', []):
                if func.value.id == mod and func.attr == attr:
                    self._viol(node, f"forbidden call: {mod}.{attr}(...)")

        # Check any_obj.method_name(...) calls
        if isinstance(func, ast.Attribute):
            for mname in self.rules.get('bad_method_names', []):
                if func.attr == mname:
                    self._viol(node, f"forbidden method call: .{mname}(...)")

        self.generic_visit(node)

    # ── Name reference checks ────────────────────────────────────────────

    def visit_Name(self, node):
        for bad in self.rules.get('bad_names', []):
            if node.id == bad:
                self._viol(node, f"forbidden name reference: {bad!r}")
        self.generic_visit(node)


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def check_layer(layer_dir: Path, rules: dict) -> list:
    """Check all .py files in layer_dir against rules. Returns violations."""
    violations = []
    for py_file in sorted(layer_dir.rglob('*.py')):
        if '__pycache__' in py_file.parts:
            continue
        try:
            source = py_file.read_text()
            tree = ast.parse(source, filename=str(py_file))
        except SyntaxError as e:
            violations.append((str(py_file), e.lineno, f"SyntaxError: {e}"))
            continue
        checker = LayerChecker(rules, str(py_file))
        checker.visit(tree)
        violations.extend(checker.violations)
    return violations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--marathon-dir', default=None,
                        help='Path to marathon/ directory (default: auto-detect)')
    args = parser.parse_args()

    if args.marathon_dir:
        marathon_dir = Path(args.marathon_dir)
    else:
        # Auto-detect: this script lives at marathon/check_imports.py
        marathon_dir = Path(__file__).parent

    total_violations = []
    for layer, rules in RULES.items():
        layer_dir = marathon_dir / layer
        if not layer_dir.is_dir():
            print(f"[SKIP] {layer}/ not found at {layer_dir}")
            continue
        viols = check_layer(layer_dir, rules)
        total_violations.extend(viols)
        status = 'PASS' if not viols else 'FAIL'
        print(f"[{status}] {layer}/  ({len(viols)} violation(s))")

    if total_violations:
        print()
        print("Violations:")
        for filepath, lineno, msg in total_violations:
            print(f"  {filepath}:{lineno}: {msg}")
        sys.exit(1)
    else:
        print("\nAll checks passed.")
        sys.exit(0)


if __name__ == '__main__':
    main()
