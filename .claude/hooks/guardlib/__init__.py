"""guardlib -- the single source of truth for the BT contract rules.

Deliberately EMPTY: no re-exports. A convenience alias here would give every module
two import paths (`guardlib.path_spec` and `guardlib.PathSpec`), and two names for one
thing is the exact disease this package exists to cure. Import the submodule you need.

Module scope across this package is restricted to imports, `re.compile` and constant
assignment. `validate_engine.hooks_import_safe()` AST-rejects a bare call statement at
module scope, because it imports these modules to reuse their rules -- anything that
executes on import would run inside the push gate.
"""
