#!/usr/bin/env python3
"""One-way bt.py -> Groot XML parameter sync (stdlib only).

For a given ``*groot.xml`` this finds the sibling ``*_bt.py`` (the runtime source
of truth), reads the *effective* parameter values of every tree node, and writes
them into the matching XML nodes, producing ``<stem>.synced.xml`` next to the
original. It is strictly one-way (bt.py wins) and touches parameters only -- never
tree structure. Anything it cannot resolve safely is skipped with a warning.

Effective value of a parameter = the class default declared in xyz_bt_lib
(``CONFIG_DEFAULTS`` dict or ``__init__`` signature default), overridden by any
value the bt.py call passes explicitly. Only values that resolve statically to a
scalar literal (int/float/str/bool/None) are synced; dicts/expressions/calls are
reported and left untouched.

Usage:
    sync_groot_xml.py <path/to/xxx_groot.xml> [--bt PATH] [--edu DIR] [--quiet]
"""
import argparse
import ast
import glob
import os
import re
import sys

UNRESOLVED = object()          # sentinel: a value we could not statically resolve
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Structural attributes that identify a node and must never be treated as params.
_IDENTITY_ATTRS = {'name', 'ID'}
# XML tags whose ID attribute names the Python class directly (safety cross-check).
_LEAF_TAGS = {'Action', 'Condition'}


# --------------------------------------------------------------------------- #
# Value resolution helpers
# --------------------------------------------------------------------------- #
def _resolve(node, src, scalar_syms):
    """Resolve an AST value node to (python_value, source_text) or (UNRESOLVED, None).

    Only scalar literals qualify (int/float/str/bool/None, incl. unary minus).
    Names are resolved against ``scalar_syms`` (module/function literal constants).
    Dicts, lists, calls, comprehensions, f-strings, arithmetic -> UNRESOLVED.
    """
    if isinstance(node, ast.Name):
        return scalar_syms.get(node.id, (UNRESOLVED, None))
    try:
        value = ast.literal_eval(node)
    except Exception:
        return (UNRESOLVED, None)
    if isinstance(value, (dict, list, tuple, set)):
        return (UNRESOLVED, None)
    text = ast.get_source_segment(src, node)
    return (value, text)


def _build_symbols(scopes, src):
    """From a list of statement bodies, build scalar + dict symbol tables.

    scalar_syms: name -> (value, text)          (simple literal constants)
    dict_syms:   name -> {key: (value, text)}   (literal dicts, for ** expansion)
    """
    scalar_syms, dict_syms = {}, {}
    for body in scopes:
        for stmt in body:
            if not isinstance(stmt, ast.Assign) or len(stmt.targets) != 1:
                continue
            target = stmt.targets[0]
            if not isinstance(target, ast.Name):
                continue
            name, val = target.id, stmt.value
            dict_items = _dict_literal_items(val, src, scalar_syms)
            if dict_items is not None:
                dict_syms[name] = dict_items
                continue
            v, t = _resolve(val, src, scalar_syms)
            if v is not UNRESOLVED:
                scalar_syms[name] = (v, t)
    return scalar_syms, dict_syms


def _dict_literal_items(node, src, scalar_syms):
    """Return {key: (value, text)} for a dict literal or dict(...) call, else None."""
    items = {}
    if isinstance(node, ast.Dict):
        for k, v in zip(node.keys, node.values):
            if not (isinstance(k, ast.Constant) and isinstance(k.value, str)):
                return None
            items[k.value] = _resolve(v, src, scalar_syms)
        return items
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) \
            and node.func.id == 'dict':
        for kw in node.keywords:
            if kw.arg is None:
                return None
            items[kw.arg] = _resolve(kw.value, src, scalar_syms)
        return items
    return None


# --------------------------------------------------------------------------- #
# xyz_bt_lib class-parameter registry (CONFIG_DEFAULTS + __init__ defaults)
# --------------------------------------------------------------------------- #
def _scan_class_params(py_files):
    """class_name -> {param: (value, text)} harvested from CONFIG_DEFAULTS + __init__."""
    registry = {}
    for path in py_files:
        try:
            with open(path, encoding='utf-8') as fh:
                src = fh.read()
            tree = ast.parse(src)
        except (OSError, SyntaxError):
            continue
        module_scalars, _ = _build_symbols([tree.body], src)
        for cls in tree.body:
            if not isinstance(cls, ast.ClassDef):
                continue
            params = {}
            # __init__ signature defaults (fills in every keyword-with-default).
            for fn in cls.body:
                if isinstance(fn, ast.FunctionDef) and fn.name == '__init__':
                    args = fn.args.args[1:]              # drop self
                    defaults = fn.args.defaults
                    for arg, dflt in zip(args[len(args) - len(defaults):], defaults):
                        params[arg.arg] = _resolve(dflt, src, module_scalars)
            # CONFIG_DEFAULTS dict overrides / adds (authoritative tunable set).
            for stmt in cls.body:
                if isinstance(stmt, ast.Assign) and any(
                        isinstance(t, ast.Name) and t.id == 'CONFIG_DEFAULTS'
                        for t in stmt.targets):
                    items = _dict_literal_items(stmt.value, src, module_scalars)
                    if items:
                        params.update(items)
            registry[cls.name] = params
    return registry


def _registry_files(bt_path, edu_dir):
    """xyz_bt_lib package (recursive) + the XML project's behaviours dir."""
    files = []
    if edu_dir and os.path.isdir(edu_dir):
        files += glob.glob(os.path.join(edu_dir, '**', '*.py'), recursive=True)
    proj_beh = _find_project_behaviours(bt_path)
    if proj_beh:
        files += glob.glob(os.path.join(proj_beh, '**', '*.py'), recursive=True)
    return files


def _find_project_behaviours(bt_path):
    """Walk up from bt.py to the nearest ancestor holding a 'behaviours' dir."""
    d = os.path.dirname(os.path.abspath(bt_path))
    while True:
        cand = os.path.join(d, 'behaviours')
        if os.path.isdir(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d or os.path.basename(d) == 'xyz_behavior':
            return None
        d = parent


# --------------------------------------------------------------------------- #
# bt.py extraction: instance-name -> effective params
# --------------------------------------------------------------------------- #
def _extract_nodes(bt_path, registry):
    """Return (index, warnings).

    index: instance_name -> (class_name, {param: (value, text)}, lineno)
    Duplicate instance names are dropped from the index and reported as ambiguous.
    """
    with open(bt_path, encoding='utf-8') as fh:
        src = fh.read()
    tree = ast.parse(src)

    scopes = [tree.body]
    for fn in ast.walk(tree):
        if isinstance(fn, ast.FunctionDef):
            scopes.append(fn.body)
    scalar_syms, dict_syms = _build_symbols(scopes, src)

    index, warnings, seen = {}, [], {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        cname = _callable_name(call.func)
        if not cname:
            continue
        iname = _instance_name(call)
        if iname is None:
            continue

        params = dict(registry.get(cname, {}))          # class defaults first
        for kw in call.keywords:
            if kw.arg is None:                          # **spread
                if isinstance(kw.value, ast.Name) and kw.value.id in dict_syms:
                    params.update(dict_syms[kw.value.id])
                continue
            if kw.arg in _IDENTITY_ATTRS:
                continue
            params[kw.arg] = _resolve(kw.value, src, scalar_syms)

        seen[iname] = seen.get(iname, 0) + 1
        index[iname] = (cname, params, call.lineno)

    for iname, count in seen.items():
        if count > 1:
            index.pop(iname, None)
            warnings.append(f"ambiguous: '{iname}' defined {count}x in "
                            f"{os.path.basename(bt_path)} -> skipped")
    return index, warnings


def _callable_name(func):
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr                                # e.g. py_trees.decorators.X -> X
    return None


def _instance_name(call):
    """First positional string arg, else a string name= kwarg, else None."""
    if call.args and isinstance(call.args[0], ast.Constant) \
            and isinstance(call.args[0].value, str):
        return call.args[0].value
    for kw in call.keywords:
        if kw.arg == 'name' and isinstance(kw.value, ast.Constant) \
                and isinstance(kw.value.value, str):
            return kw.value.value
    return None


# --------------------------------------------------------------------------- #
# XML comparison + emission
# --------------------------------------------------------------------------- #
def _to_xml_text(value, text):
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, str):
        return value
    return text if text is not None else repr(value)    # int / float: authored form


def _equal(old_str, value):
    """Semantic equality between the current XML string and a python value."""
    old = old_str.strip()
    if value is None:
        return old == ''
    if isinstance(value, bool):
        return old.lower() == ('true' if value else 'false')
    if isinstance(value, (int, float)):
        try:
            return float(old) == float(value)
        except ValueError:
            return False
    return old_str == value


def _iter_elements(xml_path):
    import xml.etree.ElementTree as ET
    root = ET.parse(xml_path).getroot()
    for elem in root.iter():
        yield elem


def _compute_changes(xml_path, index):
    """Return (changes, warnings). changes: list of (name, attr, old, new)."""
    changes, warnings = [], []
    for elem in _iter_elements(xml_path):
        name = elem.attrib.get('name')
        if not name or name not in index:
            continue
        cname, params, _ = index[name]
        xml_id = elem.attrib.get('ID')
        if elem.tag in _LEAF_TAGS and xml_id and xml_id != cname:
            warnings.append(f"id mismatch: <{elem.tag} name='{name}'> ID='{xml_id}' "
                            f"but bt.py class is '{cname}' -> skipped")
            continue
        for attr, old in elem.attrib.items():
            if attr in _IDENTITY_ATTRS or attr not in params:
                continue
            value, text = params[attr]
            if value is UNRESOLVED:
                warnings.append(f"unresolved: {name}.{attr} is dynamic in bt.py -> "
                                f"left as '{old}'")
                continue
            if not _equal(old, value):
                changes.append((name, attr, old, _to_xml_text(value, text)))
    return changes, warnings


def _apply_changes(xml_path, changes):
    """Minimal-diff text edit: rewrite only the changed attribute values. Returns
    (new_text, applied, warnings)."""
    with open(xml_path, encoding='utf-8') as fh:
        text = fh.read()

    by_node = {}
    for name, attr, old, new in changes:
        by_node.setdefault(name, []).append((attr, old, new))

    warnings, applied = [], []
    for name, attrs in by_node.items():
        tag_re = re.compile(r'<[^>]*\bname="' + re.escape(name) + r'"[^>]*>')
        matches = list(tag_re.finditer(text))
        if len(matches) != 1:
            warnings.append(f"apply: found {len(matches)} tags with name=\"{name}\" "
                            f"-> skipped (expected 1)")
            continue
        m = matches[0]
        tag = m.group(0)
        for attr, old, new in attrs:
            attr_re = re.compile(r'(\b' + re.escape(attr) + r'=")[^"]*(")')
            if not attr_re.search(tag):
                warnings.append(f"apply: attr '{attr}' not found in <{name}> -> skipped")
                continue
            tag = attr_re.sub(lambda mm: mm.group(1) + new + mm.group(2), tag, count=1)
            applied.append((name, attr, old, new))
        text = text[:m.start()] + tag + text[m.end():]
    return text, applied, warnings


# --------------------------------------------------------------------------- #
# Driver
# --------------------------------------------------------------------------- #
def _find_bt(xml_path):
    d = os.path.dirname(xml_path)
    base = os.path.basename(xml_path)
    primary = os.path.join(d, re.sub(r'_groot\.xml$', '_bt.py', base))
    if base.endswith('_groot.xml') and os.path.isfile(primary):
        return primary
    candidates = sorted(glob.glob(os.path.join(d, '*_bt.py')))
    if len(candidates) == 1:
        return candidates[0]
    return None


def sync_one(xml_path, bt_path=None, edu_dir=None, quiet=False):
    """Sync a single groot.xml. Returns (n_changes, n_warnings) or (-1, -1) on hard error."""
    if not os.path.isfile(xml_path):
        print(f"ERROR: no such file: {xml_path}", file=sys.stderr)
        return (-1, -1)

    bt_path = bt_path or _find_bt(xml_path)
    if not bt_path or not os.path.isfile(bt_path):
        print(f"SKIP {xml_path} -- no matching bt.py found", file=sys.stderr)
        return (-1, -1)

    try:
        registry = _scan_class_params(_registry_files(bt_path, edu_dir))
        index, w_extract = _extract_nodes(bt_path, registry)
    except SyntaxError as exc:
        print(f"ERROR parsing {bt_path}: {exc}", file=sys.stderr)
        return (-1, -1)
    try:
        changes, w_compute = _compute_changes(xml_path, index)
    except Exception as exc:                            # unparseable XML, etc.
        print(f"ERROR parsing {xml_path}: {exc}", file=sys.stderr)
        return (-1, -1)

    new_text, applied, w_apply = _apply_changes(xml_path, changes)
    warnings = w_extract + w_compute + w_apply

    out_path = re.sub(r'\.xml$', '.synced.xml', xml_path)
    with open(out_path, 'w', encoding='utf-8') as fh:
        fh.write(new_text)

    if applied or warnings or not quiet:
        print(f"SYNCED {out_path} -- {len(applied)} change(s), {len(warnings)} warning(s)")
        for name, attr, old, new in applied:
            print(f"  {name}.{attr}: {old} -> {new}")
        for msg in warnings:
            print(f"  WARN {msg}")
    return (len(applied), len(warnings))


def _default_edu_dir():
    guess = os.path.normpath(os.path.join(
        _SCRIPT_DIR, '..', '..', 'xyz_bt_lib', 'src', 'xyz_bt_lib'))
    return guess if os.path.isdir(guess) else None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('xml', help='path to a *groot.xml file')
    ap.add_argument('--bt', help='override the bt.py source path')
    ap.add_argument('--edu', help='xyz_bt_lib package dir (default: auto-detect)')
    ap.add_argument('--quiet', action='store_true',
                    help='suppress output for files with no changes/warnings')
    args = ap.parse_args(argv)

    edu_dir = args.edu or _default_edu_dir()
    n_changes, n_warn = sync_one(args.xml, args.bt, edu_dir, args.quiet)
    return 1 if n_changes < 0 else 0


if __name__ == '__main__':
    sys.exit(main())
