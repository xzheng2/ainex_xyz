"""Source-text noise removal, shared by every content rule.

WHAT WAS MERGED HERE
    Four full strippers with byte-identical regexes and byte-identical behaviour, under
    three different names:

      xyz_bt_lib_guard._strip_noise                three statements
      xyz_behavior_guard._strip_noise              same, plus a docstring
      xyz_bt_l1_running_guard._strip_noise         nested one-liner
      xyz_bt_tree_pre_guard._strip_strings_and_comments   same one-liner, other name

    The two forms are equivalent -- strings are blanked first, then comments -- so the
    merged `strip_noise()` produces identical output for all four callers.

WHY strip_comments() STAYS SEPARATE
    xyz_bt_tree_pre_guard uses a comments-only variant on purpose: its state_key rule
    has to SEE string literals, because a literal `state_key='grab_confirmed'` is what
    passing looks like. Folding it into strip_noise() would blank the very thing the
    rule inspects and make every literal read as a violation. Two functions, one reason.

WHY THE REGEXES ARE NOT "IMPROVED"
    `_STRING_RE` does not handle escaped quotes or f-string nesting, and `_COMMENT_RE`
    will happily strip a `#` that lives inside a string it did not match. That is the
    behaviour every existing rule was written and tuned against; a more correct
    tokenizer here would silently change verdicts across the whole rule set. Any change
    to these two patterns is a change to every rule at once.
"""
import re

#: Byte-identical to the four copies this replaces. Order of declaration differed
#: between them (cosmetic -- they are independent).
_COMMENT_RE = re.compile(r'#[^\n]*')
_STRING_RE = re.compile(r'("""[\s\S]*?"""|\'\'\'[\s\S]*?\'\'\'|"[^"\n]*"|\'[^\'\n]*\')')


def strip_strings(src):
    """Blank every string literal, leaving comments intact."""
    return _STRING_RE.sub('""', src)


def strip_comments(src):
    """Drop every comment, leaving string literals intact.

    For rules that must still see a literal -- see the module docstring.
    """
    return _COMMENT_RE.sub('', src)


def strip_noise(src):
    """Blank string literals, then drop comments.

    The order matters: blanking strings first means a `#` inside a string is already
    gone by the time the comment pass runs.
    """
    return _COMMENT_RE.sub('', _STRING_RE.sub('""', src))
