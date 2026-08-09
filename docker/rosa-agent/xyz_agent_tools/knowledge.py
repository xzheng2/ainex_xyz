"""
knowledge.py — progressive knowledge loading for the ROSA-XYZ agent.

The long ROS topic/service/tool reference lives in knowledge/ros_reference.md
instead of the base system prompt.  retrieve_knowledge(query) returns only the
Markdown sections whose keyword line matches the user query; xyz_agent.py
prepends them to the query as a [Reference context] block.

Markdown format expected:
    ## Section Title
    <!-- keywords: word, another word, phrase with spaces -->
    ...section body...

Pure stdlib, no dependencies.
"""

import re
from pathlib import Path
from typing import List, Tuple

_KNOWLEDGE_FILE = Path(__file__).parent.parent / "knowledge" / "ros_reference.md"

_KEYWORDS_RE = re.compile(r"<!--\s*keywords:\s*(.*?)\s*-->", re.IGNORECASE)


def _parse_sections(text: str) -> List[Tuple[str, List[str], str]]:
    """Split markdown into (title, keywords, body) tuples on '## ' headings."""
    sections = []
    parts = re.split(r"^## ", text, flags=re.MULTILINE)
    for part in parts[1:]:  # parts[0] is the preamble before the first section
        title, _, body = part.partition("\n")
        m = _KEYWORDS_RE.search(body)
        keywords = []
        if m:
            keywords = [k.strip().lower() for k in m.group(1).split(",") if k.strip()]
            body = _KEYWORDS_RE.sub("", body, count=1)
        sections.append((title.strip(), keywords, body.strip()))
    return sections


def _matches(query_lower: str, keywords: List[str]) -> bool:
    for kw in keywords:
        # Word-boundary match so short keywords like 'bt' don't hit 'robot'.
        if re.search(r"\b" + re.escape(kw) + r"\b", query_lower):
            return True
    return False


def retrieve_knowledge(query: str, knowledge_file: Path = None) -> str:
    """
    Return the markdown sections relevant to the query, or "" if none match
    (or the knowledge file is missing/unreadable — never raises).
    """
    path = knowledge_file or _KNOWLEDGE_FILE
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return ""

    query_lower = query.lower()
    matched = [
        f"## {title}\n{body}"
        for title, keywords, body in _parse_sections(text)
        if _matches(query_lower, keywords)
    ]
    return "\n\n".join(matched)
