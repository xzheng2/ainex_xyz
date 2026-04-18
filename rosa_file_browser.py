#!/usr/bin/env python3
"""ROSA Agent File Browser — browse the rosa-agent container filesystem at /.

Runs on the host using docker exec to list dirs and docker cp to open files.
No changes to the container required.
"""
import sys
import os
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QTreeView, QMainWindow, QMessageBox
)
from PyQt5.QtCore import (
    QAbstractItemModel, QModelIndex, Qt
)
from PyQt5.QtGui import QIcon

CONTAINER = 'rosa-agent'
SKIP_DIRS = {'/proc', '/sys', '/dev'}  # kernel virtual filesystems — would hang


class RosaFSItem:
    def __init__(self, name, path, is_dir, parent=None):
        self.name = name
        self.path = path        # absolute path inside container
        self.is_dir = is_dir
        self.parent = parent
        self.children = None    # None = not yet fetched
        self.row = 0            # row index within parent's children list


class RosaDockerFSModel(QAbstractItemModel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._root = RosaFSItem('/', '/', True, parent=None)

    # ── docker helpers ──────────────────────────────────────────────────────

    def _list_dir(self, path):
        """Return list of RosaFSItem children for a container path."""
        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER, 'ls', '-1aF', '--color=never', path],
                capture_output=True, text=True, timeout=5
            )
        except subprocess.TimeoutExpired:
            return []
        items = []
        for line in result.stdout.splitlines():
            name = line.strip()
            if not name or name in ('.', '..', './', '../'):
                continue
            is_dir = name.endswith('/')
            # ls -F appends / for dirs, * for executables, @ for symlinks, etc.
            clean_name = name.rstrip('/*@|=')
            if path == '/':
                child_path = '/' + clean_name
            else:
                child_path = path.rstrip('/') + '/' + clean_name
            items.append(RosaFSItem(clean_name, child_path, is_dir))
        return items

    # ── QAbstractItemModel API ───────────────────────────────────────────────

    def index(self, row, col, parent=QModelIndex()):
        if not self.hasIndex(row, col, parent):
            return QModelIndex()
        if not parent.isValid():
            parent_item = self._root
        else:
            parent_item = parent.internalPointer()
        if parent_item.children is None:
            return QModelIndex()
        if row >= len(parent_item.children):
            return QModelIndex()
        child = parent_item.children[row]
        return self.createIndex(row, col, child)

    def parent(self, index):
        if not index.isValid():
            return QModelIndex()
        item = index.internalPointer()
        par = item.parent
        if par is None or par is self._root:
            return QModelIndex()
        return self.createIndex(par.row, 0, par)

    def rowCount(self, parent=QModelIndex()):
        if parent.column() > 0:
            return 0
        item = parent.internalPointer() if parent.isValid() else self._root
        if item.children is None:
            return 0
        return len(item.children)

    def columnCount(self, parent=QModelIndex()):
        return 1

    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid():
            return None
        item = index.internalPointer()
        if role == Qt.DisplayRole:
            return item.name
        if role == Qt.DecorationRole:
            style = QApplication.style()
            if item.is_dir:
                return style.standardIcon(style.SP_DirIcon)
            return style.standardIcon(style.SP_FileIcon)
        return None

    def hasChildren(self, parent=QModelIndex()):
        item = parent.internalPointer() if parent.isValid() else self._root
        if not item.is_dir:
            return False
        if item.path in SKIP_DIRS:
            return False
        return True   # assume dirs have children until proven empty

    def canFetchMore(self, parent):
        item = parent.internalPointer() if parent.isValid() else self._root
        return item.is_dir and item.children is None and item.path not in SKIP_DIRS

    def fetchMore(self, parent):
        item = parent.internalPointer() if parent.isValid() else self._root
        children = self._list_dir(item.path)
        for i, child in enumerate(children):
            child.parent = item
            child.row = i
        self.beginInsertRows(parent, 0, len(children) - 1)
        item.children = children
        self.endInsertRows()

    def filePath(self, index):
        if not index.isValid():
            return '/'
        return index.internalPointer().path

    def isDir(self, index):
        if not index.isValid():
            return True
        return index.internalPointer().is_dir


class RosaFileBrowser(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f'ROSA Agent File Browser  [{CONTAINER}:/]')
        self.resize(700, 900)
        self.model = RosaDockerFSModel()
        self.tree = QTreeView()
        self.tree.setModel(self.model)
        self.tree.setColumnWidth(0, 400)
        self.tree.doubleClicked.connect(self.on_double_click)
        self.setCentralWidget(self.tree)

    def on_double_click(self, index):
        if self.model.isDir(index):
            return
        container_path = self.model.filePath(index)
        basename = os.path.basename(container_path)
        tmp_path = f'/tmp/rosa_browser_{basename}'
        try:
            subprocess.run(
                ['docker', 'cp', f'{CONTAINER}:{container_path}', tmp_path],
                check=True, timeout=10
            )
        except subprocess.CalledProcessError as e:
            QMessageBox.warning(self, 'Copy failed', str(e))
            return
        subprocess.Popen(['gedit', tmp_path])


def main():
    # Quick container check
    result = subprocess.run(
        ['docker', 'inspect', '--format={{.State.Running}}', CONTAINER],
        capture_output=True, text=True
    )
    if result.stdout.strip() != 'true':
        print(f"ERROR: container '{CONTAINER}' is not running.", file=sys.stderr)
        sys.exit(1)

    app = QApplication(sys.argv)
    w = RosaFileBrowser()
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
