#!/usr/bin/env sh
set -eu

# VS Code installed as a Snap exports GTK/Snap runtime paths into integrated
# terminals. Tauri's WebKit/GTK stack must use the host system libraries.
unset SNAP
unset SNAP_ARCH
unset SNAP_COMMON
unset SNAP_CONTEXT
unset SNAP_COOKIE
unset SNAP_DATA
unset SNAP_EUID
unset SNAP_INSTANCE_NAME
unset SNAP_LAUNCHER_ARCH_TRIPLET
unset SNAP_LIBRARY_PATH
unset SNAP_NAME
unset SNAP_REAL_HOME
unset SNAP_REVISION
unset SNAP_UID
unset SNAP_USER_COMMON
unset SNAP_USER_DATA
unset SNAP_VERSION

unset GTK_EXE_PREFIX
unset GTK_IM_MODULE_FILE
unset GTK_MODULES
unset GTK_PATH
unset LOCPATH
unset GDK_PIXBUF_MODULEDIR
unset GDK_PIXBUF_MODULE_FILE
unset GIO_MODULE_DIR
unset GSETTINGS_SCHEMA_DIR
unset XDG_DATA_HOME

if [ -n "${XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG:-}" ]; then
  export XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
fi

if [ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]; then
  export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
fi

exec "$@"
