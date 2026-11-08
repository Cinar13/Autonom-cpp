#!/usr/bin/env bash
# One-shot setup of the Autonom-cpp workspace on Ubuntu 24.04:
#   1. ROS 2 Jazzy desktop + Gazebo Harmonic bridge + build tools (apt, needs sudo)
#   2. GitHub access over SSH: if this machine cannot read the repository yet, a dedicated key
#      (~/.ssh/id_ed25519_autonom) is created and shown, to add to a GitHub account that has access
#   3. clone (or update) the repository with its submodules
#   4. rosdep dependencies and colcon build
#   5. start the simulation, only if you want to
#
# Usage (works standalone, e.g. copied to a new machine):
#   bash install.sh [--branch optimised] [--dir ~/Autonom-cpp] [--run | --no-run]
set -euo pipefail

REPO="Cinar13/Autonom-cpp"
BRANCH="optimised"
DEST="$HOME/Autonom-cpp"
RUN=""  # "", yes, no

while [[ $# -gt 0 ]]; do
  case "$1" in
    --branch) BRANCH="$2"; shift 2 ;;
    --dir) DEST="$2"; shift 2 ;;
    --run) RUN="yes"; shift ;;
    --no-run) RUN="no"; shift ;;
    -h | --help) sed -n '2,11p' "$0"; exit 0 ;;
    *) echo "Unknown option: $1 (see --help)" >&2; exit 1 ;;
  esac
done

KEY="$HOME/.ssh/id_ed25519_autonom"
HOST_ALIAS="github-autonom"
step() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m%s\033[0m\n' "$*" >&2; }

# --- 0. Platform ----------------------------------------------------------------------------------------------
# shellcheck disable=SC1091
. /etc/os-release
if [[ "${VERSION_CODENAME:-}" != "noble" ]]; then
  warn "This stack targets Ubuntu 24.04 (noble); found ${PRETTY_NAME:-unknown}. ROS 2 Jazzy may not install."
fi
if [[ $EUID -eq 0 ]]; then
  echo "Run this as your normal user (it uses sudo where needed), not as root." >&2
  exit 1
fi

# --- 1. ROS 2 Jazzy and tools ---------------------------------------------------------------------------------
step "Installing base tools"
sudo apt-get update
sudo apt-get install -y git curl openssh-client software-properties-common

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  step "Installing ROS 2 Jazzy (desktop)"
  sudo add-apt-repository -y universe
  ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
  sudo apt-get install -y ros-jazzy-desktop
else
  echo "ROS 2 Jazzy already installed."
fi

step "Installing Gazebo bridge, CycloneDDS and build tools"
sudo apt-get install -y ros-jazzy-ros-gz ros-jazzy-rmw-cyclonedds-cpp python3-rosdep python3-colcon-common-extensions

# --- 2. GitHub access -----------------------------------------------------------------------------------------
can_read() {  # can_read <url> [batch]: true when git can list the repository
  local opts="-o StrictHostKeyChecking=accept-new -o ConnectTimeout=10"
  [[ "${2:-}" == batch ]] && opts="$opts -o BatchMode=yes"
  GIT_SSH_COMMAND="ssh $opts" git ls-remote "$1" >/dev/null 2>&1
}

step "Checking GitHub access to $REPO"
REPO_URL="git@github.com:${REPO}.git"
if can_read "$REPO_URL" batch; then
  echo "This machine can already read $REPO."
else
  # A dedicated key + host alias: works even when another SSH key on this machine belongs to a GitHub account
  # without access (GitHub does not accept one key on two accounts).
  mkdir -p "$HOME/.ssh"
  chmod 700 "$HOME/.ssh"
  if [[ ! -f "$KEY" ]]; then
    echo "Creating an SSH key for GitHub: $KEY"
    echo "(You may set a passphrase; press Enter twice for none.)"
    ssh-keygen -t ed25519 -f "$KEY" -C "autonom-$(whoami)@$(hostname)"
  fi
  if ! grep -q "^Host ${HOST_ALIAS}\$" "$HOME/.ssh/config" 2>/dev/null; then
    cat >> "$HOME/.ssh/config" <<EOF

Host ${HOST_ALIAS}
  HostName github.com
  User git
  IdentityFile ${KEY}
  IdentitiesOnly yes
EOF
    chmod 600 "$HOME/.ssh/config"
  fi
  REPO_URL="git@${HOST_ALIAS}:${REPO}.git"

  until can_read "$REPO_URL"; do
    who="$(ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 -T "git@${HOST_ALIAS}" 2>&1 \
      | sed -n 's/^Hi \([^!]*\)!.*/\1/p' || true)"
    echo
    if [[ -n "$who" ]]; then
      warn "The key works for the GitHub account '$who', but that account cannot read $REPO."
      warn "Ask the repository owner to add '$who' as a collaborator, or add the key to an account that has access."
    else
      warn "GitHub does not know this key yet."
    fi
    cat <<EOF

  1. Sign in to GitHub with an account that has access to ${REPO}.
  2. Open https://github.com/settings/ssh/new
  3. Title: Autonom $(hostname)   Key type: Authentication Key
  4. Paste this public key and click "Add SSH key":

$(cat "${KEY}.pub")

EOF
    if command -v wl-copy >/dev/null; then wl-copy < "${KEY}.pub" && echo "(The key is also on your clipboard.)"
    elif command -v xclip >/dev/null; then xclip -selection clipboard < "${KEY}.pub" && echo "(The key is also on your clipboard.)"
    fi
    read -rp "Press Enter when the key is added (Ctrl+C to stop) " _
  done
  echo "GitHub access works."
fi

# --- 3. Clone or update ---------------------------------------------------------------------------------------
if [[ -d "$DEST/.git" ]]; then
  step "Updating $DEST (branch $BRANCH)"
  git -C "$DEST" fetch origin
  git -C "$DEST" checkout "$BRANCH"
  git -C "$DEST" pull --ff-only origin "$BRANCH"
  git -C "$DEST" submodule update --init --recursive
elif [[ -e "$DEST" ]]; then
  echo "$DEST exists but is not a git repository; choose another place with --dir." >&2
  exit 1
else
  step "Cloning $REPO (branch $BRANCH) into $DEST"
  git clone --recurse-submodules -b "$BRANCH" "$REPO_URL" "$DEST"
fi

# --- 4. Dependencies and build --------------------------------------------------------------------------------
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
cd "$DEST"
step "Installing ROS dependencies (rosdep)"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
# cmake_modules: an obsolete ROS 1 key in rf2o's package.xml that its CMake never uses.
rosdep install --from-paths src --ignore-src -y --skip-keys cmake_modules

step "Building the workspace (colcon)"
colcon build --symlink-install

cat <<EOF

Done. The workspace is in $DEST.
  Start the simulation:  $DEST/scripts/start_sim.sh
  Run the tests:         cd $DEST && source install/setup.bash && colcon test && colcon test-result --all
EOF

# --- 5. Start the simulation, only if wanted ------------------------------------------------------------------
if [[ -z "$RUN" ]]; then
  read -rp "Start the simulation now? [y/N] " answer
  [[ "$answer" =~ ^[Yy]([Ee][Ss])?$ ]] && RUN="yes" || RUN="no"
fi
if [[ "$RUN" == "yes" ]]; then
  exec "$DEST/scripts/start_sim.sh"
fi
