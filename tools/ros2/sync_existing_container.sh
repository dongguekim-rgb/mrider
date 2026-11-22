#!/usr/bin/env bash
# Copy committed team history to a separate checkout in an existing container.
set -euo pipefail
container_name=${1:-ros2_humble}
committed_only=${2:-}
if [ "$#" -gt 2 ] || { [ -n "$committed_only" ] && [ "$committed_only" != --committed ]; }; then
  echo "Usage: $0 [container-name] [--committed]" >&2; exit 2
fi
cd "$(dirname "$0")/../.."
branch_name=$(git symbolic-ref --quiet --short HEAD)
if [ -n "$(git status --porcelain)" ]; then
  if [ "$committed_only" != --committed ]; then
    echo "Commit changes first, or explicitly use --committed to exclude uncommitted changes." >&2
    exit 2
  fi
  echo "Synchronizing committed HEAD only; uncommitted host changes are preserved and EXCLUDED." >&2
fi
if [ "$(docker inspect --format '{{.State.Running}}' "$container_name")" != true ]; then
  echo "Container must already be running: $container_name" >&2
  exit 2
fi
sync_temp=$(mktemp -d /tmp/mrider-container-sync.XXXXXX)
container_bundle="/tmp/$(basename "$sync_temp").bundle"
cleanup() {
  docker exec "$container_name" rm -f "$container_bundle" >/dev/null 2>&1 || true
  rm -rf "$sync_temp"
}
trap cleanup EXIT
# No network or push. Include the official merge ancestry in the bundle.
git bundle create "$sync_temp/source.bundle" "refs/heads/$branch_name"
docker cp "$sync_temp/source.bundle" "$container_name:$container_bundle" >/dev/null
docker exec -i "$container_name" bash -s -- "$container_bundle" "$branch_name" "$(git config user.name)" "$(git config user.email)" <<'CONTAINER'
set -euo pipefail
bundle=$1
branch=$2
dest=/root/mrider-team
team_url=https://github.com/dongguekim-rgb/mrider.git
if [ -e "$dest" ]; then
  test -d "$dest/.git" || { echo "Destination is not a Git checkout" >&2; exit 2; }
  test "$(git -C "$dest" remote get-url origin)" = "$team_url" || {
    echo "Destination origin is not the team repository" >&2; exit 2;
  }
  test -z "$(git -C "$dest" status --porcelain)" || {
    echo "Container checkout has local changes; refusing to overwrite them" >&2; exit 2;
  }
  test "$(git -C "$dest" symbolic-ref --quiet --short HEAD)" = "$branch" || {
    echo "Container checkout is on another branch" >&2; exit 2;
  }
  git -C "$dest" fetch "$bundle" "refs/heads/$branch:refs/remotes/host/$branch"
  git -C "$dest" merge --ff-only "refs/remotes/host/$branch"
else
  git clone --branch "$branch" "$bundle" "$dest"
  git -C "$dest" remote set-url origin "$team_url"
  git -C "$dest" remote add upstream https://github.com/jrkwon/mrider.git
fi
git -C "$dest" config user.name "$3"
git -C "$dest" config user.email "$4"
printf 'Container team checkout: %s\n' "$dest"
git -C "$dest" log -1 --format='%h %s'
CONTAINER
