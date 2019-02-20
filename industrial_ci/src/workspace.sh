#!/bin/bash

# Copyright (c) 2019, Mathias Lüdtke
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

function ici_resolve_scheme {
    local url=$1; shift
    if [[ $url =~ ([^:]+):([^#@]+)[#@](.+) ]]; then
        local fragment="${BASH_REMATCH[3]}"
        local repo=${BASH_REMATCH[2]}
        local name=${repo##*/}
        local scheme=${BASH_REMATCH[1]}

        case "$scheme" in
            bitbucket | bb)
                echo "${name%.git}" "git" "https://bitbucket.org/$repo" "$fragment"
                ;;
            github | gh)
                echo "${name%.git}" "git" "https://github.com/$repo" "$fragment"
                ;;
            gitlab | gl)
                echo "${name%.git}" "git" "https://gitlab.com/$repo" "$fragment"
                ;;
            'git+file'*|'git+http'*)
                echo "${name%.git}" "git" "${scheme#git+}:$repo" "$fragment"
                ;;
            git+*)
                echo "${name%.git}" "git" "$scheme:$repo" "$fragment"
                ;;
            *)
                echo "$name" "$scheme" "$scheme:$repo" "$fragment"
                ;;
        esac
    else
        ici_error "could not parse URL '$url'"
    fi

}

function ici_exec_for_command {
  local command=$1; shift
  if ! command -v "$command" > /dev/null; then
    "$@"
  fi
}

function ici_install_pkgs_for_command {
  local command=$1; shift
  ici_exec_for_command "$command" ici_asroot apt-get -qq install --no-install-recommends -y "$@"
}

function ici_import_repository {
    local sourcespace=$1; shift
    local url=$1; shift

    ici_install_pkgs_for_command vcs python-vcstool

    IFS=" " read -r -a parts <<< "$(ici_resolve_scheme "$url")" # name, type, url, version

    case "${parts[1]}" in
        git)
          ici_install_pkgs_for_command git git-core
            ;;
        *)
            ;;
    esac
    vcs import "$sourcespace" <<< "{repositories: {'${parts[0]}': {type: '${parts[1]}', url: '${parts[2]}', version: '${parts[3]}'}}}"
}

function ici_import_file {
    local sourcespace=$1; shift
    local file=$1; shift

    case "$file" in
    *.zip|*.tar|*.tar.*|*.tgz|*.tbz2)
        ici_install_pkgs_for_command bsdtar bsdtar
        bsdtar -C "$sourcespace" -xf "$file"
        ;;
    *)
        ici_install_pkgs_for_command vcs python-vcstool
        ici_install_pkgs_for_command git git-core
        vcs import "$sourcespace" < "$file"
    ;;
    esac

}

function ici_import_url {
    local sourcespace=$1; shift
    local url=$1; shift
    local processor

    ici_install_pkgs_for_command wget wget

    case "$url" in
    *.zip|*.tar|*.tar.*|*.tgz|*.tbz2)
        ici_install_pkgs_for_command bsdtar bsdtar
        processor=(bsdtar -C "$sourcespace" -xf-)
        ;;
    *)
        ici_install_pkgs_for_command vcs python-vcstool
        ici_install_pkgs_for_command git git-core
        processor=(vcs import "$sourcespace")
    ;;
    esac

    set -o pipefail
    wget -O- -q "$url" | "${processor[@]}"
    set +o pipefail
}

function  ici_import_directory {
    local sourcespace=$1; shift
    local dir=$1; shift
    rm -rf "$sourcespace:?/$(basename "$dir")"
    cp -a "$dir" "$sourcespace"
}

function ici_prepare_sourcespace {
    local sourcespace=$1; shift
    local basepath=$TARGET_REPO_PATH

    mkdir -p "$sourcespace"

    for source in "$@"; do
        case "$source" in
        git* | bitbucket:* | bb:* | gh:* | gl:*)
            ici_import_repository "$sourcespace" "$source"
            ;;
        http://* | https://*) # When UPSTREAM_WORKSPACE is an http url, use it directly
            ici_import_url "$sourcespace" "$source"
            ;;
        -.)
            local file; file=$(basename "$basepath")
            echo "Removing '${sourcespace:?}/$file'"
            rm -r "${sourcespace:?}/$file"
            ;;
        -*)
            local file="${source:1}"
            if [ ! -e "${sourcespace:?}/$file" ]; then
              file="$(basename "$basepath")/$file"
            fi
            echo "Removing '${sourcespace:?}/$file'"
            rm -r "${sourcespace:?}/$file"
            ;;
        .)
            echo "Copying '$basepath'"
            ici_import_directory "$sourcespace" "$basepath"
            ;;
        /*)
            if [ -d "$source" ]; then
                echo "Copying '$source'"
                ici_import_directory  "$sourcespace" "$source"
            elif [ -f "$source" ]; then
                ici_import_file "$sourcespace" "$source"
            else
                ici_error "'$source' cannot be found"
            fi
            ;;
        "")
            ici_error "source is empty string"
            ;;
        *)
            if [ -d "$basepath/$source" ]; then
                echo "Copying '$source'"
                ici_import_directory "$sourcespace" "$basepath/$source"
            elif [ -f "$basepath/$source" ]; then
                ici_import_file "$sourcespace" "$basepath/$source"
            else
                ici_error "cannot read source from '$source'"
            fi
            ;;
        esac
    done
}

function ici_setup_rosdep {
    ici_install_pkgs_for_command rosdep python-rosdep
    # Setup rosdep
    rosdep --version
    if ! [ -d /etc/ros/rosdep/sources.list.d ]; then
        ici_asroot rosdep init
    fi

    update_opts=()
    if [ "$ROS_VERSION_EOL" = true ]; then
        update_opts+=(--include-eol-distros)
    fi

    ici_retry 2 rosdep update "${update_opts[@]}"
}

function ici_exec_in_workspace {
    local extend=$1; shift
    local path=$1; shift
    # shellcheck disable=SC1090
    ( { [ ! -e "$extend/setup.bash" ] || source "$extend/setup.bash"; } && cd "$path" && exec "$@")
}

function ici_install_dependencies {
    local extend=$1; shift
    local skip_keys=$1; shift
    rosdep_opts=(-q --from-paths "$@" --ignore-src -y)
    if [ -n "$skip_keys" ]; then
      rosdep_opts+=(--skip-keys "$skip_keys")
    fi
    set -o pipefail # fail if rosdep install fails
    ici_exec_in_workspace "$extend" "." rosdep install "${rosdep_opts[@]}" | { grep "executing command" || true; }
    set +o pipefail
}
