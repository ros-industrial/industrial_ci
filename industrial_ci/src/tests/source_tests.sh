#!/bin/bash

# Copyright (c) 2015, Isaac I. Y. Saito
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
## Greatly inspired by JSK travis https://github.com/jsk-ros-pkg/jsk_travis

# source_tests.sh script runs integration tests for the target ROS packages.
# It is dependent on environment variables that need to be exported in advance
# (As of version 0.4.4 most of them are defined in env.sh).

function install_catkin_lint {
    ici_install_pkgs_for_command catkin_lint "${PYTHON_VERSION_NAME}-catkin-lint"
}

function run_clang_tidy {
    local regex="$1/.*"
    local -n _run_clang_tidy_warnings=$2
    local -n _run_clang_tidy_errors=$3
    local db=$4
    shift 4

    mapfile -t files < <(grep -oP "(?<=\"file\": \")($regex)(?=\")" "$db")
    if [ "${#files[@]}" -eq 0 ]; then
        return 0
    fi

    local build; build="$(dirname "$db")"
    local name; name="$(basename "$build")"

    local max_jobs="${CLANG_TIDY_JOBS:-$(nproc)}"
    if ! [ "$max_jobs" -ge 1 ]; then
        ici_error "CLANG_TIDY_JOBS=$CLANG_TIDY_JOBS is invalid."
    fi

    rm -rf "$db".{command,warn,error}
    cat > "$db.command" << EOF
#!/bin/bash
fixes=\$(mktemp)
clang-tidy "-export-fixes=\$fixes" "-header-filter=$regex" "-p=$build" "\$@" || { touch "$db.error"; echo "Errored for '\$*'"; }
if [ -s "\$fixes" ]; then touch "$db.warn"; fi
rm -rf "\$fixes"
EOF
    chmod +x "$db.command"

    ici_time_start "clang_tidy_check_$name"
    echo "run clang-tidy for ${#files[@]} file(s) in $max_jobs process(es)."

    printf "%s\0" "${files[@]}" | xargs --null -P "$max_jobs" "$db.command" "$@"

    if [ -f "$db.error" ]; then
       _run_clang_tidy_errors+=("$name")
       ici_time_end "${ANSI_RED}"
    elif [ -f "$db.warn" ]; then
        _run_clang_tidy_warnings+=("$name")
        ici_time_end "${ANSI_YELLOW}"
    else
        ici_time_end
    fi
}

function run_clang_tidy_check {
    local target_ws=$1
    local -a errors
    local -a warnings
    local -a clang_tidy_args
    ici_parse_env_array clang_tidy_args CLANG_TIDY_ARGS

    ici_run "install_clang_tidy" ici_install_pkgs_for_command clang-tidy clang-tidy "$(apt-cache depends --recurse --important clang  | grep "^libclang-common-.*")"

    while read -r db; do
        run_clang_tidy "$target_ws/src" warnings errors "$db" "${clang_tidy_args[@]}"
    done < <(find "$target_ws/build" -name compile_commands.json)

    if [ "${#warnings[@]}" -gt "0" ]; then
        ici_warn "Clang tidy warning(s) in: ${warnings[*]}"
        if [ "$CLANG_TIDY" == "pedantic" ]; then
            errors=( "${warnings[@]}" "${errors[@]}" )
        fi
    fi

    if [ "${#errors[@]}" -gt "0" ]; then
        ici_error "Clang tidy check(s) failed: ${errors[*]}"
    fi
}

function run_pylint_check {
    local target_ws=$1
    local status=0
    local exit_code=0

    local -a pylint_versions
    ici_parse_env_array pylint_versions PYLINT_VERSIONS
    local -a pylint_args
    ici_parse_env_array pylint_args PYLINT_ARGS

    for cmd in "${pylint_versions[@]}"; do
        ici_run "install_$cmd" ici_install_pkgs_for_command "$cmd" "$cmd"
        set +e
        ici_with_ws "$target_ws" ici_run "run_$cmd" ici_exec_in_workspace "$extend" "$target_ws" "$cmd" "${pylint_args[@]}" "$(find "$target_ws/src" -iname "*.py")"; status=$?
        set -e
        ici_color_output "${ANSI_BLUE}" "STATUS: $status"
        if [ "$status" -eq 0 ]; then
            ici_color_output "${ANSI_GREEN}" "$cmd check passed"
        else
            { ici_color_output "${ANSI_YELLOW}" "$cmd check failed with status $status"; exit_code=1; }
        fi
    done
    if [ "$exit_code" -gt "0" ]; then
        ici_error "pylint check(s) failed."
    fi
}

function run_source_tests {
    # shellcheck disable=SC1090
    source "${ICI_SRC_PATH}/builders/$BUILDER.sh" || ici_error "Builder '$BUILDER' not supported"

    ici_require_run_in_docker # this script must be run in docker
    upstream_ws=~/upstream_ws
    target_ws=~/target_ws
    downstream_ws=~/downstream_ws

    if [ "$CCACHE_DIR" ]; then
        ici_run "setup_ccache" ici_asroot apt-get install -qq -y ccache
        export PATH="/usr/lib/ccache:$PATH"
    fi

    ici_run "${BUILDER}_setup" ici_quiet builder_setup

    ici_run "setup_rosdep" ici_setup_rosdep

    extend="/opt/ros/$ROS_DISTRO"

    if [ -n "$UPSTREAM_WORKSPACE" ]; then
        ici_with_ws "$upstream_ws" ici_build_workspace "upstream" "$extend" "$upstream_ws"
        extend="$upstream_ws/install"
    fi

    if [ "${CLANG_TIDY:-false}" != false ]; then
        TARGET_CMAKE_ARGS="$TARGET_CMAKE_ARGS -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    fi
    ici_with_ws "$target_ws" ici_build_workspace "target" "$extend" "$target_ws"

    if [ "$NOT_TEST_BUILD" != "true" ]; then
        ici_with_ws "$target_ws" ici_test_workspace "target" "$extend" "$target_ws"
    fi

    if [ "$CATKIN_LINT" == "true" ] || [ "$CATKIN_LINT" == "pedantic" ]; then
        ici_run "install_catkin_lint" install_catkin_lint
        local -a catkin_lint_args
        ici_parse_env_array catkin_lint_args CATKIN_LINT_ARGS
        if [ "$CATKIN_LINT" == "pedantic" ]; then
          catkin_lint_args+=(--strict -W2)
        fi
        ici_with_ws "$target_ws" ici_run "catkin_lint" ici_exec_in_workspace "$extend" "$target_ws"  catkin_lint --explain "${catkin_lint_args[@]}" src
    fi
    if [ "${CLANG_TIDY:-false}" != false ]; then
        run_clang_tidy_check "$target_ws"
    fi
    if [ "${PYLINT:-false}" != false ]; then
        run_pylint_check "$target_ws"
    fi

    extend="$target_ws/install"
    if [ -n "$DOWNSTREAM_WORKSPACE" ]; then
        ici_with_ws "$downstream_ws" ici_build_workspace "downstream" "$extend" "$downstream_ws"
        #extend="$downstream_ws/install"

        if [ "$NOT_TEST_DOWNSTREAM" != "true" ]; then
            ici_with_ws "$downstream_ws" ici_test_workspace "downstream" "$extend" "$downstream_ws"
        fi
    fi
}
