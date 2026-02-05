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
    local catkin_lint_pkg="catkin-lint"
    if [ "$ROS_PYTHON_VERSION" == "2" ]; then
        catkin_lint_pkg="catkin-lint<1.6.23"
    fi
    ici_install_pypi_pkgs_for_command catkin_lint "$catkin_lint_pkg"
}

function run_clang_tidy {
    local regex="$1/.*"
    local -n _run_clang_tidy_warnings=$2
    local -n _run_clang_tidy_errors=$3
    local db=$4
    shift 4

    local build; build="$(dirname "$db")"
    local name; name="$(basename "$build")"
    ici_time_start "clang_tidy_check_$name"

    # create an array of all files listed in $db filtered by the source tree
    mapfile -t files < <(grep -oP "(?<=\"file\": \")($regex)(?=\")" "$db")
    local num_all_files="${#files[@]}"
    if [ "$num_all_files" -gt 0 ] && [ -n "$CLANG_TIDY_BASE_REF" ] ; then
        ici_log "Filtering for files that actually changed since $CLANG_TIDY_BASE_REF"
        # Need to run git in actual source dir:  $files[@] refer to source dir and $PWD is read-only
        local src_dir
        src_dir=$(grep -oP "(?<=CMAKE_HOME_DIRECTORY:INTERNAL=).*" "$build/CMakeCache.txt")
        pushd "$src_dir" > /dev/null || true
        if git fetch -q origin "$CLANG_TIDY_BASE_REF" 2> /dev/null; then  # git might fail, e.g. operating on catkin_tools_prebuild
            # Filter for changed files, using sed to augment full path again (which git strips away)
            mapfile -t files < <(git diff --name-only --diff-filter=MA FETCH_HEAD..HEAD -- "${files[@]}" | sed "s#^#$(git rev-parse --show-toplevel)/#")
        fi
        popd > /dev/null || true
    fi
    if [ "${#files[@]}" -eq 0 ]; then
        ici_log "${#files[@]}/$num_all_files source files need checking"
        ici_time_end
        return 0
    fi

    local max_jobs="${CLANG_TIDY_JOBS:-$(nproc)}"
    if ! [ "$max_jobs" -ge 1 ]; then
        ici_error "CLANG_TIDY_JOBS=$CLANG_TIDY_JOBS is invalid."
    fi

    local err=0
    ici_log "run clang-tidy for ${#files[@]}/$num_all_files file(s) in $max_jobs process(es)."
    set -o pipefail
    printf "%s\0" "${files[@]}" | xargs --null run-clang-tidy "-j$max_jobs" "-header-filter=\"$regex\"" "-p=$build" "$@" 2>&1 | tee "$db.tidy.log" || err=$?
    set +o pipefail

    if [ "$err" -ne "0" ]; then
       _run_clang_tidy_errors+=("$name")
       ici_time_end "${ANSI_RED}" "$err"
    elif grep -q "warning: " "$db.tidy.log"; then
        _run_clang_tidy_warnings+=("$name")
        ici_time_end "${ANSI_YELLOW}"
    else
        ici_time_end
    fi
}

function run_clang_tidy_check {
    local target_ws=$1
    local errors=()
    local warnings=()
    local clang_tidy_args=()
    ici_parse_env_array clang_tidy_args CLANG_TIDY_ARGS

    ici_step "install_clang_tidy" ici_install_pkgs_for_command clang-tidy clang-tidy "$(apt-cache depends --recurse --important clang  | grep "^libclang-common-.*")"
    if [ -n "$CLANG_TIDY_BASE_REF" ]; then
        ici_setup_git_client
    fi

    ici_hook "before_clang_tidy_checks"

    # replace -export-fixes <filename> with temporary file
    local fixes_final=""
    local fixes_tmp
    local num_args=${#clang_tidy_args[@]}
    fixes_tmp=$(mktemp)
    for (( i=0; i<num_args; i++ )); do
        if [ "${clang_tidy_args[i]}" == "-export-fixes" ]; then
            fixes_final="${clang_tidy_args[i+1]}"
            clang_tidy_args[i+1]="$fixes_tmp"
        fi
    done

    # run clang-tidy checks on all build folders in target_ws
    while read -r db; do
        run_clang_tidy "$target_ws/src" warnings errors "$db" "${clang_tidy_args[@]}"
        if [ -n "${fixes_final}" ]; then
            "${ICI_SRC_PATH}/tests/merge_fixes.py" "$fixes_final" "$fixes_tmp"
        fi
    done < <(find "$target_ws/build" -mindepth 2 -name compile_commands.json)  # -mindepth 2, because colcon puts a compile_commands.json into the build folder

    if [ -n "${fixes_final}" ]; then
        # translate file names in fixes file
        sed -i "s#$target_ws/src/$TARGET_REPO_NAME/#$TARGET_REPO_PATH/#g" "${fixes_final}"
    fi
    ici_hook "after_clang_tidy_checks"

    if [ "${#warnings[@]}" -gt "0" ]; then
        ici_warn "Clang tidy warning(s) in: ${warnings[*]}"
        if [ "$CLANG_TIDY" == "pedantic" ]; then
            errors=( "${warnings[@]}" "${errors[@]}" )
        fi
    fi

    if [ "${#errors[@]}" -gt "0" ]; then
        ici_report_result clang_tidy_checks 1
        ici_error "Clang tidy check(s) failed: ${errors[*]}"
    fi
}

function run_pylint_check {
    local target_ws=$1

    local args=()
    ici_parse_env_array args PYLINT_ARGS

    local find_pattern=(-type f -iname '*.py')
    local excludes=()
    ici_parse_env_array excludes PYLINT_EXCLUDE
    for p in "${excludes[@]}"; do
        find_pattern+=(-not -path "*$p*");
    done

    local files=()
    mapfile -t files < <(ici_find_nonhidden "$target_ws/src" "${find_pattern[@]}")

    if [ "${#files[@]}" -ne 0 ]; then
        ici_step "install_pylint" ici_install_pkgs_for_command "pylint" "pylint"
        ici_step "run_pylint" ici_cmd ici_exec_in_workspace "$(ici_extend_space "$target_ws")" "$target_ws" "pylint" "${args[@]}" "${files[@]}"
    else
        ici_warn "No python files found, skipping pylint"
    fi
}

function prepare_source_tests {
    ici_check_builder
}

function run_source_tests {
    upstream_ws=$BASEDIR/${PREFIX}upstream_ws
    target_ws=$BASEDIR/${PREFIX}target_ws
    downstream_ws=$BASEDIR/${PREFIX}downstream_ws

    if [ -n "$CCACHE_DIR" ]; then
        ici_step "setup_ccache" ici_apt_install ccache
        export PATH="/usr/lib/ccache:$PATH"
    fi

    ici_source_builder
    ici_step "${BUILDER}_setup" builder_setup

    ici_step "setup_rosdep" ici_setup_rosdep

    extend=${UNDERLAY:?}
    export ROSDEP_SOURCE_FOLDERS=("${UNDERLAY:?}")  # source folders to be ignored for rosdep install

    if [ -n "$UPSTREAM_WORKSPACE" ]; then
        ici_with_ws "$upstream_ws" ici_build_workspace "upstream" "$extend" "$upstream_ws"
        extend="$(ici_extend_space "$upstream_ws")"
        ROSDEP_SOURCE_FOLDERS+=("$extend")
    fi

    if [ "${CLANG_TIDY:-false}" != false ]; then
        TARGET_CMAKE_ARGS="$TARGET_CMAKE_ARGS -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    fi
    ici_with_ws "$target_ws" ici_build_workspace "target" "$extend" "$target_ws"

    if [ "$NOT_TEST_BUILD" != "true" ]; then
        ici_with_ws "$target_ws" ici_test_workspace "target" "$extend" "$target_ws"
    fi

    if [ "$CATKIN_LINT" == "true" ] || [ "$CATKIN_LINT" == "pedantic" ]; then
        ici_step "install_catkin_lint" install_catkin_lint
        local catkin_lint_args=()
        ici_parse_env_array catkin_lint_args CATKIN_LINT_ARGS
        if [ "$CATKIN_LINT" == "pedantic" ]; then
          catkin_lint_args+=(--strict -W2)
        fi
        ici_with_ws "$target_ws" ici_step "catkin_lint" ici_cmd ici_exec_in_workspace "$extend" "$target_ws" catkin_lint --explain "${catkin_lint_args[@]}" src

    fi
    if [ "${CLANG_TIDY:-false}" != false ]; then
        run_clang_tidy_check "$target_ws"
    fi
    if [ "$PYLINT_CHECK" == "true" ]; then
        run_pylint_check "$target_ws"
    fi

    extend="$(ici_extend_space "$target_ws")"
    ROSDEP_SOURCE_FOLDERS+=("$extend")
    if [ -n "$DOWNSTREAM_WORKSPACE" ]; then
        ici_with_ws "$downstream_ws" ici_build_workspace "downstream" "$extend" "$downstream_ws"
        #extend="$(ici_extend_space "$downstream_ws")"

        if [ "$NOT_TEST_DOWNSTREAM" != "true" ]; then
            ici_with_ws "$downstream_ws" ici_test_workspace "downstream" "$extend" "$downstream_ws"
        fi
    fi
}
