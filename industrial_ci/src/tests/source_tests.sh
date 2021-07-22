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
    ici_install_pypi_pkgs_for_command catkin_lint "catkin-lint"
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

    rm -rf "$db".{command,warn,error}
    cat > "$db.command" << EOF
#!/bin/bash
num_non_file_args=\$1; shift
args=("\${@:1:\$num_non_file_args}")
files=("\${@:\$((num_non_file_args+1))}")
fixes=\$(mktemp)
rm -f /tmp/clang_tidy_output.\$\$
for f in "\${files[@]}" ; do
  ( cd \$(dirname \$f); clang-tidy "-export-fixes=\$fixes" "-header-filter=$regex" "-p=$build" "\${args[@]}" \$f &>> /tmp/clang_tidy_output.\$\$ 2>&1 || { touch "$db.error"; } )
  if [ -s "\$fixes" ]; then touch "$db.warn"; fi
done
rm -rf "\$fixes"
EOF
    chmod +x "$db.command"

    echo "run clang-tidy for ${#files[@]}/$num_all_files file(s) in $max_jobs process(es)."

    printf "%s\0" "${files[@]}" | xargs --null -P "$max_jobs" -n "$(( (${#files[@]} + max_jobs-1) / max_jobs))" "$db.command" "$#" "$@"
    cat /tmp/clang_tidy_output.* | grep -vP "^([0-9]+ warnings generated|Use .* to display errors from system headers as well)\.$" || true
    rm -rf /tmp/clang_tidy_output.*

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
    local errors=()
    local warnings=()
    local clang_tidy_args=()
    ici_parse_env_array clang_tidy_args CLANG_TIDY_ARGS

    ici_run "install_clang_tidy" ici_install_pkgs_for_command clang-tidy clang-tidy "$(apt-cache depends --recurse --important clang  | grep "^libclang-common-.*")"
    if [ -n "$CLANG_TIDY_BASE_REF" ]; then
        ici_setup_git_client
    fi

    ici_hook "before_clang_tidy_checks"

    while read -r db; do
        run_clang_tidy "$target_ws/src" warnings errors "$db" "${clang_tidy_args[@]}"
    done < <(find "$target_ws/build" -mindepth 2 -name compile_commands.json)  # -mindepth 2, because colcon puts a compile_commands.json into the build folder

    ici_hook "after_clang_tidy_checks"

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
        ici_run "install_pylint" ici_quiet ici_install_pkgs_for_command "pylint" "pylint"
        ici_run "run_pylint" ici_exec_in_workspace "$(ici_extend_space "$target_ws")" "$target_ws" "pylint" "${args[@]}" "${files[@]}"
    else
        ici_warn "No python files found, skipping pylint"
    fi
}

function prepare_source_tests {
    ici_check_builder
}

function run_source_tests {
    upstream_ws=$BASEDIR/upstream_ws
    target_ws=$BASEDIR/target_ws
    downstream_ws=$BASEDIR/downstream_ws

    if [ -n "$CCACHE_DIR" ]; then
        ici_run "setup_ccache" ici_apt_install ccache
        export PATH="/usr/lib/ccache:$PATH"
    fi

    ici_source_builder
    ici_run "${BUILDER}_setup" ici_quiet builder_setup

    ici_run "setup_rosdep" ici_setup_rosdep

    extend=${UNDERLAY:?}

    if [ -n "$UPSTREAM_WORKSPACE" ]; then
        ici_with_ws "$upstream_ws" ici_build_workspace "upstream" "$extend" "$upstream_ws"
        extend="$(ici_extend_space "$upstream_ws")"
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
        local catkin_lint_args=()
        ici_parse_env_array catkin_lint_args CATKIN_LINT_ARGS
        if [ "$CATKIN_LINT" == "pedantic" ]; then
          catkin_lint_args+=(--strict -W2)
        fi
        ici_with_ws "$target_ws" ici_run "catkin_lint" ici_exec_in_workspace "$extend" "$target_ws"  catkin_lint --explain "${catkin_lint_args[@]}" src

    fi
    if [ "${CLANG_TIDY:-false}" != false ]; then
        run_clang_tidy_check "$target_ws"
    fi
    if [ "$PYLINT_CHECK" == "true" ]; then
        run_pylint_check "$target_ws"
    fi

    extend="$(ici_extend_space "$target_ws")"
    if [ -n "$DOWNSTREAM_WORKSPACE" ]; then
        ici_with_ws "$downstream_ws" ici_build_workspace "downstream" "$extend" "$downstream_ws"
        #extend="$(ici_extend_space "$downstream_ws")"

        if [ "$NOT_TEST_DOWNSTREAM" != "true" ]; then
            ici_with_ws "$downstream_ws" ici_test_workspace "downstream" "$extend" "$downstream_ws"
        fi
    fi
}
