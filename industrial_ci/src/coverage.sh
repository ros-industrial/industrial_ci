#!/bin/bash

# Copyright (c) 2020, Chen Bainian
# Copyright (c) 2020, Mathias Lüdtke
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

function ici_combine_cpp_reports {
  ici_install_pkgs_for_command lcov lcov
  # Capture initial coverage info
  lcov --capture --initial \
       --directory build \
       --output-file initial_coverage.info | grep -ve "^Processing"
  # Capture tested coverage info
  lcov --capture \
       --directory build \
       --output-file test_coverage.info | grep -ve "^Processing"
  # Combine two report (exit function  when none of the records are valid)
  lcov --add-tracefile initial_coverage.info \
       --add-tracefile test_coverage.info \
       --output-file coverage.info || return 0 \
    && rm initial_coverage.info test_coverage.info
  # Extract repository files
  lcov --extract coverage.info "$(pwd)/src/$TARGET_REPO_NAME/*" \
       --output-file coverage.info | grep -ve "^Extracting"
  # Filter out test files
  lcov --remove coverage.info "*/test/*" \
       --output-file coverage.info | grep -ve "^removing"
  # Some sed magic to remove identifiable absolute path
  sed -i "s~$(pwd)/src/$TARGET_REPO_NAME/~~g" coverage.info
  lcov --list coverage.info
}

function ici_combine_python_reports {
  # Find all .coverage file
  local python_reports
  python_reports=$(find "./build" \
                        -type f \
                        -name ".coverage" \
                        -printf "%p ")
  IFS=" " read -r -a python_reports <<< "$python_reports"
  # Combine coverage files
  if "$PYTHON_VERSION_NAME" -m coverage combine "${python_reports[@]}"; then
    # Generate report
    "$PYTHON_VERSION_NAME" -m coverage report --include="src/$TARGET_REPO_NAME/*" --omit="*/test/*,*/setup.py" || return 0
    "$PYTHON_VERSION_NAME" -m coverage xml --include="src/$TARGET_REPO_NAME/*" --omit="*/test/*,*/setup.py"
  fi
}

function ici_collect_coverage_report {
  local target_ws=$1

  # Run combine reports command securely
  # within workspace directory
  ( cd "$target_ws" && \
    ici_combine_cpp_reports && \
    ici_combine_python_reports )

  case "$CODE_COVERAGE" in
    "coveralls.io")
      # Combine cpp and python reports locally and expose coverage.json file for coveralls
      cp "$target_ws"/.coverage "$target_ws/src/$TARGET_REPO_NAME" || echo "No python coverage report"
      ici_setup_git_client
      ici_install_pkgs_for_command pip "${PYTHON_VERSION_NAME}-pip" \
        "${PYTHON_VERSION_NAME}-dev" "${PYTHON_VERSION_NAME}-wheel"
      "${PYTHON_VERSION_NAME}" -m pip install coveralls
      # Use coveragerc file used for coveralls ignore
      printf "[report]\ninclude = \n\t%s/src/%s/*\nomit = \n\t*/test/*\n\t*/setup.py" "$target_ws" "$TARGET_REPO_NAME" \
        > "$target_ws/src/$TARGET_REPO_NAME/.default.coveragerc"
      if [ -f "$target_ws"/coverage.info ]; then
        # Install and run coveralls-lcov within git directory
        cp "$target_ws"/coverage.info "$target_ws/src/$TARGET_REPO_NAME"
        ici_install_pkgs_for_command gem ruby
        gem install coveralls-lcov
        ( cd "$target_ws/src/$TARGET_REPO_NAME" && \
          coveralls-lcov -v -n coverage.info > coverage.c.json && \
          coveralls --merge coverage.c.json \
                    --rcfile .default.coveragerc \
                    --output "$COVERAGE_REPORT_PATH"/coverage.json )
      else
        echo "No cpp covearge report"
        ( cd "$target_ws/src/$TARGET_REPO_NAME" && \
          coveralls --rcfile .default.coveragerc \
                    --output "$COVERAGE_REPORT_PATH"/coverage.json )
      fi
      ;;
    *) # Expose LCOV and Cobertura XML files by default
      cp "$target_ws"/coverage.info "$COVERAGE_REPORT_PATH" || echo "No cpp coverage report"
      cp "$target_ws"/coverage.xml "$COVERAGE_REPORT_PATH" || echo "No python coverage report"
      ;;
  esac
}

function upload_coverage_report {
  case "$CODE_COVERAGE" in
    "codecov.io")
      bash <(curl -s https://codecov.io/bash) -Z
      ;;
    "coveralls.io")
      # Check pip/pip3 installation (prioritize using python3)
      local PYTHON_VERSION
      if command -v python3 &> /dev/null; then
        if ! command -v pip3 &> /dev/null; then
          ici_warn "pip3 not found. Try installing pip3..."
          ici_apt_install python3-pip
        fi
        # Check setuptools installation
        if ! pip3 show setuptools &> /dev/null; then
          ici_warn "pip3 setup_tools not found. Try installing pip3 setup_tools..."
          ici_apt_install python3-setuptools python3-dev python3-wheel
        fi
        PYTHON_VERSION=3
      elif command -v python &> /dev/null; then
        if ! command -v pip &> /dev/null; then
          ici_warn "pip not found. Try installing pip3..."
          ici_apt_install python-pip
        fi
        # Check setuptools installation
        if ! pip show setuptools &> /dev/null; then
          ici_warn "pip setup_tools not found. Try installing pip setup_tools..."
          ici_apt_install python-setuptools python-dev python-wheel
        fi
      fi

      python"$PYTHON_VERSION" -m pip install --user --upgrade pip
      python"$PYTHON_VERSION" -m pip install --user coveralls
      python"$PYTHON_VERSION" -m coveralls --merge=.ici_coverage_report/coverage.json
      ;;
  esac
}
