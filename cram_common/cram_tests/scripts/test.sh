#!/usr/bin/env bash

# pre-compile cram packages
/usr/bin/sbcl --non-interactive --eval "(progn (load (parse-namestring (concatenate 'string (sb-ext:posix-getenv \"ROS_ROOT\") \"lisp/scripts/roslisp-sbcl-init\"))) (asdf:load-system :cram-pr2-pick-place-demo))" --quit
# launch bullet environment
roslaunch cram_pr2_pick_place_demo sandbox.launch &
# kill the launchfile when this script is done
trap 'kill $(jobs -p)' EXIT
# give the launchfile time to boot
sleep 2

# contains all systems/packages that should be tested
systems_under_test=(":cram-process-modules-test"\
                        ":cram-math-test"\
                        ":cl-bullet-vis-tests"\
                        ":cram-btr-spatial-relations-costmap-tests"\
                        ":cram-urdf-environment-manipulation-tests"\
                        ":cram-beginner-tutorial-tests"\
                        ":cram-manipulation-interfaces-tests"\
                        ":cram-common-failures-tests"\
                        ":cram-task-tree-export-tests"\
                        ":cram-utilities-tests"\
                        ":cram-prolog-tests"\
                        ":cram-projection-tests"\
                        ":cram-fetch-deliver-plans-tests"\
                        ":cram-bullet-reasoning-tests"\
                        ":cram-pr2-pick-place-demo-tests"\
                        ":cram-designators-test"\
                        ":cram-bullet-reasoning-belief-state-tests")

report_path=$(echo "$(rospack find cram_tests)/reports")
mkdir -p $report_path
# testreport will be put into this file
report_file=$(echo "$report_path/$(date "+%F-%T")")
# temporary package-specific lisp file
tmp_test_script=$(echo "$(rospack find cram_tests)/scripts/tmp-test-script.lisp")

# writes the temporary lisp file
create_test_script () {
    cat > $tmp_test_script <<EOF
(load (parse-namestring (concatenate 'string (sb-ext:posix-getenv "ROS_ROOT") "lisp/scripts/roslisp-sbcl-init")))
(asdf:load-system "cram-tests")
(asdf:load-system $1)
(cram-tests:run-package-tests $1 "$2")
EOF
}

# create lisp script and run tests for every system under test
for system in "${systems_under_test[@]}"; do
    echo "-------- System $system under test --------"
    create_test_script $system $report_file
    let starttime=$(date +%s)
    # if the tests get stuck, e.g. on a memory fault, terminate them after 3 minutes to continue
    timeout 3m /usr/bin/sbcl --dynamic-space-size 8192 --noinform --load $tmp_test_script --quit
    if [ $? -eq 124 ]
    then
        echo "Tests for system $system timed out after 3 minutes. Terminating."
    else
        echo "Finished in $(expr $(date +%s) - $starttime) seconds."
    fi
done

# print the test report
cat $report_file

