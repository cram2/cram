# CRAM_TESTS

Run single tests from emacs with `(cram-tests:run-package-tests :cram-process-modules-test)` for example.

Run all tests for the CRAM project with `rosrun cram_tests test.sh`. All test results will be stored in the `reports/` directory.

To add packages that use lisp-unit for automatic testing, modify the `systems_under_test` list in `scripts/test.sh`.
