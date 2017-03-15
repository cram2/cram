
ChangeLog
=========

### [0.2.2] - 2016-11-15

2016-03-08 [desig] macros for initializing designators with A and AN: an improved version of old similar macros, but these ones work with nested key-value pairs, nested A / AN calls, and don't use WITH-DESIGNATORS but directly MAKE-DESIGNATOR
2016-03-16 [desig] fixed for the initialization macro: add support for numerical values
2016-07-19 [desig] in desig init macros added support for variables, lexical variables and non-symbol values in variables
2016-07-21 [cram_process_modules] added a small utility function: matching-available-process-modules finds the module to execute an action designator and issues an failure when none found if fail-if-none is set
2016-08-03 [desig] in initialization macros fixed a bug: nested designators were getting one bracket too much.
2016-10-21 Additional logging of task invocation parameters
2016-11-15 bumped ROS package version and cleaned up package.xmls

### [0.2.1] - 2016-01-28
