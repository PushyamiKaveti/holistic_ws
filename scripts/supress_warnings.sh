#!/bin/bash

rosservice call /anymal_estimator_node/set_logger_level "{logger: 'ros.roscpp.superdebug', level: 'error'}"
rosservice call /anymal_estimator_node/set_logger_level "{logger: 'ros.roscpp', level: 'error'}"
rosservice call /anymal_estimator_node/set_logger_level "{logger: 'ros', level: 'error'}"
rosservice call /anymal_estimator_node/get_loggers
