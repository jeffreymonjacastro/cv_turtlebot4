unzip ~/Downloads/turtlebot_nav_real_log_iteration_package.zip -d /tmp/nav_real_log_iter

mkdir -p docs
cp /tmp/nav_real_log_iter/turtlebot_nav_real_log_iteration/docs/*.md docs/

cat /tmp/nav_real_log_iter/turtlebot_nav_real_log_iteration/AGENTS_REAL_LOG_ITERATION_APPEND.md >> AGENTS.md
