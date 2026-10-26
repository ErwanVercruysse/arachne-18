#!/bin/sh
# prints running motion scripts (python only), exit 1 if any
ps -eo pid,comm,args | awk "\$2 ~ /^python/ && \$0 ~ /tools\/(spin|walk|dance|crab_walk|policy_walk|grab)\.py/" | grep . && exit 1 || exit 0
