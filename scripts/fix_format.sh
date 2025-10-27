#!/bin/bash
echo "🔧 Running auto-format and basic lint checks..."
find src include -name "*.cpp" -o -name "*.hpp" | xargs clang-format -style=file -i
cppcheck --enable=warning,style -
