#!/bin/bash

echo "===== CS293 Project Test Suite ====="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

# Build
echo "Building..."
make clean > /dev/null 2>&1
if make all > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Build successful"
else
    echo -e "${RED}✗${NC} Build failed"
    exit 1
fi

# Generate tests
echo "Generating test cases..."
if python3 generate_tests.py > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Test generation successful"
else
    echo -e "${RED}✗${NC} Test generation failed"
    exit 1
fi

# Test Phase 1
echo ""
echo "Testing Phase 1..."
if ./phase1 test_graph.json test_queries_phase1.json output1.json 2>&1 | grep -q "Done"; then
    echo -e "${GREEN}✓${NC} Phase 1 completed"
    queries=$(cat output1.json | python3 -c "import json, sys; print(len(json.load(sys.stdin)['results']))" 2>/dev/null)
    echo "  Processed $queries queries"
else
    echo -e "${RED}✗${NC} Phase 1 failed"
fi

# Test Phase 2
echo ""
echo "Testing Phase 2..."
if ./phase2 test_graph.json test_queries_phase2.json output2.json 2>&1 | grep -q "Done"; then
    echo -e "${GREEN}✓${NC} Phase 2 completed"
    queries=$(cat output2.json | python3 -c "import json, sys; print(len(json.load(sys.stdin)['results']))" 2>/dev/null)
    echo "  Processed $queries queries"
else
    echo -e "${RED}✗${NC} Phase 2 failed"
fi

echo ""
echo "===== All Tests Complete ====="
