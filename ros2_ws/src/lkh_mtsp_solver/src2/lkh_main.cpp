/*
 * Standalone LKH-3 executable, built from the solver core already vendored in
 * this package.
 *
 * The ACVRP branch of mtsp_node (problem 3 - the contiguity-driven task
 * allocation) runs the solver as a separate process rather than in-process:
 * LKH keeps ~150 mutable globals and calls exit() on malformed input, so a bad
 * problem file would take the whole exploration stack down with it, and
 * consecutive solves would inherit each other's state. Upstream C2-Explorer
 * solved that by shelling out to a system-wide /usr/local/bin/LKH, i.e. an
 * undeclared external dependency; building the same binary from the sources
 * already in this package keeps the workspace self-contained.
 */
#include <lkh_mtsp_solver/lkh3_interface.h>

#include <cstdio>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <parameter_file>\n", argv[0]);
    return 1;
  }
  return solveMTSPWithLKH3(argv[1]);
}
