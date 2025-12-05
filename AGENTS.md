# AGENTS.md - AI Assistant Development Guidelines

**Purpose:** Project-specific instructions for AI coding assistants (GitHub Copilot, Claude, Cursor, etc.)

**Organization Standards:** See [SPS 05-copilot-instructions.md](https://github.com/EdgeFirstAI/SPS) for Au-Zone universal rules

**Version:** 1.0
**Last Updated:** 2025-12-04

---

## Overview

This file provides **project-specific** guidelines for the bno08x-rs IMU driver library. ALL contributors (human and AI) must also follow:

- **Organization-wide:** SPS 05-copilot-instructions.md - License policy, security, Git/JIRA
- **Process docs:** SPS 00-README through 11-cicd-pipelines
- **This file:** Project conventions, module structure, testing patterns

**Hierarchy:** Org standards (mandatory) → SPS processes (required) → This file (project-specific)

---

## Project Overview

**bno08x-rs** is a Rust userspace driver for the BNO08x family of 9-axis IMU sensors from Bosch/Hillcrest Labs. It provides:

- SPI communication via Linux spidev
- GPIO control via gpiod (interrupt and reset pins)
- SHTP (Sensor Hub Transport Protocol) implementation
- High-level API for sensor data access (accelerometer, gyroscope, magnetometer, rotation vectors)

### Technology Stack

- **Language:** Rust (edition 2021, MSRV 1.90)
- **Build:** Cargo
- **Key deps:** gpiod (0.3.0), spidev (0.7.1), log (0.4.25)
- **Targets:** Linux ARM64 (NXP i.MX8M Plus, Raivin platform)

### Module Structure

```
src/
├── lib.rs          # Entry point, re-exports, Error types
├── driver.rs       # BNO08x struct, main API, SHTP protocol
├── constants.rs    # Protocol IDs, Q-points, channel definitions
├── reports.rs      # SensorData parsing and storage
├── frs.rs          # Flash Record System operations
└── interface/
    ├── mod.rs      # SensorInterface trait
    ├── spi.rs      # SpiInterface implementation
    ├── spidev.rs   # Linux spidev wrapper
    ├── gpio.rs     # gpiod GPIO wrapper
    └── delay.rs    # Timing utilities
```

---

## Git Workflow

**Branch:** `<type>/EDGEAI-###[-desc]` (feature/bugfix/hotfix/release, JIRA key required)
**Commit:** `EDGEAI-###: Brief description` (50-72 chars, what done not how)
**PR:** main=2 approvals, develop=1. Link JIRA, squash features.

---

## ⚠️ CRITICAL RULES

### #1: NEVER Use cd Commands

```bash
# ✅ Use direct paths or -C flag
cargo build --release
make -C /path/to/project test

# ❌ AI loses directory context
cd build && make  # Where are we now?
```

### #2: No System Python - Use venv

**NEVER install pip packages outside of the local venv!** This includes `--user` installs.

```bash
# ✅ Direct venv invocation - ALWAYS use local venv
venv/bin/python script.py
venv/bin/pip install package-name
venv/bin/pytest tests/

# ❌ FORBIDDEN - System Python pollution
python script.py
pip install package-name
pip install package-name --user
pip install package-name --break-system-packages  # NEVER!
```

**Why?** System/user pip installs:
- Pollute global environment
- Cause version conflicts
- Break reproducibility
- May require `--break-system-packages` which damages the system

### #3: env.sh for Credentials (Optional)

```bash
# env.sh - LOCAL ONLY (.gitignore, NEVER commit!)
export API_TOKEN="expires-in-24h"  # Ephemeral only!
```

**Usage:** `[[ -f env.sh ]] && source env.sh`

### #4: ALWAYS Use Makefile for Format and Lint

```bash
# ✅ MUST use Makefile targets (handles nightly fmt properly)
make format
make lint

# ❌ NEVER run cargo fmt/clippy directly - inconsistent behavior
cargo fmt --all           # Missing nightly fallback
cargo clippy ...          # May miss flags
```

### #5: ALWAYS Use cargo-zigbuild for Cross-Compilation

Cross-compiled binaries MUST be built with `cargo zigbuild` for manylinux2014 glibc compatibility with target devices.

```bash
# ✅ MUST use zigbuild for ARM64 cross-compilation
cargo zigbuild --target aarch64-unknown-linux-gnu --release
cargo zigbuild --target aarch64-unknown-linux-gnu --release --tests

# ❌ NEVER use regular cargo for cross-compilation - glibc version mismatch
cargo build --target aarch64-unknown-linux-gnu --release  # GLIBC_2.39 errors on target!
```

**Why?** Host glibc is newer than target device. zigbuild uses zig's bundled libc for consistent ABI.

---

## Build Commands

```bash
# Build (native)
cargo build --release

# Build for ARM64 cross-compilation (MUST use zigbuild!)
cargo zigbuild --target aarch64-unknown-linux-gnu --release

# Build tests for ARM64 (for on-target testing)
cargo zigbuild --target aarch64-unknown-linux-gnu --release --tests

# Test (unit tests only - no hardware required)
cargo test

# Test with coverage
cargo llvm-cov nextest --all-features --workspace

# Format/Lint (MUST use Makefile!)
make format
make lint

# Documentation
cargo doc --all-features --no-deps

# Pre-release validation
make pre-release
```

---

## Testing Strategy

### Test Categories

1. **Unit Tests** (in-source `#[cfg(test)]` modules)
   - Run on any platform, no hardware required
   - Cover packet parsing, data conversion, error handling
   - Located in same file as code being tested

2. **Hardware Integration Tests** (`tests/hardware_integration.rs`)
   - Require real BNO08x hardware
   - Marked with `#[ignore]` attribute
   - Run with `cargo test -- --ignored --test-threads=1`

### Coverage Requirements

- **Minimum:** 70% overall
- **Critical paths:** 90%+ (driver initialization, data parsing, error handling)
- **Tools:** cargo-llvm-cov + cargo-nextest

### Running Unit Tests

```bash
# Standard unit tests
cargo test

# With coverage report
cargo llvm-cov nextest --all-features --workspace \
    --lcov --output-path target/rust-coverage.lcov
```

---

## On-Target Hardware Testing

### Target Platform

The BNO08x driver targets ARM64 Linux systems, specifically:

- **Raivin platform** (NXP i.MX8M Plus based)
- Torizon OS with minimal BSP
- **NO build toolchain on target** - cross-compile required

### Local Development Workflow

For local on-target testing during development:

#### 1. Cross-Compile on Host (x86_64 → ARM64)

```bash
# Install cross-compilation target
rustup target add aarch64-unknown-linux-gnu

# Build test binaries with coverage instrumentation
cargo llvm-cov nextest --profile profiling --cargo-profile profiling \
    --all-features --no-run

# Or build without coverage for simple testing (MUST use zigbuild!)
cargo zigbuild --target aarch64-unknown-linux-gnu --release
cargo zigbuild --target aarch64-unknown-linux-gnu --release --tests
```

#### 2. Deploy to Target

```bash
# Copy test binary to target (replace <target-host> with actual hostname/IP)
# Test binaries are in target/llvm-cov-target/profiling/deps/ or target/aarch64-unknown-linux-gnu/release/deps/

# Find the hardware integration test binary
TEST_BIN=$(find target/llvm-cov-target/profiling/deps -name "hardware_integration*" -type f -executable | head -1)

# Deploy to target
scp "$TEST_BIN" torizon@<target-host>:/tmp/hardware_integration_test

# Also copy any required shared libraries if needed
# scp target/aarch64-unknown-linux-gnu/release/libbno08x_rs.so torizon@<target-host>:/tmp/
```

#### 3. Run Tests on Target

```bash
# SSH to target and run tests
ssh torizon@<target-host> << 'EOF'
    # Set up coverage output if instrumented
    export LLVM_PROFILE_FILE=/tmp/profraw/%p-%m.profraw
    mkdir -p /tmp/profraw

    # Run hardware tests (--include-ignored runs #[ignore] tests)
    chmod +x /tmp/hardware_integration_test
    /tmp/hardware_integration_test --include-ignored --test-threads=1
EOF
```

#### 4. Collect Coverage Data (if instrumented)

```bash
# Copy profraw files back to host
scp -r torizon@<target-host>:/tmp/profraw/ ./target/hardware-profraw/

# Merge profraw files
TOOLCHAIN_ROOT=$(rustc --print sysroot)
LLVM_PROFDATA=$(find "$TOOLCHAIN_ROOT" -name "llvm-profdata" -type f | head -1)

"$LLVM_PROFDATA" merge -sparse \
    $(find target/hardware-profraw -name "*.profraw" -type f) \
    -o target/hardware.profdata

# Generate coverage report
LLVM_COV=$(find "$TOOLCHAIN_ROOT" -name "llvm-cov" -type f | head -1)

# Find instrumented objects (use same profile as build!)
OBJECT_FILES=""
for obj in $(find target/llvm-cov-target/profiling/deps -maxdepth 1 -type f \
             ! -name "*.d" ! -name "*.rlib" ! -name "*.rmeta"); do
    if file "$obj" | grep -q "ELF"; then
        OBJECT_FILES="$OBJECT_FILES --object=$obj"
    fi
done

"$LLVM_COV" export --format=lcov \
    --instr-profile=target/hardware.profdata \
    --ignore-filename-regex='/.cargo/registry|/rustc/' \
    $OBJECT_FILES > target/hardware-coverage.lcov
```

### CI/CD Pattern (Three-Phase)

For CI/CD, hardware testing follows the **three-phase pattern** (see SPS 11-cicd-pipelines.md):

1. **Phase 1 (Build):** Cross-compile on `ubuntu-22.04-arm-private` with coverage instrumentation
2. **Phase 2 (Test):** Run on `nxp-imx8mp-latest` hardware runner, collect profraw files
3. **Phase 3 (Process):** Process coverage on same runner type as build

**Why?** Hardware runners have no toolchains - they can only execute pre-built binaries.

---

## Code Quality

### Standards

- **Rust:** Latest stable, `cargo fmt`, `cargo clippy -- -D warnings`
- **MSRV:** 1.90 (minimum supported Rust version)
- **Edition:** 2021

### Performance Considerations

- **Edge-First:** Target platforms have 512MB-2GB RAM
- Minimize heap allocations in hot paths
- Use fixed-size buffers where possible
- Profile on actual target hardware

### Error Handling

- Use `Result<T, Error>` for fallible operations
- Provide context in error messages
- Map low-level errors to driver-level errors

---

## License Policy (ZERO TOLERANCE)

**✅ Allowed:** MIT, Apache-2.0, BSD-2/3, ISC, 0BSD, Unlicense, Zlib, BSL-1.0
**⚠️ Conditional:** MPL-2.0/EPL-2.0 (deps ONLY), LGPL (**FORBIDDEN in Rust** - static linking)
**❌ BLOCKED:** GPL, AGPL, SSPL, Commons Clause

**Current License:** Apache-2.0

**SBOM:** `make sbom` generates CycloneDX SBOM. CI/CD blocks license violations.

---

## Security

**Input:** Validate all sensor data, handle malformed packets gracefully
**GPIO:** Verify pin states, handle GPIO errors
**Scans:** `cargo audit` must pass - no known vulnerabilities
**Report:** support@au-zone.com "Security Vulnerability"

---

## Release

**Semver:** MAJOR.MINOR.PATCH (currently 2.0.0)
**CHANGELOG:** Update during dev under [Unreleased], move to [X.Y.Z] at release
**Pre-release:** `make pre-release` (lint+test+coverage+sbom)
**Tag:** After main merge + CI green. `git tag -a vX.Y.Z && git push origin vX.Y.Z`

**Version files:** Cargo.toml only (single source of truth)

---

## Documentation

**Required:**
- README.md - Features, installation, usage examples
- ARCHITECTURE.md - System design, module structure, data flow
- CHANGELOG.md - Version history
- API docs - `cargo doc` generates rustdoc

**Doc Comments:**
- All public APIs must have rustdoc comments
- Include examples in doc comments where helpful
- Document error conditions and edge cases

---

## AI Assistant Practices

**Verify:**
- APIs exist before using them
- Licenses are approved
- Linters pass (`make format && make lint`)
- Tests cover edge cases
- Match existing code patterns

**Avoid:**
- Hallucinated APIs
- GPL/AGPL dependencies
- System Python (NEVER pip install outside venv!)
- cd commands
- Hardcoded secrets
- Over-engineering simple solutions
- Running cargo fmt/clippy directly (use Makefile!)
- Using cargo build for cross-compilation (use zigbuild!)

**Review:** ALL code. YOU are author (AI = tool). Test thoroughly.

---

## Quick Reference

**Branch:** `feature/EDGEAI-123-desc`
**Commit:** `EDGEAI-123: Brief description`
**PR:** 2 approvals (main), 1 (develop)
**Licenses:** ✅ MIT/Apache/BSD | ❌ GPL/AGPL
**Tests:** 70% min, 90%+ critical
**Security:** support@au-zone.com
**Release:** Semver, make pre-release, wait CI, tag vX.Y.Z

**Build:** `cargo build --release`
**Cross-build:** `cargo zigbuild --target aarch64-unknown-linux-gnu --release`
**Test:** `cargo test` (unit) | `cargo test -- --ignored` (hardware)
**Lint:** `make format && make lint` (NEVER cargo fmt/clippy directly!)
**Coverage:** `cargo llvm-cov nextest --all-features`

---

**Process docs:** See SPS repository 00-README through 11-cicd-pipelines
**v1.0** | 2025-12-04 | sebastien@au-zone.com

*This file helps AI assistants contribute effectively while maintaining quality, security, and consistency.*
