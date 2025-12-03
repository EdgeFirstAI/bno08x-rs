# Rust Project Makefile
#
# This Makefile implements SPS v2.1 requirements for Rust projects
# Reference: ~/Documents/SPS/08-repository-setup.md
#
# CUSTOMIZE THESE VARIABLES FOR YOUR PROJECT:
# ===========================================================================

# Project name (for version verification)
PROJECT_NAME ?= bno08x

# Rust features to test (default: all features)
RUST_FEATURES ?= --all-features

# Additional test flags
TEST_FLAGS ?=

# Virtual environment for SBOM tools
VENV_ACTIVATE := $(shell if [ -d "venv" ]; then echo "source venv/bin/activate &&"; fi)

# ===========================================================================
# STANDARD TARGETS
# ===========================================================================

.PHONY: help
help:
	@echo "Available targets:"
	@echo "  make format         - Format Rust code with rustfmt"
	@echo "  make lint           - Run clippy with strict settings"
	@echo "  make build          - Build the project"
	@echo "  make test           - Run tests with coverage (nextest + llvm-cov)"
	@echo "  make sbom           - Generate SBOM and check license policy"
	@echo "  make verify-version - Verify version consistency"
	@echo "  make pre-release    - Complete pre-release validation"
	@echo "  make clean          - Remove build artifacts"

# Format source code
.PHONY: format
format:
	@echo "Formatting Rust code..."
	cargo +nightly fmt --all || cargo fmt --all
	@echo "✓ Formatting complete"

# Run linters
.PHONY: lint
lint:
	@echo "Running clippy (strict mode)..."
	cargo clippy --all-targets $(RUST_FEATURES) -- -D warnings
	@echo "✓ Linting complete"

# Build
.PHONY: build
build:
	@echo "Building..."
	cargo build $(RUST_FEATURES)
	@echo "✓ Build complete"

# Run tests with coverage
.PHONY: test
test: build
	@echo "Running tests with cargo-nextest and llvm-cov..."
	@if ! cargo nextest --version >/dev/null 2>&1; then \
		echo "ERROR: cargo nextest not installed"; \
		echo "Install with: cargo install cargo-nextest"; \
		exit 1; \
	fi
	@if ! cargo llvm-cov --version >/dev/null 2>&1; then \
		echo "ERROR: cargo llvm-cov not installed"; \
		echo "Install with: cargo install cargo-llvm-cov"; \
		exit 1; \
	fi

	# Run Rust tests with coverage (unit tests only, not hardware tests)
	cargo llvm-cov nextest $(RUST_FEATURES) --workspace \
		--lcov --output-path target/rust-coverage.lcov \
		$(TEST_FLAGS)

	@echo "✓ Tests passed with coverage"
	@echo "Coverage report: target/rust-coverage.lcov"

# Generate SBOM and check licenses
.PHONY: sbom
sbom:
	@echo "Generating SBOM..."
	@if [ ! -f "venv/bin/scancode" ]; then \
		echo "ERROR: scancode not found. Please install:"; \
		echo "  python3 -m venv venv"; \
		echo "  venv/bin/pip install scancode-toolkit"; \
		exit 1; \
	fi
	@if ! cargo cyclonedx --version >/dev/null 2>&1; then \
		echo "ERROR: cargo-cyclonedx not installed"; \
		echo "Install with: cargo install cargo-cyclonedx"; \
		exit 1; \
	fi
	@if [ ! -f ".github/scripts/generate_sbom.sh" ]; then \
		echo "ERROR: .github/scripts/generate_sbom.sh not found"; \
		exit 1; \
	fi
	@.github/scripts/generate_sbom.sh || true
	@if [ ! -f "sbom.json" ]; then \
		echo "ERROR: SBOM generation failed"; \
		exit 1; \
	fi
	@echo "✓ SBOM generated (sbom.json)"

# Verify version consistency
.PHONY: verify-version
verify-version:
	@echo "Verifying version consistency..."
	@if [ ! -f "Cargo.toml" ]; then \
		echo "ERROR: Cargo.toml not found"; \
		exit 1; \
	fi
	@CARGO_VERSION=$$(grep -m1 '^version = ' Cargo.toml | sed 's/version = "\(.*\)"/\1/'); \
	echo "Cargo.toml version: $$CARGO_VERSION"; \
	if [ -f "CHANGELOG.md" ]; then \
		if ! grep -q "\[$$CARGO_VERSION\]" CHANGELOG.md; then \
			echo "ERROR: Version $$CARGO_VERSION not found in CHANGELOG.md"; \
			exit 1; \
		fi; \
		echo "CHANGELOG.md: ✓"; \
	fi
	@echo "✓ Version verification complete"

# Pre-release checks
.PHONY: pre-release
pre-release: format lint verify-version test sbom
	@echo "=================================================="
	@echo "✓ All pre-release checks passed"
	@echo "=================================================="
	@echo ""
	@echo "Next steps:"
	@echo "  1. Review changes: git status && git diff"
	@echo "  2. Commit: git add -A && git commit -m 'Prepare release'"
	@echo "  3. Push: git push origin main"
	@echo "  4. Wait for CI/CD to pass"
	@CARGO_VERSION=$$(grep -m1 '^version = ' Cargo.toml | sed 's/version = "\(.*\)"/\1/'); \
	echo "  5. Tag: git tag -a -m 'Version $$CARGO_VERSION' v$$CARGO_VERSION"; \
	echo "  6. Push tag: git push origin v$$CARGO_VERSION"

# Clean build artifacts
.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	cargo clean
	rm -rf target/rust-coverage.lcov test-results.xml
	rm -f sbom.json *-sbom.json *.cdx.json
	@echo "✓ Clean complete"
