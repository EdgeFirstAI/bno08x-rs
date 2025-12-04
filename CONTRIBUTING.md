# Contributing to BNO08x Driver

Thank you for your interest in contributing to the BNO08x driver! This document provides guidelines for contributing to this project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [How to Contribute](#how-to-contribute)
- [Code Style Guidelines](#code-style-guidelines)
- [Dependency Management](#dependency-management)
- [Testing Requirements](#testing-requirements)
- [Pull Request Process](#pull-request-process)
- [Developer Certificate of Origin (DCO)](#developer-certificate-of-origin-dco)
- [License](#license)

## Code of Conduct

This project adheres to the [Contributor Covenant Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior to support@au-zone.com.

## Getting Started

The BNO08x driver is a Rust userspace library for communicating with BNO08x 9-axis IMUs via SPI. Before contributing:

1. Read the [README.md](README.md) to understand the project
2. Review the [ARCHITECTURE.md](ARCHITECTURE.md) for design patterns
3. Check existing [issues](https://github.com/EdgeFirstAI/bno08x-rs/issues) and [discussions](https://github.com/EdgeFirstAI/bno08x-rs/discussions)
4. Browse the [EdgeFirst Documentation](https://doc.edgefirst.ai/latest/) for context

### Ways to Contribute

- **Code**: Bug fixes, new features, performance improvements
- **Documentation**: README improvements, code comments, examples
- **Testing**: Unit tests, integration tests, validation on real hardware
- **Community**: Answer questions, write tutorials, share use cases

## Development Setup

### Prerequisites

- **Rust**: 1.90 or later ([install instructions](https://rustup.rs()))
- **Git**: For version control
- **Hardware** (optional): BNO08x sensor with SPI/GPIO for integration testing

### Clone and Build

```bash
# Clone the repository
git clone https://github.com/EdgeFirstAI/bno08x-rs.git
cd bno08x-rs

# Build
cargo build
```

### Rust Development

```bash
cargo fmt         # Format code
cargo clippy      # Run linter
cargo test        # Run tests
cargo doc         # Generate documentation
```

## How to Contribute

### Reporting Bugs

Before creating bug reports, please check existing issues to avoid duplicates.

**Good Bug Reports** include:

- Clear, descriptive title
- Steps to reproduce the behavior
- Expected vs. actual behavior
- Environment details (OS, Rust version, hardware)
- Minimal code example demonstrating the issue
- Sensor model and firmware version if applicable

### Suggesting Enhancements

Enhancement suggestions are tracked as GitHub issues. Provide:

- Clear, descriptive title
- Detailed description of the proposed functionality
- Use cases and motivation
- Examples of how the feature would be used
- Possible implementation approach (optional)

### Contributing Code

1. **Fork the repository** and create your branch from `main`
2. **Make your changes** following our code style guidelines
3. **Add tests** for new functionality (minimum 70% coverage)
4. **Ensure all tests pass** (`cargo test`)
5. **Update documentation** for API changes
6. **Run formatters and linters** (`cargo fmt`, `cargo clippy`)
7. **Submit a pull request** with a clear description

## Code Style Guidelines

### Rust Guidelines

- Follow [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/)
- Use `cargo fmt` (enforced in CI)
- Address all `cargo clippy` warnings
- Write doc comments for public APIs
- Maximum line length: 100 characters
- Use descriptive variable names

## Dependency Management

### Adding New Dependencies

When adding dependencies to `Cargo.toml`, follow this process:

1. **Check License Compatibility**
   - Only use permissive licenses: MIT, Apache-2.0, BSD (2/3-clause), ISC
   - Avoid GPL, AGPL, or restrictive licenses
   - Review the entire dependency tree for license compliance

2. **Update Lock File**

   ```bash
   # After modifying Cargo.toml
   cargo build
   git add Cargo.lock
   ```

3. **Regenerate NOTICE File**

   ```bash
   # Generate complete SBOM and updated NOTICE
   .github/scripts/generate_sbom.sh

   # Review the updated NOTICE file
   git diff NOTICE

   # Commit if changes are present
   git add NOTICE
   ```

4. **Verify License Policy**

   ```bash
   # Check for license violations
   python3 .github/scripts/check_license_policy.py sbom.json
   ```

5. **Include in Commit**
   - Commit `Cargo.lock`
   - Commit updated `NOTICE` file
   - Reference NOTICE update in commit message or PR description

## Testing Requirements

### Minimum Coverage

All contributions with new functionality must include tests:

- **Unit Tests**: Minimum 70% code coverage
- Critical paths require 100% coverage

### Running Tests

```bash
cargo test              # Run all tests
cargo test --lib        # Run library tests only
cargo test --coverage   # Generate coverage report (requires cargo-llvm-cov)
```

### Integration Testing

Integration tests require physical BNO08x hardware connected via SPI. These tests are skipped in CI:

```bash
# On target hardware with sensor connected
cargo test --test '*'
```

## Pull Request Process

### Branch Naming

For external contributors:

```text
feature/<description>       # New features
bugfix/<description>        # Bug fixes
docs/<description>          # Documentation updates
```

**Examples:**

- `feature/add-magnetometer-calibration`
- `bugfix/fix-spi-timeout`
- `docs/update-readme`

### Commit Messages

Write clear, concise commit messages:

```text
Add [feature] for [purpose]

- Implementation detail 1
- Implementation detail 2
- Implementation detail 3

Addresses issue #123
```

**Guidelines:**

- Use imperative mood ("Add feature" not "Added feature")
- First line: 50 characters or less
- Body: Wrap at 72 characters
- Reference issues/discussions when applicable

### Pull Request Checklist

Before submitting, ensure:

- [ ] Code follows style guidelines (`cargo fmt`, `cargo clippy`)
- [ ] All tests pass (`cargo test`)
- [ ] New tests added for new functionality (70% coverage minimum)
- [ ] Documentation updated for API changes
- [ ] Commit messages are clear and descriptive
- [ ] Branch is up-to-date with `main`
- [ ] PR description clearly explains changes and motivation
- [ ] SPDX headers present in new files

### Review Process

1. **Automated Checks**: CI/CD runs tests, linters, formatters
2. **Maintainer Review**: Code review by project maintainers
3. **Community Feedback**: Other contributors may provide input
4. **Approval**: At least one maintainer approval required
5. **Merge**: Maintainer merges upon approval

### After Your Pull Request is Merged

- Update your local repository: `git pull upstream main`
- Delete your feature branch (optional)
- Celebrate! Thank you for contributing!

---

## Developer Certificate of Origin (DCO)

All contributors must sign off their commits to certify they have the right to submit the code under the project's open source license. This is done by adding a `Signed-off-by` line to commit messages.

### How to Sign Off Commits

Sign off your commits using the `--signoff` or `-s` flag:

```bash
git commit -s -m "Add new feature"
```

This automatically adds a line like this to your commit message:

```text
Signed-off-by: Your Name <your.email@example.com>
```

**Configure git with your real name and email:**

```bash
git config user.name "Your Name"
git config user.email "your.email@example.com"
```

### DCO Enforcement

- **All commits** in a pull request **must be signed off**
- Pull requests with unsigned commits will fail automated checks
- You can check your commits with: `git log --show-signature`

## License

By contributing to the BNO08x driver, you agree that your contributions will be licensed under the [Apache License 2.0](LICENSE).

All source files must include the SPDX license header:

```rust
// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0
```

## Questions?

- **Documentation**: https://doc.edgefirst.ai/latest/
- **Discussions**: https://github.com/EdgeFirstAI/bno08x-rs/discussions
- **Issues**: https://github.com/EdgeFirstAI/bno08x-rs/issues
- **Email**: support@au-zone.com

Thank you for helping make the BNO08x driver better!
