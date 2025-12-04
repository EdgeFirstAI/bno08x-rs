# Security Policy

## Reporting a Vulnerability

We take the security of BNO08x driver seriously. If you discover a security vulnerability, please report it responsibly.

**DO NOT** report security vulnerabilities through public GitHub issues.

Instead, please use one of the following methods:

1. **Email**: Send details to support@au-zone.com with subject line: "Security Vulnerability - BNO08x Driver"
2. **GitHub Security Advisory**: Use the [private reporting feature](https://github.com/EdgeFirstAI/bno08x-rs/security/advisories/new)

### Information to Include

Help us understand and resolve the issue quickly by providing:

- **Type of vulnerability** (e.g., buffer overflow, injection, authentication bypass)
- **Full paths** of affected source files
- **Location of the affected code** (tag/branch/commit or direct URL)
- **Step-by-step instructions** to reproduce the issue
- **Proof-of-concept or exploit code** (if possible)
- **Impact assessment** (what an attacker could achieve)
- **Suggested fix** (if you have one)

## What to Expect

After you submit a vulnerability report:

1. **Acknowledgment**: We will acknowledge receipt within **48 hours**
2. **Initial Assessment**: We will provide an initial assessment within **7 days**, including:
   - Confirmation of the vulnerability
   - Severity classification (Critical, High, Medium, Low)
   - Estimated timeline for a fix
3. **Resolution Timeline**:
   - **Critical**: Fix within 7 days
   - **High**: Fix within 30 days
   - **Medium**: Fix within 90 days
   - **Low**: Fix in next scheduled release
4. **Disclosure**: We will work with you on responsible disclosure timing

## Vulnerability Severity

We use the [CVSS v3.1](https://www.first.org/cvss/v3.1/specification-document) scoring system:

- **Critical (9.0-10.0)**: Remote code execution, authentication bypass
- **High (7.0-8.9)**: Significant data disclosure, privilege escalation
- **Medium (4.0-6.9)**: Denial of service, limited information disclosure
- **Low (0.1-3.9)**: Minor information leak, low-impact issues

## Responsible Disclosure

We kindly ask security researchers to:

- Give us reasonable time to address the issue before public disclosure
- Make a good faith effort to avoid privacy violations, data destruction, and service disruption
- Not access, modify, or delete data without explicit permission
- Not exploit the vulnerability beyond what is necessary to demonstrate it

---

**Last Updated:** 2025-11-27

For general questions, see our [Contributing Guide](CONTRIBUTING.md) or visit [EdgeFirst Documentation](https://doc.edgefirst.ai/latest/).
