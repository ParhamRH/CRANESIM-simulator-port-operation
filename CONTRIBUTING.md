# Contributing to CraneSim

Thank you for taking the time to contribute! Here's everything you need to know.

---

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How to Report a Bug](#how-to-report-a-bug)
- [How to Request a Feature](#how-to-request-a-feature)
- [Development Setup](#development-setup)
- [Branch & Commit Conventions](#branch--commit-conventions)
- [Pull Request Process](#pull-request-process)
- [Code Style](#code-style)
- [Testing](#testing)

---

## Code of Conduct

Be respectful and constructive. We follow the [Contributor Covenant](https://www.contributor-covenant.org/version/2/1/code_of_conduct/).

---

## How to Report a Bug

1. Check [existing issues](../../issues) first.
2. Open a **Bug Report** issue using the template.
3. Include: Python version, OS, steps to reproduce, expected vs actual behaviour, and any error output.

---

## How to Request a Feature

1. Open a **Feature Request** issue using the template.
2. Describe the use case and why it belongs in the core simulator (vs a personal fork).

---

## Development Setup

```bash
git clone https://github.com/your-username/CRANESIM-Simulator-Port-operation.git
cd CRANESIM-Simulator-Port-operation
python -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
pip install pytest ruff         # dev tools
```

Place your mesh files in `assets/meshes/` before running.

---

## Branch & Commit Conventions

| Branch prefix | Use for |
|---------------|---------|
| `feat/`       | New features |
| `fix/`        | Bug fixes |
| `docs/`       | Documentation only |
| `refactor/`   | Code restructuring without behaviour change |
| `test/`       | Adding or updating tests |

Commit messages follow **Conventional Commits**:

```
feat(physics): add wind disturbance force model
fix(ui): correct cable-length display after detach
docs(readme): add dataset column descriptions
```

---

## Pull Request Process

1. Fork the repo and create your branch from `main`.
2. Write tests for new behaviour where possible (`tests/`).
3. Run `ruff check .` and ensure no lint errors.
4. Run `pytest tests/` — all tests must pass.
5. Update `CHANGELOG.md` under `[Unreleased]`.
6. Open a PR with a clear description of *what* and *why*.
7. At least one maintainer review is required before merging.

