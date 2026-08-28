"""Allow ``python -m pipe_cad`` as a short export entry point."""

from .cli import main


if __name__ == "__main__":
    raise SystemExit(main())
