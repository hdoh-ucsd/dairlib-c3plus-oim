"""Public check command."""
def main(argv=None):
    from c3plus.runtime.environment import main as command
    return command(argv)

if __name__ == "__main__":
    raise SystemExit(main())
