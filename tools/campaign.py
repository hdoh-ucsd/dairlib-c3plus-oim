"""Public campaign command."""
def main(argv=None):
    from c3plus.experiments.campaign import main as command
    return command(argv)

if __name__ == "__main__":
    raise SystemExit(main())
