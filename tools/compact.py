"""Consolidate a completed C3+ run into its verified JSON and MP4."""


def main(argv=None):
    from c3plus.evaluation.package import main as command
    return command(argv)


if __name__ == "__main__":
    main()
