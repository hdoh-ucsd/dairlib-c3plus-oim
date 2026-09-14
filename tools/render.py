"""Render a saved C3+ trajectory to MP4."""


def main(argv=None):
    from c3plus.visualization.render import main as command

    return command(argv)


if __name__ == "__main__":
    main()
