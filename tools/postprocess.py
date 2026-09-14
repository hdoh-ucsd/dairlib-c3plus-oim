"""Postprocess or enrich a saved C3+ run."""


def main(argv=None):
    from c3plus.evaluation.postprocess import main as command
    return command(argv)


if __name__ == "__main__":
    main()
