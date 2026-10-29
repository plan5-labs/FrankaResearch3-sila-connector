import sys

from unitelabs.cdk.cli import connector


def main() -> None:
    """Start this connector using the CDK configuration-aware CLI."""
    connector(args=["start", "--app", "unitelabs.fr3_connector:create_app", *sys.argv[1:]])


if __name__ == "__main__":
    main()
